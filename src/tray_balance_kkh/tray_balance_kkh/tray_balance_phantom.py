#!/usr/bin/env python3
# =============================================================
# tray_balance_phantom.py — tray_balance에 "레버 팬텀 모멘트" 보상 추가판
# -------------------------------------------------------------
# 실행: ros2 run move tray_balance_phantom  (또는 python3 이 파일)
# 노드: tray_balance_phantom | 토픽/서비스: tray_balance.py와 동일
#       (com_point_marker, com_axes TF, offset_debug_mm, control_debug_deg,
#        motion/move_stop) — 한 번에 하나만 실행할 것
# 의존: rclpy, doosan-robot2, tray_balance_kkh.tray_balance(수학/필터/상수 재사용)
# =============================================================
"""tray_balance.py의 제어 루프에 연장팔(레버) 팬텀 모멘트 보상을 넣은 버전.

왜 필요한가 (지난 실기 로그 진단)
--------------------------------
TCP에서 판 중심까지 중력수직 방향으로 큰 레버(실측 ~0.5m)가 있어, tare
토크가 ~1.27Nm였다. 판을 몇 도 기울이면 이 레버 모멘트가 sin(tilt)만큼
변하는데(3도에 ~26mm 상당의 가짜 offset), tray_balance.py는 tare 토크를
"고정값 1회"만 빼므로 그 변화분이 팬텀 offset으로 남는다. 그러면 기울임
→ 팬텀 발생 → 더 기울임의 양의 피드백으로 자기 진동(limit cycle)이 생긴다.

무엇을 바꿨나
-------------
- estimate_lever_arm(): tare wrench에서 레버팔의 중력수직 성분 r_perp를
  "측정"한다(추측 아님). 물체가 중심일 때 tare torque = r_ext x (W0 g_hat0)
  이므로 r_perp = (g_hat0 x tau0)/W0. 중력평행 성분은 관측 불가하지만 tilt가
  작을 때 2차 오차라 무시.
- tilt_phantom_torque(): 매 루프, 현재 tilt에서 예측되는 레버 모멘트에서
  tare 시점 레버 모멘트를 뺀 "팬텀"을 만든다(tare에서 0, tilt 커질수록 증가).
- delta_torque에서 (raw-tare) 뒤에 이 팬텀을 한 번 더 빼서 물체 기여만 남긴다.
- 비교를 위해 보상 전(raw) offset과 보상 후 offset을 상태 로그에 같이 찍는다.
  둘의 차이가 곧 제거된 팬텀 크기다.

한계
----
- r_perp의 중력평행 성분 미관측 → 큰 tilt(수십 도)에선 잔차. 서빙 tilt(≤15deg)
  범위에선 충분.
- 부호/유도는 이 파일의 _demo_phantom_closed_form self-test로 검증했다.
"""

import time as pytime

import rclpy
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
import DR_init

from tray_balance_kkh.tray_balance import (
    ROBOT_ID,
    ROBOT_MODEL,
    LOOP_HZ,
    KP,
    KD,
    MAX_TILT_DEG,
    COM_OK_THRESHOLD_M,
    CONTROL_DEADBAND_M,
    MEDIAN_WINDOW,
    FILTER_ALPHA,
    DEBUG_LOG_PERIOD_SEC,
    COMMAND_HZ,
    COMMAND_PERIOD_SEC,
    RETURN_TO_HOME,
    STABLE_OFFSET_M,
    STABLE_VEL_MPS,
    STABLE_HOLD_SEC,
    REARM_OFFSET_M,
    RETURN_VEL,
    BALL_RADIUS_M,
    SAFE_RADIUS_M,
    MIN_WEIGHT_N,
    OBJECT_WEIGHT_N,
    TARE_SAMPLE_COUNT,
    TARE_SAMPLE_PERIOD_SEC,
    COM_MARKER_FRAME_ID,
    COM_MARKER_TOPIC,
    COM_AXES_CHILD_FRAME_ID,
    OFFSET_DEBUG_TOPIC,
    CONTROL_DEBUG_TOPIC,
    _cross,
    _sub,
    _add,
    _scale,
    _norm,
    _basis_to_quaternion,
    calibrate_gravity_frame,
    estimate_plane_offset,
    measure_tare_offset,
    MedianPrefilter,
    ExponentialMovingAverage,
)

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# --- A안: g_hat0 고정 + full 레버 팬텀 보상 -----------------------------------
# 왜: tare로는 레버의 "중력수직 성분(r_perp)"만 관측된다. 실제로 TCP에서 판
# 중심까지의 큰 거리는 대부분 "중력평행" 방향(판이 TCP 아래 매달림)이라 tare에
# 안 잡히는데, 판을 기울이면 이 평행 레버가 수직 모멘트로 회전해 들어와 물체
# 신호(골프공 최대 0.045Nm)보다 큰 가짜 토크를 만든다 -> 기울임→팬텀→더 기울임
# 의 양의 피드백(발산). 평행 성분은 g_hat0 방향이므로 "거리 스칼라 하나"만
# 자로 재면 r_ext = r_perp + d*g_hat0 로 전체 레버를 복원해 full 팬텀을 뺄 수 있다.
D_TCP_TO_TRAY_M = 0.0   # TCP와 판중심이 중력방향으로 같은 높이인 rig 구조 -> 0.
                        #   (레버는 순수 수평=중력수직이라 tare의 r_perp가 곧 전체 레버.)
                        #   판중심이 TCP보다 아래로 매달리는 구조로 바꾸면 그 수직거리[m]를
                        #   자로 재서 넣을 것(부호: 아래로 +). 0이면 수직 레버만 보상.
G_HAT0_TOOL = None      # TODO(선택): joint6 정렬을 매번 재현하면 그때의 툴프레임 중력
                        #   단위벡터를 여기 상수로 박아 run간 g_hat0 흔들림(로그상 X: -0.07~0.47)을 없앤다.
                        #   예: (0.08, -0.78, -0.55). None이면 매 run tare에서 측정.


def estimate_lever_arm(tare_offset, d_parallel=0.0, g_hat0_fixed=None):
    """tare(물체 중심) wrench에서 TCP->판중심 "전체" 레버팔 r_ext를 복원.

    물체가 중심이라 tare 토크는 순수 연장팔 모멘트: tau0 = r_ext x (W0 g_hat0).
    양변에 g_hat0을 왼쪽 외적하면 g0 x tau0 = W0 * (r_ext의 g0수직 성분)이므로
    관측 가능한 부분은 r_perp = (g_hat0 x tau0) / W0 뿐이다 (g0평행 성분은 외적에서
    사라져 관측 불가). 그 미관측 평행 성분을 자로 잰 d_parallel[m]로 채워
    r_ext = r_perp + d_parallel * g_hat0 로 전체 레버를 만든다.
    g_hat0_fixed가 주어지면 노이즈 섞인 tare 방향 대신 그 상수를 기준 중력으로 쓴다.
    반환: (r_ext, g_hat0, weight0)."""
    g_vec = (tare_offset[0], tare_offset[1], tare_offset[2])
    weight0 = _norm(g_vec)
    if g_hat0_fixed is not None:
        g_hat0 = _scale(g_hat0_fixed, 1.0 / _norm(g_hat0_fixed))
    else:
        g_hat0 = _scale(g_vec, 1.0 / weight0)
    tau0 = (tare_offset[3], tare_offset[4], tare_offset[5])
    r_perp = _scale(_cross(g_hat0, tau0), 1.0 / weight0)
    r_ext = _add(r_perp, _scale(g_hat0, d_parallel))
    return r_ext, g_hat0, weight0


def tilt_phantom_torque(r_ext, g_hat0, weight0, g_hat_live, weight_live):
    """tilt로 변하는 레버 모멘트(=팬텀). tare에서 0, tilt 커질수록 증가.

    레버 모멘트 = r_ext x (W * g_hat). 현재 자세 값에서 tare 자세 값을 빼서,
    tilt 때문에 늘어난 만큼만 돌려준다(이걸 delta_torque에서 추가로 뺀다).
    r_ext에 중력평행 성분(estimate_lever_arm의 d_parallel)이 포함돼야 발산을
    일으키는 큰 항까지 상쇄된다 -- r_perp만 주면 예전처럼 미보상 잔차가 남는다."""
    moment_now = _scale(_cross(r_ext, g_hat_live), weight_live)
    moment_tare = _scale(_cross(r_ext, g_hat0), weight0)
    return _sub(moment_now, moment_tare)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("tray_balance_phantom", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            get_tool_force,
            get_current_posx,
            get_current_posj,
            movel,
            DR_TOOL,
            DR_BASE,
            DR_MV_MOD_REL,
            DR_MV_MOD_ABS,
            DR_MV_RA_DUPLICATE,
            DR_QSTOP,
        )
        from dsr_msgs2.srv import MoveStop
    except ImportError as error:
        node.get_logger().error(f"두산 로봇 모듈 import 실패: {error}")
        node.destroy_node()
        rclpy.shutdown()
        return

    stop_client = node.create_client(MoveStop, "motion/move_stop")

    def stop_robot(stop_mode=DR_QSTOP):
        if not stop_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"/{ROBOT_ID}/motion/move_stop 서비스를 찾을 수 없습니다.")
        request = MoveStop.Request()
        request.stop_mode = int(stop_mode)
        future = stop_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError("MoveStop 서비스 호출 실패")

    set_tool("Tool Weight_gripper")
    set_tcp("GripperDA_v1")

    com_marker_pub = node.create_publisher(Marker, COM_MARKER_TOPIC, 10)
    tf_broadcaster = TransformBroadcaster(node)
    offset_debug_pub = node.create_publisher(Vector3Stamped, OFFSET_DEBUG_TOPIC, 10)
    control_debug_pub = node.create_publisher(Vector3Stamped, CONTROL_DEBUG_TOPIC, 10)

    median_pre = MedianPrefilter(MEDIAN_WINDOW)
    filt = ExponentialMovingAverage(FILTER_ALPHA)
    last_debug_log = 0.0
    last_status_log = 0.0
    start_time = pytime.monotonic()

    prev_a, prev_b = 0.0, 0.0
    prev_time = start_time          # KD 속도항 기준 시점 -- 명령을 낼 때만 갱신
    last_command_time = start_time  # 마지막 movel 발사 시각 -- 명령율 게이팅용
    commanded_u_deg, commanded_v_deg = 0.0, 0.0
    was_balanced = False
    was_saturated = False

    # 안정 시 복귀용: 시작(쟁반 평행) 자세를 저장. get_current_posx는 (pose, sol)
    # 튜플을 반환하므로 pose만 꺼낸다. 실패하면 복귀 기능만 끈다.
    home_pose = None
    if RETURN_TO_HOME:
        try:
            _hp = get_current_posx(ref=DR_BASE)
            home_pose = _hp[0] if isinstance(_hp, (list, tuple)) and len(_hp) == 2 else _hp
            node.get_logger().info(f"홈(평행) 자세 저장: {[round(v, 2) for v in home_pose]}")
        except Exception as home_error:
            node.get_logger().warn(f"홈 자세 저장 실패, 복귀 기능 비활성: {home_error}")
    stable_since = None   # 안정 상태가 시작된 시각(연속 유지 판정)
    idle = False          # 홈 복귀 후 대기 상태(교란 전까지 movel 생략)

    node.get_logger().info(
        "물체를 판 정중앙에 놓고 손을 뗀 뒤 tare 측정을 시작합니다 "
        f"({TARE_SAMPLE_COUNT}개 샘플, 약 {TARE_SAMPLE_COUNT * TARE_SAMPLE_PERIOD_SEC:.1f}초)."
    )
    tare_offset, tare_std = measure_tare_offset(
        get_tool_force, DR_TOOL, return_std=True
    )
    node.get_logger().info(f"tare 기준값: {[round(v, 4) for v in tare_offset]}")
    node.get_logger().info(
        f"tare 노이즈(std): 힘={[round(v, 4) for v in tare_std[:3]]}N, "
        f"토크={[round(v, 4) for v in tare_std[3:]]}Nm "
        f"({TARE_SAMPLE_COUNT}샘플 {TARE_SAMPLE_COUNT * TARE_SAMPLE_PERIOD_SEC:.1f}초 평균) "
        f"-- std가 크면 영점 불량, 판/물체 고정 후 재측정"
    )

    g_hat, u_hat, v_hat, weight_n = calibrate_gravity_frame(tare_offset)

    # 레버 팬텀 보상: tare에서 수직 레버(r_perp)를 측정하고, 관측 불가한 평행
    # 성분은 자로 잰 D_TCP_TO_TRAY_M로 채워 "전체" 레버 r_ext를 만든다.
    r_ext, g_hat0, weight0 = estimate_lever_arm(
        tare_offset, d_parallel=D_TCP_TO_TRAY_M, g_hat0_fixed=G_HAT0_TOOL
    )
    node.get_logger().info(
        f"중력/레버 실측 보정: g_hat0={[round(v, 3) for v in g_hat0]}"
        f"{'(고정)' if G_HAT0_TOOL is not None else ''}, "
        f"weight0={weight0:.3f}N, 전체레버 r_ext={[round(v, 4) for v in r_ext]}m "
        f"(|r_ext|={_norm(r_ext)*1000:.1f}mm, 평행 d={D_TCP_TO_TRAY_M*1000:.0f}mm) "
        f"-- 이 레버로 tilt별 팬텀을 보상"
    )

    if weight_n < MIN_WEIGHT_N:
        node.get_logger().error(
            f"tare 시점 측정 무게={weight_n:.3f}N < {MIN_WEIGHT_N:.3f}N -- "
            "물체가 판 위에 없었을 수 있습니다. 시작하지 않습니다."
        )
        try:
            node.get_logger().error(
                f"진단용 현재 자세: posj={get_current_posj()}, "
                f"posx(BASE)={get_current_posx(ref=DR_BASE)}"
            )
        except Exception as pose_error:
            node.get_logger().error(f"현재 자세 조회 실패: {pose_error}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    # OBJECT_WEIGHT_N은 "판 위 물체만의 무게"이므로 전체 하중(weight0)보다 클 수 없다.
    # 크면 offset 배율이 틀려(작게 읽혀) 게인을 과하게 올리게 된다 -- 실측값 재확인 필요.
    if OBJECT_WEIGHT_N > weight0:
        node.get_logger().warn(
            f"OBJECT_WEIGHT_N={OBJECT_WEIGHT_N:.2f}N > 전체무게 weight0={weight0:.2f}N -- "
            "물리적으로 불가(물체가 전체보다 무거울 수 없음). offset이 실제보다 작게 읽혀 "
            "보상이 약해지고 KP를 과하게 키우게 됨. 물체만의 무게로 재설정할 것."
        )

    node.get_logger().info(
        f"tray_balance_phantom 시작: Kp={KP}, Kd={KD}, loop={LOOP_HZ}Hz, "
        f"명령율={COMMAND_HZ}Hz, safe_radius={SAFE_RADIUS_M*1000:.1f}mm, "
        f"물체무게={OBJECT_WEIGHT_N:.3f}N(전체 {weight_n:.2f}N 중) | 레버 팬텀 보상 ON | "
        "offset/tilt는 매 루프 계산·movel은 명령율로만 발사"
    )

    try:
        while rclpy.ok():
            raw_force = get_tool_force(ref=DR_TOOL)

            if raw_force == -1:
                node.get_logger().error(
                    "get_tool_force 읽기 실패 - 서비스 응답 없음. 루프를 정지합니다."
                )
                break

            raw_filtered = filt.update(median_pre.update(raw_force))
            g_hat, u_hat, v_hat, weight_n = calibrate_gravity_frame(raw_filtered)

            # offset은 "물체만의 무게"로 나눠야 실제 이동량(mm)이 된다. g_hat 방향은
            # 전체무게 weight_n으로 잡되(중력 방향), 크기 환산은 OBJECT_WEIGHT_N으로.
            # (전체무게로 나누면 물체 이동이 m전체/m물체배 축소돼 보상이 약해짐)
            # (1) 기존 방식: tare 토크만 뺀 raw offset (비교용)
            delta_raw = _sub(raw_filtered[3:6], tare_offset[3:6])
            a_raw, b_raw = estimate_plane_offset(g_hat, u_hat, v_hat, OBJECT_WEIGHT_N, delta_raw)

            # (2) 레버 팬텀 보상: tilt로 변한 레버 모멘트를 추가로 뺀다.
            #     팬텀 자체는 전체 하중이 만드는 모멘트라 weight_n(전체) 기준으로 계산.
            phantom = tilt_phantom_torque(r_ext, g_hat0, weight0, g_hat, weight_n)
            delta_torque = _sub(delta_raw, phantom)
            a, b = estimate_plane_offset(g_hat, u_hat, v_hat, OBJECT_WEIGHT_N, delta_torque)
            offset_norm = (a ** 2 + b ** 2) ** 0.5 *3
            # 제거된 팬텀 크기(보상 전후 offset 차) -- 로그로 효과 확인
            phantom_mm = ((a_raw - a) ** 2 + (b_raw - b) ** 2) ** 0.5 * 1000.0

            stamp = node.get_clock().now().to_msg()

            com_point = _add(_scale(u_hat, a), _scale(v_hat, b))
            marker = Marker()
            marker.header.frame_id = COM_MARKER_FRAME_ID
            marker.header.stamp = stamp
            marker.ns = "tray_balance"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = marker.scale.y = marker.scale.z = BALL_RADIUS_M * 2
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = com_point
            marker.pose.orientation.w = 1.0
            marker.color.a = 1.0
            marker.color.g = 1.0 if offset_norm <= COM_OK_THRESHOLD_M else 0.0
            marker.color.r = 1.0 if offset_norm > COM_OK_THRESHOLD_M else 0.0
            com_marker_pub.publish(marker)

            axes_tf = TransformStamped()
            axes_tf.header.frame_id = COM_MARKER_FRAME_ID
            axes_tf.header.stamp = stamp
            axes_tf.child_frame_id = COM_AXES_CHILD_FRAME_ID
            axes_tf.transform.translation.x = com_point[0]
            axes_tf.transform.translation.y = com_point[1]
            axes_tf.transform.translation.z = com_point[2]
            (
                axes_tf.transform.rotation.x,
                axes_tf.transform.rotation.y,
                axes_tf.transform.rotation.z,
                axes_tf.transform.rotation.w,
            ) = _basis_to_quaternion(u_hat, v_hat, g_hat)
            tf_broadcaster.sendTransform(axes_tf)

            offset_msg = Vector3Stamped()
            offset_msg.header.frame_id = COM_MARKER_FRAME_ID
            offset_msg.header.stamp = stamp
            offset_msg.vector.x, offset_msg.vector.y, offset_msg.vector.z = (
                a * 1000.0, b * 1000.0, offset_norm * 1000.0
            )
            offset_debug_pub.publish(offset_msg)

            if offset_norm > SAFE_RADIUS_M:
                node.get_logger().error(
                    f"물체 이탈 감지: offset={offset_norm*1000:.1f}mm > "
                    f"safe_radius={SAFE_RADIUS_M*1000:.1f}mm. 정지합니다."
                )
                try:
                    stop_robot(DR_QSTOP)
                except Exception as stop_error:
                    node.get_logger().error(f"비동기 모션 정지 실패: {stop_error}")
                break

            # 속도항은 "직전 명령 이후" 변화로 계산(prev는 명령 낼 때만 갱신).
            # 루프(10ms)마다 미분하면 노이즈가 폭증해 가짜 tilt를 쐈다 -- 명령
            # 간격(≈1/COMMAND_HZ)으로 미분하면 실제 이동속도에 가까워 훨씬 안정.
            now_wall = pytime.monotonic()
            dt = max(1e-3, now_wall - prev_time)
            a_dot = (a - prev_a) / dt
            b_dot = (b - prev_b) / dt

            raw_u_deg = -(KP * b + KD * b_dot)
            raw_v_deg = KP * a + KD * a_dot
            target_u_deg = max(-MAX_TILT_DEG, min(MAX_TILT_DEG, raw_u_deg))
            target_v_deg = max(-MAX_TILT_DEG, min(MAX_TILT_DEG, raw_v_deg))
            is_saturated = (
                abs(raw_u_deg) > MAX_TILT_DEG or abs(raw_v_deg) > MAX_TILT_DEG
            )

            u_delta_deg = target_u_deg - commanded_u_deg
            v_delta_deg = target_v_deg - commanded_v_deg

            control_target_vec = _add(
                _scale(u_hat, target_u_deg), _scale(v_hat, target_v_deg)
            )
            control_msg = Vector3Stamped()
            control_msg.header.frame_id = COM_MARKER_FRAME_ID
            control_msg.header.stamp = stamp
            control_msg.vector.x, control_msg.vector.y, control_msg.vector.z = control_target_vec
            control_debug_pub.publish(control_msg)

            now = pytime.monotonic() - start_time
            if now - last_debug_log >= DEBUG_LOG_PERIOD_SEC:
                node.get_logger().debug(
                    f"raw={[round(v, 3) for v in raw_force]} "
                    f"filt={[round(v, 3) for v in raw_filtered]} | "
                    f"offset_raw=({a_raw*1000:+.1f},{b_raw*1000:+.1f}) "
                    f"offset_comp=({a*1000:+.1f},{b*1000:+.1f})mm phantom={phantom_mm:.1f}mm | "
                    f"tilt=(u={target_u_deg:+.2f},v={target_v_deg:+.2f})deg"
                )
                last_debug_log = now

            is_balanced = offset_norm <= COM_OK_THRESHOLD_M
            if is_balanced != was_balanced:
                node.get_logger().info(
                    f"{'balanced' if is_balanced else 'unbalanced'}: "
                    f"offset={offset_norm*1000:.1f}mm "
                    f"(threshold={COM_OK_THRESHOLD_M*1000:.0f}mm)"
                )
                was_balanced = is_balanced

            if is_saturated != was_saturated:
                node.get_logger().warn(
                    f"{'tilt limit hit' if is_saturated else 'tilt back within range'}: "
                    f"raw target=(u={raw_u_deg:+.1f},v={raw_v_deg:+.1f})deg, "
                    f"limit=±{MAX_TILT_DEG}deg"
                )
                was_saturated = is_saturated

            active = offset_norm > CONTROL_DEADBAND_M   # 데드밴드 밖 = 보상 대상
            # 명령 발사는 COMMAND_HZ로만: 매 루프 계산은 하되 movel은 한 주기에 한 번.
            time_to_command = (now_wall - last_command_time) >= COMMAND_PERIOD_SEC

            # --- 안정 시 홈(쟁반 평행) 복귀 + idle ---------------------------------
            # 물체가 중심에 정착(offset 작음)하고 속도도 작은 상태가 STABLE_HOLD_SEC
            # 연속되면, 잔여 tilt를 홈 자세로 되돌리고 명령을 멈춰 미세 진동을 없앤다.
            # idle 중 offset이 REARM_OFFSET_M을 넘으면(교란) 제어를 자동 재개한다.
            offset_speed = (a_dot ** 2 + b_dot ** 2) ** 0.5
            if RETURN_TO_HOME and home_pose is not None:
                is_stable = (
                    offset_norm < STABLE_OFFSET_M and offset_speed < STABLE_VEL_MPS
                )
                if idle:
                    if offset_norm > REARM_OFFSET_M:
                        idle = False
                        stable_since = None
                        node.get_logger().info(
                            f"교란 감지(offset={offset_norm*1000:.1f}mm) -> 제어 재개"
                        )
                elif is_stable:
                    if stable_since is None:
                        stable_since = now_wall
                    elif (now_wall - stable_since) >= STABLE_HOLD_SEC:
                        node.get_logger().info(
                            "안정 감지 -> 초기(쟁반 평행) 자세로 복귀 후 대기"
                        )
                        try:
                            movel(
                                home_pose,
                                vel=RETURN_VEL,
                                acc=RETURN_VEL,
                                ref=DR_BASE,
                                mod=DR_MV_MOD_ABS,
                                ra=DR_MV_RA_DUPLICATE,
                            )
                        except Exception as return_error:
                            node.get_logger().error(f"홈 복귀 실패: {return_error}")
                        commanded_u_deg, commanded_v_deg = 0.0, 0.0
                        prev_a, prev_b, prev_time = a, b, now_wall
                        last_command_time = now_wall
                        idle = True
                else:
                    stable_since = None

            if now - last_status_log >= DEBUG_LOG_PERIOD_SEC:
                node.get_logger().info(
                    f"[{'대기(홈)' if idle else ('보상 ON' if active else '대기')}] "
                    f"offset={offset_norm*1000:5.1f}mm (a={a*1000:+.1f},b={b*1000:+.1f}) | "
                    f"raw={((a_raw**2+b_raw**2)**0.5)*1000:5.1f}mm 팬텀제거={phantom_mm:4.1f}mm | "
                    f"tilt목표(u={target_u_deg:+.2f},v={target_v_deg:+.2f})deg | "
                    # tare 뺀 토크변화 3축[Nm] -- 물체를 각 방향으로 밀며 어느 축이
                    # 반응 약한지(=감지 약한 방향) 확인용
                    f"dT=({delta_raw[0]:+.4f},{delta_raw[1]:+.4f},{delta_raw[2]:+.4f})Nm"
                )
                last_status_log = now

            if active and time_to_command and not idle:
                commanded_u_deg, commanded_v_deg = target_u_deg, target_v_deg
                # 이 시점(정착된 샘플)을 다음 KD 속도항의 기준으로 삼는다.
                prev_a, prev_b, prev_time = a, b, now_wall
                last_command_time = now_wall

                # movel REL + ref=DR_TOOL의 자세 슬롯 [rx,ry,rz]는 ZYZ 오일러가
                # 아니라 "툴 축 기준 회전벡터(axis-angle, deg)"로 해석된다 -- 실기
                # 검증됨(2026-07: movel([0,0,0,5,0,0],REL,DR_TOOL)이 툴 X축으로 회전).
                # 따라서 u_hat/v_hat 회전벡터를 그대로 합성해 넣으면 되고, 그립
                # 방향(g_hat)이 바뀌어도 축이 안 틀어진다 (오일러 변환 불필요).
                rotation_vec = _add(
                    _scale(u_hat, u_delta_deg),
                    _scale(v_hat, v_delta_deg),
                )
                movel(
                    [0, 0, 0, rotation_vec[0], rotation_vec[1], rotation_vec[2]],
                    vel=30,
                    acc=30,
                    ref=DR_TOOL,
                    mod=DR_MV_MOD_REL,
                    ra=DR_MV_RA_DUPLICATE,
                )

            pytime.sleep(1.0 / LOOP_HZ)

    except KeyboardInterrupt:
        node.get_logger().info("사용자 요청으로 정지")

    except Exception as error:
        node.get_logger().error(f"Robot Error: {error}")

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _demo_estimate_lever_arm():
    # 임의 레버(중력수직)와 중력방향으로 tare torque를 만들고 되돌려 확인
    from tray_balance_kkh.tray_balance import _normalize, _dot
    weight0 = 2.5
    g_hat0 = _normalize((0.37, -0.56, -0.74))
    r_true = (0.05, 0.10, -0.02)
    r_perp_true = _sub(r_true, _scale(g_hat0, _dot(r_true, g_hat0)))  # 관측 가능한 성분
    d_true = _dot(r_true, g_hat0)                                     # 미관측 평행 성분(자로 잰 값에 해당)
    tau0 = _scale(_cross(r_true, g_hat0), weight0)                    # = r_perp x (W g)
    tare = [weight0 * g_hat0[0], weight0 * g_hat0[1], weight0 * g_hat0[2],
            tau0[0], tau0[1], tau0[2]]
    # d_parallel=0: 수직 성분만 복원 (기존 동작)
    r_ext0, g_out, w_out = estimate_lever_arm(tare)
    assert abs(w_out - weight0) < 1e-9
    assert all(abs(r_ext0[i] - r_perp_true[i]) < 1e-9 for i in range(3)), (r_ext0, r_perp_true)
    # d_parallel=실측 -> 전체 레버 r_true를 그대로 복원해야 함
    r_ext, _, _ = estimate_lever_arm(tare, d_parallel=d_true)
    assert all(abs(r_ext[i] - r_true[i]) < 1e-9 for i in range(3)), (r_ext, r_true)
    print("estimate_lever_arm self-check OK")


def _demo_phantom_closed_form():
    """핵심 검증: 레버+물체가 같이 실린 토크에서, tilt 팬텀을 보상하면
    물체 offset이 그대로 복원되는가 (기울어도 팬텀에 안 속는가)."""
    from tray_balance_kkh.tray_balance import _normalize, _dot

    weight = 2.5
    g_hat0 = _normalize((0.37, -0.56, -0.74))
    # 실제 레버(0.5m급) -- 대부분 중력평행(판이 TCP 아래 매달림). tare에선 관측
    # 불가하지만 자로 잰 d_parallel로 채워 넣으면 전체 레버가 복원된다.
    r_lever = _scale(_normalize((0.2, 0.9, -0.3)), 0.5)
    d_parallel = _dot(r_lever, g_hat0)   # 자로 잰 TCP->판중심 중력방향 거리에 해당
    tau0 = _scale(_cross(r_lever, g_hat0), weight)
    tare = [weight * g_hat0[0], weight * g_hat0[1], weight * g_hat0[2],
            tau0[0], tau0[1], tau0[2]]
    r_ext, g0, w0 = estimate_lever_arm(tare, d_parallel=d_parallel)
    # 참고: d_parallel 없이(=0)는 평행 레버 미복원 -> 아래에서 발산성 잔차 확인
    r_perp_only, _, _ = estimate_lever_arm(tare)

    # 판을 기울여 중력이 tool 프레임에서 g_live로 회전, 물체는 판 평면 내 r_obj
    g_live = _normalize((0.45, -0.62, -0.64))
    g_hatL, u_L, v_L, wL = calibrate_gravity_frame(
        [weight * g_live[0], weight * g_live[1], weight * g_live[2], 0, 0, 0]
    )
    r_obj = _add(_scale(u_L, 0.03), _scale(v_L, -0.02))  # 실제 물체 offset

    # 현재 자세에서 측정될 토크 = (레버 + 물체) x (W g_live)
    tau_now = _scale(_cross(_add(r_lever, r_obj), g_live), weight)
    raw = [weight * g_live[0], weight * g_live[1], weight * g_live[2],
           tau_now[0], tau_now[1], tau_now[2]]

    # 보상 없이 -> 팬텀 섞임 / full 레버 보상 후 -> 물체만
    delta_raw = _sub(raw[3:6], tare[3:6])
    a_raw, b_raw = estimate_plane_offset(g_hatL, u_L, v_L, wL, delta_raw)
    phantom = tilt_phantom_torque(r_ext, g0, w0, g_hatL, wL)
    a, b = estimate_plane_offset(g_hatL, u_L, v_L, wL, _sub(delta_raw, phantom))

    # 평행 성분 미보상(r_perp만) -> 잔차가 남아 발산성 오차
    phantom_perp = tilt_phantom_torque(r_perp_only, g0, w0, g_hatL, wL)
    a_p, b_p = estimate_plane_offset(g_hatL, u_L, v_L, wL, _sub(delta_raw, phantom_perp))

    assert abs(a - 0.03) < 1e-6 and abs(b - (-0.02)) < 1e-6, (a, b)
    # 보상 전에는 눈에 띄게 틀렸어야 의미가 있음(팬텀이 실재)
    assert (abs(a_raw - 0.03) + abs(b_raw + 0.02)) > 0.005, (a_raw, b_raw)
    # r_perp만으로는 여전히 크게 틀림 -> full 레버가 필요함을 증명
    assert (abs(a_p - 0.03) + abs(b_p + 0.02)) > 0.005, (a_p, b_p)
    print(f"phantom closed-form self-check OK "
          f"(보상 전 offset=({a_raw*1000:+.1f},{b_raw*1000:+.1f}) "
          f"-> 보상 후=({a*1000:+.1f},{b*1000:+.1f})mm, 참값=(+30.0,-20.0))")


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
        _demo_estimate_lever_arm()
        _demo_phantom_closed_form()
    else:
        main()
