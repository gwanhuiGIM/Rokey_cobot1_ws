#!/usr/bin/env python3
"""tray_balance.py -- TCP 힘/모멘트 피드백 기반 쟁반 위 물체 중심 유지.

그립 자세 가정 -- 이제 없음 (실측으로 자동 보정, 4장 참고)
-------------------------------------------------------
이전 버전들은 "툴 +X가 아래" 또는 "툴 +Z가 아래"처럼 특정 축이 중력과
나란하다고 가정했다. 실제 로봇(DART 플랫폼, GripperDA_v1 TCP)에서
+X 정렬로 판을 수평 잡은 상태로 측정해보니 Fx,Fy,Fz가 모두 비슷한
크기로 나왔다 -- 즉 TCP 좌표계에서 중력 방향이 어느 좌표축과도 깨끗이
나란하지 않다(대략 40도 정도 기울어진 방향). 축을 하드코딩해 맞히는
대신, calibrate_gravity_frame()이 tare 측정값(정지 상태 raw force)
자체를 "중력 방향"으로 직접 사용한다 -- 그립을 다시 바꾸거나 TCP를
재정의해도 이 코드는 그대로 동작한다.

핵심 기능
---------
- measure_tare_offset()으로 물체를 판 정중앙에 둔 상태의 raw force를
  측정한다. 이 값의 선형 힘 성분(Fx,Fy,Fz)이 곧 TCP 좌표계에서의 중력
  방향이자 크기이므로, calibrate_gravity_frame()이 이를 정규화해 단위
  벡터 g_hat과 그에 수직인 판 평면 내 정규직교 기저(u_hat, v_hat)를
  만든다 -- 특정 축이 아래를 향한다고 가정하지 않는다.
- estimate_plane_offset()이 tau = r x F를 이 기저로 일반화해서 판 위
  물체의 평면 내 오프셋(a: u_hat 성분, b: v_hat 성분)을 역산한다.
- PD 제어(KP/KD)로 오프셋에 비례한 회전 보정량을 계산한다: u_hat 축
  회전이 b를, v_hat 축 회전이 a를 되돌린다(부호는 표준 시뮬레이션 없이
  독립 스크립트로 검증 -- 아래 한계 참고). 두 회전을 벡터로 합성해
  movel에 [rx,ry,rz] 3축 모두로 보낸다(g_hat 축 회전은 판 법선을 축으로
  제자리 회전이라 기울기에 기여하지 않으므로 자연히 성분이 0에 가까움).
- 오프셋이 CONTROL_DEADBAND_M(채터링 방지용 최소값) 이내일 때만 명령을
  생략한다 -- COM_OK_THRESHOLD_M(더 큰 값)은 "balanced" 로그 판정에만
  쓰고 제어는 안 멈춘다 (둘을 같은 값으로 쓰면 물체가 중심이 아니라
  threshold 경계에 걸쳐서 안착하는 문제가 있었음).
- tare 직후 측정된 무게(weight_n)가 너무 작으면(물체가 없었을 가능성)
  제어 루프를 시작하지 않고 즉시 종료한다.
- offset_norm이 SAFE_RADIUS_M(판 반지름-공 반지름)을 넘으면 물체가
  이미 가장자리를 벗어난 것으로 보고 MoveStop 서비스로 즉시 정지한다.
- 실시간 디버깅: 무게 작용점을 Marker(구)+TF(u_hat/v_hat/g_hat 축, rviz2
  Axes로 확인)로 COM_MARKER_TOPIC/COM_AXES_CHILD_FRAME_ID에 퍼블리시하고,
  오프셋(mm)/목표 tilt(deg)를 Vector3Stamped로 OFFSET_DEBUG_TOPIC/
  CONTROL_DEBUG_TOPIC에 퍼블리시한다 (rqt_plot, PlotJuggler용).

전제 및 한계 (현장 투입 전 반드시 확인)
--------------------------------------
- TRAY_RADIUS_M/BALL_RADIUS_M은 "지름 200mm 판 + 골프공" 기준값이다.
  다른 판/물체로 바꾸면 이 2개 상수를 교체해야 한다 (물체 질량은 더 이상
  하드코딩하지 않고 tare에서 직접 측정하므로 GOLF_BALL_MASS_KG는 이제
  최소 무게 판정에만 참고로 쓰인다).
- tare는 시작 시 1회만 측정한다. 측정 중 손으로 물체를 잡고 있으면 그
  힘이 기준값에 섞이므로, 로그 안내대로 손을 뗀 뒤 측정해야 한다.
- calibrate_gravity_frame/estimate_plane_offset/제어 부호는
  tray_balance_sim.py가 아니라 이 파일 자체의 _demo_general_closed_loop
  self-test(임의의 중력 방향 30개로 폐루프 수렴 검증)로 확인했다 --
  tray_balance_sim.py/viz.py/debug.py는 아직 옛 "Z가 아래" 가정을 쓰므로
  이 파일의 실제 동작과 더 이상 일치하지 않는다 (요청에 따라 그대로 둠).
- 마찰 모델(가속도=g*sin(tilt) 소각 근사)은 실제 물체의 구름/미끄럼
  저항과 다르다. 게인은 참고용이며 실기 튜닝이 필요하다.
"""

import math
import time as pytime

import rclpy
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# 무게 작용점 실시간 디버깅용 -- tray_balance_viz.py의 PARENT_FRAME과 동일하게
# 실제 로봇 flange TF 이름으로 맞출 것 (다르면 rviz2에서 마커가 로봇에 안 붙어 보임)
COM_MARKER_FRAME_ID = "link_6"
COM_MARKER_TOPIC = "com_point_marker"
# 무게 작용점에 (u_hat,v_hat,g_hat) 기저로 그린 TCP 기준 xyz축 -- rviz2에서
# TF > Axes로 바로 보임 (커스텀 화살표 마커 불필요)
COM_AXES_CHILD_FRAME_ID = "com_axes"
# rqt_plot/PlotJuggler로 실시간 그래프 확인용 시계열 토픽
OFFSET_DEBUG_TOPIC = "offset_debug_mm"      # x=a, y=b, z=offset_norm (mm)
CONTROL_DEBUG_TOPIC = "control_debug_deg"   # x,y,z = 목표 tilt rx,ry,rz (deg, TCP 기준)

LOOP_HZ = 70.0
KP = 300.0                     # deg of tilt per (m of offset). 실기 시작값 -- 이전 3.0은
                              #   75mm 벗어나도 tilt 0.2deg뿐이라 구르는 물체를 못 되돌렸다.
                              #   약하면 100~150까지 올릴 것(30mm를 5deg로 되돌리려면 ~167).
KD = 0.0                     # deg of tilt per (m/s of offset velocity). KP 올린 만큼 같이 올려
                              #   감쇠 유지 -- P만 키우면 물체가 중심을 오가며 진동한다.
MAX_TILT_DEG = 30.0
COM_OK_THRESHOLD_M = 0.008     # "balanced" 로그 판정 기준 (2cm) -- 제어는 이걸로 안 멈춤
CONTROL_DEADBAND_M = 0.0005    # 이 안쪽이면만 movel 전송 생략 (채터링 방지). 그 밖은 항상 중심(0)으로 계속 보정
MEDIAN_WINDOW = 7            # 토크센서 스파이크/모터 진동 제거용 중앙값 창(홀수, EMA 앞단).
                            #   5->7로 키움(진동 억제↑, 지연 ~3샘플). 명령을 COMMAND_HZ로만 내므로 지연 여유 있음.
FILTER_ALPHA = 0.15          # EMA smoothing factor, lower = smoother/slower. 0.2->0.15(진동 억제↑)
DEBUG_LOG_PERIOD_SEC = 0.5     # how often to print per-loop sensor debug

# 측정/명령 분리 (모터 진동↔센서 커플링 완화, tray_balance_phantom에서 사용).
# F/T 센서는 중력 모멘트 + 모션 관성/진동을 같이 잰다. 서보가 계속 돌면 진동이
# offset에 섞이고, 높은 게인이 이를 명령으로 증폭해 리밋사이클이 된다. 해법:
# offset/필요 tilt는 매 루프(LOOP_HZ)로 계속 계산하되, movel은 COMMAND_HZ로만
# 발사한다(직전 명령 후 한 주기 경과 = 모션 정착 후의 샘플로만 명령). KD 속도항도
# 이 명령 간격 기준으로 미분해 노이즈를 억제한다.
COMMAND_HZ = 30.0             # movel 발사 주기[Hz]. 낮출수록 안정(진동↓)·반응↓.
                             #   주기(1/HZ)는 한 스텝 movel 완료시간보다 커야 정착본을 읽는다
                             #   -- 스텝이 크면 COMMAND_HZ를 더 낮추거나 movel vel을 올릴 것.
COMMAND_PERIOD_SEC = 1.0 / COMMAND_HZ

# 판/물체 실측값 -- 다른 판이나 물체로 바꾸면 이 4개를 같이 교체할 것
# 주의: 이름은 legacy(골프공)지만 값은 "판 위에서 움직이는 물체 전체" 질량이다.
#   현재 실측: 골프공 47g + 플라스틱컵 17g + 쇠막대 65g*2 = 194g. (sim/viz/debug도 import)
GOLF_BALL_MASS_KG = 0.194    # 판 위 물체 전체 질량[kg] (offset 스케일 OBJECT_WEIGHT_N에 사용)
TRAY_RADIUS_M = 80          # 지름 200mm 원형 판
BALL_RADIUS_M = 0.04133       # 골프공 지름 42.67mm
SAFE_RADIUS_M = TRAY_RADIUS_M  # 공 중심이 이 밖이면 이미 가장자리 이탈


# 물체 위치 추정용 "판에 올리는 물체만의 무게" -- offset = 토크변화 / 이 무게.
# 판+그리퍼 포함 전체무게(weight_n, ~5N)로 나누면, 위치에 따라 변하는 토크는
# 물체 무게에만 비례하므로 실제 이동이 (m전체/m물체)배(~12x)만큼 축소돼 보인다
# -> 보상이 약해지고 SAFE_RADIUS도 무의미해짐. 반드시 물체만의 무게로 나눌 것.
OBJECT_WEIGHT_N = GOLF_BALL_MASS_KG * 9.81  # TODO(확정): 저울로 물체 실측 후 교체(골프공 기본값)

# TCP-판 중심 간 연장 거리 tare (판 중심에 물체를 두고 측정한 기준값을 빼서 상쇄)
# 센서 노이즈/저주파 드리프트가 커서 길게(약 3초) 평균해야 영점이 안정된다
# (0.4초=20샘플일 때 run간 무게가 ±0.2N 흔들렸다). 백색잡음은 √N로 줄지만
# 드리프트는 안 줄므로, 이 시간 동안 판/물체를 흔들리지 않게 고정할 것.
TARE_SAMPLE_COUNT = 150
TARE_SAMPLE_PERIOD_SEC = 0.02


# tare 시점 측정 무게가 이보다 작으면 "물체가 없었다"로 판정 (골프공의 30% 정도)
MIN_WEIGHT_N = GOLF_BALL_MASS_KG * 9.81 * 0.3

# 안정 시 초기(쟁반 평행) 자세 복귀 + idle (tray_balance_phantom에서 사용).
# 물체가 중심에 정착해 한동안 안 움직이면, 잔여 tilt를 홈 자세로 되돌리고 명령을
# 멈춘다(계속되는 미세 보정=진동 방지). 다시 교란되면 자동 재개.
RETURN_TO_HOME = True         # 복귀 기능 on/off (연속 제어만 원하면 False)
STABLE_OFFSET_M = 0.002       # 이 안이고 (idle 진입 기준 -- 노이즈 바닥 ~0.7mm보다 커야 함)
STABLE_VEL_MPS = 0.010        # offset 변화속도도 이 안이며
STABLE_HOLD_SEC = 2.0         # 이만큼 연속 유지되면 "안정" -> 홈 복귀 후 idle
# [STABLE_OFFSET, REARM_OFFSET] = 물체를 무시하는 사각지대. 이 밴드가 물체 실제
# 이동폭보다 크면 idle에 갇혀 영영 재개 안 됨(실기: 10mm였는데 물체가 4.5mm까지만
# 움직여 갇힘). 물체 이동폭 안쪽으로, 단 노이즈(~0.7mm) 위로 잡을 것. REARM>STABLE 유지.
REARM_OFFSET_M = 0.004        # idle 중 offset이 이보다 커지면(교란) 제어 재개
RETURN_VEL = 15               # 홈 복귀 movel 속도[%]/가속 -- 천천히 되돌림

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def estimate_com_offset(force_xyzrxryrz, payload_mass_kg=1, g=9.81):
    """tau = r x F -> 판 위 물체의 (dx, dy) 역산 (툴 프레임).

    가정: 판의 법선 = 툴 +Z이고 그 +Z가 아래(중력) 방향과 나란함 (실제
    그립 방향 -- tray_balance.py의 실제 제어 루프가 쓰는 함수). 판은 툴
    X-Y 평면에 위치. F ~= (0, 0, +weight) (Z가 중력과 같은 방향이므로
    물체 무게가 +Z로 실림, 안 기울어진 상태 근사). r=(dx, dy, r_z)로
    두고 풀면 Tx = dy*weight, Ty = -dx*weight.
    """
    tx, ty = force_xyzrxryrz[3], force_xyzrxryrz[4]
    weight = payload_mass_kg * g
    dx = -ty / weight
    dy = tx / weight
    return dx, dy


def estimate_plate_offset(force_xyzrxryrz, payload_mass_kg=1, g=9.81):
    """tau = r x F -> 판 위 물체의 (offset_y, offset_z) 역산 (툴 프레임).

    가정: 판의 법선 = 툴 +X (아래/중력 방향), 판은 툴 Y-Z 평면에 위치.
    이 저장소의 실제 그립은 +Z가 법선이라 tray_balance.py의 제어 루프는
    이 함수를 쓰지 않는다 -- 그립 방향을 다시 바꿀 경우를 위해 남겨둔
    대안 물리식 (그때는 아래 estimate_com_offset 대신 이걸 쓰고, movel
    회전축도 Y/Z로 바꿔야 한다).
    F ~= (+weight, 0, 0) (안 기울어진 상태 근사). r=(0, offset_y, r_z)로
    두고 풀면 Ty = r_z*weight, Tz = -offset_y*weight.
    r_z(=연장거리+offset_z)의 연장거리 성분은 tare로 상쇄된 뒤 입력된다고
    가정하므로, 여기서는 Ty를 그대로 offset_z*weight로 취급한다.
    """
    ty, tz = force_xyzrxryrz[4], force_xyzrxryrz[5]
    weight = payload_mass_kg * g
    offset_y = -tz / weight
    offset_z = ty / weight
    return offset_y, offset_z


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _norm(a):
    return math.sqrt(_dot(a, a))


def _normalize(a):
    return _scale(a, 1.0 / _norm(a))


def _basis_to_quaternion(u_hat, v_hat, g_hat):
    """(u_hat,v_hat,g_hat)를 열로 하는 회전행렬 -> 쿼터니언(x,y,z,w).
    표준 trace 기반 변환 (Shepperd's method)."""
    m00, m10, m20 = u_hat
    m01, m11, m21 = v_hat
    m02, m12, m22 = g_hat
    trace = m00 + m11 + m22
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m21 - m12) * s
        y = (m02 - m20) * s
        z = (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    return x, y, z, w


def calibrate_gravity_frame(tare_offset):
    """tare 시점 raw force(정지 상태)의 힘 성분(Fx,Fy,Fz) 자체를 중력
    방향/크기로 직접 사용한다 -- 특정 축이 아래를 향한다고 가정하지 않음.
    F0 = weight_n * g_hat 로 정의(둘 다 이 tare 측정에서 나온 값)하므로,
    이후 모든 계산은 반드시 이 부호 관례를 그대로 따라야 한다.

    반환: (g_hat, u_hat, v_hat, weight_n). (u_hat, v_hat, g_hat)는
    u_hat x v_hat = g_hat인 우수 정규직교 기저 -- u_hat,v_hat이 판 평면을
    이룬다."""
    g_vec = (tare_offset[0], tare_offset[1], tare_offset[2])
    weight_n = _norm(g_vec)
    g_hat = _scale(g_vec, 1.0 / weight_n)

    reference = (1.0, 0.0, 0.0) if abs(g_hat[0]) < 0.9 else (0.0, 1.0, 0.0)
    u_hat = _normalize(_sub(reference, _scale(g_hat, _dot(reference, g_hat))))
    v_hat = _cross(g_hat, u_hat)

    return g_hat, u_hat, v_hat, weight_n


def estimate_plane_offset(g_hat, u_hat, v_hat, weight_n, delta_torque):
    """tau = r x F를 특정 좌표축에 의존하지 않게 일반화 -- 판 평면
    (calibrate_gravity_frame이 만든 u_hat,v_hat) 내 2D 오프셋(a,b)을
    delta_torque(tare 기준 대비 토크 변화)로부터 역산한다.

    유도: F = weight_n*g_hat, tau = r x F = weight_n*(r x g_hat).
    r가 g_hat에 수직(판 평면 내)이라는 조건과 (r x g_hat) x g_hat = -r
    항등식을 쓰면 r = (g_hat x tau) / weight_n. calibrate_gravity_frame의
    F0 = weight_n*g_hat 부호 관례와 반드시 짝이 맞아야 하며, 독립
    스크립트로 폐루프 수렴을 검증한 뒤 정한 부호다 (_demo_general_closed_loop
    참고)."""
    r_perp = _scale(_cross(g_hat, delta_torque), 1.0 / weight_n)
    a = _dot(r_perp, u_hat)
    b = _dot(r_perp, v_hat)
    return a, b


def measure_tare_offset(get_tool_force, dr_tool, sample_count=TARE_SAMPLE_COUNT,
                        return_std=False):
    """물체를 판 정중앙에 둔 상태로 여러 샘플을 평균 -- TCP에서 판 중심까지의
    연장 거리 때문에 생기는 상수 모멘트를 이 기준값으로 상쇄한다.
    return_std=True면 채널별 표준편차도 반환한다 (영점 품질/노이즈 확인용)."""
    samples = []
    for _ in range(sample_count):
        samples.append(get_tool_force(ref=dr_tool))
        pytime.sleep(TARE_SAMPLE_PERIOD_SEC)
    means = [sum(axis) / len(axis) for axis in zip(*samples)]
    if not return_std:
        return means
    stds = [
        (sum((s - m) ** 2 for s in axis) / len(axis)) ** 0.5
        for axis, m in zip(zip(*samples), means)
    ]
    return means, stds


def check_grip_orientation(tare_offset, payload_mass_kg=1, g=9.81, tolerance_ratio=0.3):
    """tare 시점의 raw force가 그립 가정(판 법선 = 툴 +Z)과 맞는지 점검.

    맞다면 물체 무게가 거의 전부 Fz로 실리고 Fx,Fy는 작아야 한다. 둘 중
    하나라도 어긋나면 (1) tare 시점에 물체가 판 위에 없었거나 (2) 실제
    그립 방향이 이 코드가 가정한 축과 다르다는 뜻이다 -- 문제 설명을
    반환하고, 문제 없으면 None."""
    expected_weight = payload_mass_kg * g
    fx, fy, fz = tare_offset[0], tare_offset[1], tare_offset[2]
    lateral_mag = math.hypot(fx, fy)

    if abs(fz) < expected_weight * (1.0 - tolerance_ratio):
        return (
            f"tare 시점 Fz={fz:.2f}N이 예상 무게 {expected_weight:.2f}N보다 작습니다 -- "
            "tare 측정 중 물체가 판 위에 없었을 수 있습니다."
        )

    if lateral_mag > expected_weight * tolerance_ratio:
        return (
            f"tare 시점 Fx,Fy 크기={lateral_mag:.2f}N가 무시하기엔 큽니다 -- "
            "그립 가정(툴 +Z=중력 방향)과 실제 자세가 다를 수 있습니다."
        )

    return None


class MedianPrefilter:
    """채널별 최근 N개 중앙값 -- 토크센서의 튀는 스파이크(outlier)를 EMA
    앞단에서 제거한다. EMA는 스파이크를 평균에 섞어버려 못 잡으므로, 중앙값
    으로 먼저 거른 뒤 EMA로 부드럽게 한다. N은 홀수 권장(지연 ~N/2 샘플)."""

    def __init__(self, window):
        self.window = window
        self.buffers = None

    def update(self, sample):
        if self.buffers is None:
            self.buffers = [[float(s)] * self.window for s in sample]
        out = []
        for i, s in enumerate(sample):
            buf = self.buffers[i]
            buf.append(float(s))
            del buf[0]
            out.append(sorted(buf)[len(buf) // 2])
        return out


class ExponentialMovingAverage:
    def __init__(self, alpha):
        self.alpha = alpha
        self.value = None

    def update(self, sample):
        if self.value is None:
            self.value = list(sample)
        else:
            self.value = [
                self.alpha * s + (1 - self.alpha) * v
                for s, v in zip(sample, self.value)
            ]
        return self.value


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("tray_balance", namespace=ROBOT_ID)
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
    prev_time = start_time
    commanded_u_deg, commanded_v_deg = 0.0, 0.0
    was_balanced = False
    was_saturated = False

    node.get_logger().info(
        "물체를 판 정중앙에 놓고 손을 뗀 뒤 tare 측정을 시작합니다 "
        f"({TARE_SAMPLE_COUNT}개 샘플, 약 {TARE_SAMPLE_COUNT * TARE_SAMPLE_PERIOD_SEC:.1f}초)."
    )
    tare_offset = measure_tare_offset(get_tool_force, DR_TOOL)
    node.get_logger().info(
        f"tare 기준값: {[round(v, 4) for v in tare_offset]}"
    )

    g_hat, u_hat, v_hat, weight_n = calibrate_gravity_frame(tare_offset)
    node.get_logger().info(
        f"중력 방향 실측 보정: g_hat(TCP좌표)={[round(v, 3) for v in g_hat]}, "
        f"weight={weight_n:.3f}N -- 특정 축을 가정하지 않고 이 값을 그대로 씀"
    )

    if weight_n < MIN_WEIGHT_N:
        node.get_logger().error(
            f"tare 시점 측정 무게={weight_n:.3f}N < {MIN_WEIGHT_N:.3f}N -- "
            "물체가 판 위에 없었을 수 있습니다. 시작하지 않습니다."
        )
        # 진단용으로 현재 관절각/TCP pose도 같이 남긴다.
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

    node.get_logger().info(
        f"tray_balance 시작: Kp={KP}, loop={LOOP_HZ}Hz, "
        f"safe_radius={SAFE_RADIUS_M*1000:.1f}mm | "
        f"rviz2: marker=/{ROBOT_ID}/{COM_MARKER_TOPIC}, "
        f"tf={COM_MARKER_FRAME_ID}->{COM_AXES_CHILD_FRAME_ID} | "
        f"rqt_plot/PlotJuggler: /{ROBOT_ID}/{OFFSET_DEBUG_TOPIC}, "
        f"/{ROBOT_ID}/{CONTROL_DEBUG_TOPIC}"
    )

    try:
        while rclpy.ok():
            raw_force = get_tool_force(ref=DR_TOOL)

            if raw_force == -1:
                # get_tool_force 서비스 호출 실패 시 -1을 반환한다.
                node.get_logger().error(
                    "get_tool_force 읽기 실패 - 서비스 응답 없음. 루프를 정지합니다."
                )
                break

            # tare 시점 고정 기저가 아니라, 매 루프 "지금 이 자세"의 중력
            # 기저를 다시 잡는다. get_tool_force(ref=DR_TOOL)의 선형 성분
            # (Fx,Fy,Fz)은 언제나 현재(회전했을 수도 있는) TCP 프레임 기준
            # 중력 방향이므로, 이 값을 그대로 쓰면 로봇이 기울어도 역산이
            # 어긋나지 않는다 -- 누적 명령각을 Rodrigues로 돌리는 것보다
            # 코드도 적고, 실제로 실행된 각이 아니라 측정값이라 더 정확하다.
            # 단, 현재 게인(KP=3deg/m)에선 tilt가 <0.25deg라 tare 고정 대비
            # 차이는 ~1mm(센서 노이즈 이하)다. 게인을 크게 올려 tilt가 수 deg가
            # 되면 이 보정이 의미를 갖는다.
            # ponytail: 여기서 남는 미보상 항 -- TCP~판중심 연장팔(r_ext)이
            #   만드는 모멘트도 tilt에 따라 변한다(r_ext x g). tare로 1회만
            #   빼므로 큰 tilt에선 이게 지배적 오차. 필요해지면 r_ext 모델링 추가.
            raw_filtered = filt.update(median_pre.update(raw_force))
            g_hat, u_hat, v_hat, weight_n = calibrate_gravity_frame(raw_filtered)
            delta_torque = _sub(raw_filtered[3:6], tare_offset[3:6])
            a, b = estimate_plane_offset(g_hat, u_hat, v_hat, weight_n, delta_torque)
            offset_norm = (a ** 2 + b ** 2) ** 0.5

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

            # TCP 기준 (u_hat,v_hat,g_hat) 축을 무게 작용점 위치에 그대로 그림
            # -- rviz2에서 TF > Axes 켜면 바로 보임
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

            now_wall = pytime.monotonic()
            dt = max(1e-3, now_wall - prev_time)
            a_dot = (a - prev_a) / dt
            b_dot = (b - prev_b) / dt
            prev_a, prev_b, prev_time = a, b, now_wall

            # ponytail: 소각 근사로 회전량 산출. u_hat축 회전이 b를,
            # v_hat축 회전이 a를 되돌린다 -- 부호는 tray_balance.py 자체의
            # _demo_general_closed_loop self-test(임의 중력방향 30개)로
            # 검증했다 (실제 물리 축을 가정하지 않으므로 tray_balance_sim.py
            # 로는 검증할 수 없음).
            raw_u_deg = -(KP * b + KD * b_dot)
            raw_v_deg = KP * a + KD * a_dot
            target_u_deg = max(-MAX_TILT_DEG, min(MAX_TILT_DEG, raw_u_deg))
            target_v_deg = max(-MAX_TILT_DEG, min(MAX_TILT_DEG, raw_v_deg))
            is_saturated = (
                abs(raw_u_deg) > MAX_TILT_DEG or abs(raw_v_deg) > MAX_TILT_DEG
            )

            u_delta_deg = target_u_deg - commanded_u_deg
            v_delta_deg = target_v_deg - commanded_v_deg

            # 목표 tilt(u,v 성분)를 TCP xyz(rx,ry,rz)로 합성 -- movel에 실제
            # 보내는 값(delta)이 아니라 절대 목표값이라 시계열 그래프에서 보기 쉬움
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
                    f"force raw={[round(v, 3) for v in raw_force]} "
                    f"filt={[round(v, 3) for v in raw_filtered]} | "
                    f"offset=({a*1000:+.1f},{b*1000:+.1f})mm "
                    f"vel=({a_dot*1000:+.1f},{b_dot*1000:+.1f})mm/s | "
                    f"tilt_target=(u={target_u_deg:+.2f},v={target_v_deg:+.2f})deg "
                    f"delta=(u={u_delta_deg:+.3f},v={v_delta_deg:+.3f})deg"
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

            # 항상 켜진 상태 로그 -- 데드밴드 안이면 "대기", 밖이면 "보상 ON".
            # 기존 balanced/unbalanced 로그는 상태가 바뀔 때만 찍혀서 공이
            # 가만히 있으면 화면이 조용했다. 이건 매 주기(2Hz) 무조건 찍어
            # 루프가 살아있고 보상 중인지 한눈에 보이게 한다.
            will_compensate = offset_norm > CONTROL_DEADBAND_M
            if now - last_status_log >= DEBUG_LOG_PERIOD_SEC:
                node.get_logger().info(
                    f"[{'보상 ON' if will_compensate else '대기'}] "
                    f"offset={offset_norm*1000:5.1f}mm (a={a*1000:+.1f},b={b*1000:+.1f}) | "
                    f"tilt목표(u={target_u_deg:+.2f},v={target_v_deg:+.2f})deg | "
                    f"weight={weight_n:.2f}N g_hat={[round(v,2) for v in g_hat]}"
                )
                last_status_log = now

            if not will_compensate:
                pytime.sleep(1.0 / LOOP_HZ)
                continue

            commanded_u_deg, commanded_v_deg = target_u_deg, target_v_deg

            # u_hat축 회전(u_delta_deg)과 v_hat축 회전(v_delta_deg)을
            # 소각 근사로 벡터 합성해 TCP의 [rx,ry,rz] 3축 전부로 보낸다
            # (더 이상 특정 좌표축 하나로만 회전한다고 가정하지 않는다).
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


def _demo_estimate_com_offset():
    force = [0, 0, 0, 0.0, 0.0, 0.0]
    dx, dy = estimate_com_offset(force, payload_mass_kg=1)
    assert abs(dx) < 1e-9 and abs(dy) < 1e-9

    # Ty만 있으면 dx에만 반영되어야 함 (부호: dx = -ty/weight)
    force = [0, 0, 0, 0.0, 0.5, 0.0]
    dx, dy = estimate_com_offset(force, payload_mass_kg=1)
    assert abs(dx - (-0.5 / 9.81)) < 1e-9
    assert abs(dy) < 1e-9

    # Tx만 있으면 dy에만 반영되어야 함 (부호: dy = tx/weight)
    force = [0, 0, 0, 0.3, 0.0, 0.0]
    dx, dy = estimate_com_offset(force, payload_mass_kg=1)
    assert abs(dy - (0.3 / 9.81)) < 1e-9
    assert abs(dx) < 1e-9

    print("estimate_com_offset self-check OK")


def _demo_estimate_plate_offset():
    force = [0, 0, 0, 0.0, 0.0, 0.0]
    offset_y, offset_z = estimate_plate_offset(force, payload_mass_kg=1)
    assert abs(offset_y) < 1e-9 and abs(offset_z) < 1e-9

    # Tz만 있으면 offset_y에만 반영되어야 함 (부호: offset_y = -tz/weight)
    force = [0, 0, 0, 0.0, 0.0, 0.5]
    offset_y, offset_z = estimate_plate_offset(force, payload_mass_kg=1)
    assert abs(offset_y - (-0.5 / 9.81)) < 1e-9
    assert abs(offset_z) < 1e-9

    # Ty만 있으면 offset_z에만 반영되어야 함 (부호: offset_z = ty/weight)
    force = [0, 0, 0, 0.0, 0.3, 0.0]
    offset_y, offset_z = estimate_plate_offset(force, payload_mass_kg=1)
    assert abs(offset_z - (0.3 / 9.81)) < 1e-9
    assert abs(offset_y) < 1e-9

    print("estimate_plate_offset self-check OK")


def _demo_check_grip_orientation():
    weight = 1.0 * 9.81

    # 그립 정상: 무게가 거의 전부 Fz, Fx/Fy는 작음
    assert check_grip_orientation([0.1, -0.1, weight, 0, 0, 0], payload_mass_kg=1) is None

    # tare 시점에 물체가 없었던 경우: Fz가 예상 무게보다 훨씬 작음
    assert check_grip_orientation([0.0, 0.0, 0.2, 0, 0, 0], payload_mass_kg=1) is not None

    # 그립 방향이 가정과 다른 경우: 무게가 Fx/Fy 쪽으로 크게 새어나감
    assert check_grip_orientation([weight * 0.7, 0.0, weight * 0.5, 0, 0, 0], payload_mass_kg=1) is not None

    print("check_grip_orientation self-check OK")


def _demo_calibrate_and_estimate_plane_offset():
    # 임의의(축과 정렬되지 않은) tare 힘 벡터 -- 실제 로봇에서 관측된 것과
    # 비슷하게 세 성분이 비슷한 크기.
    tare = [0.18, -1.28, -1.55, 0.03, -0.16, -1.26]
    g_hat, u_hat, v_hat, weight_n = calibrate_gravity_frame(tare)

    assert abs(_norm(g_hat) - 1.0) < 1e-9
    assert abs(_dot(u_hat, g_hat)) < 1e-9
    assert abs(_dot(v_hat, g_hat)) < 1e-9
    assert abs(_dot(u_hat, v_hat)) < 1e-9
    assert abs(_norm(u_hat) - 1.0) < 1e-9
    assert abs(_norm(v_hat) - 1.0) < 1e-9
    # u_hat x v_hat == g_hat (우수 좌표계)
    cross_uv = _cross(u_hat, v_hat)
    assert all(abs(cross_uv[i] - g_hat[i]) < 1e-9 for i in range(3))

    # round-trip: 알려진 판 평면 내 오프셋 r0로 torque를 만들고 되돌려 확인
    r0 = _add(_scale(u_hat, 0.03), _scale(v_hat, -0.02))
    force = _scale(g_hat, weight_n)
    tau = _cross(r0, force)
    a, b = estimate_plane_offset(g_hat, u_hat, v_hat, weight_n, tau)
    assert abs(a - 0.03) < 1e-9 and abs(b - (-0.02)) < 1e-9

    print("calibrate_gravity_frame / estimate_plane_offset self-check OK")


def _demo_median_prefilter():
    m = MedianPrefilter(5)
    # 안정된 값(2.0) 사이에 스파이크(99) 하나 -> 중앙값이 스파이크를 무시해야 함
    for v in (2.0, 2.0, 2.0, 2.0):
        m.update([v])
    out = m.update([99.0])          # 창=[2,2,2,2,99] -> median 2
    assert abs(out[0] - 2.0) < 1e-9, out
    # 채널 독립성 확인
    m2 = MedianPrefilter(3)
    m2.update([1.0, 10.0])
    m2.update([1.0, 10.0])
    out2 = m2.update([100.0, 10.0])  # ch0=[1,1,100]->1, ch1=[10,10,10]->10
    assert abs(out2[0] - 1.0) < 1e-9 and abs(out2[1] - 10.0) < 1e-9, out2
    print("MedianPrefilter self-check OK")


def _demo_general_closed_loop():
    """실제 물리 축을 가정하지 않는 제어식이므로 tray_balance_sim.py로는
    검증할 수 없다 -- 임의의 중력 방향 여러 개에 대해 이 파일 자체에서
    폐루프 수렴을 확인한다 (판이 어느 각도로 그립됐든 동작해야 함)."""
    import random

    g_accel = 9.81
    dt = 0.02
    friction = 1.5
    steps = 2000
    rng = random.Random(0)

    def run_one(g_raw, a0, b0, weight_n=1.95):
        g_hat, u_hat, v_hat, weight = calibrate_gravity_frame(
            (*_scale(_normalize(g_raw), weight_n), 0.0, 0.0, 0.0)
        )
        a, b = a0, b0
        va, vb = 0.0, 0.0
        u_deg, v_deg = 0.0, 0.0
        prev_a, prev_b = a, b

        for _ in range(steps):
            r = _add(_scale(u_hat, a), _scale(v_hat, b))
            force = _scale(g_hat, weight)
            tau = _cross(r, force)
            est_a, est_b = estimate_plane_offset(g_hat, u_hat, v_hat, weight, tau)

            a_dot = (est_a - prev_a) / dt
            b_dot = (est_b - prev_b) / dt
            prev_a, prev_b = est_a, est_b

            u_deg = max(-MAX_TILT_DEG, min(MAX_TILT_DEG, -(KP * est_b + KD * b_dot)))
            v_deg = max(-MAX_TILT_DEG, min(MAX_TILT_DEG, KP * est_a + KD * a_dot))

            aa = -g_accel * math.sin(math.radians(v_deg))
            ab = g_accel * math.sin(math.radians(u_deg))
            va = (va + aa * dt) * max(0.0, 1.0 - friction * dt)
            vb = (vb + ab * dt) * max(0.0, 1.0 - friction * dt)
            a += va * dt
            b += vb * dt

        return math.hypot(a, b)

    worst = 0.0
    for _ in range(30):
        g_raw = (rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-1, 1))
        if _norm(g_raw) < 1e-6:
            continue
        angle = rng.uniform(0, 2 * math.pi)
        radius = 0.06
        a0, b0 = radius * math.cos(angle), radius * math.sin(angle)
        final_offset = run_one(g_raw, a0, b0)
        worst = max(worst, final_offset)

    assert worst < 0.001, f"임의 중력 방향 폐루프 수렴 실패, worst={worst*1000:.3f}mm"
    print("_demo_general_closed_loop self-check OK "
          f"(worst final offset={worst*1000:.4f}mm over 30 random gravity directions)")


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
        _demo_median_prefilter()
        _demo_estimate_com_offset()
        _demo_estimate_plate_offset()
        _demo_check_grip_orientation()
        _demo_calibrate_and_estimate_plane_offset()
        _demo_general_closed_loop()
    else:
        main()
