# M0609 + RG2 ROS2 Workspace

Doosan M0609 협동로봇 + OnRobot RG2 그리퍼 통합 ROS2 워크스페이스

---

## 요구사항

- Ubuntu 22.04
- ROS2 Humble
- Intel RealSense SDK 2.0

```bash
sudo apt update

# 기본 도구 및 라이브러리
sudo apt install libpoco-dev

# ROS2 빌드 및 실행 관련
sudo apt install ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-gazebo-ros-pkgs

# 제어 및 하드웨어 인터페이스
sudo apt install ros-humble-hardware-interface \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers

# OnRobot 그리퍼 드라이버 의존성
pip3 install pymodbus==3.3.2
```

---

## 패키지 설치

```bash
mkdir -p ~/ws_cobot_pjt/ws_dsr/src

# ahnisinc 공식 패키지
cd ~/ws_cobot_pjt/ws_dsr/src
git clone https://github.com/ahnisinc/cobot_rg2

# package.xml 의존성 자동 설치 (MoveIt2 등 누락 키 보강)
cd ~/ws_cobot_pjt/ws_dsr
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

> `onrobot_rg_control` 의 `message_runtime` 키는 ROS1 잔재라 경고가 나오지만 `-r` 플래그로 무시되어 빌드엔 영향 없음.

---

## 초기 설정 (최초 1회)

### DRCF 에뮬레이터 (virtual 모드 motion service용)

virtual 모드에서 `movej` 등 motion service를 사용하려면 Doosan DRCF 에뮬레이터(Docker) 설치가 필요.

```bash
# Docker engine 미설치 시 먼저 설치: https://docs.docker.com/engine/install/ubuntu/

# 현재 사용자를 docker 그룹에 추가 (launch에서 docker run 호출 시 필수)
sudo usermod -aG docker $USER
newgrp docker

# 에뮬레이터 이미지 pull
cd ~/ws_cobot_pjt/ws_dsr/src/doosan-robot2
chmod +x ./install_emulator.sh
sudo ./install_emulator.sh
```

> docker 그룹 가입 후 sudo 없이도 동작하지만 upstream 안내를 따라 sudo 형태로 기재. 그룹 변경 사항 적용을 위해 새 셸 또는 재로그인 필요.

### Real 모드 사전 조건

- 로봇 IP: `192.168.1.100`
- 그리퍼 IP: `192.168.1.1` (OnRobot 컴퓨트박스, 고정)
- UDP 포트 권한 설정:
  ```bash
  sudo sysctl -w net.ipv4.ip_unprivileged_port_start=0
  # 재부팅 후에도 유지:
  echo 'net.ipv4.ip_unprivileged_port_start=0' | sudo tee /etc/sysctl.d/99-ros2-doosan.conf
  ```

### RealSense udev rules

udev rules 미설치 시 스트리밍 중 `xioctl(VIDIOC_QBUF) failed — No such device` 에러 발생.

```bash
sudo curl https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules \
  -o /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

적용 후 USB 재연결 필요.

---

## 빌드

```bash
cd ~/ws_cobot_pjt/ws_dsr
colcon build --symlink-install
source install/setup.bash
```

---

## 실행

환경 설정:

```bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
```

### Virtual 모드 (시뮬레이션)

```bash
# 브링업 (그리퍼만)
ros2 launch m0609_rg2_bringup bringup.launch.py

# 브링업 (RealSense 카메라 포함)
ros2 launch m0609_rg2_bringup bringup_camera.launch.py

```

### Real 모드 (실제 로봇)

```bash
# 브링업 (그리퍼만)
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 model:=m0609

# 브링업 (RealSense 카메라 포함)
ros2 launch m0609_rg2_bringup bringup_camera.launch.py mode:=real host:=192.168.1.100 model:=m0609

```

### 간단한 로봇 HMI

`control_GUI`는 상태, 통신, START/STOP/E-STOP, XYZ 수동 이동, TCP 위치,
힘/접촉 상태, 이벤트 로그만 한 화면에 표시한다. 기본 실행은 DRY-RUN이며
START 버튼을 누르기 전에는 모션 스트리밍을 보내지 않는다.

```bash
cd ~/ws_cobot_pjt/ws_dsr
source install/setup.bash

# 안전한 기본 실행(DRY-RUN)
ros2 run rokey control_GUI

# 실제 로봇 연결 상태로 시작
ros2 run rokey control_GUI --ros-args -p dry_run_default:=false
```

실기에서는 상단 상태가 `IDLE`인지 확인하고 `START`를 누른 뒤 XYZ 버튼을
사용한다. `현재 위치를 A/B로 기록` 버튼은 활성 TCP의 DR_BASE 절대 자세를
저장하며, `A/B 좌표 클립보드 복사`로 `contact_stop` 입력 형식을 복사할 수 있다.

### 힘 감지 접촉 정지 (실제 M0609 전용)

`contact_stop`은 Base 좌표계의 절대 TCP 자세 A에서 절대 TCP 자세 B까지
직선 이동한다. 이동 중 Base X/Y/Z 중 어느 축에서든 설정 힘을 감지하면
Quick Stop Category 2로 정지한다. A/B의 `[x, y, z, rx, ry, rz]`는 플랜지
좌표가 아니라 활성 TCP(`GripperDA_v1`)의 자세다.

실행 시 컨트롤러에 미리 등록된 다음 프리셋을 명시적으로 활성화하고 이름을
재확인한다.

- Tool: `Tool Weight_1` — 무게, 무게중심, 관성
- TCP: `GripperDA_v1` — 플랜지에서 실제 작업점까지의 위치/자세 오프셋

프리셋의 실제 수치는 이 저장소가 아니라 로봇 컨트롤러에 저장되어 있다.
그리퍼 외형 크기나 안전 충돌 형상은 TCP와 별도이므로 컨트롤러의 Tool Shape/
안전 설정에서 확인해야 한다. 가상 에뮬레이터에서는 힘/컴플라이언스 제어가
정상 동작하지 않을 수 있으므로 실기에서만 사용한다.

먼저 위의 real 모드 브링업을 실행하고, 별도 터미널에서 다음을 실행한다.
아래 A/B 숫자는 명령 형식 예시일 뿐이므로 반드시 티칭한 TCP 좌표로 교체한다.

```bash
cd ~/ws_cobot_pjt/ws_dsr
source install/setup.bash

# arm 기본값은 false이므로 이 명령은 좌표를 검사하되 움직이지 않는다.
ros2 run rokey contact_stop --ros-args \
  -p position_a:="[345.90,-64.66,55.51,103.22,179.55,105.41]" \
  -p position_b:="[345.90,-64.66,35.51,103.22,179.55,105.41]"

# 로봇을 티치펜던트 등으로 A에 안전하게 위치시킨 뒤 실제 A→B 감시 이동
ros2 run rokey contact_stop --ros-args \
  -p arm:=true \
  -p position_a:="[345.90,-64.66,55.51,103.22,179.55,105.41]" \
  -p position_b:="[345.90,-64.66,35.51,103.22,179.55,105.41]" \
  -p motion_speed_mm_s:=2.0 \
  -p contact_force_n:=10.0
```

기본적으로 현재 TCP가 A와 위치 2 mm, 자세 성분 2도 이내로 일치하지 않으면
동작을 거부한다. `-p move_to_a:=true`를 주면 먼저 A로 이동하지만, 현재 위치에서
A까지의 구간에는 접촉 감시가 적용되지 않으므로 이미 충돌 없는 경로로 검증된
경우에만 사용한다. A→B 병진 거리는 코드에서 최대 100 mm, 이동 속도는 최대
10 mm/s로 제한한다.

이 노드는 안전 정격 보호 기능이 아니다. 저속/축소 모드에서 사람이 비상정지
버튼을 잡고 시험하고, 티치펜던트의 Tool Weight/TCP가 실제 RG2 구성과 일치하는지
먼저 확인해야 한다. 시작 시 이미 힘 임계값을 넘으면 새 접촉으로 오인하지 않도록
동작을 거부한다.

### Virtual / Real 모드 그리퍼 동작 차이

virtual 모드에서 `gripper_virtual_node`(bringup에 포함)가 `/onrobot/sendCommand` 서비스로 RViz 시각화 담당. OnRobot RG2 Modbus 제어 미포함.

| 항목 | real 모드 | virtual 모드 |
|------|-----------|-------------|
| 그리퍼 제어 | OnRobot 드라이버 (Modbus TCP) | Modbus 미포함 |
| 완료 신호 | 디지털 입력 핀 감지 | `/onrobot/sendCommand` 응답 (애니메이션 완료 시) |
| RViz 그리퍼 상태 | `/gripper_joint_states` (OnRobot 드라이버) | `/gripper_joint_states` (gripper_virtual_node) |
| 파지력 / 접촉 | 실제 물리 동작 | 시뮬레이션 없음 |
| Tool/TCP 프리셋 | DRCF 등록값 사용 | 설정 스킵 (에뮬레이터 미등록) |

### RealSense 주요 토픽

| 토픽 | 설명 |
|------|------|
| `/camera/color/image_raw` | RGB 컬러 이미지 |
| `/camera/aligned_depth_to_color/image_raw` | 컬러 정렬 뎁스 이미지 |
| `/camera/depth/color/points` | RGB 포인트클라우드 |
| `/camera/color/camera_info` | 컬러 카메라 내부 파라미터 |

`default.rviz` 사전 구성 display:
- **Color Image** — `/camera/color/image_raw`
- **Depth Image** — `/camera/aligned_depth_to_color/image_raw`
- **PointCloud2** — `/camera/depth/color/points`

---

## TF 구조

### bringup.launch.py (그리퍼만)

```
world
└── base_link
    └── link1 → link2 → link3 → link4 → link5 → link6
                                                    └── tool0
                                                        └── rg2_base_link
                                                            ├── rg2_left_outer_knuckle
                                                            │   ├── rg2_left_inner_knuckle
                                                            │   └── rg2_left_inner_finger
                                                            └── rg2_right_outer_knuckle
                                                                ├── rg2_right_inner_knuckle
                                                                └── rg2_right_inner_finger
```

### bringup_camera.launch.py (카메라 포함)

```
world
└── base_link
    └── link1 → ... → tool0
                       ├── rg2_base_link          (그리퍼, 위와 동일)
                       └── bracket_link           (마운트 브라켓)
                           └── camera_link
                               ├── camera_color_frame / camera_color_optical_frame
                               ├── camera_depth_frame / camera_depth_optical_frame
                               ├── camera_infra1_frame / camera_infra1_optical_frame
                               └── camera_infra2_frame / camera_infra2_optical_frame
```

- `world → base_link`: `static_transform_publisher` (identity)
- `tool0 → rg2_base_link`: `joint0` (fixed)
- `tool0 → bracket_link`: `tool0_to_bracket` (fixed)
- `rg2_left/right_inner_knuckle`: mimic joint, `rg2_finger_joint` 기준 연동

---

## 디렉토리 구조

```

└── src
    ├── README.md
    ├── doosan-robot2                    # 외부 패키지 — read-only
    │   ├── LICENSE
    │   ├── README.md
    │   ├── dsr_bringup2
    │   ├── dsr_common2
    │   ├── dsr_controller2
    │   ├── dsr_description2
    │   ├── dsr_example2
    │   ├── dsr_gazebo2
    │   ├── dsr_hardware2
    │   ├── dsr_moveit2
    │   ├── dsr_msgs2
    │   ├── dsr_mujoco
    │   ├── dsr_tests
    │   ├── install_emulator.sh
    │   ├── test.sh
    │   └── uninstall_emulator.sh
    ├── onrobot-ros2                   # 외부 패키지 — read-only
    │   ├── LICENSE
    │   ├── README.md
    │   ├── _onrobot_rg_modbus_tcp
    │   ├── onrobot_rg_control
    │   ├── onrobot_rg_description
    │   └── onrobot_rg_msgs
    └── rg2
        ├── m0609_rg2_bringup         # 커스텀 브링업 패키지
        └── m0609_rg2_moveit
