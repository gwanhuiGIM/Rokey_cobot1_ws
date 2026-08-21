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
