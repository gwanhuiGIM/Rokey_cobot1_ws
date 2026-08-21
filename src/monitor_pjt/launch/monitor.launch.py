# =============================================================
# monitor.launch.py — 시스템 모니터 노드 + Qt 대시보드를 각각 직접 실행
# -------------------------------------------------------------
# 실행: ros2 launch monitor_pjt monitor.launch.py
#       ros2 launch monitor_pjt monitor.launch.py control_enabled:=true
# 전제: 로봇 드라이버가 이미 떠 있어야 데이터가 들어온다.
#   ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
#       mode:=real host:=192.168.137.100 model:=m0609 name:=dsr01
# -------------------------------------------------------------
# system_monitor(로봇/dsr_msgs2 담당)와 dashboard(Qt, 토픽으로만 통신)는
# 서로 다른 프로세스다 — 중간에 둘을 묶어 실행하는 글루 노드는 없다.
# 둘 다 죽어도 서로 영향 없고, dashboard만 따로 여러 대 띄워도 된다.
# =============================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    control_enabled = LaunchConfiguration("control_enabled")

    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="dsr01"),
        # 기본 false = 읽기 전용. true로 켜도 GUI의 "제어 활성화" 체크는 별도로 눌러야 함.
        DeclareLaunchArgument("control_enabled", default_value="false"),
        Node(
            package="monitor_pjt",
            executable="system_monitor",
            name="system_monitor",
            output="screen",
            parameters=[{
                "robot_id": robot_id,
                "control_enabled": control_enabled,
            }],
        ),
        Node(
            package="monitor_pjt",
            executable="dashboard",
            name="monitor_dashboard",
            output="screen",
        ),
    ])
