# simple_input.py 
import rclpy
import DR_init
import time

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30
ON, OFF = 1, 0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_move", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
            set_digital_output,
            get_digital_input,
            wait,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        return

    set_tool("Tool Weight_1")
    set_tcp("GripperDA_v1")

    homej = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
    posj1 = posj([0.0, 0.0, 90.0, 0.0, 30.0, 0.0])

    posx13 = posx([350.0, 34.5, 350.0, 45.0, 180.0, 45.0])
    posx14 = posx([350.0, 34.5, 300.0, 45.0, 180.0, 45.0])
    posx15 = posx([350.0, 34.5, 250.0, 45.0, 180.0, 45.0])
    posx16 = posx([350.0, 34.5, 400.0, 45.0, 180.0, 45.0])

    movej(homej, vel=VELOCITY, acc=ACC)

    try:
        movej(posj1, vel=VELOCITY, acc=ACC)

        while rclpy.ok():

            di13 = get_digital_input(13)
            di14 = get_digital_input(14)
            di15 = get_digital_input(15)
            di16 = get_digital_input(16)

            # 13번 버튼 눌림 감지
            if di13 and not old_di13:
                node.get_logger().info("Button 13 ON")
                movel(posx13, vel=VELOCITY, acc=ACC)
                wait(1)
                movej(posj1, vel=VELOCITY, acc=ACC)

            # 14번 버튼 눌림 감지
            elif di14 and not old_di14:
                node.get_logger().info("Button 14 ON")
                movel(posx14, vel=VELOCITY, acc=ACC)
                wait(1)
                movej(posj1, vel=VELOCITY, acc=ACC)

            # 15번 버튼 눌림 감지
            elif di15 and not old_di15:
                node.get_logger().info("Button 15 ON")
                movel(posx15, vel=VELOCITY, acc=ACC)
                wait(1)
                movej(posj1, vel=VELOCITY, acc=ACC)

            # 16번 버튼 눌림 감지
            elif di16 and not old_di16:
                node.get_logger().info("Button 16 ON")
                movel(posx16, vel=VELOCITY, acc=ACC)
                wait(1)
                movej(posj1, vel=VELOCITY, acc=ACC)

            # 현재 상태 저장
            old_di13 = di13
            old_di14 = di14
            old_di15 = di15
            old_di16 = di16

            time.sleep(0.05)

    except KeyboardInterrupt:
        node.get_logger().info("Program Stopped")

    except Exception as e:
        node.get_logger().info(f"Robot Error: {e}")

    finally:
        movej(homej, vel=VELOCITY, acc=ACC)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
