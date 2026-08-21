import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("gear_move", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            set_singular_handling,
            set_velj,
            set_accj,
            set_velx,
            set_accx,
            set_digital_output,
            set_ref_coord,
            task_compliance_ctrl,
            set_stiffnessx,
            set_desired_force,
            release_force,
            release_compliance_ctrl,
            amove_periodic,
            check_position_condition,
            drl_script_stop,
            wait,
            movej,
            movel,
            DR_AVOID,
            DR_MV_MOD_ABS,
            DR_MV_MOD_REL,
            DR_MV_RA_DUPLICATE,
            DR_FC_MOD_REL,
            DR_QSTOP,
            DR_AXIS_Z,
            DR_BASE,
            ON,
            OFF,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        return

    # System.drvar
    System_T1 = posx(525.32, -156.54, 23.33, 126.12, 179.56, 126.68)
    System_T3 = posx(512.11, -260.58, 22.08, 91.75, 179.83, 92.35)
    System_T2 = posx(428.71, -197.66, 23.31, 68.75, 179.45, 69.58)
    System_T0 = posx(488.61, -204.43, 22.33, 112.8, 179.54, 113.58)
    System_T3_dst = posx(393.28, -101.28, 22.55, 115.49, 179.53, 116.68)
    System_T2_dst = posx(289.61, -87.2, 22.52, 76.68, 179.68, 78.21)
    System_T1_dst = posx(354.22, -4.61, 22.8, 131.46, 179.78, 133.1)
    System_T0_dst = posx(345.9, -64.66, 55.51, 103.22, 179.55, 105.41)

    def gripper_off():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(0.50)

    def gripper_on():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.50)

    set_singular_handling(DR_AVOID)
    set_velj(30.0)
    set_accj(50.0)
    set_velx(25.0, 80.625)
    set_accx(100.0, 322.5)

    set_tool("Tool Weight_gripper")
    set_tcp("GripperDA_v1")

    try:
        gripper_off()
        movej(posj(-25.31, 25.28, 68.30, -1.98, 79.07, -9.51), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        movel(System_T3, radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        gripper_on()
        movel(posx(0.00, 0.00, 150.00, 0.00, 0.00, 0.00), radius=0.00, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        movej(posj(-10.08, 3.25, 95.71, -4.23, 78.75, -9.51), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        movel(System_T3_dst, radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        gripper_off()
        node.get_logger().info("T3끝")

        movej(posj(-13.51, 8.20, 93.67, -2.40, 78.66, -14.21), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        movej(posj(-16.24, 26.94, 66.66, -1.22, 85.91, -14.21), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        movel(System_T1, radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        gripper_on()
        movel(posx(0.00, 0.00, 150.00, 0.00, 0.00, 0.00), radius=0.00, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        movej(posj(-3.07, 0.98, 101.29, 1.20, 78.34, -14.21), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        movel(System_T1_dst, radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        gripper_off()
        node.get_logger().info("T1 끝")

        movej(posj(-3.07, 0.98, 101.29, 1.20, 78.34, -14.21), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        movej(posj(-25.24, 17.03, 83.23, 1.10, 80.81, -36.35), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        movel(System_T2, radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        gripper_on()
        movel(posx(0.00, 0.00, 150.00, 0.00, 0.00, 0.00), radius=0.00, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        movej(posj(-18.67, -6.18, 108.24, 1.15, 78.97, -29.85), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        movel(System_T2_dst, radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        gripper_off()
        node.get_logger().info("T2 끝")

        movej(posj(-17.34, -7.85, 108.01, 0.34, 79.73, -16.27), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        movej(posj(-23.18, 24.37, 70.91, 0.58, 84.70, -22.04), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        movel(System_T0, radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        gripper_on()
        movel(posx(0.00, 0.00, 150.00, 0.00, 0.00, 0.00), radius=0.00, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        movej(posj(-11.67, -0.90, 101.69, 0.58, 79.15, -10.60), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        node.get_logger().info("왔다갔다 시작")
        movel(System_T0_dst, radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

        set_ref_coord(1)
        task_compliance_ctrl()
        set_stiffnessx([3000.00, 3000.00, 3000.00, 200.00, 200.00, 200.00], time=0.0)
        set_desired_force([0.00, 0.00, 25.00, 0.00, 0.00, 0.00], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_REL)
        amove_periodic(amp=[0.00, 0.00, 0.00, 0.00, 0.00, 10.00], period=[0.00, 0.00, 0.00, 0.00, 0.00, 1.50], atime=0.00, repeat=5, ref=1)

        while True:
            if check_position_condition(axis=DR_AXIS_Z, max=35, ref=DR_BASE):
                drl_script_stop(DR_QSTOP)
                gripper_off()
                release_force(time=0.0)
                release_compliance_ctrl()
                movel(posx(0.00, 0.00, 150.00, 0.00, 0.00, 0.00), radius=0.00, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                break

    except KeyboardInterrupt:
        node.get_logger().info("Program Stopped")

    except Exception as e:
        node.get_logger().info(f"Robot Error: {e}")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
