import math
import time
import threading
import rclpy
from rclpy.node import Node

import numpy as np

from mycobot_msgs_2.msg import MycobotPose, MycobotSetAngles
from mycobot_client_2.ik_pybullet import CobotIK, RADIAN_TO_DEGREES


def get_zero_joints_msg(speed: int):
    zero_joints = MycobotSetAngles()
    zero_val = 0.0
    zero_joints.joint_1 = zero_val
    zero_joints.joint_2 = zero_val
    zero_joints.joint_3 = zero_val
    zero_joints.joint_4 = zero_val
    zero_joints.joint_5 = zero_val
    zero_joints.joint_6 = zero_val
    zero_joints.speed = speed
    return zero_joints

# TODO make a datastructure to keep track of desired pose, gripper status, and perhaps wait time

def main(args=None):
    rclpy.init(args=args)

    cobot_ik = CobotIK(visualize=False)
    # Spin in a separate thread
    thread = threading.Thread(
        target=rclpy.spin, args=(cobot_ik, ), daemon=True)
    thread.start()

    demo_time = 60
    start_time = time.time()
    loop_rate = 30
    loop_seconds = 1 / loop_rate
    speed = 30

    rate = cobot_ik.create_rate(loop_rate)

    zero_msg = get_zero_joints_msg(speed)
    cobot_ik.cmd_angle_pub.publish(zero_msg)

    time.sleep(5)
    frame = "gripper"

    cur_angles = cobot_ik.get_real_angles()

    while rclpy.ok() and time.time() - start_time < demo_time:

        x = 0.2
        y = 0.0
        z = 0.1
        rx = 180.0
        ry = 25.0
        rz = 0.0
        close_gripper = False
        counter = 0

        # TODO: modify the below code to use your datastructure and pick up the cube and deposit it at known positions
        
        command_angles = cobot_ik.calculate_ik(np.array([x, y, z]),
                                                   np.array([rx, ry, rz]), frame, pybullet_max_iterations = 100)
        p1, o1 = cobot_ik.get_pose(
                        cur_joint_angles=command_angles, target_frame=frame)
        print(f"goal: {np.array([x, y, z]), np.array([rx, ry, rz])}")
        print("ik would result in: ")
        print(f"position1 {p1}")
        print(f"orientation1 {o1}")
        print(f"goal angles: {command_angles}")
        print(f"cur angles: {cur_angles}")
        cobot_ik.publish_angles(command_angles)
        if close_gripper:
            cobot_ik.close_gripper()
        else:
            cobot_ik.open_gripper()
        if counter % 20 == 0:
            pose = (np.array([x, y, z]), np.array([rx, ry, rz]))
            print(f"goal: {pose}")
            p1, o1 = cobot_ik.get_pose(
                cur_joint_angles=None, target_frame=frame)
            cur_angles = cobot_ik.get_real_angles()
            print(f"position1 {p1}")
            print(f"orientation1 {o1}")
            print(f"goal angles: {command_angles}")
            print(f"cur angles: {cur_angles}")
        counter += 1

        rate.sleep()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cobot_ik.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
