import math
import time
import threading
import rclpy
from rclpy.node import Node

import numpy as np

from mycobot_msgs_2.msg import MycobotPose, MycobotSetAngles
from mycobot_client_2.ik_simple import CobotIK, RADIAN_TO_DEGREES

# positions_times = []
# start = 0.0
# end = 0.8
# num_iterations = 10
# increment_val = (end - start) / num_iterations
# for i in range(num_iterations):
#     new_x = start + (increment_val * i)
#     pose = MycobotPose(frame="gripper", x=0.15, y=0.0, z=new_x, rx=180.0, ry=0.0, rz=0.0)
#     positions_times.append((pose, 3.0))

positions_times = [(MycobotPose(frame="gripper", x=0.15, y=0.0, z=0.2, rx=180.0, ry=0.0, rz=0.0), 5.0),
                   (MycobotPose(frame="gripper", x=0.15, y=0.0, z=0.005, rx=180.0, ry=0.0, rz=0.0), 5.0),
                   (MycobotPose(frame="gripper", x=0.2, y=-0.15, z=0.4, rx=180.0, ry=0.0, rz=0.0), 5.0)]




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


def main(args=None):
    rclpy.init(args=args)

    cobot_ik = CobotIK()
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

        for pose_time in positions_times:
            pose = pose_time[0]
            slp_time = pose_time[1]
            command_angles = cobot_ik.calculate_ik(np.array([pose.x, pose.y, pose.z]),
                                     np.array([pose.rx, pose.ry, pose.rz]), frame)
            command_angles = RADIAN_TO_DEGREES * command_angles
            cobot_ik.set_pose(pose)
            start_loop_time = time.time()
            counter = 0
            while time.time() - start_loop_time < slp_time:
                if counter % 20 == 0:
                    print(f"goal: {pose}")
                    p1, o1 = cobot_ik.get_pose(
                        cur_joint_angles=None, target_frame=frame)
                    cur_angles = cobot_ik.get_real_angles()
                    print(f"position1 {p1}")
                    print(f"orientation1 {o1}")
                    print(f"goal angles: {command_angles}")
                    print(f"cur angles: {cur_angles}")
                counter += 1
                time.sleep(0.01)

        rate.sleep()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cobot_ik.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
