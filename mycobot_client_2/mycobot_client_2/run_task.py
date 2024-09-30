import math
import time
import threading
import rclpy
from rclpy.node import Node

from mycobot_msgs_2.msg import MycobotPose, MycobotSetAngles
from mycobot_client_2.ik import CobotIK



positions_times = [(MycobotPose(frame="gripper", x=0.2, y=0, z=0.2, rx=math.pi, ry=0, rz=0), 5.0),
                   (MycobotPose(frame="gripper", x=0.2, y=0.15, z=0.2, rx=math.pi, ry=0, rz=0), 5.0),]


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

    demo_time = 30
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
    cur_position, cur_orientation_euler_angles_degrees = cobot_ik.get_pose(
        cur_joint_angles=cur_angles, target_frame=frame)

    min_angle = -210.0
    max_angle = 90.0
    cur_angle = min_angle
    cur_sign = 1

    angular_change_per_second = 15

    angle_diff = angular_change_per_second / loop_rate

    while rclpy.ok() and time.time() - start_time < demo_time:

        for pose_time in positions_times:
            pose = pose_time[0]
            slp_time = pose_time[1]
            cobot_ik.set_pose(pose)
            rclpy.sleep(slp_time)

        rate.sleep()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cobot_ik.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
