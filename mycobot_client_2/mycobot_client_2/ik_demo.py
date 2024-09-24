import math
import time
import threading
import rclpy
from rclpy.node import Node

from mycobot_msgs_2.msg import MycobotPose, MycobotSetAngles
from mycobot_client_2.ik import CobotIK

def get_zero_joints_msg(speed: int):
    zero_joints = MycobotSetAngles()
    zero_joints.joint_1 = 0
    zero_joints.joint_2 = 0
    zero_joints.joint_3 = 0
    zero_joints.joint_4 = 0
    zero_joints.joint_5 = 0
    zero_joints.joint_6 = 0
    zero_joints.speed = speed
    return zero_joints



def main(args=None):
    rclpy.init(args=args)

    cobot_ik = CobotIK()
    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(cobot_ik, ), daemon=True)
    thread.start()

    demo_time = 30
    start_time = time.time()
    loop_rate = 30
    loop_seconds = 1 / loop_rate
    speed = 50

    rate = cobot_ik.create_rate(loop_rate)

    zero_msg = get_zero_joints_msg(speed)
    cobot_ik.cmd_angle_pub.publish(zero_msg)

    time.sleep(5)

    frame = "gripper"
    min_angle = -70
    max_angle = 70
    cur_angle = -70
    cur_sign = 1

    angular_change_per_second = 5

    angle_diff = angular_change_per_second / loop_rate
    my_pose = MycobotPose()
    my_pose.frame = frame
    my_pose.x = 0.04
    my_pose.y = -0.06
    my_pose.z = 0.45
    my_pose.rx = -40
    my_pose.ry = 0
    my_pose.rz = cur_angle

    while rclpy.ok() and time.time() - start_time < demo_time:
        my_pose.rz = cur_angle
        cobot_ik.set_pose(my_pose)
        cur_angle += angle_diff * cur_sign
        if cur_angle > max_angle:
            cur_sign = -1
        elif cur_angle < min_angle:
            cur_sign = 1
        rate.sleep()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cobot_ik.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()