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

# TODO make a datastructure to keep track of objects found and their counts
# consider using the xyz and checking the vector norm to the previous points with a tolerance

OBJECTS_FOUND = {}

def main(args=None):
    rclpy.init(args=args)

    cobot_ik = CobotIK(visualize=False)
    # Spin in a separate thread
    thread = threading.Thread(
        target=rclpy.spin, args=(cobot_ik, ), daemon=True)
    thread.start()

    rate = cobot_ik.create_rate(5)

    zero_msg = get_zero_joints_msg(30)
    cobot_ik.cmd_angle_pub.publish(zero_msg)

    time.sleep(5)
    object_found_min_count = 2


    while rclpy.ok() :

        obj_found_dict = cobot_ik.get_obj_found_dict()
        obj_to_pickup = None
        for object_id in obj_found_dict.keys():
            cnt_coords = obj_found_dict[object_id]
            if cnt_coords[0] > object_found_min_count:
                obj_to_pickup = object_id
                break
        
        if obj_to_pickup:
            cobot_ik.pickup_object(obj_to_pickup)

        rate.sleep()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cobot_ik.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
