#!/usr/bin/env python
import math
import time

import rospy

from mycobot_msgs.msg import (
    MycobotAngles,
    MycobotSetAngles,
    MycobotCoords,
    MycobotGripperStatus,
    MycobotPumpStatus,
)

COBOT_JOINT_GOAL_TOPIC = "mycobot/angles_goal"
COBOT_JOIN_REAL_TOPIC = "mycobot/angles_real"
COBOT_GRIPPER_STATUS_TOPIC = "mycobot/gripper_status"
COBOT_PUMP_STATUS_TOPIC = "mycobot/pump_status"
COBOT_END_EFFECTOR_COORDS_TOPIC = "mycobot/coords_real"
NUM_JOINTS = 6


class CurRealAngles:
    def __init__(self, angles):
        self.angles = angles

    def __eq__(self, other):
        if isinstance(other, CurAngles):
            return self.angles == other.angles
        return False

    def __ne__(self, other):
        # needed in python 2
        return not self.__eq__(other)
    
    def __str__(self):
        joint_str = "joint_"
        debug_str = ""
        for i in range(len(self.angles)):
            debug_str += joint_str + str(i + 1) + ": " + str(self.angles[i]) + "\n"
        debug_str = debug_str[:-1]
        return debug_str


class CurAngles:
    def __init__(self, angles, speed):
        self.angles = angles
        self.speed = speed

    def __eq__(self, other):
        if isinstance(other, CurAngles):
            return self.angles == other.angles and self.speed == other.speed
        return False

    def __ne__(self, other):
        # needed in python 2
        return not self.__eq__(other)


class CurGripperState:
    def __init__(self, state, speed):
        self.state = state
        self.speed = speed

    def __eq__(self, other):
        if isinstance(other, CurGripperState):
            return self.state == other.state and self.speed == other.speed
        return False

    def __ne__(self, other):
        # needed in python 2
        return not self.__eq__(other)

class MyCobotClient:
    def __init__(self):
        rospy.init_node('mycobot_client')
        self.real_angle_sub = rospy.Subscriber(
            COBOT_JOIN_REAL_TOPIC, MycobotAngles, self.real_angle_cb)
        self.cmd_angle_pub = rospy.Publisher(
            COBOT_JOINT_GOAL_TOPIC, MycobotSetAngles, queue_size=1)
        self.gripper_status_pub = rospy.Publisher(
            COBOT_GRIPPER_STATUS_TOPIC, MycobotGripperStatus, queue_size=1)

        self.cur_real_angles = CurRealAngles([])

    def real_angle_cb(self, msg):
        angles = [0] * NUM_JOINTS
        angles[0] = msg.joint_1
        angles[1] = msg.joint_2
        angles[2] = msg.joint_3
        angles[3] = msg.joint_4
        angles[4] = msg.joint_5
        angles[5] = msg.joint_6
        self.cur_real_angles = CurRealAngles(angles)
        print(self.cur_real_angles)


    def main(self):
        max_angle = 50
        cur_counter = 0
        counter_incr = math.pi / 16
        joint_tol = 2
        speed = 80
        joint_idx = 0
        SLP_TIME = 0.01
        while not rospy.is_shutdown():
            joint_msg = MycobotSetAngles()
            joint_msg.speed = speed
            goal_angle_degrees = math.sin(cur_counter) * max_angle
            joint_msg.joint_1 = goal_angle_degrees

            cur_counter += counter_incr
            self.cmd_angle_pub.publish(joint_msg)
            time.sleep(SLP_TIME)
            # while not rospy.is_shutdown() and len(self.cur_real_angles.angles) == NUM_JOINTS and (abs(self.cur_real_angles.angles[joint_idx] - goal_angle_degrees) < joint_tol):
            #     time.sleep(SLP_TIME)

if __name__ == '__main__':
    try:
        cobot_client = MyCobotClient()
        cobot_client.main()
    except rospy.ROSInterruptException:
        pass