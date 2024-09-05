import math

import rclpy
from rclpy.node import Node

from mycobot_msgs_2.msg import (
    MycobotAngles,
    MycobotSetAngles,
    MycobotCoords,
    MycobotGripperStatus,
    MycobotPumpStatus,
)

from pymycobot.mycobot import MyCobot
from pymycobot.error import MyCobotDataException

COBOT_JOINT_GOAL_TOPIC = "mycobot/angles_goal"
COBOT_JOIN_REAL_TOPIC = "mycobot/angles_real"
COBOT_GRIPPER_STATUS_TOPIC = "mycobot/gripper_status"
COBOT_PUMP_STATUS_TOPIC = "mycobot/pump_status"
COBOT_END_EFFECTOR_COORDS_TOPIC = "mycobot/coords_real"


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


class CurPumpStatus:
    def __init__(self, state, pin_1, pin_2):
        self.state = state
        self.pin_1 = pin_1
        self.pin_2 = pin_2

    def __eq__(self, other):
        if isinstance(other, CurPumpStatus):
            return self.state == other.state and self.pin_1 == other.pin_1 and self.pin_2 == other.pin_2
        return False

    def __ne__(self, other):
        # needed in python 2
        return not self.__eq__(other)

class MycobotClient(Node):

    def __init__(self):
        super().__init__('mycobot_client')

        self.declare_parameter('pub_cmd_timer', 100)

        self.get_logger().info("start ...")
        timer_hz = self.get_parameter('pub_real_coords_timer').value
        self.get_logger().info("Params: %s" % (timer_hz))


        self.cmd_angle_pub = self.create_publisher(MycobotSetAngles, COBOT_JOINT_GOAL_TOPIC, 1)

        self.timer_cmd_angles = self.create_timer(timer_hz,
                 self.publish_cmd_angles)

        self.ax_angle = 50
        self.cur_counter = 0.0
        self.counter_incr = (1) * math.pi / 180
        self.speed = 80
        self.joint_idx = 0
    
    def publish_cmd_angles(self):
        joint_msg = MycobotSetAngles()
        joint_msg.speed = self.speed
        goal_angle_degrees = math.sin(self.cur_counter) * self.max_angle
        joint_msg.joint_1 = goal_angle_degrees
        self.cur_counter += self.counter_incr
        self.cmd_angle_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)

    mycobot_client = MycobotClient()

    rclpy.spin(mycobot_client)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mycobot_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
