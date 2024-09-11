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


class MycobotClient(Node):

    def __init__(self):
        super().__init__('mycobot_client')

        self.declare_parameter('pub_cmd_timer', 0.01)

        self.get_logger().info("start ...")
        timer_hz = self.get_parameter('pub_cmd_timer').value
        self.get_logger().info("Params: %s" % (timer_hz))

        self.cmd_angle_pub = self.create_publisher(MycobotSetAngles, COBOT_JOINT_GOAL_TOPIC, 1)

        self.timer_cmd_angles = self.create_timer(timer_hz,
                 self.move_joints)

        self.max_angle = 50
        self.cur_counter = 0
        self.counter_incr = 1
        self.speed = 80
        self.joint_idx = 0
    
    def publish_cmd_angles(self, cmd_angles: CurAngles):
        joint_msg = MycobotSetAngles()
        joint_msg.speed = cmd_angles.speed
        joint_msg.joint_1 = cmd_angles.angles[0]
        joint_msg.joint_2 = cmd_angles.angles[1]
        joint_msg.joint_3 = cmd_angles.angles[2]
        joint_msg.joint_4 = cmd_angles.angles[3]
        joint_msg.joint_5 = cmd_angles.angles[4]
        joint_msg.joint_6 = cmd_angles.angles[5]
        self.publish_cmd_angles_ros(joint_msg)
    
    def publish_cmd_angles_ros(self, cmd_msg: MycobotSetAngles):
        self.get_logger().debug(cmd_msg)
        self.cmd_angle_pub.publish(cmd_msg)

    
    def move_joints(self):
        joint_msg = MycobotSetAngles()
        joint_msg.speed = self.speed
        goal_angle_degrees = math.sin(self.cur_counter * math.pi / 180) * self.max_angle
        joint_msg.joint_1 = goal_angle_degrees
        joint_msg.joint_2 = goal_angle_degrees
        joint_msg.joint_3 = goal_angle_degrees
        joint_msg.joint_4 = goal_angle_degrees
        joint_msg.joint_5 = goal_angle_degrees
        joint_msg.joint_6 = goal_angle_degrees
        self.cur_counter += self.counter_incr
        self.publish_cmd_angles_ros(joint_msg)

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
