import rclpy
from rclpy.node import Node

from mycobot_client_2.cobot_client import MycobotClient, CurAngles
from mycobot_msgs_2.msg import (MycobotSetAngles,
                                MycobotPose,
                                MycobotGripperStatus)

import numpy as np
import time
import os
from ament_index_python.packages import get_package_share_directory
import simulation_and_control as sac
from simulation_and_control.sim import pybullet_robot_interface as pb
from simulation_and_control.controllers.servo_motor import MotorCommands
from simulation_and_control.controllers.pin_wrapper import PinWrapper


COBOT_JOINT_GOAL_TOPIC = "mycobot/angles_goal"
COBOT_POSE_GOAL_TOPIC = "mycobot/pose_goal"
COBOT_GRIPPER_STATUS_TOPIC = "mycobot/gripper_status"

class CobotIK(Node):
    def __init__(self):
        super().__init__('mycobot_ik_client')
        
        self.cmd_angle_pub = self.create_publisher(MycobotSetAngles, COBOT_JOINT_GOAL_TOPIC, 1)
        self.gripper_status_pub = self.create_publisher(MycobotGripperStatus, COBOT_GRIPPER_STATUS_TOPIC, 1)

        self.pose_sub = self.create_subscription(
            MycobotPose,
            COBOT_POSE_GOAL_TOPIC,
            self.set_pose,
            1)

        self.declare_parameter('max_iterations', 500)
        self.declare_parameter('solution_tol', 0.1)

        self.get_logger().info("start ...")
        
        self.package_share_directory = get_package_share_directory('mycobot_client_2')
        self.conf_file_name = "elephantconfig.json"  # Configuration file for the robot
        
        self.get_logger().info(f"share directory {self.package_share_directory}")
        self.sim = pb.SimInterface(self.conf_file_name, conf_file_path_ext = self.package_share_directory)  # Initialize simulation interface

        self.source_names = ["pybullet"]  # Define the source for dynamic modeling
        # Get active joint names from the simulation
        self.ext_names = self.sim.getNameActiveJoints()
        self.ext_names = np.expand_dims(np.array(self.ext_names), axis=0)  # Adjust the shape for compatibility

        # Create a dynamic model of the robot
        self.dyn_model = PinWrapper(self.conf_file_name, "pybullet", self.ext_names, self.source_names, False, 0, self.package_share_directory)
        # Display dynamics information
        self.get_logger().info("Joint info simulator:")
        self.get_logger().info(self.sim.GetBotJointsInfo())

        self.get_logger().info("Link info simulator:")
        self.get_logger().info(self.sim.GetBotDynamicsInfo())

        self.get_logger().info("Link info pinocchio:")
        self.get_logger().info(self.dyn_model.getDynamicsInfo())


    
    def set_pose(self, msg):
        q_k = np.array([0, 0, 0, 0, 0, 0])
        end_effector_frame = "joint6_flange"
        local_or_global = "local_global"
        q_k_plus_one = np.copy(q_k)
        q_k_plus_one[:] = 99
        target_pose = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info("target pose")
        self.get_logger().info(np.array_str(target_pose))
        num_iterations = 0
        success = False
        while num_iterations < self.get_parameter('max_iterations').value and not success:
            q_k = np.copy(q_k_plus_one)
            jacobian = self.dyn_model.ComputeJacobian(q_k, end_effector_frame, local_or_global).J
            trimmed_jacobian = np.copy(jacobian[0:3, :])
            self.get_logger().debug("jacobian")
            self.get_logger().debug(np.array_str(jacobian))
            position, orientation = self.dyn_model.ComputeFK(q_k, end_effector_frame)
            self.get_logger().debug("position at k")
            self.get_logger().debug(np.array_str(position))
            self.get_logger().debug("orientation at k")
            self.get_logger().debug(np.array_str(orientation))
            inverted_j = np.linalg.pinv(trimmed_jacobian)
            self.get_logger().debug(f"{q_k.shape} + {inverted_j.shape} @ ({target_pose.shape} - {position.shape})")
            q_k_plus_one = q_k + np.linalg.pinv(trimmed_jacobian) @ (target_pose - position)
            num_iterations += 1
            success = np.linalg.norm(q_k_plus_one - q_k) < self.get_parameter('solution_tol').value
        if not success:
            self.get_logger().error(f"could not solve for solution in {self.get_parameter('max_iterations').value} iterations")
            return
        else:
            self.get_logger().info(f"found solution in {num_iterations} iterations")
            self.get_logger().info(np.array_str(q_k_plus_one))

            self.get_logger().info(f"forward kinematics with the solution results in:")
            position, orientation = self.dyn_model.ComputeFK(q_k_plus_one, end_effector_frame)
            self.get_logger().info(np.array_str(position))


        new_joint_msg = MycobotSetAngles()
        new_joint_msg.joint_1 = q_k_plus_one[0]
        new_joint_msg.joint_2 = q_k_plus_one[1]
        new_joint_msg.joint_3 = q_k_plus_one[2]
        new_joint_msg.joint_4 = q_k_plus_one[3]
        new_joint_msg.joint_5 = q_k_plus_one[4]
        new_joint_msg.joint_6 = q_k_plus_one[5]
        new_joint_msg.speed = 80

        self.cmd_angle_pub.publish(new_joint_msg)


def main(args=None):
    rclpy.init(args=args)

    cobot_ik = CobotIK()

    rclpy.spin(cobot_ik)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cobot_ik.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()