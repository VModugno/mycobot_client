import rclpy
from rclpy.node import Node

from mycobot_client_2.cobot_client import MycobotClient, CurAngles
from mycobot_msgs_2.msg import (MycobotSetAngles,
                                MycobotAngles,
                                MycobotPose,
                                MycobotGripperStatus)

import numpy as np
import numpy.typing as npt
import time
import os
from typing import Tuple, Optional
from ament_index_python.packages import get_package_share_directory
import simulation_and_control as sac
from simulation_and_control.sim import pybullet_robot_interface as pb
from simulation_and_control.controllers.servo_motor import MotorCommands
from simulation_and_control.controllers.pin_wrapper import PinWrapper
import pinocchio as pin


COBOT_JOINT_GOAL_TOPIC = "mycobot/angles_goal"
COBOT_POSE_GOAL_TOPIC = "mycobot/pose_goal"
COBOT_GRIPPER_STATUS_TOPIC = "mycobot/gripper_status"
COBOT_JOIN_REAL_TOPIC = "mycobot/angles_real"

NUM_ANGLES = 6
RADIAN_TO_DEGREES = (1/np.pi) * 180
DEGREES_TO_RADIANS = np.pi / 180
# the joints have the below min/maxes (from https://www.elephantrobotics.com/en/mycobot-280-pi-2023-specifications/)
# J1 -165 ~ +165
# J2 -165 ~ +165
# J3 -165 ~ +165
# J4 -165 ~ +165
# J5 -165 ~ +165
# J6 -179 ~ +179
JOINT_LIMITS = [[-165, 165], [-165, 165], [-165, 165],
                [-165, 165], [-165, 165], [-179, 179]]


def do_dampened_pseudo_inverse(matrix, dampening_factor):
    # https://robotics.caltech.edu/~jwb/courses/ME115/handouts/damped.pdf
    return matrix.T @ (matrix @ matrix.T + dampening_factor**2 * np.eye(matrix.shape[0]))


class CobotIK(Node):
    def __init__(self):
        super().__init__('mycobot_ik_client')

        self.cmd_angle_pub = self.create_publisher(
            MycobotSetAngles, COBOT_JOINT_GOAL_TOPIC, 1)
        self.gripper_status_pub = self.create_publisher(
            MycobotGripperStatus, COBOT_GRIPPER_STATUS_TOPIC, 1)

        self.pose_sub = self.create_subscription(
            MycobotPose,
            COBOT_POSE_GOAL_TOPIC,
            self.set_pose,
            1)

        self.real_angle_sub = self.create_subscription(
            MycobotAngles,
            COBOT_JOIN_REAL_TOPIC,
            self.update_real_angles,
            1)

        self.real_angles = np.zeros(NUM_ANGLES)

        self.declare_parameter('max_iterations', 500)
        self.declare_parameter('solution_tol', 0.001)
        self.declare_parameter('move_speed', 30)
        self.declare_parameter('step_size', 0.1)
        self.declare_parameter('dampening_factor', 0.1)
        self.declare_parameter('initial_guess_is_cur_pos', True)
        self.declare_parameter('use_pybullet', True)

        self.get_logger().info("start ...")

        self.package_share_directory = get_package_share_directory(
            'mycobot_client_2')
        self.conf_file_name = "elephantconfig.json"  # Configuration file for the robot

        self.get_logger().info(
            f"share directory {self.package_share_directory}")
        # Initialize simulation interface
        self.sim = pb.SimInterface(
            self.conf_file_name, conf_file_path_ext=self.package_share_directory)

        # Define the source for dynamic modeling
        self.source_names = ["pybullet"]
        # Get active joint names from the simulation
        self.ext_names = self.sim.getNameActiveJoints()
        # Adjust the shape for compatibility
        self.ext_names = np.expand_dims(np.array(self.ext_names), axis=0)

        # Create a dynamic model of the robot
        self.dyn_model = PinWrapper(self.conf_file_name, "pybullet", self.ext_names,
                                    self.source_names, False, 0, self.package_share_directory)
        # Display dynamics information
        self.get_logger().info("Joint info simulator:")
        self.get_logger().info(self.sim.GetBotJointsInfo())

        self.get_logger().info("Link info simulator:")
        self.get_logger().info(self.sim.GetBotDynamicsInfo())

        self.get_logger().info("Link info pinocchio:")
        self.get_logger().info(self.dyn_model.getDynamicsInfo())

    def update_real_angles(self, msg: MycobotAngles):
        """Helper function to be called in a ROS2 callback that takes the message and stores it in a numpy array in the class.

        Args:
            msg (MycobotAngles): msg from mycobot_msgs_2
        """
        angles = np.zeros(NUM_ANGLES)
        angles[0] = msg.joint_1
        angles[1] = msg.joint_2
        angles[2] = msg.joint_3
        angles[3] = msg.joint_4
        angles[4] = msg.joint_5
        angles[5] = msg.joint_6
        self.real_angles = angles

    def get_real_angles(self) -> npt.NDArray[float]:
        """Helper function to return the current angles.

        Returns:
            npt.NDArray[float]: numpy array with the joint angles 1-6.
        """
        return np.copy(self.real_angles)

    def get_pose(self, cur_joint_angles: Optional[npt.NDArray[float]] = None,
                 target_frame: str = "gripper") -> Tuple[npt.NDArray[float], npt.NDArray[float]]:
        """Helper function to get the current pose of the robot from the current angles.

        Args:
            cur_joint_angles (Optional, optional): What current angles to calculate direct kinematics with? 6D numpy array. Defaults to None.
            target_frame (str, optional): What frame should the pose be in, from frames in the URDF. Defaults to "gripper".

        Returns:
            Tuple[npt.NDArray[float], npt.NDArray[float]]: returns position (x, y, z, meters) and orientation (rx, ry, rz, euler angles degrees)
        """

        if cur_joint_angles is None:
            cur_joint_angles = self.get_real_angles()
            cur_joint_angles = DEGREES_TO_RADIANS * cur_joint_angles
        position, orientation = self.dyn_model.ComputeFK(
            cur_joint_angles, target_frame)
        euler_angles = RADIAN_TO_DEGREES * pin.rpy.matrixToRpy(orientation)
        return position, euler_angles

    def adjust_angles(self, target_angles):
        new_angles = np.copy(target_angles)
        for i in range(new_angles.shape[0]):
            angle_degrees = RADIAN_TO_DEGREES * target_angles[i]
            angle_degrees_bounded = angle_degrees % 360

            if angle_degrees_bounded < 0:
                angle_degrees_bounded = 360 - (angle_degrees_bounded * (-1))

            # now i need to remap 0 to 360 to -180 to 180
            # keeping 0-180 where it is, changing where 180-360 is, but this creates some wrapping issues
            if angle_degrees_bounded > 180:
                angle_degrees_wrapped = 360 - angle_degrees_bounded
                angle_degrees_wrapped *= -1
            else:
                angle_degrees_wrapped = angle_degrees_bounded
            joint_min, joint_max = JOINT_LIMITS[i]

            if angle_degrees_wrapped < joint_min:
                self.get_logger().error(
                    f"joint_index {i} had angle {angle_degrees_wrapped}, min {joint_min}, bounding it.")
                angle_degrees_wrapped = joint_min
            if angle_degrees_wrapped > joint_max:
                self.get_logger().error(
                    f"joint_index {i} had angle {angle_degrees_wrapped}, max {joint_max}, bounding it.")
                angle_degrees_wrapped = joint_max
            new_angles[i] = angle_degrees_wrapped
        return new_angles

    def calculate_ik(self, target_position, euler_angles_degrees, target_frame, tolerance, step_size, dampening_factor,
                     initial_guess_is_cur_pos, max_iterations, use_pybullet=True):
        """_summary_

        Args:
            position (_type_): np.array 3D
            euler_angles_degrees (_type_): np.array 3D
            target_frame (_type_): _description_
            tolerance (_type_): _description_
            step_size (_type_): _description_
            dampening_factor (_type_): _description_
            initial_guess_is_cur_pos (_type_): _description_
            use_pybullet (bool, optional): _description_. Defaults to True.
        """
        if initial_guess_is_cur_pos:
            q_k = np.copy(self.real_angles)
        else:
            q_k = np.zeros(len(self.real_angles))

        q_k_plus_one = np.copy(q_k)

        x, y, z = target_position
        rx, ry, rz = euler_angles_degrees

        orientation_only = (x == -1 and y == -1 and z == -
                            1) and (rx != -1 and ry != -1 and rz != -1)
        position_only = (x != -1 and y != -1 and z != -
                         1) and (rx == -1 and ry == -1 and rz == -1)

        ori_des_euler_degrees = np.array([rx, ry, rz])
        ori_des_euler_radians = ori_des_euler_degrees * DEGREES_TO_RADIANS
        ori_des = pin.rpy.rpyToMatrix(ori_des_euler_radians)
        ori_des_quat = pin.Quaternion(ori_des)
        ori_des_quat = ori_des_quat.normalize()

        joint_angles = None

        if not use_pybullet:
            local_or_global = "local_global"
            self.get_logger().info("position target")
            self.get_logger().info(np.array_str(target_position))
            self.get_logger().info("orientation target (degrees)")
            self.get_logger().info(np.array_str(ori_des_euler_degrees))
            num_iterations = 0
            success = False
            while num_iterations < max_iterations and not success:
                q_k = np.copy(q_k_plus_one)
                jacobian = self.dyn_model.ComputeJacobian(
                    q_k, target_frame, local_or_global).J
                position, orientation = self.dyn_model.ComputeFK(
                    q_k, target_frame)

                cur_quat = pin.Quaternion(orientation)
                cur_quat = cur_quat.normalize()

                # Ensure quaternion is in the same hemisphere as the desired orientation
                cur_quat_coeff = cur_quat.coeffs()
                ori_des_quat_coeff = ori_des_quat.coeffs()
                if np.dot(cur_quat_coeff, ori_des_quat_coeff) < 0.0:
                    cur_quat_coeff = cur_quat_coeff * -1.0
                    cur_quat = pin.Quaternion(cur_quat_coeff)

                # Compute the "difference" quaternion (assuming orientation_d is also a pin.Quaternion object)
                angle_error_quat = cur_quat.inverse() * ori_des_quat
                # extract coefficient x y z from the quaternion
                angle_error = angle_error_quat.coeffs()
                angle_error = angle_error[:3]

                # rotate the angle error in the base frame
                angle_error_base_frame = orientation@angle_error

                # computing position error
                pos_error = target_position - position

                if position_only:
                    trimmed_jacobian = np.copy(jacobian[0:3, :])
                    cur_error = pos_error
                elif orientation_only:
                    trimmed_jacobian = np.copy(jacobian[3:, :])
                    cur_error = angle_error_base_frame
                else:
                    trimmed_jacobian = np.copy(jacobian)
                    cur_error = np.concatenate(
                        (pos_error, angle_error_base_frame), axis=0)
                self.get_logger().debug("jacobian")
                self.get_logger().debug(np.array_str(jacobian))
                self.get_logger().debug("position at k")
                self.get_logger().debug(np.array_str(position))
                self.get_logger().debug("orientation at k")
                self.get_logger().debug(np.array_str(orientation))
                self.get_logger().debug(f"jacobian: {trimmed_jacobian.shape}")
                # q_k_plus_one = q_k + step_size * do_dampened_pseudo_inverse(trimmed_jacobian, dampening_factor) @ (cur_error)
                q_k_plus_one = q_k + step_size * \
                    np.linalg.pinv(trimmed_jacobian) @ (cur_error)
                num_iterations += 1
                success = np.linalg.norm(q_k_plus_one - q_k) < tolerance
                # success = np.linalg.norm(angle_error_base_frame) < tolerance and np.linalg.norm(pos_error) < tolerance

            self.get_logger().debug("forward kinematics with the solution results in:")
            self.get_logger().debug(f"q_k:\n{np.array_str(q_k)}")
            self.get_logger().debug(
                f"q_k_plus_one:\n{np.array_str(q_k_plus_one)}")
            position, orientation = self.get_pose(q_k_plus_one, target_frame)
            self.get_logger().debug("position:")
            self.get_logger().debug(np.array_str(position))
            self.get_logger().debug("orientation:")
            self.get_logger().debug(np.array_str(orientation))
            self.get_logger().debug("error (orientation error in quat)")
            self.get_logger().debug(np.array_str(cur_error))
            if not success:
                self.get_logger().error(
                    f"could not solve for solution in {max_iterations} iterations")
            else:
                self.get_logger().info(
                    f"found solution in {num_iterations} iterations")
                joint_angles = q_k_plus_one
        else:
            pybullet_client = self.sim.GetPyBulletClient()
            bot_id = 0
            link_id = self.sim.bot[bot_id].link_name_to_id[target_frame]
            joint_limits = np.array(JOINT_LIMITS)
            pybullet_robot_index = self.sim.bot[bot_id].bot_pybullet

            joint_poses_pybullet = pybullet_client.calculateInverseKinematics(pybullet_robot_index,
                                                                              link_id,
                                                                              target_position,
                                                                              ori_des_quat.coeffs(),
                                                                              lowerLimits=joint_limits[:, 0],
                                                                              upperLimits=joint_limits[:, 1])
            #   jointRanges=jr,
            #   restPoses=rp
            joint_poses_pybullet = np.array(joint_poses_pybullet)

            self.get_logger().debug("pybullet poses")
            self.get_logger().debug(f"{np.array_str(joint_poses_pybullet)}")
            joint_angles = joint_poses_pybullet
        return joint_angles

    def set_pose(self, msg):
        position = np.array([msg.x, msg.y, msg.z])
        euler_angles_degrees = np.array([msg.rx, msg.ry, msg.rz])
        target_frame = msg.frame
        if not target_frame:
            target_frame = "gripper"

        tolerance = self.get_parameter('solution_tol').value
        step_size = self.get_parameter('step_size').value
        dampening_factor = self.get_parameter('dampening_factor').value
        initial_guess_is_cur_pos = self.get_parameter(
            'initial_guess_is_cur_pos').value
        max_iterations = self.get_parameter('max_iterations').value
        use_pybullet = self.get_parameter('use_pybullet').value

        angles = self.calculate_ik(position, euler_angles_degrees, target_frame, tolerance,
                                   step_size, dampening_factor, initial_guess_is_cur_pos, max_iterations, use_pybullet)

        if angles is None:
            return

        adjusted_angles = self.adjust_angles(angles)

        new_joint_msg = MycobotSetAngles()
        new_joint_msg.joint_1 = adjusted_angles[0]
        new_joint_msg.joint_2 = adjusted_angles[1]
        new_joint_msg.joint_3 = adjusted_angles[2]
        new_joint_msg.joint_4 = adjusted_angles[3]
        new_joint_msg.joint_5 = adjusted_angles[4]
        new_joint_msg.joint_6 = adjusted_angles[5]
        new_joint_msg.speed = self.get_parameter('move_speed').value

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
