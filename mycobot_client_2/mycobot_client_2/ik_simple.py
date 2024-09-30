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
import pybullet as pb
import pybullet_data
from pybullet_utils import bullet_client


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

SPEED_MIN = 0
SPEED_MAX = 100


class CobotIK(Node):
    def __init__(self, speed:int = 30):
        if speed < SPEED_MIN or speed > SPEED_MAX:
            raise ValueError(f"speed must be between {SPEED_MIN} and {SPEED_MAX}, {speed} was given")
        self.speed = speed
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

        self.get_logger().info("start ...")

        self.package_share_directory = get_package_share_directory(
            'mycobot_client_2')
        self.conf_file_name = "elephantconfig.json"  # Configuration file for the robot
        self.urdf_file_name = "mycobot_280_pi.urdf"
        self.urdf_file_full_path_name = os.path.join(self.package_share_directory, "models", "elephant_description", self.urdf_file_name)

        self.get_logger().info(
            f"share directory {self.package_share_directory}")

        # here i need to create the environment and get the robot object
        self.pybullet_client = bullet_client.BulletClient(connection_mode=pb.GUI)
        self.pybullet_client.setPhysicsEngineParameter(numSolverIterations=30)
        self.pybullet_client.setTimeStep(0.001)
        self.pybullet_client.setGravity(0, 0, -9.81)
        self.pybullet_client.setPhysicsEngineParameter(enableConeFriction=0)
        self.pybullet_client.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.ground_body = self.pybullet_client.loadURDF("plane.urdf")
        
        flags = self.pybullet_client.URDF_USE_INERTIA_FROM_FILE | self.pybullet_client.URDF_USE_SELF_COLLISION
        self.bot_pybullet = self.pybullet_client.loadURDF(
            self.urdf_file_full_path_name,
            [0, 0, 0],
            [0, 0, 0, 1],
            useFixedBase=True,
            flags=flags)
        self.pybullet_client.resetBasePositionAndOrientation(self.bot_pybullet, [0, 0, 0], [0, 0, 0, 1])
        self.link_name_to_id = {}
        self.buildLinkNameToId(self.pybullet_client)
    

    def buildLinkNameToId(self, pybullet_client):
        num_joints = pybullet_client.getNumJoints(self.bot_pybullet)
        self.link_name_to_id = {}
        for i in range(num_joints):
            joint_info = pybullet_client.getJointInfo(self.bot_pybullet, i)
            self.link_name_to_id[joint_info[12].decode("UTF-8")] = joint_info[0]

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
        for i in range(6):
            joint_id = self.link_name_to_id[f"joint{i+1}"]
            self.pybullet_client.resetJointState(self.bot_pybullet, joint_id, DEGREES_TO_RADIANS * angles[i])
        self.real_angles = angles

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
        # getJointState
        joint_id = self.link_name_to_id[target_frame]
        link_state = self.pybullet_client.getLinkState(self.bot_pybullet, joint_id, computeForwardKinematics=True)

        linkWorldPosition = link_state[0]
        linkWorldOrientation = link_state[1]
        worldLinkFramePosition = link_state[4]
        worldLinkFrameOrientation = link_state[5]

        world_orientation = pb.getEulerFromQuaternion(linkWorldOrientation)
        world_orientation = np.array(world_orientation)

        euler_angles = RADIAN_TO_DEGREES * world_orientation
        return linkWorldPosition, euler_angles


    def get_real_angles(self) -> npt.NDArray[float]:
        """Helper function to return the current angles.

        Returns:
            npt.NDArray[float]: numpy array with the joint angles 1-6.
        """
        return np.copy(self.real_angles)

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

    def calculate_ik(self, target_position, euler_angles_degrees, target_frame):
        """_summary_

        Args:
            position (_type_): np.array 3D
            euler_angles_degrees (_type_): np.array 3D
            target_frame (_type_): _description_
        """
        rx, ry, rz = euler_angles_degrees

        ori_des_euler_degrees = np.array([rx, ry, rz])
        ori_des_euler_radians = ori_des_euler_degrees * DEGREES_TO_RADIANS

        ori_des_quat = pb.getQuaternionFromEuler(ori_des_euler_radians)
        
        link_id = self.link_name_to_id[target_frame]
        joint_limits = np.array(JOINT_LIMITS)
        pybullet_robot_index = self.bot_pybullet

        joint_poses_pybullet = self.pybullet_client.calculateInverseKinematics(pybullet_robot_index,
                                                                            link_id,
                                                                            target_position,
                                                                            ori_des_quat,
                                                                            lowerLimits=joint_limits[:, 0],
                                                                            upperLimits=joint_limits[:, 1])
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

        angles = self.calculate_ik(position, euler_angles_degrees, target_frame)

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
        new_joint_msg.speed = self.speed

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
