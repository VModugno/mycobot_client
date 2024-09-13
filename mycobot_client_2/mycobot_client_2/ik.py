from mycobot_client_2.cobot_client import MycobotClient, CurAngles

import numpy as np
import time
import os
from ament_index_python.packages import get_package_share_directory
import simulation_and_control as sac
from simulation_and_control.sim import pybullet_robot_interface as pb
from simulation_and_control.controllers.servo_motor import MotorCommands
from simulation_and_control.controllers.pin_wrapper import PinWrapper

def main():
    # Configuration for the simulation
    package_share_directory = get_package_share_directory('mycobot_client_2')
    conf_file_name = "elephantconfig.json"  # Configuration file for the robot
    print(package_share_directory)
    dir_with_configs = package_share_directory
    sim = pb.SimInterface(conf_file_name, conf_file_path_ext = dir_with_configs)  # Initialize simulation interface

    # Get active joint names from the simulation
    ext_names = sim.getNameActiveJoints()
    ext_names = np.expand_dims(np.array(ext_names), axis=0)  # Adjust the shape for compatibility

    source_names = ["pybullet"]  # Define the source for dynamic modeling

    # Create a dynamic model of the robot
    dyn_model = PinWrapper(conf_file_name, "pybullet", ext_names, source_names, False,0, dir_with_configs)  

    # Display dynamics information
    print("Joint info simulator:")
    sim.GetBotJointsInfo()

    print("Link info simulator:")
    sim.GetBotDynamicsInfo()

    print("Link info pinocchio:")
    dyn_model.getDynamicsInfo()

    q_k = np.array([0, 0, 0, 0, 0, 0])
    end_effector_frame = "joint_6"
    local_or_global = "local_global"
    q_k_plus_one = np.copy(q_k)
    q_k_plus_one[:] = 99
    target_pose = np.array([0.2, -0.1, 0, 0, 0, 0])
    num_iterations = 0
    while np.linalg.norm(q_k_plus_one - q_k) > 0.5:
        q_k = np.copy(q_k_plus_one)
        jacobian = dyn_model.ComputeJacobian(q_k, end_effector_frame, local_or_global)
        pose_at_k = dyn_model.ComputeFK(q_k, end_effector_frame)
        q_k_plus_one = q_k + np.linalg.inv(jacobian) @ (target_pose - pose_at_k)
        num_iterations += 1
    print(f"found solution in {num_iterations} iterations")
    print(q_k_plus_one)
    
    # use ComputeJacobian(self,q0,frame_name,local_or_global)
    # to get jacobian, and numerical solver from allesandro's slides

    # joint6

    # Command and control loop
    # cmd = MotorCommands()  # Initialize command structure for motors
    # while True:
    #     cmd.tau_cmd = np.zeros((dyn_model.getNumberofActuatedJoints(),))  # Zero torque command
    #     sim.Step(cmd, "torque")  # Simulation step with torque command

    #     if dyn_model.visualizer: 
    #         for index in range(len(sim.bot)): # Conditionally display the robot model
    #             q = sim.GetMotorAngles(index)
    #             dyn_model.DisplayModel(q)  # Update the display of the robot model

    #     # Exit logic with 'q' key
    #     keys = sim.GetPyBulletClient().getKeyboardEvents()
    #     qKey = ord('q')
    #     if qKey in keys and keys[qKey] and sim.GetPyBulletClient().KEY_WAS_TRIGGERED:
    #         break

    #     time.sleep(0.1)  # Slow down the loop for better visualization

if __name__ == '__main__':
    main()