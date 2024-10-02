# mycobot_client

We recommend ros2.

## ros1
Checkout the branch `ros-noetic-1.0.0`.

## ros2

For this, ensure that the robot arm is running and connected to the same network.

You can [RoboEnv](https://github.com/VModugno/RoboEnv) to install the environment. From there, copy the mycobot_msgs_2 folder to this subfolder. Then set the `ROS_DOMAIN_ID` variable to whatever is set on the robot. From here, you can run one of the example scripts or write your own using the classes.

### Example Script
```
colcon build --packages-select mycobot_client_2 mycobot_msgs_2
source install/setup.bash
export ROS_DOMAIN_ID=10
ros2 run mycobot_client_2 ik_demo
```

### Writing your Own

For this make a new script in the `mycobot_client_2/mycobot_client_2` folder and structure it off of the `run_task_simple.py` script. You can import the `from mycobot_client_2.ik_simple import CobotIK` class and use it to control the arm in your script. To add your script to the client package you will have to edit the `setup.py` file and then build the package `colcon build --symlink-install` and source it `source install/setup.bash`.

Docs for the `CobotIk` class are below.

```
Help on module ik_simple:

NAME
    ik_simple

CLASSES
    rclpy.node.Node(builtins.object)
        CobotIK
    
    class CobotIK(rclpy.node.Node)
     |  CobotIK(speed: int = 30)
     |  
     |  Class that exposes functions to talk to the robot arm given numpy arrays. It translates these to the needed ros messages. It also calculates IK.
     |  
     |  Method resolution order:
     |      CobotIK
     |      rclpy.node.Node
     |      builtins.object
     |  
     |  Methods defined here:
     |  
     |  __init__(self, speed: int = 30)
     |      Initialize the sim and such.
     |      
     |      Args:
     |          speed (int, optional): how fast should the arm move when given commands. 0-100. Defaults to 30.
     |      
     |      Raises:
     |          ValueError: if speed is wrong.
     |  
     |  adjust_angles(self, target_angles: numpy.ndarray[typing.Any, numpy.dtype[float]]) -> numpy.ndarray[typing.Any, numpy.dtype[float]]
     |      Takes angles in degrees. Bounds them and wraps them as needed.
     |      
     |      Args:
     |          target_angles (npt.NDArray[float]): input angles
     |      
     |      Returns:
     |          npt.NDArray[float]: bounded angles
     |  
     |  calculate_ik(self, target_position: numpy.ndarray[typing.Any, numpy.dtype[float]], euler_angles_degrees: Optional[numpy.ndarray[Any, numpy.dtype[float]]] = None, target_frame: Optional[str] = None) -> numpy.ndarray[typing.Any, numpy.dtype[float]]
     |      calculate what angles the robot joints should have to reach the targets. Takes angles in degrees.
     |      
     |      Args:
     |          target_position (npt.NDArray[float]): x, y, z
     |          euler_angles_degrees (Optional[npt.NDArray[float]], optional): rx, ry, rz, or None. Easier to solve if None. Defaults to None.
     |          target_frame (Optional[str], optional): what frame to use for the calcs. Defaults to None.
     |      
     |      Returns:
     |          npt.NDArray[float]: the joint angles
     |  
     |  close_gripper(self)
     |      Close the gripper.
     |  
     |  get_pose(self, cur_joint_angles: Optional[numpy.ndarray[Any, numpy.dtype[float]]] = None, target_frame: str = 'gripper') -> Tuple[numpy.ndarray[Any, numpy.dtype[float]], numpy.ndarray[Any, numpy.dtype[float]]]
     |      Helper function to get the current pose of the robot from the current angles.
     |      
     |      Args:
     |          cur_joint_angles (Optional[npt.NDArray[float]], optional): What current angles to calculate direct kinematics with? 6D numpy array. Defaults to None.. Defaults to None.
     |          target_frame (str, optional): What frame should the pose be in, from frames in the URDF. Defaults to "gripper".. Defaults to "gripper".
     |      
     |      Returns:
     |          Tuple[npt.NDArray[float], npt.NDArray[float]]: _description_
     |  
     |  get_real_angles(self) -> numpy.ndarray[typing.Any, numpy.dtype[float]]
     |      Helper function to return the current angles.
     |      
     |      Returns:
     |          npt.NDArray[float]: numpy array with the joint angles 1-6.
     |  
     |  open_gripper(self)
     |      Open the gripper.
     |  
     |  publish_angles(self, angles: numpy.ndarray[typing.Any, numpy.dtype[float]])
     |      Takes command angles in degrees and publishes them to ros.
     |      
     |      Args:
     |          angles (npt.NDArray[float]): _description_
     |  
     |  set_pose(self, x: float, y: float, z: float, rx: float, ry: float, rz: float, frame: str = 'gripper')
     |      Takes angles in degrees, calculates inverse kinematics, and then publishes the output to ROS. Wrapper function.
     |      
     |      Args:
     |          x (float): _description_
     |          y (float): _description_
     |          z (float): _description_
     |          rx (float): _description_
     |          ry (float): _description_
     |          rz (float): _description_
     |          frame (str, optional): _description_. Defaults to "gripper".
     |  
     |  update_angles_msg(self, msg: mycobot_msgs_2.msg._mycobot_angles.MycobotAngles)
     |      Helper function to be called in a ROS2 callback that takes the message and stores it in a numpy array in the class.
     |      
     |      Args:
     |          msg (MycobotAngles): msg from mycobot_msgs_2
     |  
     |  update_pybullet(self, angles: numpy.ndarray[typing.Any, numpy.dtype[float]])
     |      Helper function to update the joint positions in pybullet given the joint angles that the robot is at.
     |      
     |      Args:
     |          angles (npt.NDArray[float]): 6D array of the joint angles in degrees
     |  
     |  update_real_angles(self, angles: numpy.ndarray[typing.Any, numpy.dtype[float]])
     |      Helper function to take a numpy array and update pybullet as well as the classes' member.
     |      
     |      Args:
     |          angles (npt.NDArray[float]): 6D array of the joint angles in degrees

FUNCTIONS
    main(args=None)

DATA
    COBOT_GRIPPER_STATUS_TOPIC = 'mycobot/gripper_status'
    COBOT_JOINT_GOAL_TOPIC = 'mycobot/angles_goal'
    COBOT_JOIN_REAL_TOPIC = 'mycobot/angles_real'
    COBOT_POSE_GOAL_TOPIC = 'mycobot/pose_goal'
    DEGREES_TO_RADIANS = 0.017453292519943295
    JOINT_LIMITS = [[-165, 165], [-165, 165], [-165, 165], [-165, 165], [-...
    JOINT_NAMES = ['joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'join...
    JOINT_SIGNS = [1, 1, 1, 1, 1, 1]
    NUM_ANGLES = 6
    Optional = typing.Optional
        Optional[X] is equivalent to Union[X, None].
    
    RADIAN_TO_DEGREES = 57.29577951308232
    SPEED_MAX = 100
    SPEED_MIN = 0
    Tuple = typing.Tuple
        Deprecated alias to builtins.tuple.
        
        Tuple[X, Y] is the cross-product type of X and Y.
        
        Example: Tuple[T1, T2] is a tuple of two elements corresponding
        to type variables T1 and T2.  Tuple[int, float, str] is a tuple
        of an int, a float and a string.
        
        To specify a variable-length tuple of homogeneous type, use Tuple[T, ...].

FILE
    /home/mz/mycobot_client/mycobot_client_2/mycobot_client_2/ik_simple.py
```

## Troubleshooting
If you are running client examples and the pybullet window is coming up but with nothing inside, and you are on a virtual machine, try turning off hardware acceleration in your virtual machine settings.