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

For this make a new script in the `mycobot_client_2/mycobot_client_2` folder and structure it off of the `run_task.py` script. You can import the `from mycobot_client_2.ik_pybullet import CobotIK` class and use it to control the arm in your script. To add your script to the client package you will have to edit the `setup.py` file and then build the package `colcon build --symlink-install` and source it `source install/setup.bash`.

Docs for the `CobotIk` class are below.

```
Help on module ik_pybullet:

NAME
    ik_pybullet

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
    /home/mz/mycobot_client/mycobot_client_2/mycobot_client_2/ik_pybullet.py
```

Docs for the `CameraCalculator` class are below.
```
Help on module camera_calculator:

NAME
    camera_calculator

CLASSES
    builtins.object
        Images
    rclpy.node.Node(builtins.object)
        CameraCalculator
    
    class CameraCalculator(rclpy.node.Node)
     |  This is a class to run calculations on the camera images. Like making pointclouds from them and 
     |  getting the world coordinates of a pixel in the image. This intended to make it easier for students to write logic
     |  to find things in the image and move the robot arm.
     |  
     |  Args:
     |      Node (_type_): _description_
     |  
     |  Method resolution order:
     |      CameraCalculator
     |      rclpy.node.Node
     |      builtins.object
     |  
     |  Methods defined here:
     |  
     |  display_images(self, img: camera_calculator.Images, markpoint_u_v: Optional[Tuple[int, int]] = None)
     |      Helper function to display an image pair and optionally to mark a point on the image with a cross.
     |      
     |      Args:
     |          img (Images): _description_
     |          markpoint_u_v (Optional[Tuple[int, int]], optional): _description_. Defaults to None.
     |  
     |  get_3d_points(self, color_img: numpy.ndarray[typing.Any, numpy.dtype[float]], depth_img: numpy.ndarray[typing.Any, numpy.dtype[float]], intrinsics: numpy.ndarray[typing.Any, numpy.dtype[+_ScalarType_co]]) -> numpy.ndarray[typing.Any, numpy.dtype[numpy.float32]]
     |      Helper function to get a pointcloud from an image pair and intrinsics. In the Camera frame. It will then take the RGB element and 
     |      pack those 3 bytes into a 4 byte float in a way that rviz2 can read.
     |      
     |      Args:
     |          color_img (npt.NDArray[float]): _description_
     |          depth_img (npt.NDArray[float]): _description_
     |          intrinsics (npt.NDArray): _description_
     |      
     |      Returns:
     |          npt.NDArray[np.float32]: _description_
     |  
     |  get_3d_points_from_pixel_point_on_color(self, img: camera_calculator.Images, u: int, v: int) -> Tuple[numpy.ndarray[Any, numpy.dtype[numpy.float32]], numpy.ndarray[Any, numpy.dtype[numpy.float32]]]
     |      Function to take image pair and intrinsics, and calculate the 3d location in world frame of a pixel. You could use a color mask or 
     |      contour stuff to get the center of an object in pixel coordinates, then pass it to this function to get the 3d location
     |      of the object in the world frame.
     |      
     |      Args:
     |          img (Images): image pair and intrinsics
     |          u (int): the u pixel coordinate. Note that in a numpy image the rows correspond to v, the columns to u.
     |          v (int): the v pixel coordinate. Note that in a numpy image the rows correspond to v, the columns to u.
     |      
     |      Returns:
     |          Tuple[npt.NDArray[np.float32], npt.NDArray[np.float32]]: the first array is points in the camera frame, second array is points in the world frame
     |  
     |  get_images(self) -> camera_calculator.Images
     |      Helper function to get an image pair and calculate the pointcloud. It will return None if not available, so check for that.
     |      
     |      Returns:
     |          Images: object with the image pair, intrinsics, and pointcloud. It's a chunky boy so don't keep too many in memory.
     |  
     |  points_to_pountcloud(self, xyz_rgb: numpy.ndarray[typing.Any, numpy.dtype[numpy.float32]], frame_id: str) -> sensor_msgs.msg._point_cloud2.PointCloud2
     |      Helper function to take in a numpy pointcloud nx4, where the 4 is x, y, z, and a float containing RGB in a format for rviz.
     |      It packs this into a PointCloud2 object that can be published to ROS.
     |      
     |      Args:
     |          xyz_rgb (npt.NDArray[np.float32]): numpy pointcloud
     |          frame_id (str): what frame to use for the pointcloud.
     |      
     |      Returns:
     |          PointCloud2: _description_
     |  
     |  translate_to_world_frame(self, xyz_rgb: numpy.ndarray[typing.Any, numpy.dtype[numpy.float32]]) -> numpy.ndarray[typing.Any, numpy.dtype[numpy.float32]]
     |      Helper function to translate a pointcloud with rgb data from camera to world frame.
     |      
     |      Args:
     |          xyz_rgb (npt.NDArray[np.float32]): nx4 array. the 4 is x, y, z, and a float containing r,g,b, compressed for rviz.
     |      
     |      Returns:
     |          _type_: npt.NDArray[np.float32]: nx4 array, in world coordinates.
     |  
    class Images(builtins.object)
     |  Images(color: numpy.ndarray[typing.Any, numpy.dtype[float]], depth: numpy.ndarray[typing.Any, numpy.dtype[float]], intrinsics: numpy.ndarray[typing.Any, numpy.dtype[float]], xyz_rgb: numpy.ndarray[typing.Any, numpy.dtype[numpy.float32]], xyz_rgb_frame: str) -> None
     |  
     |  This is a datastructure to hold color/depth images and intrinsics, as well as the pointcloud.
     |  It's a chunky boy, don't hold too many in memory at once.
     |  
     |  __init__(self, color: numpy.ndarray[typing.Any, numpy.dtype[float]], depth: numpy.ndarray[typing.Any, numpy.dtype[float]], intrinsics: numpy.ndarray[typing.Any, numpy.dtype[float]], xyz_rgb: numpy.ndarray[typing.Any, numpy.dtype[numpy.float32]], xyz_rgb_frame: str) -> None
     |      Initialize self.  See help(type(self)) for accurate signature.
     |  
DATA
    CAMERA_FRAME_TO_ROBOT_FRAME_EXTRINSICS = array([[-0.6905175 , -0.50300...
    COLOR_CAMERA_FRAME_ID = 'camera_color_optical_frame'
    COLOR_CAMERA_INFO_TOPIC_NAME = '/camera/realsense2_camera_node/color/i...
    COLOR_CAMERA_TOPIC_NAME = '/camera/realsense2_camera_node/color/image_...
    DEPTH_CAMERA_FRAME_ID = 'camera_depth_optical_frame'
    DEPTH_CAMERA_INFO_TOPIC_NAME = '/camera/realsense2_camera_node/depth/i...
    DEPTH_CAMERA_TOPIC_NAME = '/camera/realsense2_camera_node/depth/image_...
    DEPTH_SCALE = 0.001
    Optional = typing.Optional
        Optional[X] is equivalent to Union[X, None].
    
    POINTCLOUD_TOPIC_NAME = '/camera/pointcloud'
    Tuple = typing.Tuple
        Deprecated alias to builtins.tuple.
        
        Tuple[X, Y] is the cross-product type of X and Y.
        
        Example: Tuple[T1, T2] is a tuple of two elements corresponding
        to type variables T1 and T2.  Tuple[int, float, str] is a tuple
        of an int, a float and a string.
        
        To specify a variable-length tuple of homogeneous type, use Tuple[T, ...].
    
    WORLD_FRAME_ID = 'map'
```


## Troubleshooting
If you are running client examples and the pybullet window is coming up but with nothing inside, and you are on a virtual machine, try turning off hardware acceleration in your virtual machine settings.