
```
ros2 run mycobot_client_2 cobot_ik
```


```
ros2 topic pub /mycobot/pose_goal mycobot_msgs_2/msg/MycobotPose "{x: 0.1, y: 0, z: 0}" --once
```

To visualize
```
apt install ros-humble-joint-state-publisher
apt install ros-humble-joint-state-publisher-gui
apt-get install ros-humble-xacro
ros2 launch mycobot_client_2 display.launch.py urdf_package:=mycobot_client_2 urdf_package_path:=models/elephant_description/mycobot_280_pi.urdf
```