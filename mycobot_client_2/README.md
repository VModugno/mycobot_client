

```
xhost +local:root; sudo docker run -it --network host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" mzandtheraspberrypi/mycobot-client-humble:1.0.0; xhost -local:root;
ros2 run mycobot_client_2 cobot_ik
```


```
ros2 topic pub /mycobot/pose_goal mycobot_msgs_2/msg/MycobotPose "{frame: gripper, x: 0.04, y: -0.06, z: 0.45, rx: -40, ry: 0, rz: -75}" --once
ros2 topic pub /mycobot/pose_goal mycobot_msgs_2/msg/MycobotPose "{frame: gripper, x: 0.04, y: -0.06, z: 0.45, rx: -40, ry: 0, rz: -55}" --once
```




To visualize
```
apt install ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-xacro
ros2 launch mycobot_client_2 display.launch.py urdf_package:=mycobot_client_2 urdf_package_path:=models/elephant_description/mycobot_280_pi.urdf
```