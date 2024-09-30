# mycobot_client

## ros1
Checkout the branch `ros-noetic-1.0.0`.

## ros2

For this, ensure that the robot arm is running and connected to the same network.

You can RoboEnv to install the environment. From there, copy the mycobot_msgs_2 folder to this subfolder. Then set the `ROS_DOMAIN_ID` variable to whatever is set on the robot. Then, build and run the client

```
colcon build --packages-select mycobot_client_2 mycobot_msgs_2
source install/setup.bash
export ROS_DOMAIN_ID=10
ros2 run mycobot_client_2 cobot_client
```


## Troubleshooting
If you are running client examples and the pybullet window is coming up but with nothing inside, and you are on a virtual machine, try turning off hardware acceleration in your virtual machine settings.