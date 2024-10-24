FROM ros:humble-ros-base

# xhost +local:root; sudo docker run -it --network host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="/home/mz/mycobot_client:/mycobot_client" --volume "/home/er/img_plots:/root/img_plots" mzandtheraspberrypi/mycobot-client-humble:1.0.0; xhost -local:root;

# libgl1 is hidden dep of opencv-python
RUN apt update && apt install nano git ros-humble-pinocchio ros-humble-image-transport ros-humble-compressed-image-transport python3-colcon-common-extensions libgl1 python3-pip -y

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN pip install pybullet==3.1.0 robot_descriptions opencv-python  git+https://github.com/VModugno/simulation_and_control/

RUN mkdir -p /stale_workspace/src
WORKDIR /stale_workspace
WORKDIR /stale_workspace/src
RUN git clone https://github.com/VModugno/mycobot_client
WORKDIR /stale_workspace
RUN /bin/bash -c '. /opt/ros/humble/setup.bash; colcon build --symlink-install'
WORKDIR /
# RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc