FROM ros:humble as base

RUN apt update && apt install -y ros-humble-pinocchio

RUN mkdir -p /workspace/src
WORKDIR /workspace

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /workspace/src
RUN git clone -b client-examples https://github.com/VModugno/mycobot_client
RUN git clone -b ros2-seperating https://github.com/VModugno/mycobot_communications

WORKDIR /workspace
RUN /bin/bash -c '. /opt/ros/humble/setup.bash; colcon build --symlink-install'