FROM ros:humble as base

RUN apt update && apt install -y ros-humble-pinocchio

RUN mkdir -p /workspace/src
WORKDIR /workspace

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /workspace/src
RUN git clone -b client-examples https://github.com/VModugno/mycobot_client
RUN git clone -b ros2-seperating https://github.com/VModugno/mycobot_communications

COPY ./simulation_and_control-0.1-py3-none-any.whl /tmp/simulation_and_control-0.1-py3-none-any.whl
RUN pip install /tmp/simulation_and_control-0.1-py3-none-any.whl

WORKDIR /workspace
RUN /bin/bash -c '. /opt/ros/humble/setup.bash; colcon build --symlink-install'