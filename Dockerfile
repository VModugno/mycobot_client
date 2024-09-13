FROM osrf/ros:humble-desktop

# xhost +local:root; sudo docker run -it --network host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" test-humble-pinocchio; xhost -local:root;

RUN apt update && apt install -y ros-humble-pinocchio

RUN mkdir -p /workspace/src
WORKDIR /workspace

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

COPY ./simulation_and_control-0.1-py3-none-any.whl /tmp/simulation_and_control-0.1-py3-none-any.whl
RUN apt install python3-pip -y
RUN pip install /tmp/simulation_and_control-0.1-py3-none-any.whl
RUN pip install pybullet==3.1.0 robot_descriptions

WORKDIR /workspace/src
RUN git clone -b client-examples https://github.com/VModugno/mycobot_client
RUN git clone -b ros2-seperating https://github.com/VModugno/mycobot_communications

WORKDIR /workspace
RUN /bin/bash -c '. /opt/ros/humble/setup.bash; colcon build --symlink-install'
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc