FROM osrf/ros:humble-desktop

# xhost +local:root; sudo docker run -it --network host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="/home/mz/mycobot_client:/mycobot_client" --volume "/home/er/img_plots:/root/img_plots" mzandtheraspberrypi/mycobot-client-humble:1.0.0; xhost -local:root;

RUN apt update && apt install -y ros-humble-pinocchio

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN apt install python3-pip -y
RUN pip install git+https://github.com/VModugno/simulation_and_control/
RUN pip install pybullet==3.1.0 robot_descriptions

RUN mkdir -p /workspace/src
WORKDIR /workspace
WORKDIR /workspace/src
RUN git clone https://github.com/VModugno/mycobot_client
WORKDIR /workspace
RUN /bin/bash -c '. /opt/ros/humble/setup.bash; colcon build --symlink-install'
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc