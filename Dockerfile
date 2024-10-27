# note, by default dds will try to use shared memory to communicate if using host networking with docker
# , so containers should have access to this https://answers.ros.org/question/370595/ros2-foxy-nodes-cant-communicate-through-docker-container-border/

FROM ros:humble-ros-base

# https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#the-ros-localhost-only-variable
ENV ROS_LOCALHOST_ONLY=0
# docker run -it --network host --volume="/home/mz/mycobot_client:/mycobot_client" --volume "/home/mz/img_plots:/root/img_plots" test

# libgl1 is hidden dep of opencv-python
RUN apt update && apt install nano git ros-humble-pinocchio ros-humble-image-transport ros-humble-compressed-image-transport python3-colcon-common-extensions libgl1 python3-pip -y
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# building pybullet freezes the raspberrypi so manually limit parallelization
RUN git clone --depth 1 --branch 3.25 https://github.com/bulletphysics/bullet3/
WORKDIR /bullet3
RUN sed -i 's/.*multiprocessing.cpu_count.*/  N = 1/' setup.py
RUN pip install .

WORKDIR /
# numpy 2.0 brings attribute error
RUN pip install numpy==1.26.* robot_descriptions matplotlib opencv-python  git+https://github.com/VModugno/simulation_and_control/

RUN mkdir -m 777 /root/img_plots
RUN echo "umask 0000" >> /root/.bashrc

RUN mkdir -p /stale_workspace/src
WORKDIR /stale_workspace
WORKDIR /stale_workspace/src
RUN git clone https://github.com/VModugno/mycobot_client
WORKDIR /stale_workspace
RUN /bin/bash -c '. /opt/ros/humble/setup.bash; colcon build --symlink-install'
WORKDIR /
# RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc