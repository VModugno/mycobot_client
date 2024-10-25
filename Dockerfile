FROM ros:humble-ros-base

# docker run -it --network host --volume="/home/mz/mycobot_client:/mycobot_client" --volume "/home/mz/img_plots:/root/img_plots" test

# libgl1 is hidden dep of opencv-python
RUN apt update && apt install nano git ros-humble-pinocchio ros-humble-image-transport ros-humble-compressed-image-transport python3-colcon-common-extensions libgl1 python3-pip -y
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# numpy 2.0 brings attribute error
RUN pip install pybullet==3.1.0 numpy==1.26.* robot_descriptions matplotlib opencv-python  git+https://github.com/VModugno/simulation_and_control/

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