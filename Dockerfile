FROM ros:noetic-ros-core

RUN sudo apt-get update
RUN sudo apt-get install build-essential cmake git nano python3-pip -y

RUN mkdir -p /home/ubuntu/catkin_ws/src
WORKDIR /home/ubuntu/catkin_ws/src
RUN git clone https://github.com/VModugno/mycobot_communications.git
COPY . mycobot_client/
WORKDIR /home/ubuntu/catkin_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/ubuntu/catkin_ws; catkin_make'
RUN echo ". /home/ubuntu/catkin_ws/devel/setup.bash">>/home/ubuntu/.bashrc