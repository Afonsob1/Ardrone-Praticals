FROM ubuntu:20.04 AS base

# install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends
RUN apt-get -y install libsdl1.2-dev libsdl2-dev

# install ros
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt -y install curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update
RUN apt-get -y install ros-noetic-desktop-full

# source
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc
RUN mkdir -p ~/ardrone_ws/src

# catkin workspace
RUN cd ~/ardrone_ws/src && catkin_init_workspace

# get ardrone_autonomy repo
RUN apt-get -y install git
RUN cd ~/ardrone_ws/src && git clone -b indigo-devel https://github.com/sleutenegger/ardrone_autonomy.git

# get parrow_ardrone drivers
RUN cd ~/ardrone_ws/src && git clone -b kinetic-gazebo9 https://sleutenegger@bitbucket.org/sleutenegger/parrot_ardrone.git

# get ardrone practicals
RUN cd ~/ardrone_ws/src && git clone https://github.com/atraboulsi27/ardrone_practicals

# build & source devel
RUN cd ~/ardrone_ws && catkin_make -DCMAKE_BUILD_TYPE=Release
RUN source ~/ardrone_ws/devel/setup.bash

RUN roslaunch ardrone_practicals arp_simulation_challenge.launch