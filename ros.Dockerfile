FROM ros:noetic-ros-core AS base

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  libsdl1.2-dev libsdl2-dev \
  git \
  ros-noetic-catkin \
  && rm -rf /var/lib/apt/lists/*

# Set up ROS environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Create Catkin workspace
RUN mkdir -p /root/ardrone_ws/src
WORKDIR /root/ardrone_ws/src
RUN source /opt/ros/noetic/setup.bash && catkin_init_workspace

# Clone required repositories
RUN git clone https://github.com/sleutenegger/ardrone_autonomy.git
RUN git clone https://sleutenegger@bitbucket.org/sleutenegger/parrot_ardrone.git
RUN git clone https://github.com/atraboulsi27/ardrone_practicals.git

# Build Catkin workspace
WORKDIR /root/ardrone_ws
RUN source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release
RUN echo "source /root/ardrone_ws/devel/setup.bash" >> /root/.bashrc

# Run roslaunch at container startup
CMD ["roslaunch", "ardrone_practicals", "arp_simulation_challenge.launch"]