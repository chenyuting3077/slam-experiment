FROM ros:noetic-ros-base-focal
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
      git curl build-essential cmake python3-catkin-tools \
      libpcl-dev libeigen3-dev ros-noetic-pcl-ros ros-noetic-pcl-conversions \
      ros-noetic-tf ros-noetic-eigen-conversions ros-noetic-tf-conversions ros-noetic-nav-msgs ros-noetic-geometry-msgs ros-noetic-rosbag \
    && rm -rf /var/lib/apt/lists/*
RUN mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src \
    && git clone https://github.com/Livox-SDK/livox_ros_driver.git \
    && git clone https://github.com/hku-mars/FAST_LIO.git \
    && cd FAST_LIO && git submodule update --init
RUN source /opt/ros/noetic/setup.bash && cd ~/catkin_ws && (catkin_make -DCMAKE_BUILD_TYPE=Release -j2 || catkin_make -DCMAKE_BUILD_TYPE=Release -j2)  # first pass fails once: FAST_LIO does not depend on its own generated Pose6D.h
WORKDIR /root/catkin_ws
