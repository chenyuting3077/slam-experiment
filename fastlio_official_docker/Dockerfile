FROM osrf/ros:noetic-desktop-full-focal

RUN apt-get update \
    && apt-get install -y git curl \
    && apt-get install -y python3-catkin-tools \
    && apt-get install -y libpcl-dev libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

RUN mkdir -p ~/catkin_ws/src \
    && cd ~/catkin_ws/src \
    && git clone https://github.com/Livox-SDK/livox_ros_driver.git \
    && cd ~/catkin_ws \
    && source /opt/ros/noetic/setup.bash \
    && catkin_make -DCMAKE_BUILD_TYPE=Release

RUN cd ~/catkin_ws/src \
    && git clone https://github.com/hku-mars/FAST_LIO.git \
    && cd FAST_LIO \
    && git submodule update --init \
    && cd ~/catkin_ws \
    && source /opt/ros/noetic/setup.bash \
    && source devel/setup.bash \
    && catkin_make -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
    && echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

WORKDIR /root/catkin_ws
