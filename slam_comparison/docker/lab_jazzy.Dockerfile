FROM ros:jazzy
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-cartographer-ros ros-jazzy-rtabmap-ros ros-jazzy-rosbag2-storage-mcap \
      ros-jazzy-robot-state-publisher ros-jazzy-tf2-ros ros-jazzy-pcl-ros \
      python3-pip python3-colcon-common-extensions git build-essential \
    && rm -rf /var/lib/apt/lists/*
COPY src /root/colcon_ws/src
RUN . /opt/ros/jazzy/setup.sh && cd /root/colcon_ws && colcon build --packages-select compal_amr_description spot_cartographer
