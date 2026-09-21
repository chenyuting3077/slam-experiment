FROM slam-lab-jazzy
RUN apt-get update && apt-get install -y --no-install-recommends \
      libgtsam-dev libpcl-dev libopencv-dev ros-jazzy-tf2-eigen ros-jazzy-tf2-sensor-msgs \
      ros-jazzy-tf2-geometry-msgs ros-jazzy-pcl-msgs ros-jazzy-pcl-conversions ros-jazzy-cv-bridge \
      ros-jazzy-rosidl-default-generators ros-jazzy-visualization-msgs ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*
COPY lio_sam_official_build_fix.patch /tmp/lio_sam_official_build_fix.patch
# Official TixiaoShan/LIO-SAM (ros2 branch); the patch only fixes the Eigen find_package for jazzy.
RUN cd /root/colcon_ws/src && git clone -b ros2 https://github.com/TixiaoShan/LIO-SAM lio_sam \
    && cd lio_sam && git checkout 08af3f32f01725372d4269838dc44c19c6d9e76b && git apply /tmp/lio_sam_official_build_fix.patch
RUN . /opt/ros/jazzy/setup.sh && cd /root/colcon_ws && colcon build --packages-select lio_sam --cmake-args -DCMAKE_BUILD_TYPE=Release
