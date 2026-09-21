#!/bin/bash
# Cartographer on the Livox cloud but with the robot's Xsens IMU (/imu/data, frame imu_link) instead of /livox/imu,
# using the lab's own spot_offline_mapping.lua unchanged (tracking_frame imu_link).
#
# The Xsens IMU is merged into a copy of the converted Livox bag first: Cartographer's offline node would treat
# two bags as two trajectories.
#
# usage: run_lab_livox_carto_xsens.sh TAG XSENS_IMU_BAG
#   TAG            a tag already processed by run_lab_livox_all.sh (needs data/<TAG>_ros2)
#   XSENS_IMU_BAG  rosbag2 dir holding only /imu/data (scripts/slice_bag.py --topics /imu/data)
set -euo pipefail
cd "$(dirname "$0")/../.."
TAG=$1; XBAG=$(realpath "$2")
REPO=$PWD; U=$(id -u):$(id -g); W=/workspace
RUN=compal_amr/${OUTNAME:-$TAG}   # results go to outputs/$RUN/<system>/
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
D=$RUN/cartographer_xsens
set +u; source /opt/ros/jazzy/setup.bash; set -u
[ -d data/${TAG}_xsens_ros2 ] || python3 slam_comparison/scripts/merge_bags.py data/${TAG}_xsens_ros2 data/${TAG}_ros2 $XBAG
mkdir -p outputs/$D
docker run --rm -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID -v $REPO:$W slam-lab-jazzy bash -c "source /opt/ros/jazzy/setup.bash; source /root/colcon_ws/install/setup.bash; cd $W/outputs/$D
ros2 launch $W/slam_comparison/launch/cartographer_lab_livox.launch.py bag_filenames:=$W/data/${TAG}_xsens_ros2 save_state_filename:=$W/outputs/$D/$TAG.pbstream configuration_directory:=/root/colcon_ws/install/spot_cartographer/share/spot_cartographer/configuration_files configuration_basenames:=spot_offline_mapping.lua imu_topic:=/imu/data > offline_run.log 2>&1
chown -R $U $W/outputs/$D"
data/venv/bin/python slam_comparison/scripts/export_cartographer_pbstream_trajectory.py data/cartographer_pbpy outputs/$D/$TAG.pbstream outputs/$D/trajectory_tum.txt
python3 slam_comparison/scripts/compare_to_reference.py outputs/$RUN/reference/odometry_tum.txt Cartographer_xsens=outputs/$D/trajectory_tum.txt
python3 slam_comparison/scripts/compute_endpoint_error.py outputs/$D/trajectory_tum.txt
