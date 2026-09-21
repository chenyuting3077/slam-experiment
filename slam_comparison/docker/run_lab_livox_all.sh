#!/bin/bash
# Run Cartographer / RTAB-Map / official FAST_LIO / official LIO-SAM in parallel on the Livox Mid360 part of a
# Compal AMR lab bag (/livox/lidar CustomMsg + /livox/imu + /odometry + /tf_static).
#
# usage: run_lab_livox_all.sh SLICED_BAG_DIR TAG
#   SLICED_BAG_DIR  rosbag2 dir (mcap) with /livox/lidar /livox/imu /odometry /tf_static (scripts/slice_bag.py)
#   TAG             names the converted inputs (data/<TAG>_*) and, unless OUTNAME is set, the output folder
#                   outputs/compal_amr/<TAG>/<system>/ (set OUTNAME=dataset3_vanjee_full to use another folder name)
#
# Set ONLY=liosam (or e.g. fastlio2,rtabmap) to re-run just those systems.
# FAST_LIO and LIO-SAM use /livox/imu (frame livox_frame); FAST_LIO runs the official mid360.yaml unchanged.
# Needs the docker images slam-lab-jazzy, slam-lab-liosam, slam-lab-noetic-fastlio, host ROS jazzy, data/venv,
# data/cartographer_pbpy.
set -euo pipefail
cd "$(dirname "$0")/../.."
BAG=$(realpath "$1"); TAG=$2
REPO=$PWD; U=$(id -u):$(id -g)
RUN=compal_amr/${OUTNAME:-$TAG}   # results go to outputs/$RUN/<system>/
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
set +u; source /opt/ros/jazzy/setup.bash; set -u
PY=$REPO/data/venv/bin/python
SC=$REPO/slam_comparison/scripts
OUT=$REPO/outputs
mkdir -p $OUT/$RUN/{cartographer,rtabmap,fastlio2,liosam,reference}

echo "== converting inputs"
[ -d $REPO/data/${TAG}_ros2 ] || python3 $SC/convert_lab_livox_ros2.py $BAG $REPO/data/${TAG}_ros2
[ -f $REPO/data/${TAG}_fastlio.bag ] || $PY $SC/convert_lab_livox_ros1.py $(ls $BAG/*.mcap | head -1) $REPO/data/${TAG}_fastlio.bag
python3 $SC/export_bag_odometry_tum.py $BAG /odometry $OUT/$RUN/reference/odometry_tum.txt

want() { [ -z "${ONLY:-}" ] || [[ ",$ONLY," == *",$1,"* ]]; }
W=/workspace
ROS2BAG=$W/data/${TAG}_ros2
echo "== starting the four systems in parallel"
D=$RUN/cartographer
want cartographer && docker run --rm -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID -v $REPO:$W slam-lab-jazzy bash -c "source /opt/ros/jazzy/setup.bash; source /root/colcon_ws/install/setup.bash; cd $W/outputs/$D
ros2 launch $W/slam_comparison/launch/cartographer_lab_livox.launch.py bag_filenames:=$ROS2BAG save_state_filename:=$W/outputs/$D/$TAG.pbstream configuration_directory:=$W/slam_comparison/config > offline_run.log 2>&1
chown -R $U $W/outputs/$D" > /dev/null 2>&1 &

D=$RUN/rtabmap
want rtabmap && docker run --rm -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID -v $REPO:$W slam-lab-jazzy bash -c "source /opt/ros/jazzy/setup.bash; cd $W/outputs/$D
ros2 launch $W/slam_comparison/launch/rtabmap_lab.launch.py database_path:=$W/outputs/$D/rtabmap.db scan_topic:=/livox/points > rtabmap.log 2>&1 &
sleep 8
ros2 bag play $ROS2BAG --topics /tf_static /odometry /livox/points > bag_play.log 2>&1
sleep 10; pkill -INT -f '[r]tabmap'; sleep 8; chown -R $U $W/outputs/$D" > /dev/null 2>&1 &

D=$RUN/fastlio2
want fastlio2 && docker run --rm -v $REPO:$W slam-lab-noetic-fastlio bash -c "source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash; cd $W/outputs/$D
roscore > roscore.log 2>&1 & sleep 4
roslaunch fast_lio mapping_mid360.launch rviz:=false > fastlio_run.log 2>&1 &
sleep 5
rosbag record -O odom.bag /Odometry > record.log 2>&1 &
sleep 2
rosbag play $W/data/${TAG}_fastlio.bag > bag_play.log 2>&1
sleep 5; pkill -INT -f '[r]osbag'; sleep 3; pkill -INT -f '[f]astlio_mapping'; sleep 20
cp -r ~/catkin_ws/src/FAST_LIO/PCD $W/outputs/$D/ 2>/dev/null; chown -R $U $W/outputs/$D" > /dev/null 2>&1 &

D=$RUN/liosam
want liosam && docker run --rm -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID -e HOME=$W/outputs/$D -v $REPO:$W slam-lab-liosam bash -c "source /opt/ros/jazzy/setup.bash; source /root/colcon_ws/install/setup.bash; cd $W/outputs/$D
ros2 launch $W/slam_comparison/launch/liosam_lab.launch.py params_file:=$W/slam_comparison/config/liosam_livox_params.yaml > liosam_run.log 2>&1 &
sleep 10
python3 $W/slam_comparison/scripts/dump_odometry_tum.py /lio_sam/mapping/odometry $W/outputs/$D/liosam_trajectory_tum.txt > record.log 2>&1 &
sleep 3
ros2 bag play $ROS2BAG --topics /livox/points /livox/imu > bag_play.log 2>&1
sleep 5
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap '{resolution: 0.2, destination: \"/liosam_out/\"}' > save_map.log 2>&1
sleep 5; pkill -INT -f '[l]io_sam'; sleep 5; chown -R $U $W/outputs/$D" > /dev/null 2>&1 &

wait
echo "== exporting trajectories"
C=$OUT/$RUN/cartographer; R=$OUT/$RUN/rtabmap; F=$OUT/$RUN/fastlio2; L=$OUT/$RUN/liosam
$PY $SC/export_cartographer_pbstream_trajectory.py $REPO/data/cartographer_pbpy $C/$TAG.pbstream $C/trajectory_tum.txt || true
docker run --rm -v $REPO:$W slam-lab-jazzy bash -c "source /opt/ros/jazzy/setup.bash; cd $W/outputs/$RUN/rtabmap && rtabmap-export --poses --opt 0 --output_dir $W/outputs/$RUN/rtabmap/export rtabmap.db > /dev/null 2>&1; chown -R $U export" || true
grep -v "^#" $R/export/rtabmap_poses.txt | awk '{print $1,$2,$3,$4,$5,$6,$7,$8}' > $R/rtabmap_optimized_tum.txt || true
$PY $SC/export_ros1_odometry_tum.py $F/odom.bag /Odometry $F/fastlio_trajectory_tum.txt || true
python3 $SC/compare_to_reference.py $OUT/$RUN/reference/odometry_tum.txt \
  Cartographer=$C/trajectory_tum.txt RTAB-Map=$R/rtabmap_optimized_tum.txt \
  FAST-LIO2=$F/fastlio_trajectory_tum.txt LIO-SAM=$L/liosam_trajectory_tum.txt || true
