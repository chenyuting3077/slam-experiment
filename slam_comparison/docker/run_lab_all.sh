#!/bin/bash
# Run Cartographer / RTAB-Map / official FAST_LIO / official LIO-SAM in parallel on a Compal AMR lab bag.
#
# usage: run_lab_all.sh SLICED_BAG_DIR TAG
#   SLICED_BAG_DIR  rosbag2 dir (mcap) with /vanjee_points719e_merged /imu/data /odometry /tf_static
#                   (make one with scripts/slice_bag.py)
#   TAG             names the converted inputs (data/<TAG>_*) and, unless OUTNAME is set, the output folder
#                   outputs/compal_amr/<TAG>/<system>/ (set OUTNAME=dataset3_vanjee_full to use another folder name)
#
# Needs the docker images slam-lab-jazzy, slam-lab-liosam, slam-lab-noetic-fastlio
# (restore after a wipe with: docker load -i data/docker_images/<image>.tar),
# plus host ROS jazzy and data/venv, data/cartographer_pbpy.
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
[ -d $REPO/data/${TAG}_liosam ] || python3 $SC/convert_lab_bag_for_liosam.py $BAG $REPO/data/${TAG}_liosam
[ -f $REPO/data/${TAG}_fastlio.bag ] || $PY $SC/convert_lab_bag_for_fastlio.py $(ls $BAG/*.mcap | head -1) $REPO/data/${TAG}_fastlio.bag
python3 $SC/export_bag_odometry_tum.py $BAG /odometry $OUT/$RUN/reference/odometry_tum.txt

W=/workspace
echo "== starting the four systems in parallel"
D=$RUN/cartographer
docker run --rm -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID -v $REPO:$W slam-lab-jazzy bash -c "source /opt/ros/jazzy/setup.bash; source /root/colcon_ws/install/setup.bash; cd $W/outputs/$D
ros2 launch spot_cartographer offline_3d.launch.py bag_filenames:=${BAG/$REPO/$W} save_state_filename:=$W/outputs/$D/$TAG.pbstream no_rviz:=true > offline_run.log 2>&1 &
PID=\$!
until grep -q 'cartographer_offline_node.*process has finished' offline_run.log; do sleep 5; done
sleep 3; kill -INT \$PID; sleep 3; chown -R $U $W/outputs/$D" > /dev/null 2>&1 &

D=$RUN/rtabmap
docker run --rm -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID -v $REPO:$W slam-lab-jazzy bash -c "source /opt/ros/jazzy/setup.bash; cd $W/outputs/$D
ros2 launch $W/slam_comparison/launch/rtabmap_lab.launch.py database_path:=$W/outputs/$D/rtabmap.db > rtabmap.log 2>&1 &
sleep 8
ros2 bag play ${BAG/$REPO/$W} --topics /tf_static /odometry /vanjee_points719e_merged > bag_play.log 2>&1
sleep 10; pkill -INT -f '[r]tabmap'; sleep 8; chown -R $U $W/outputs/$D" > /dev/null 2>&1 &

D=$RUN/fastlio2
docker run --rm -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID -v $REPO:$W -v $REPO/fastlio_official_docker/config:/lab_config slam-lab-noetic-fastlio bash -c "source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash; cd $W/outputs/$D
roscore > roscore.log 2>&1 & sleep 4
roslaunch $W/fastlio_official_docker/compal_lab_velodyne.launch > fastlio_run.log 2>&1 &
sleep 5
rosbag record -O odom.bag /Odometry > record.log 2>&1 &
sleep 2
rosbag play $W/data/${TAG}_fastlio.bag > bag_play.log 2>&1
sleep 5; pkill -INT -f '[r]osbag'; sleep 3; pkill -INT -f '[f]astlio_mapping'; sleep 20
cp -r ~/catkin_ws/src/FAST_LIO/PCD $W/outputs/$D/ 2>/dev/null; chown -R $U $W/outputs/$D" > /dev/null 2>&1 &

D=$RUN/liosam
docker run --rm -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID -e HOME=$W/outputs/$D -v $REPO:$W slam-lab-liosam bash -c "source /opt/ros/jazzy/setup.bash; source /root/colcon_ws/install/setup.bash; cd $W/outputs/$D
ros2 launch $W/slam_comparison/launch/liosam_lab.launch.py params_file:=$W/slam_comparison/config/liosam_lab_params.yaml > liosam_run.log 2>&1 &
sleep 10
ros2 bag play $W/data/${TAG}_liosam > bag_play.log 2>&1
sleep 5
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap '{resolution: 0.2, destination: \"/liosam_out/\"}' > save_map.log 2>&1
sleep 5; pkill -INT -f '[l]io_sam'; sleep 5; chown -R $U $W/outputs/$D" > /dev/null 2>&1 &

wait
echo "== exporting trajectories"
C=$OUT/$RUN/cartographer; R=$OUT/$RUN/rtabmap; F=$OUT/$RUN/fastlio2; L=$OUT/$RUN/liosam
$PY $SC/export_cartographer_pbstream_trajectory.py $REPO/data/cartographer_pbpy $C/$TAG.pbstream $C/trajectory_tum.txt
python3 $SC/export_rtabmap_trajectory.py $R/rtabmap.db $R/rtabmap_trajectory_tum.txt
$PY $SC/export_ros1_odometry_tum.py $F/odom.bag /Odometry $F/fastlio_trajectory_tum.txt
python3 $SC/export_liosam_trajectory.py $L/liosam_out/transformations.pcd $L/liosam_trajectory_tum.txt
python3 $SC/compare_to_reference.py $OUT/$RUN/reference/odometry_tum.txt \
  Cartographer=$C/trajectory_tum.txt RTAB-Map=$R/rtabmap_trajectory_tum.txt \
  FAST-LIO2=$F/fastlio_trajectory_tum.txt LIO-SAM=$L/liosam_trajectory_tum.txt
