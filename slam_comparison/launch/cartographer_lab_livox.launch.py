"""Cartographer 3D offline node for the Livox part of a Compal AMR lab bag (converted by convert_lab_livox_ros2.py).

Same as spot_cartographer's offline_3d.launch.py (same URDF, same Lua apart from tracking_frame) but with the Livox
topics remapped in and without rviz / occupancy grid, so the launch ends when the offline node finishes.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf = FindPackageShare('compal_amr_description').find('compal_amr_description') + '/urdf_file/compalamr/urdf/compal_amr.urdf'
    return LaunchDescription([
        DeclareLaunchArgument('bag_filenames'),
        DeclareLaunchArgument('save_state_filename'),
        DeclareLaunchArgument('configuration_directory'),
        DeclareLaunchArgument('configuration_basenames', default_value='spot_offline_mapping_livox.lua'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        SetRemap('odometry', 'odom'),
        SetRemap('/livox/points', 'points2'),
        SetRemap(LaunchConfiguration('imu_topic'), 'imu'),
        Node(package='cartographer_ros', executable='cartographer_offline_node', output='screen',
             parameters=[{'use_sim_time': True}],
             arguments=['-configuration_directory', LaunchConfiguration('configuration_directory'),
                        '-configuration_basenames', LaunchConfiguration('configuration_basenames'),
                        '-urdf_filenames', urdf,
                        '-bag_filenames', LaunchConfiguration('bag_filenames'),
                        '-save_state_filename', LaunchConfiguration('save_state_filename')]),
    ])
