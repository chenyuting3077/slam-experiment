"""Cartographer 3D live node for the Zenodo Mid360 outdoor_hard_01 dataset."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory',
            FindPackageShare('slam_comparison').find('slam_comparison') + '/config',
            '-configuration_basename', 'mid360_3d.lua'],
        remappings=[
            ('points2', '/livox/points'),
            ('imu', '/livox/imu'),
        ],
        output='screen',
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        parameters=[{'use_sim_time': use_sim_time}, {'resolution': 0.05}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
