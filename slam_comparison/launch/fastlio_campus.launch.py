"""FAST-LIO2 (spark-fast-lio) for the LIO-SAM Campus dataset."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = FindPackageShare('slam_comparison').find('slam_comparison') + '/config/fastlio_campus.yaml'

    static_tf_base_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_velodyne',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'velodyne'],
    )

    lio_node = Node(
        package='spark_fast_lio',
        executable='spark_lio_mapping',
        name='lio_mapping',
        output='screen',
        parameters=[
            config,
            {'common.save_dir': '/home/allen/slam-experiment/outputs/pointclouds/'},
            {'common.sequence_name': 'fastlio_campus'},
        ],
        remappings=[
            ('lidar', 'points_raw'),
            ('imu', 'imu_correct'),
        ],
    )

    return LaunchDescription([static_tf_base_to_velodyne, lio_node])
