"""Cartographer 3D live node for LegKilo's corridor.bag, with real leg
odometry (/state_SDK) fed in via use_odometry=true -- see
legkilo_corridor_odom.lua and CARTOGRAPHER_FAILURE.md.

Static transforms approximate the rigid offsets from the dataset's own
config (legkilo_go1_velodyne.yaml: extrinsic_T=[0,0,0.20] lidar-in-imu-frame)
and an identity approximation between imu_link and the odometry message's
child_frame_id (sdk_base_link_3d), since no better calibration is published.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    static_tf_imu_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_link_to_velodyne',
        arguments=['0', '0', '0.20', '0', '0', '0', 'imu_link', 'velodyne'],
    )

    static_tf_imu_to_sdk_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_link_to_sdk_base_link_3d',
        arguments=['0', '0', '0', '0', '0', '0', 'imu_link', 'sdk_base_link_3d'],
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory',
            FindPackageShare('slam_comparison').find('slam_comparison') + '/config',
            '-configuration_basename', 'legkilo_corridor_odom.lua'],
        remappings=[
            ('points2', '/points_raw'),
            ('imu', '/imu_raw'),
            ('odom', '/state_SDK'),
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
        static_tf_imu_to_velodyne,
        static_tf_imu_to_sdk_base,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
