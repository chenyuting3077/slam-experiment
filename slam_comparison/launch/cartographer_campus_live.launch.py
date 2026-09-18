"""
Cartographer 3D LIVE node (not cartographer_offline_node) for the LIO-SAM
Campus dataset. Switched from the offline node because its ROS2 port doesn't
honor -r remaps when matching its hardcoded expected topics ("points2",
"imu") against actual bag topic names (confirmed empirically: even with the
bag's topics renamed to match exactly, it still never delivered point cloud
data to the algorithm -- 0 submaps, 0 constraints). The live node uses a
normal rclcpp subscription + remapping, the same mechanism already proven to
work for RTAB-Map and FAST-LIO2, so it's driven by a real-time `ros2 bag
play` instead of reading the bag file directly.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    static_tf_base_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_velodyne',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'velodyne'],
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory',
            FindPackageShare('slam_comparison').find('slam_comparison') + '/config',
            '-configuration_basename', 'campus_3d.lua'],
        remappings=[
            ('points2', 'points_raw'),
            ('imu', 'imu_correct'),
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
        static_tf_base_to_velodyne,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
