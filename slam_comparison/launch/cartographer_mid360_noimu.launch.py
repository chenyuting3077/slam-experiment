"""Cartographer 3D live node for Mid360-family datasets, WITHOUT feeding it
any IMU data at all -- testing whether PoseExtrapolator's "no IMU data"
fallback (a constant, fake UnitZ() gravity observation, see
pose_extrapolator.cc::AdvanceImuTracker) avoids the gravity-tracking CHECK
failure/divergence root-caused in CARTOGRAPHER_FAILURE.md, since that
fallback never mixes in real (and possibly gravity-corrupting) accelerometer
readings. The 'imu' topic is deliberately left unremapped (and nothing is
played to it), so cartographer_node's IMU subscription just never receives
any message -- imu_data_ stays empty for the whole run.
"""

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
            # Deliberately no 'imu' remap -- see module docstring.
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
