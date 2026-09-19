"""RTAB-Map pure-LiDAR ICP SLAM for the Zenodo Mid360 outdoor_hard_01 dataset."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    icp_odom_parameters = {
        'frame_id': 'livox_frame',
        'odom_frame_id': 'icp_odom',
        'wait_imu_to_init': True,
        'expected_update_rate': 15.0,
        'OdomF2M/ScanSubtractRadius': '0.3',
        'OdomF2M/ScanMaxSize': '10000',
        'Icp/VoxelSize': '0.3',
        'Icp/MaxCorrespondenceDistance': '3',
        'Icp/PointToPlane': 'true',
        'Icp/RangeMin': '0.3',
        'Icp/RangeMax': '60',
        'Icp/MaxTranslation': '2',
    }

    rtabmap_parameters = {
        'frame_id': 'livox_frame',
        'database_path': '/home/allen/slam-experiment/outputs/rtabmap_mid360.db',
        'subscribe_rgb': False,
        'subscribe_depth': False,
        'subscribe_scan_cloud': True,
        'Reg/Strategy': '1',
        'Mem/NotLinkedNodesKept': 'false',
        'Icp/VoxelSize': '0.3',
        'Icp/MaxCorrespondenceDistance': '3',
        'Icp/PointToPlane': 'true',
        'Icp/RangeMin': '0.3',
        'Icp/RangeMax': '60',
        'RGBD/OptimizeMaxError': '0.3',
        # Same search-radius fix as the Campus run -- start wide from the
        # start instead of re-discovering the same default-too-small bug.
        'RGBD/LocalRadius': '60',
        'RGBD/ProximityMaxGraphDepth': '0',
    }

    shared_parameters = {
        'use_sim_time': use_sim_time,
    }

    remappings = [
        ('scan_cloud', '/livox/points'),
        ('imu', '/livox/imu'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[icp_odom_parameters, shared_parameters],
            remappings=remappings,
        ),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_parameters, shared_parameters],
            remappings=remappings,
            arguments=['-d'],
        ),
    ])
