"""
RTAB-Map pure-LiDAR ICP SLAM for LegKilo's corridor.bag (VLP-16 + raw IMU,
no camera). Same pattern as rtabmap_campus.launch.py, just pointed at
/imu_raw instead of /imu_correct (LegKilo has no pre-rotated IMU topic).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    icp_odom_parameters = {
        'frame_id': 'base_link',
        'odom_frame_id': 'icp_odom',
        'wait_imu_to_init': True,
        'expected_update_rate': 15.0,
        'OdomF2M/ScanSubtractRadius': '0.3',
        'OdomF2M/ScanMaxSize': '10000',
        'Icp/VoxelSize': '0.3',
        'Icp/MaxCorrespondenceDistance': '3',
        'Icp/PointToPlane': 'true',
        'Icp/RangeMin': '0.5',
        'Icp/RangeMax': '100',
        'Icp/MaxTranslation': '2',
    }

    rtabmap_parameters = {
        'frame_id': 'base_link',
        'database_path': '/home/allen/slam-experiment/outputs/rtabmap_legkilo_corridor.db',
        'subscribe_rgb': False,
        'subscribe_depth': False,
        'subscribe_scan_cloud': True,
        'Reg/Strategy': '1',
        'Mem/NotLinkedNodesKept': 'false',
        'Icp/VoxelSize': '0.3',
        'Icp/MaxCorrespondenceDistance': '3',
        'Icp/PointToPlane': 'true',
        'Icp/RangeMin': '0.5',
        'Icp/RangeMax': '100',
        'RGBD/OptimizeMaxError': '0.3',
        'RGBD/LocalRadius': '60',
        'RGBD/ProximityMaxGraphDepth': '0',
    }

    shared_parameters = {
        'use_sim_time': use_sim_time,
    }

    remappings = [
        ('scan_cloud', 'points_raw'),
        ('imu', 'imu_raw'),
    ]

    static_tf_base_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_velodyne',
        arguments=['0', '0', '0.20', '0', '0', '0', 'base_link', 'velodyne'],
    )

    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        static_tf_base_to_velodyne,
        static_tf_base_to_imu,

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
