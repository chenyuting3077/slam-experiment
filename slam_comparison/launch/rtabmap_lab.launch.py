"""RTAB-Map on the Compal AMR lab bags: /odometry as odometry backbone, ICP loop closure on the merged Vanjee cloud."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    parameters = {
        'frame_id': 'base_link',
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_scan_cloud': True,
        'approx_sync': True,
        'use_sim_time': False,
        'qos_scan': 1,
        'qos_odom': 1,
        'topic_queue_size': 30,
        'sync_queue_size': 30,
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'Icp/VoxelSize': '0.1',
        'Icp/PointToPlane': 'true',
        'Icp/PointToPlaneK': '20',
        'Icp/PointToPlaneRadius': '0',
        'Icp/MaxCorrespondenceDistance': '1',
        'Icp/MaxTranslation': '3',
        'Icp/Iterations': '30',
        'Icp/Epsilon': '0.001',
        'Icp/PMOutlierRatio': '0.7',
        'Icp/CorrespondenceRatio': '0.01',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'Grid/CellSize': '0.1',
        'Grid/RangeMax': '60',
        'Grid/Sensor': '0',
        'Mem/IncrementalMemory': 'true',
        'database_path': LaunchConfiguration('database_path'),
    }
    return LaunchDescription([
        DeclareLaunchArgument('database_path'),
        DeclareLaunchArgument('scan_topic', default_value='/vanjee_points719e_merged'),
        Node(package='rtabmap_slam', executable='rtabmap', output='screen',
             parameters=[parameters],
             remappings=[('odom', '/odometry'), ('scan_cloud', LaunchConfiguration('scan_topic'))],
             arguments=['--delete_db_on_start']),
    ])
