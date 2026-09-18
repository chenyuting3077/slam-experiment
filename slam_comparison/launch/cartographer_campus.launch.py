"""
Cartographer 3D offline node for the LIO-SAM Campus dataset.

The bag has no TF at all: /points_raw is in frame "velodyne", /imu_correct is
already expressed in frame "base_link" (LIO-SAM's imuConverter pre-rotates
raw IMU using extrinsicRot/RPY before recording -imu_correct, with
extrinsicTrans = [0,0,0]). So base_link and velodyne share an origin and we
only need an identity static transform between them.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    bag_filenames_arg = DeclareLaunchArgument('bag_filenames')
    no_rviz_arg = DeclareLaunchArgument('no_rviz', default_value='true')
    # offline_node.launch.py doesn't expose a save_state_filename passthrough,
    # so cartographer_offline_node falls back to its own default:
    # "<bag_filenames[0]>.pbstream" -- we move it into outputs/ afterward.
    configuration_directory_arg = DeclareLaunchArgument(
        'configuration_directory',
        default_value=FindPackageShare('slam_comparison').find('slam_comparison') + '/config')
    configuration_basenames_arg = DeclareLaunchArgument(
        'configuration_basenames', default_value='campus_3d.lua')

    static_tf_base_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_velodyne',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'velodyne'],
    )

    # offline_node reads bags directly (not via a live ROS graph subscription),
    # so the remap direction here is (topic-name-as-recorded-in-the-bag,
    # name-cartographer's-code-expects) -- the same direction the official
    # offline_backpack_3d.launch.py uses for its own sample bag.
    # cartographer_offline_node's ROS2 port doesn't honor -r remaps when it
    # matches its hardcoded expected topics ("points2", "imu") against the
    # bag's actual topic names -- confirmed empirically (empty pbstream, 0
    # submaps) even though the remap args were correctly on its argv. So the
    # bag passed in here must already use those literal topic names (see
    # scripts/rename_bag_topics_for_cartographer.py); no SetRemap needed.

    offline_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            FindPackageShare('cartographer_ros').find('cartographer_ros') + '/launch/offline_node.launch.py'),
        launch_arguments={
            'bag_filenames': LaunchConfiguration('bag_filenames'),
            'no_rviz': LaunchConfiguration('no_rviz'),
            'rviz_config': '',
            'configuration_directory': LaunchConfiguration('configuration_directory'),
            'configuration_basenames': LaunchConfiguration('configuration_basenames'),
            'urdf_filenames': '',
        }.items(),
    )

    return LaunchDescription([
        bag_filenames_arg,
        no_rviz_arg,
        configuration_directory_arg,
        configuration_basenames_arg,
        static_tf_base_to_velodyne,
        offline_node_launch,
    ])
