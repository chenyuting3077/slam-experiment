"""cartographer_assets_writer for the Campus dataset -- exports the pbstream
pose graph + renamed bag's point clouds into a single PCD point cloud."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # cartographer_assets_writer reads the bag directly and does a single,
    # non-retrying lookupTransform -- an external static_transform_publisher
    # node never reliably wins that race regardless of startup delay. It
    # loads -urdf_filename itself and builds its own internal tf tree from
    # it, so base_link->velodyne needs to come from there instead.
    assets_writer_node = Node(
        package='cartographer_ros',
        executable='cartographer_assets_writer',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory',
            FindPackageShare('slam_comparison').find('slam_comparison') + '/config',
            '-configuration_basename', 'assets_writer_campus.lua',
            '-urdf_filename',
            FindPackageShare('slam_comparison').find('slam_comparison') + '/config/campus.urdf',
            '-bag_filenames', LaunchConfiguration('bag_filenames'),
            '-pose_graph_filename', LaunchConfiguration('pose_graph_filename')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('bag_filenames'),
        DeclareLaunchArgument('pose_graph_filename'),
        assets_writer_node,
    ])
