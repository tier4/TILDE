from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument,LogInfo
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    path_list_file = 'path_list.yaml'
    path_list = PathJoinSubstitution([FindPackagePrefix('tilde_aggregator'), path_list_file])
    path_list_arg = DeclareLaunchArgument("path_list",
                                           default_value='',
                                           description='target path list')
    mode_arg = DeclareLaunchArgument("mode",
                                      default_value='normal',
                                      description='use path list file')
    tilde_aggregator_node = Node(package='tilde_aggregator',
                                 executable='tilde_aggregator_node',
                                 output='screen',
                                 parameters = [{
                                 "mode": LaunchConfiguration('mode'),
                                 "path_list": LaunchConfiguration('path_list'),
                                 }]
                                )

    return LaunchDescription([
        mode_arg,
        path_list_arg,
        tilde_aggregator_node
    ])

