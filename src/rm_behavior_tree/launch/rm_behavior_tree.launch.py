import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetUseSimTime

def generate_launch_description():
    package_path = get_package_share_directory('rm_behavior_tree')
    default_config_path = os.path.join(package_path, 'config', 'decision.yaml')
    config_path = LaunchConfiguration('config_path')

    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )

    rm_behavior_tree_node = Node(
        package='rm_behavior_tree',
        executable='rm_behavior_tree',
        parameters=[config_path,
                    # {'use_sim_time': use_sim_time}
                    ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_config_path_cmd)
    ld.add_action(rm_behavior_tree_node)

    return ld
