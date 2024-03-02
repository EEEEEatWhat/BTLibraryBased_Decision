from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    goalActionServerNode = Node(
        package='goal_action_server',
        executable='goalActionServer',
        output='screen'
    )    
    goalActionServerWithPosesNode = Node(
        package='goal_action_server',
        executable='goalActionServerWithPoses',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(goalActionServerNode)
    ld.add_action(goalActionServerWithPosesNode)

    return ld