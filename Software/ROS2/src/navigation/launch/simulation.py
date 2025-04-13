from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation',
            executable='mock_odometry',
            output='screen'
        ),
        Node(
            package='navigation',
            executable='pathfinder',
            output='screen'
        ),
        Node(
            package='navigation',
            executable='planner',
            output='screen'
        ),
    ])