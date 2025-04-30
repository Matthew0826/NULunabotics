from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Simulates the navigation system using a mock_odometry which pretends
    to move the robot as well as a mock_obstacle_detector which pretends to find obstacles."""
    return LaunchDescription([
        Node(
            package='navigation',
            executable='odometry',
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
        # Node(
        #     package='navigation',
        #     executable='mock_obstacle_detector',
        #     output='screen'
        # ),
        Node(
            package='sensors',
            executable='mock_positioning',
            output='screen'
        )
    ])
