import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launches the website using Node.js."""
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    share_directory = get_package_share_directory('website')

	# revise path to your nodejs start file
    start_js_file = os.path.join(
        share_directory,
        '.next/standalone',
        'server.js')

    start_javascript_node = Node(
        name='test_node',
        executable='/home/selene/.nvm/versions/node/v22.14.0/bin/node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            start_js_file
        ],
        cwd=share_directory)

    ld = LaunchDescription()
    ld.add_action(start_javascript_node)

    return ld
