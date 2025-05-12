from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Runs the serial port manager and positioning nodes, as well as the website to visualize them."""
    
    website_launch_path = os.path.join(
        get_package_share_directory('website'),
        'launch',
        'nodejs.launch.py'
    )
    
    return LaunchDescription([
        Node(
            package='sensors',
            executable='serial_port_manager',
            output='screen'
        ),
        Node(
            package='sensors',
            executable='positioning',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(website_launch_path),
        )
    ])
