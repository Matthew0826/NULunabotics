from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Runs obstacle detection using the real LiDAR sensor.
    Also includes the actual LIDAR launch file from ldlidar_stl_ros2 package.
    """
    
    ld19_launch_path = os.path.join(
        get_package_share_directory('ldlidar_stl_ros2'),
        'launch',
        'ld19.launch.py'
    )
    website_launch_path = os.path.join(
        get_package_share_directory('website'),
        'launch',
        'nodejs.launch.py'
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ld19_launch_path)
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(website_launch_path),
        # ),
        Node(
            package='navigation',
            executable='obstacle_detector',
            output='screen'
        ),
        Node(
            package='sensors',
            executable='lidar_forwarder',
            output='screen'
        )
    ])
