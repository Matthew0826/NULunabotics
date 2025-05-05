from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Runs the entire robot! Including positioning, obstacle detection, and navigation."""
    
    ld19_launch_path = os.path.join(
        get_package_share_directory('ldlidar_stl_ros2'),
        'launch',
        'ld19.launch.py'
    )
    
    return LaunchDescription([
        # obstacle detection nodes
        Node(
            package='navigation',
            executable='obstacle_detector',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ld19_launch_path)
        ),
        Node(
            package='sensors',
            executable='lidar_forwarder',
            output='screen'
        ),
        
        # serial port manager
        Node(
            package='sensors',
            executable='serial_port_manager',
            output='screen'
        ),
        
        # navigation nodes
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
        
        # sensor board nodes
        Node(
            package='sensors',
            executable='positioning',
            output='screen'
        ),
        
        # driving nodes
        Node(
            package='navigation',
            executable='odometry',
            output='screen'
        ),
        Node(
            package='physical_robot',
            executable='motor_driver',
            output='screen'
        )
    ])
