#!/bin/bash
cd ~/NULunabotics/Software/ROS2/
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run sensors serial_port_server &
#ros2 run physical_robot drive &
#ros2 run sensors positioning &
#ros2 run sensors lidar &
ros2 run navigation pathfinding &
ros2 run navigation obstacle_detector &
ros2 launch website example.launch.py &
