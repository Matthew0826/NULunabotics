#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/selene/NULunabotics/Software/ROS2/install/setup.bash
ros2 run sensors serial_port_server &
#ros2 run physical_robot drive &
#ros2 run sensors positioning &
#ros2 run sensors lidar &
ros2 launch website example.launch.py &
ros2 run navigation pathfinder &
ros2 run navigation obstacle_detector &

wait
