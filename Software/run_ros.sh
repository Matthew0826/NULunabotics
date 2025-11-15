#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/selene/NULunabotics/Software/ROS2/install/setup.bash
cd /home/selene/NULunabotics/Software/ROS2
ros2 launch website nodejs.launch.py &
ros2 launch navigation robot.py &
wait
