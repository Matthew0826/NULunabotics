colcon build
source install/setup.bash
ros2 launch navigation robot.py &
ros2 launch website nodejs.launch.py &
wait
