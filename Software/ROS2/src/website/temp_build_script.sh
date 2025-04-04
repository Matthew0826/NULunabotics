cd ~/NULunabotics/Software/ROS2/
git pull
source install/setup.bash
cd src/website
npm install
npm run deploy
cd ~/NULunabotics/Software/ROS2/
rm -rf install/website/share/website/.next
colcon build --packages-select website
cp -r src/website/.next install/website/share/website/.next
ros2 launch website example.launch.py
