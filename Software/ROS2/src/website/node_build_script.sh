cd ~/NULunabotics/Software/ROS2/
git pull
source install/setup.bash
colcon build --packages-select lunabotics_interfaces
cd src/website
npm install
npx generate-ros-messages
npm run deploy
cd ~/NULunabotics/Software/ROS2/
rm -rf install/website/share/website/.next
colcon build --packages-select website
cp -r src/website/.next install/website/share/website/.next
