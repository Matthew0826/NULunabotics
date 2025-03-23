cd ~/ros2_stuff/src/NULunabotics/
git pull
cd Software/NextJS
npm install
npm run deploy
cd ~/ros2_stuff
colcon build --packages-select website
cp -r src/NULunabotics/Software/NextJS/.next install/website/share/website/.next
source install/setup.bash
ros2 launch website example.launch.py
