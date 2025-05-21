#!/bin/bash

# Get the directory of the script itself
ROS2_ROOT="$(cd "$(dirname "$0")" && pwd)"

# === Step 1: Run ROS2 Build ===
if [ -d "$ROS2_ROOT" ]; then
    echo "Building ROS2 navigation..."
    cd "$ROS2_ROOT" || exit 1
    source /opt/ros/jazzy/setup.bash
    colcon build
    source install/setup.bash
else
    echo "ERROR: ROS2 folder not found at $ROS2_DIR"
fi

# === Step 2: Run Website Dev Server ===
WEB_DIR="$ROS2_ROOT/src/website"
if [ -d "$WEB_DIR" ]; then
    echo "Running website dev server..."
    cd "$WEB_DIR" || exit 1
    npm install
    npx generate-ros-messages
    npm run dev &
else
    echo "ERROR: Website folder not found at $WEB_DIR"
fi

# Step 3: Profit
if [ -d "$ROS2_ROOT" ]; then
    echo "Building ROS2 navigation..."
    cd "$ROS2_ROOT" || exit 1
    ros2 launch navigation realistic_simulation.py
else
    echo "ERROR: ROS2 folder not found at $ROS2_DIR"
fi


