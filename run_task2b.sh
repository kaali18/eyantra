#!/bin/bash

echo "=========================================="
echo "Task 2B - Pick and Place"
echo "=========================================="
echo ""
echo "This will run the manipulation task."
echo "Make sure:"
echo "1. Gazebo simulation is running"
echo "2. Detection node is running in another terminal"
echo ""
echo "Starting in 3 seconds..."
sleep 1
echo "2..."
sleep 1  
echo "1..."
sleep 1

cd /home/kasinath/colcon_ws
source install/setup.bash

echo ""
echo "Running manipulation node..."
echo "=========================================="
ros2 run ur5_control task2b_manipulation.py
