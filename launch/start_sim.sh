#!/bin/bash
export DISPLAY=:0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=happy_mini
source /usr/share/gazebo/setup.sh
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash

echo "Starting TurtleBot3 Gazebo simulation with CycloneDDS..."
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py x_pose:=-0.05 y_pose:=-1.8