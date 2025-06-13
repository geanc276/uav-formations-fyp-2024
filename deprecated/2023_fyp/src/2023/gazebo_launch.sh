#!/bin/bash

# A script to launch the gazebo simulation from a python process.
# Sets the required environment variables and launches a simulation 
# within the required number of drones. 


cd ../..;
source /opt/ros/noetic/setup.bash; 
source ~/catkin_ws/devel/setup.bash; 
cd PX4-Autopilot; 
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default; 
roslaunch ~/fyp/uav-formations-fyp-2024/src/2023/launch/full_swarm_simulated.launch #gui:=false
