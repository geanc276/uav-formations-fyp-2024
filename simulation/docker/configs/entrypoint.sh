#!/bin/bash
set -e

# Setup environment
source $HOME/.bashrc
export ROS_DOMAIN_ID=0

# Start in the home directory
cd /home/ros/drones/uav-formations-fyp-2024/simulation

# Remove /etc/drone_config.conf if it exists
if [ -f /etc/drone_config.conf ]; then
    sudo rm /etc/drone_config.conf
fi
chmod +x ./docker/configs/setup_sim_docker.bash
sudo -E ./docker/configs/setup_sim_docker.bash
sudo chmod 660 /etc/drone_config.conf
sudo chmod +x ./sim_launch.bash
sudo ./sim_launch.bash

# Pass all arguments to the provided command
exec "$@"

# Keep the shell open
exec bash -i
