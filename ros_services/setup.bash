#!/bin/bash

DRONE_ID=$(hostname | grep -o '[0-9]*')
CONFIG_FOLDER_PATH=/home/swarm_software/config
STATE_FOLDER_PATH=/home/swarm_software/state

sudo touch /etc/drone_config.conf
sudo echo "DRONE_ID=$DRONE_ID" | sudo tee /etc/drone_config.conf
sudo echo "DRONE_CONFIG_FOLDER_PATH=$CONFIG_FOLDER_PATH" | sudo tee -a /etc/drone_config.conf
sudo echo "DRONE_STATE_FOLDER_PATH=$STATE_FOLDER_PATH" | sudo tee -a /etc/drone_config.conf
sudo echo "ROS_DOMAIN_ID=0" | sudo tee -a /etc/drone_config.conf

sudo chown root:drone_swarm /etc/drone_config.conf
sudo chmod 660 /etc/drone_config.conf

