#!/bin/bash

# Define log file paths
AGENT_LOG=/home/swarm_software/logs/manual/micro_xrce_dds_agent.log
ROS2_LOG=/home/swarm_software/logs/manual/ros2_launch.log
LOGGING_CONFIG=/home/swarm_software/logs/logging_config.yaml

# Start a new tmux session named 'ros2_session'
tmux new-session -d -s ros2_session

# Create a new window for the Micro XRCE-DDS Agent
tmux rename-window -t ros2_session:0 'Agent'
# Redirect Agent output to a log file
tmux send-keys -t ros2_session:0 "micro-xrce-dds-agent udp4 -p 8888" C-m

# Create a new window for the ROS 2 launch
tmux new-window -t ros2_session -n 'ROS2 Launch'

# Set environment variables for logging and launch ROS 2
# Set environment variables for logging and launch ROS 2
tmux send-keys -t ros2_session:1 'export DRONE_ID=${DRONE_ID}' C-m
tmux send-keys -t ros2_session:1 'export DRONE_CONFIG_FOLDER_PATH=${DRONE_CONFIG_FOLDER_PATH}' C-m

# Set ROS 2 logging to use the YAML configuration file
tmux send-keys -t ros2_session:1 "export RCUTILS_LOGGING_CONFIGURATION_FILE=${LOGGING_CONFIG}" C-m

# Launch ROS 2 with logging configuration
tmux send-keys -t ros2_session:1 'source ./install/local_setup.bash && ros2 launch drone.launch.py' C-m

# Set ROS 2 logging to use the YAML configuration file
tmux send-keys -t ros2_session:1 "export RCUTILS_LOGGING_CONFIGURATION_FILE=${LOGGING_CONFIG}" C-m

# Launch ROS 2 with logging configuration
tmux send-keys -t ros2_session:1 'source ./install/local_setup.bash && ros2 launch drone.launch.py' C-m

# Attach to the tmux session
tmux attach -t ros2_session
