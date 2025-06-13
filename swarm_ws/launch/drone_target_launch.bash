#!/bin/bash

# Start a new tmux session named 'ros2_session'
tmux new-session -d -s ros2_session

# Create a new window for the Micro XRCE-DDS Agent
tmux rename-window -t ros2_session:0 'Agent'
tmux send-keys -t ros2_session:0 'micro-xrce-dds-agent udp4 -p 8888' C-m

# Create a new window for the ROS 2 launch
tmux new-window -t ros2_session -n 'ROS2 Launch'

# Launch ROS 2
tmux send-keys -t ros2_session:1 'source ../install/local_setup.bash && ros2 launch target.launch.py' C-m

# Attach to the tmux session
tmux attach -t ros2_session
