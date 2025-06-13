#!/bin/bash

# Default value for DRONE_NAME
DRONE_NAME="uav"

# Function to display usage
usage() {
    echo "Usage: $0 [-d DRONE_NAME] [-t TIMEOUT]"
    echo "  -d DRONE_NAME : Set the name of the drone (default: uav)"
    echo "  -t TIMEOUT    : Set the timeout duration in seconds (default: 33)"
    exit 1
}

# Parse command-line arguments
while getopts ":d:t:" opt; do
  case $opt in
    d) DRONE_NAME="$OPTARG" ;;
    t) TIMEOUT="$OPTARG" ;;
    *) usage ;;
  esac
done


# Default value for TIMEOUT if not specified
TIMEOUT=${TIMEOUT:-33}

# Start the ROS2 launch file in the background in a new process group
sudo bash -c "source install/setup.sh; export DRONE_NAME='${DRONE_NAME}'; ros2 launch dcp launchDataDisseminationProtocol.xml" &
LAUNCH_PID=$!

# Get the process group ID of the launched process
LAUNCH_PGID=$(ps -o pgid= -p $LAUNCH_PID | grep -o '[0-9]*')

# Start a timer in the background to kill the ROS2 process group after the specified duration
(
    sleep $TIMEOUT
    echo "Sending SIGINT (Ctrl + C) to ROS2 launch process group after $TIMEOUT seconds"
    sudo kill -SIGINT -$LAUNCH_PGID  # Send SIGINT to the entire process group
    sudo killall "vardis_client_node"
    sudo killall "beaconing_client_node"
)

# Wait for the ROS2 launch process to finish
wait $LAUNCH_PID
echo "ROS2 nodes terminated."