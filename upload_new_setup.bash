#!/bin/bash

# Read the drone number from the user
echo -n "Enter drone number: "
read CUSTOM_NUMBER
# Calculate the new IP address dynamically to start at .10
# NEW_IP="192.168.0.$((CUSTOM_NUMBER + 9))"

# Define the remote path
REMOTE_PATH="drone_swarm@$NEW_IP:~/swarm/__new_drone_setup/"

# Create the remote directory if it does not exist
ssh drone_swarm@$NEW_IP 'mkdir -p ~/swarm/__new_drone_setup/'

# Copy the necessary files using scp
scp -r ./__new_drone_setup/install_on_drone.bash $REMOTE_PATH
scp -r ./__new_drone_setup/01-px4-netplan.yaml $REMOTE_PATH
scp -r ./__new_drone_setup/50-cloud-init.yaml $REMOTE_PATH