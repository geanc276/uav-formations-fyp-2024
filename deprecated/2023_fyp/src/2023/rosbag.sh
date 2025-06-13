#!/bin/bash

# Get the current working directory of the script
current_dir="$PWD"

# Define the bagfiles directory path
bagfiles_dir="$current_dir/bagfiles"

# Check if the bagfiles directory exists, and create it if it doesn't
if [ ! -d "$bagfiles_dir" ]; then
    mkdir "$bagfiles_dir"
fi

# Change directory to 'bagfiles'
cd "$bagfiles_dir"

# Start recording rosbag with all topics (-a)
rosbag record -a

# Run the python script afterwards
sleep 1
cd $current_dir
last_bagfile=$(ls -t "$bagfiles_dir" | head -n1)
echo 
echo Opening latest bagfile $last_bagfile
python3 analytics.py $bagfiles_dir/$last_bagfile uav0 uav1 uav2 uav3 uav4
