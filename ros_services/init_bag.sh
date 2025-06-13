#!/bin/bash

CURRENT_TIME=$(date +"y%Ym%md%d_H%H-M%M-S%S")
echo "Current Time: $CURRENT_TIME"
ID=$1
echo "Drone ID: $ID"

if [ -z "$ID" ]; then
    echo "Drone ID Unkown"
    DRONE_NAME="drone_UNKOWN"
else
    DRONE_NAME="drone_$ID"
fi
echo "Drone Name: $DRONE_NAME"

check_exist() {
    local dir="$1"
    if [ -d "$dir" ]; then
        echo "Directory exists"
        return 1
    else
        echo "Directory does not exist"
        return 0
    fi
}

move_bag() {
    local bag="$1"
    # Move the bag file to the bag_files directory
    BAG_DIR="unowned"
    if [ ! -d "$BAG_DIR" ]; then
        mkdir -p "$BAG_DIR"
    fi
    mv "$bag" "${BAG_DIR}/${DRONE_NAME}_${CURRENT_TIME}"
}

main() {
    # Check if the current bag file exists
    CURRENT_BAG="current"
    if check_exist "$CURRENT_BAG"; then
        echo "Bag file does not exist"
    else
        echo "Bag file exists"
        move_bag "$CURRENT_BAG"
    fi
}

main
