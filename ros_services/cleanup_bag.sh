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


check_and_make_dir() {
    local dir=$1
    if [ ! -d $dir ]; then
        mkdir -p $dir
    fi
}

check_exist() {
    local file=$1
    if [ -f $file ]; then
        echo "File exists"
        return 1
    else
        echo "File does not exist"
        return 0
    fi
}


get_files_in_dir() {
    local dir=$1
    local files=$(ls $dir)
    return $files
}

move_bag() {
    local bag=$1
    local BAG_DIR=$2
    # Move the bag file to the bag_files directory
    RECENT_DIR="recent"
    BAG_DIR="bag_files"

    if [ ! -d $RECENT_DIR ]; then
        mkdir -p $RECENT_DIR
    fi

    if [ ! -d $BAG_DIR ]; then
        mkdir -p $BAG_DIR 
    fi


    for file in $(ls $RECENT_DIR); do
        mv $RECENT_DIR/$file $BAG_DIR
    done    

    mv $bag "$RECENT_DIR/${DRONE_NAME}_${CURRENT_TIME}"

    # mv $bag "${BAG_DIR}/${DRONE_NAME}_${CURRENT_TIME}"
}


main() {
    # Check if the directory exists
    SAVE_DIR="bag_files"
    check_and_make_dir $SAVE_DIR
    BAG_NAME="current"
    check_exist $BAG_NAME
    move_bag $BAG_NAME $SAVE_DIR
}


main


