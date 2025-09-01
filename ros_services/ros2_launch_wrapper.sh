#!/bin/bash

set -e

# Timestamp
CURRENT_DATE=$(date '+%Y-%m-%d_%H-%M-%S')

# Directories
LOG_DIR="/home/swarm_software/logs/ros"
CURRENT_LOG_DIR="$LOG_DIR/current"
LAST_FIVE_LOGS_DIR="$LOG_DIR/last_five"
OLD_LOGS_DIR="$LOG_DIR/old"

create_dirs() {
    mkdir -p "$CURRENT_LOG_DIR" "$LAST_FIVE_LOGS_DIR" "$OLD_LOGS_DIR"
}

move_oldest_log() {
    oldest=$(find "$LAST_FIVE_LOGS_DIR" -maxdepth 1 -type d -name 'ros2_launch_*' | sort | head -n1)
    [ -n "$oldest" ] && mv -n "$oldest" "$OLD_LOGS_DIR/"
    filename=$(basename "$oldest")

    if [ ! -f "$oldest" ]; then
        return
    fi

    if [ -e "$OLD_LOGS_DIR/$filename" ]; then
        mv -n "$oldest" "$OLD_LOGS_DIR/${filename}_ERROR_DUPLICATE"
    else
        mv -n "$oldest" "$OLD_LOGS_DIR/"
    fi

}

rotate_logs() {
    count=$(find "$LAST_FIVE_LOGS_DIR" -maxdepth 1 -type d -name 'ros2_launch_*' | wc -l)
    if [ "$count" -gt 4 ]; then
        move_oldest_log
    fi
}

move_current_logs() {
    [ "$(ls -A "$CURRENT_LOG_DIR" 2>/dev/null)" ] || return 0
    target="$LAST_FIVE_LOGS_DIR/ros2_launch_${CURRENT_DATE}"
    if [ -e "$target" ]; then
        target="$LAST_FIVE_LOGS_DIR/ros2_launch_${CURRENT_DATE}_ERROR_DUPLICATE"
    fi 
    mkdir -p "$target"
    mv "$CURRENT_LOG_DIR"/* "$target/"
}

main() {
    echo "Starting ROS2 launch wrapper"
    create_dirs
    echo "Directories created"
    rotate_logs
    echo "Logs rotated"
    move_current_logs
    echo "Current logs moved"
    STDOUT_LOG="$CURRENT_LOG_DIR/ros2_launch_${CURRENT_DATE}.log"
    STDERR_LOG="$CURRENT_LOG_DIR/ros2_launch_error_${CURRENT_DATE}.log"
    exec /opt/ros/humble/bin/ros2 launch /home/swarm_software/launch/drone_launch/drone.launch.py > "$STDOUT_LOG" 2> "$STDERR_LOG"
}

source /home/swarm_software/launch/drone_launch/install/setup.bash


main
