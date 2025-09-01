#!/usr/bin/env bash
set -e

# Fallback si ROS_DISTRO n'est pas défini
export ROS_DISTRO="${ROS_DISTRO:-humble}"

# Source ROS 2 (silencieux si absent)
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
  source "/opt/ros/humble/setup.bash"
fi

# Source l’overlay du workspace si dispo (ne bloque pas)
if [ -f /home/ros/drones/uav-formations-fyp-2024/swarm_ws/install/setup.bash ]; then
  source /home/ros/drones/uav-formations-fyp-2024/swarm_ws/install/setup.bash || true
fi

exec "$@"
