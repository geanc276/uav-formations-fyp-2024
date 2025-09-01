#!/usr/bin/env bash
set -e

BASHRC="/home/ros/.bashrc"

# 1) Nettoyage des anciennes lignes cassées (facultatif mais sûr)
sed -i '/source \/opt\/ros\/\/setup.bash/d' "$BASHRC" || true

# 2) Ajout d’un bloc correct, en fin de fichier
{
  echo ""
  echo "## ROS"
  echo 'export ROS_DISTRO=${ROS_DISTRO:-humble}'
  echo '[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ] && source "/opt/ros/${ROS_DISTRO}/setup.bash"'
  echo 'export ROS_DOMAIN_ID=0'
  echo '[ -f "/home/ros/drones/uav-formations-fyp-2024/swarm_ws/install/setup.bash" ] && source "/home/ros/drones/uav-formations-fyp-2024/swarm_ws/install/setup.bash"'
  echo ''
  echo 'stop_ros() { ps aux | grep _node | grep -v grep | awk '"'"'{print $2}'"'"' | xargs -r kill; }'
} >> "$BASHRC"
