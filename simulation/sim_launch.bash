# Remet le bon workspace
sed -i 's|^SWARM_PATH=.*|SWARM_PATH="/home/ros/drones/uav-formations-fyp-2024/swarm_ws"|' \
  /home/ros/drones/uav-formations-fyp-2024/simulation/sim_launch.bash

# Remplace tout chemin cassé "/home/Documents/..." par le bon "install/setup.bash"
sed -i 's|/home/Documents/uav-formations-fyp-2024/swarm_ws/drone_launch/setup.bash|/home/ros/drones/uav-formations-fyp-2024/swarm_ws/install/setup.bash|g' \
  /home/ros/drones/uav-formations-fyp-2024/simulation/sim_launch.bash

# S’assure qu’il source bien ROS Humble + l’overlay
grep -n 'opt/ros/humble' /home/ros/drones/uav-formations-fyp-2024/simulation/sim_launch.bash || \
  sed -i '1i source /opt/ros/humble/setup.bash' /home/ros/drones/uav-formations-fyp-2024/simulation/sim_launch.bash

chmod +x /home/ros/drones/uav-formations-fyp-2024/simulation/sim_launch.bash
