# Adding all the necessary ros sourcing
echo "" >> ~/.bashrc
echo "## ROS" >> ~/.bashrc
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "source /home/ros/drones/uav-formations-fyp-2024/swarm_ws/install/setup.bash" >> ~/.bashrc

echo 'stop_ros() {
  ps aux | grep _node | grep -v grep | awk '\''{print $2}'\'' | xargs -r sudo kill
}' >> ~/.bashrc
