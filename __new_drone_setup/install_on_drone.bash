#!/bin/bash

## Bash script for setting up ROS2 Humble (with Gazebo Fortress) development environment for PX4 on Ubuntu Jammy LTS (22.04). 
## It installs the common dependencies for all targets (including Qt Creator)
##
## Installs:
## - ROS2 Humble (including Gazebo Fortress)
## - uXRCE-DDS Agent
cd ..
if [[ $(lsb_release -sc) != "jammy" ]]; then
  echo "OS version detected as $(lsb_release -sc) $(lsb_release -sr)." 
  echo "This ROS2 Humble install requires at least Ubuntu "jammy" 22.04."
  echo "Exiting ...."
  kill -INT $$
fi

### ROS2 INSTALL ### 

# Make sure you have a locale which supports UTF-8:
# locale

# Add the ROS 2 apt repository
# You will need to add the ROS 2 apt repository to your system.
# First ensure that the Ubuntu Universe repository is enabled.

sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe

# Now add the ROS 2 GPG key with apt.

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Then add the repository to your sources list.

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update your apt repository caches after setting up the repositories.
# ROS 2 packages are built on frequently updated Ubuntu systems. It is always recommended that you ensure your system is up to date before installing new packages.
# Desktop Install (Recommended): ROS, RViz, demos, tutorials.
# Development tools: Compilers and other tools to build ROS packages

sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop



# Packages
sudo apt-get install ros-humble-control-toolbox
sudo apt-get install nlohmann-json3-dev

# Configuring environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

# Install common packages and dependencies

sudo apt update
sudo apt install ~nros-humble-rqt*
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update

pip install --user -U empy==3.3.4 pyros-genmsg setuptools


### uXRCE-DDS Agent Install ###
sudo snap install micro-xrce-dds-agent --edge

#idk if this is needed yet
# cd ..
# git clone https://github.com/PX4/PX4-Autopilot.git --recursive
# bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

sudo apt install python3-colcon-common-extensions
sudo apt install libreadline-dev
sudo apt-get install unzip

#stops boot up hold
sudo sed -i '/^ExecStart=\/lib\/systemd\/systemd-networkd-wait-online/s|$| --timeout=10|' /etc/systemd/system/network-online.target.wants/systemd-networkd-wait-online.service
echo "export DRONE_ID=$(hostname | grep -o '[0-9]*')" >> ~/.bashrc
echo "alias drone_id='echo $DRONE_ID'" >> ~/.bashrc
source ~/.bashrc

sudo apt install network-manager
sudo systemctl enable NetworkManager
sudo systemctl start NetworkManager

sudo apt install iw
sudo apt install net-tools
sudo apt install rfkill

# Configure network settings
sudo cp /etc/netplan/01-px4-netplan.yaml /etc/netplan/01-px4-netplan.yaml.bak
sudo cp $(pwd)/__new_drone_setup/01-px4-netplan.yaml /etc/netplan/01-px4-netplan.yaml
sudo cp /etc/netplan/50-cloud-init.yaml /etc/netplan/50-cloud-init.yaml.bak
sudo cp $(pwd)/__new_drone_setup/50-cloud-init.yaml /etc/netplan/50-cloud-init.yaml
sudo netplan apply

sudo apt-get install python3-pip
python3 -m pip install --upgrade packaging




## WIFI SETUP
sudo nmcli device wifi connect TP-Link_B45E password 41139219
# read -p "Enter drone number: " CUSTOM_NUMBER
# # Calculate the new IP address dynamically to start at .10
# NEW_IP="192.168.0.$((CUSTOM_NUMBER + 9))/24"
# Use the calculated IP address in the nmcli command
# sudo nmcli connection modify "TP-Link_B45E" ipv4.addresses "$NEW_IP"
# sudo nmcli connection modify "TP-Link_B45E" ipv4.gateway "192.168.0.1"
sudo nmcli connection modify "TP-Link_B45E" ipv4.dns "8.8.8.8 8.8.4.4"
sudo nmcli connection modify "TP-Link_B45E" ipv4.method manual
sudo systemctl restart NetworkManager


#https://docs.px4.io/main/en/companion_computer/pixhawk_rpi.html
#https://docs.px4.io/main/en/advanced_config/ethernet_setup.html#px4-ethernet-network-setup
#UXRCE_DDS_AG_IP = 170461697
#https://raspberrypi.stackexchange.com/questions/94047/how-to-setup-an-unprotected-ad-hoc-ibss-network-and-if-possible-with-wpa-encry/94048#94048
# uxrce_dds_client start -p 8888 -h 10.41.10.1 -n px4_1