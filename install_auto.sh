#!/bin/bash

AUTO="-y"
# Update git submodules
git submodule update --init --recursive
# Fetch tags for the main repository
git fetch --tags

# Fetch tags for all submodules
git submodule foreach 'git fetch --tags'
if [ "$1" == "-n" ]; then
  AUTO=""
fi

## Bash script for setting up ROS2 Humble (with Gazebo Fortress) development environment for PX4 on Ubuntu Jammy LTS (22.04). 
## It installs the common dependencies for all targets (including Qt Creator)
##
## Installs:
## - ROS2 Humble (including Gazebo Fortress)
## - uXRCE-DDS Agent

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

sudo apt install ${AUTO} software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install ${AUTO} curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade ${AUTO}
sudo apt install ${AUTO} ros-humble-desktop
sudo apt install ${AUTO} ros-dev-tools
sudo apt-get install ${AUTO} libreadline-dev

# Packages
sudo apt-get install ${AUTO} ros-humble-control-toolbox
sudo apt-get install ${AUTO} nlohmann-json3-dev

# Configuring environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

# Install common packages and dependencies

sudo apt update
sudo apt install ${AUTO} ros-humble-turtlesim
sudo apt install ${AUTO} ~nros-humble-rqt*
sudo apt install ${AUTO} python3-colcon-common-extensions
sudo apt-get install ${AUTO} python3-rosdep
sudo rosdep init
rosdep update

pip install --user -U empy==3.3.4 pyros-genmsg setuptools

### GAZEBO INSTALL & PX4 Setup ###
sudo apt-get install ${AUTO} ros-humble-ros-gz

### uXRCE-DDS Agent Install ###
sudo snap install micro-xrce-dds-agent --edge


# python3.11
# python3-pip
# sudo apt install python3.11-venv python3.11-distutils -y
# curl -sS https://bootstrap.pypa.io/get-pip.py | /usr/bin/python3.11
# sudo apt update
# sudo apt install python3.11-venv -y
# venv
 