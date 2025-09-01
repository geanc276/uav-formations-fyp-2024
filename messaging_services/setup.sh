#!/bin/bash

# Check for -n flag
NO_INSTALL=false
PERMISSIONS_SET=false
while getopts ":np" opt; do
    case $opt in
        n)
            NO_INSTALL=true
            ;;
        p)
            PERMISSIONS_SET=true
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            exit 1
            ;;
    esac
done




if [ "$NO_INSTALL" = false ]; then
    # Update package list and install pip for Python 3
    echo "Updating package list and installing pip for Python 3..."
    sudo apt update
    sudo apt install -y python3-pip
    echo "Python 3 pip installed successfully."

    # Install Flask
    echo "Installing Flask..."
    sudo -H pip3 install flask
    echo "Flask installed successfully."

    # Install watchdog
    echo "Installing watchdog..."
    sudo -H pip3 install watchdog
    echo "Watchdog installed successfully."

    sudo apt-get install sshpass
    sudo apt install -y inotify-tools

    # Install required packages for mDNS
    echo "Installing required packages for mDNS..."
    sudo apt update
    sudo apt install -y avahi-daemon avahi-utils python3-dbus
    sudo apt-get install -y libzmq3-dev
    sudo -H pip3 install pyzmq
    sudo -H pip3 install netifaces
    sudo -H pip3 install inotify
    sudo -H pip3 install gunicorn


    echo "mDNS packages installed successfully."
fi

# Define the software folder path
SW_FOLDER="/home/swarm_software"

# Check if SW_FOLDER exists, if not, create it
if [ ! -d "$SW_FOLDER" ]; then
    echo "$SW_FOLDER does not exist. Creating it..."
    sudo mkdir -p "$SW_FOLDER"
    echo "$SW_FOLDER created successfully."
else
    echo "$SW_FOLDER already exists."
fi

if [ "$PERMISSIONS_SET" = true ]; then
    # Set permissions for the software folder
    sudo groupadd swarm_group
    echo "created group swarm_group"
    sudo usermod -aG swarm_group drone_swarm
    echo "added drone_swarm to swarm_group"
    sudo chown -R root:swarm_group /home/swarm_software/launch
    echo "changed ownership of /home/swarm_software/launch to root:swarm_group"
    # sudo chmod -R 775 /home/swarm_software
    sudo chmod 775 /home/swarm_software/launch
    sudo chmod g+s /home/swarm_software/launch

fi


# Create necessary directories
echo "Creating necessary directories in $SW_FOLDER..."
sudo mkdir -p "$SW_FOLDER/messaging_services"
sudo mkdir -p "$SW_FOLDER/executables"
sudo mkdir -p "$SW_FOLDER/meta_data"
sudo mkdir -p "$SW_FOLDER/logs"
echo "Directories created successfully."

# Copy mDNS files to the appropriate location
echo "Copying mDNS files to $SW_FOLDER/messaging_services..."
sudo cp -r . "$SW_FOLDER/messaging_services"
echo "mDNS files copied successfully."

# Enable and start the Avahi Daemon service
echo "Enabling and starting the Avahi Daemon service..."
sudo systemctl enable avahi-daemon
sudo systemctl start avahi-daemon
echo "Avahi Daemon service started successfully."

# Move service files to systemd directory
echo "Moving service files to /etc/systemd/system/..."
sudo mv "$SW_FOLDER/messaging_services/api/api_server.service" /etc/systemd/system/api_server.service
sudo mv "$SW_FOLDER/messaging_services/advertise/mdns_advertise.service" /etc/systemd/system/mdns_advertise.service
sudo mv "$SW_FOLDER/messaging_services/status/status_pub.service" /etc/systemd/system/status_pub.service
sudo mv "$SW_FOLDER/messaging_services/mission/mission_status.service" /etc/systemd/system/mission_status.service
echo "Service files moved successfully."

# Reload systemd to recognize new services
echo "Reloading systemd to recognize new services..."
sudo systemctl daemon-reload
echo "Systemd reloaded successfully."

# Enable, start, and check the status of the mDNS advertise service
echo "Enabling, starting, and checking the status of mDNS advertise service..."
sudo systemctl enable mdns_advertise.service
sudo systemctl start mdns_advertise.service

# Enable, start, and check the status of the API server service
echo "Enabling, starting, and checking the status of API server service..."
sudo systemctl enable api_server.service
sudo systemctl start api_server.service


# Enable, start, and check the status of the status publisher service
echo "Enabling, starting, and checking the status of status publisher service..."
sudo systemctl enable status_pub.service
sudo systemctl start status_pub.service


# Enable, start, and check the status of the mission status service
echo "Enabling, starting, and checking the status of mission status service..."
sudo systemctl enable mission_status.service
sudo systemctl start mission_status.service


sudo systemctl daemon-reload