-- runs all needed ros services to launch the droen
# ROS Services Overview

This directory contains all the systemd-managed ROS services and helper scripts required to launch and manage the drone’s ROS 2 stack and related components. Each service runs in the background to ensure continuous operation of critical processes.

## Contents

- **setup.bash**  
  Bash script to initialize `/etc/drone_config.conf` with drone ID and folder paths.
- **setup_services.py**  
  Python utility to install and verify all `.service` files, move wrapper scripts, and prepare bag recording scripts.
- **micro_xrce_dds_agent.service**  
  Launches the Micro XRCE-DDS Agent for uORB message transport.
- **ros_bag_movement.service**  
  Starts and stops ROS 2 bag recording of movement topics.
- **ros2_launch.service**  
  Main ROS 2 launch wrapper service for core drone operations.
- **target_launch.service**  
  Secondary ROS 2 launch wrapper service for target-specific operations.
- **ros2_launch_wrapper.sh** & **target_launch_wrapper.sh**  
  Wrapper scripts invoked by the corresponding launch services.
- **init_bag.sh** & **cleanup_bag.sh**  
  Helper scripts for initializing and cleaning up bag recording directories.

---

## 1. setup.bash

**Location:** `ros_services/setup.bash`  
**Purpose:**  
- Creates or overwrites `/etc/drone_config.conf` with:
  ```ini
  DRONE_ID=<last segment of hostname>
  DRONE_CONFIG_FOLDER_PATH=/home/swarm_software/config
  DRONE_STATE_FOLDER_PATH=/home/swarm_software/state
  ROS_DOMAIN_ID=0
  ```
- Sets ownership to `root:drone_swarm` and permissions to `660`.

**Usage:**  
```bash
sudo bash setup.bash
```

---

## 2. setup_services.py

**Location:** `ros_services/setup_services.py`  
**Purpose:**  
Automates the installation of all service units and supporting files:
1. Generates `/etc/drone_config.conf` (same content as `setup.bash`).
2. Copies each `*.service` file into `/etc/systemd/system/`, backing up any existing unit.
3. Moves `ros2_launch_wrapper.sh` and `target_launch_wrapper.sh` into `/home/swarm_software/launch/`.
4. Copies `init_bag.sh` and `cleanup_bag.sh` into `/home/swarm_software/logs/bag/`.
5. Reloads the systemd daemon and checks service statuses.

**Key Commands:**
```bash
sudo python3 setup_services.py
```

---

## 3. micro_xrce_dds_agent.service

**Location:** `/etc/systemd/system/micro_xrce_dds_agent.service`  
**Description:**  
- Starts the Micro XRCE-DDS Agent over UDP on port 8888 for uORB message transport.  
- Logs standard output and error to `/home/swarm_software/logs/uorb/`.

```ini
[Unit]
Description=Micro XRCE-DDS Agent Service
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=/home/swarm_software/launch
ExecStart=/snap/bin/micro-xrce-dds-agent udp4 -p 8888
Restart=on-failure
StandardOutput=append:/home/swarm_software/logs/uorb/micro_xrce_dds_agent.log
StandardError=append:/home/swarm_software/logs/uorb/micro_xrce_dds_agent_error.log

[Install]
WantedBy=multi-user.target
```

---

## 4. ros_bag_movement.service

**Location:** `/etc/systemd/system/ros_bag_movement.service`  
**Description:**  
- Records ROS 2 topics related to movement into a bag file using `ros2 bag record`.  
- Invokes `init_bag.sh` with `DRONE_ID` on start, and `cleanup_bag.sh` on stop.

```ini
[Unit]
Description=Service to start ros bag for movements
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=/home/swarm_software/logs/bag
EnvironmentFile=/etc/drone_config.conf
ExecStart=/bin/bash -c "/home/swarm_software/logs/bag/init_bag.sh $DRONE_ID && \
  source /opt/ros/humble/setup.bash && \
  ros2 bag record -o current"
ExecStop=/bin/bash -c "/home/swarm_software/logs/bag/cleanup_bag.sh $DRONE_ID"
Restart=on-failure
RestartSec=5
StartLimitInterval=60
StartLimitBurst=5

[Install]
WantedBy=multi-user.target
```

---

## 5. ros2_launch.service & target_launch.service

**Locations:**  
- `/etc/systemd/system/ros2_launch.service`  
- `/etc/systemd/system/target_launch.service`  

**Description:**  
Both services invoke their respective wrapper scripts to launch ROS 2 launch files for core or target-specific operations.

```ini
[Unit]
Description=ROS2 Launch Service
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=/home/swarm_software/launch
ExecStart=/home/swarm_software/launch/ros2_launch_wrapper.sh
EnvironmentFile=/etc/drone_config.conf
Environment=NUM_DRONES=1
Environment=RCUTILS_LOGGING_CONFIGURATION_FILE=/home/swarm_software/logs/logging_config.yaml
Environment="PATH=/opt/ros/humble/bin:/usr/local/bin:/usr/bin:/bin"
Restart=no

[Install]
WantedBy=multi-user.target
```

*(For `target_launch.service`, change `ExecStart` to `target_launch_wrapper.sh` and add `Conflicts=ros2_launch.service`.)*

---

## 6. Wrapper and Helper Scripts

- **ros2_launch_wrapper.sh**  
  Prepares environment and invokes `ros2 launch` commands.
- **target_launch_wrapper.sh**  
  Similar to above but for target-specific launch files.
- **init_bag.sh** & **cleanup_bag.sh**  
  Create bag directories, timestamp files, and cleanup old recordings.

---

## Linux Services and systemd

### What is a Linux Service?

A Linux service (daemon) is a background process managed by `systemd`. Services:
- Start automatically at boot (if enabled).
- Can be monitored, restarted on failure, and have defined dependencies.

### Common Commands

```bash
# Reload service definitions
sudo systemctl daemon-reload

# Enable a service to start on boot
sudo systemctl enable ros2_launch.service

# Start/stop a service immediately
sudo systemctl start ros2_launch.service
sudo systemctl stop ros2_launch.service

# Check status and follow logs
sudo systemctl status ros2_launch.service
sudo journalctl -u ros2_launch.service -f
```

Use `setup_services.py` to automate installation and verification of all services listed above.

---

*End of ROS Services Documentation*