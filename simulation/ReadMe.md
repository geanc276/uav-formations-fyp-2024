# The following is a read me for a highly experimental use of docker to simulate gazebo on apple silicon using docker. This was made quickly and tested only a couple of times. This should not be treated as a fully developed solution and guides and use may contain errors. **

# Raspberry Pi–Based Simulation on macOS via Docker

This README explains how to run a Raspberry Pi–emulated ROS 2/Gazebo simulation on macOS using Docker. It allows you to test and develop your multi-drone swarm code locally on a Mac, without requiring physical Raspberry Pi hardware. Docker is used to emulate the Raspberry Pi environment, load ROS 2 Humble, and run Gazebo in an X11 window.

---

## Table of Contents

1. [Overview](#overview)  
2. [Prerequisites](#prerequisites)  
3. [Directory Structure](#directory-structure)  
4. [Key Components](#key-components)  
   1. [setup.sh](#1-setupsh)  
   2. [build.sh](#2-buildsh)  
   3. [Dockerfile](#3-dockerfile)  
   4. [docker-compose.yml](#4-docker-composeyml)  
   5. [run.sh](#5-runsh)  
   6. [sim_launch.bash](#6-sim_launchbash)  
5. [Usage](#usage)  
   1. [Initial Setup](#initial-setup)  
   2. [Building the Docker Image](#building-the-docker-image)  
   3. [Launching the Simulation](#launching-the-simulation)  
   4. [Running Multiple Drone Instances](#running-multiple-drone-instances)  
6. [Log Management](#log-management)  
7. [Troubleshooting](#troubleshooting)  
8. [License & Acknowledgements](#license--acknowledgements)  

---

## Overview

This project provides a complete Dockerized environment that emulates a Raspberry Pi running Ubuntu and ROS 2 Humble, allowing you to develop and test your multi-drone swarm code on a macOS host. The setup includes:

- **Docker images** configured to mimic Raspberry Pi architecture via Buildx.
- **ROS 2 Humble** and Gazebo installed inside the container.
- **X11 forwarding** to display Gazebo’s GUI on your Mac.
- **Bind mounts** to share your local workspace with the container, so code edits are immediately available.
- **Per-drone configuration** (via `DRONE_ID`) so you can spin up multiple simulated drones concurrently.

---

## Prerequisites

1. **macOS 10.15+** (Catalina, Big Sur, Monterey, or newer)  
2. **Docker Desktop for Mac**  
   - Install from [https://www.docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop).  
   - Ensure “Use virtualization framework” is enabled in Docker’s settings.  
   - Confirm that Buildx support is available (`docker buildx version`).  
3. **XQuartz (X11)**  
   - Install from [https://www.xquartz.org](https://www.xquartz.org).  
   - After installation, launch XQuartz and go to **XQuartz → Preferences → Security**, then enable “Allow connections from network clients.”  
   - Open a terminal and run:
     ```bash
     xhost + 127.0.0.1
     ```
     to allow local X11 connections.
4. **Git**  
   ```bash
   brew install git
   ```
5. **ROS 2 Humble source code** (optional)  
   - This container installs ROS 2 Humble inside; you do not need a native ROS installation on the Mac.

---

## Directory Structure

```
simulation/
├── Dockerfile
├── build.sh
├── setup.sh
├── run.sh
├── docker-compose.yml
├── sim_launch.bash
└── ReadMe.md
```

- `Dockerfile` – Defines how to build the `drone:humble` image emulating Raspberry Pi.  
- `build.sh` – Builds the Docker image with Buildx, targeting ARM architecture (Raspberry Pi).  
- `setup.sh` – Ensures Docker Desktop/Buildx is installed, pulls the ROS 2 base image, and invokes `build.sh`.  
- `run.sh` – Wrapper script to launch a per-drone container, set environment variables, and open an interactive shell.  
- `docker-compose.yml` – Describes the “drone” service, binds the workspace, and sets up X11 forwarding.  
- `sim_launch.bash` – Inside-container script to rotate logs and run `ros2 launch` with Gazebo.  
- `ReadMe.md` – This file.

---

## Key Components

### 1. setup.sh

Bootstrap script to ensure Docker Desktop (including Buildx) is available on macOS, then:

1. Checks if Docker is accessible (`docker` CLI).  
2. Configures Buildx if not already set.  
3. Pulls the `ros:humble` ARM base image.  
4. Calls:
   ```bash
   ./build.sh humble
   ```
   to build `drone:humble` (ARM) via Buildx.

Usage from the `simulation/` folder:
```bash
chmod +x setup.sh
./setup.sh
```

---

### 2. build.sh

Builds the `drone:humble` Docker image with Raspberry Pi (ARM) emulation:

1. Parses optional `-r` flag for no-cache rebuild:
   ```bash
   ./build.sh -r humble
   ```
2. Requires one argument: ROS distro (e.g., `humble`).
3. Uses Buildx to target ARM architecture:
   ```bash
   docker buildx build \
     --platform linux/arm64 \
     --build-arg BASE_IMAGE=ros \
     --build-arg BASE_TAG=humble \
     --build-arg UID=$(id -u) \
     --build-arg GID=$(id -g) \
     -t drone:humble \
     .
   ```
4. Sets file permissions so that files created inside the container match your macOS user.

---

### 3. Dockerfile

Defines how to extend the ROS 2 Humble image to emulate a Raspberry Pi environment:

```dockerfile
# 1. Base on ROS Humble (ARM64)
FROM --platform=linux/arm64 ros:humble AS base
ENV DEBIAN_FRONTEND=noninteractive

# 2. Install system dependencies (build tools, Gazebo, ROS packages, etc.)
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    python3-colcon-common-extensions \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-control-toolbox \
    libreadline-dev \
    libboost-all-dev \
    libtbb-dev \
    sudo \
    bash-completion \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# 3. Create 'ros' user matching host UID/GID
ARG UID=1000
ARG GID=1000

RUN groupadd --gid "$GID" ros || true \
 && useradd --uid "$UID" --gid "$GID" --create-home ros \
 && usermod -aG sudo,dialout ros \
 && echo 'ros ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ros \
 && chmod 0440 /etc/sudoers.d/ros

# 4. Switch to 'ros' user to set up environment
USER ros
WORKDIR /home/ros

# 5. Copy and source a bashrc updater to include ROS environment
COPY configs/update_bashrc.sh /home/ros/update_bashrc.sh
RUN chmod +x /home/ros/update_bashrc.sh \
 && /home/ros/update_bashrc.sh \
 && rm /home/ros/update_bashrc.sh

# 6. Set entrypoint to source ROS 2 and workspace overlays
COPY configs/entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
```

Key points:
- `--platform=linux/arm64` instructs Buildx to produce an ARM64 image (Raspberry Pi’s architecture).
- Installs Gazebo and any ROS 2 packages needed for simulation.
- Creates a `ros` user with the same UID/GID as your macOS user for file permission consistency.
- Entry point (`entrypoint.sh`) sources `/opt/ros/humble/setup.bash` so that launching `bash` immediately has ROS 2 available.

---

### 4. docker-compose.yml

Defines a single service, `drone`, with settings tailored for macOS:

```yaml
version: "3.9"

services:
  drone:
    image: drone:humble
    container_name: "drone_${DRONE_ID}_container"
    platform: linux/arm64
    stdin_open: true
    tty: true
    environment:
      - DISPLAY=host.docker.internal:0
      - DRONE_ID=${DRONE_ID}
      - DRONE_CONFIG_FOLDER_PATH=/home/ros/drones/configs/drone_${DRONE_ID}
      - DRONE_STATE_FOLDER_PATH=/home/ros/drones/config_states
    volumes:
      - type: bind
        source: ${HOST_WS_PATH}
        target: /home/ros/drones/uav-formations-fyp-2024
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    network_mode: "bridge"
    privileged: true
    command: bash
```

- `platform: linux/arm64` ensures Docker pulls the ARM64 variant and runs it via emulation.
- `DISPLAY=host.docker.internal:0` forwards X11 from the container to your Mac’s XQuartz server.
- Binds your local project root (`HOST_WS_PATH`) into `/home/ros/drones/uav-formations-fyp-2024` inside the container.
- Maps macOS’s X11 socket (`/tmp/.X11-unix`) so Gazebo can render on your screen.
- `privileged: true` is needed for Gazebo’s graphical and physics acceleration.

---

### 5. run.sh

Wrapper script to launch a per-drone container:

```bash
#!/usr/bin/env bash

# Usage check
if [ $# -lt 1 ]; then
  echo "Usage: ./run.sh <DRONE_ID>"
  exit 1
fi

# Set DRONE_ID
export DRONE_ID="$1"
echo "Starting Docker container for Drone ID ${DRONE_ID}..."

# Determine project root (HOST_WS_PATH) automatically
HOST_WS_PATH="$(cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd)"
export HOST_WS_PATH

# Create a per-drone .env file
ENV_FILE=".env_drone_${DRONE_ID}"
cat <<EOF > "${ENV_FILE}"
HOST_WS_PATH=${HOST_WS_PATH}
DRONE_ID=${DRONE_ID}
EOF

# Unique Docker Compose project name
export COMPOSE_PROJECT_NAME="drone_project_${DRONE_ID}"

# Tear down existing container if present
EXISTING=$(docker ps -a --filter "name=drone_${DRONE_ID}_container" --format "{{.Names}}")
if [ -n "${EXISTING}" ]; then
  docker rm -f "${EXISTING}"
fi

# Launch the container
docker compose --env-file "${ENV_FILE}" up -d

# Wait a moment for the container to start
sleep 3

# Open an interactive shell inside the container
docker exec -it "drone_${DRONE_ID}_container" bash
```

Key steps:
1. Requires a `DRONE_ID` argument (e.g., `1`, `A`, `swarm1`).
2. Computes `HOST_WS_PATH` as the parent directory (the project root).
3. Writes `.env_drone_<ID>` with `HOST_WS_PATH` and `DRONE_ID`.
4. Sets a unique Compose project name to avoid conflicts.
5. Removes any existing container named `drone_<ID>_container`.
6. Runs `docker compose up -d` to start the container detached.
7. Opens an interactive shell in the newly launched container.

---

### 6. sim_launch.bash

Script to rotate previous logs and run the ROS 2 launch file inside the container:

```bash
#!/usr/bin/env bash

# Timestamp for naming logs
CURRENT_DATE=$(date '+%Y-%m-%d_%H-%M-%S')

# Log directories inside the container
LOG_BASE="/home/ros/logs/ros"
CURRENT_LOG="${LOG_BASE}/current"
HISTORY_LOG="${LOG_BASE}/last_five"
ARCHIVE_LOG="${LOG_BASE}/old"
SWARM_WS="/home/ros/drones/uav-formations-fyp-2024/swarm_ws"

# Ensure log directories exist
mkdir -p "${CURRENT_LOG}" "${HISTORY_LOG}" "${ARCHIVE_LOG}"

# Rotate old logs: if more than 4 in last_five, move oldest to 'old'
count=$(find "${HISTORY_LOG}" -maxdepth 1 -type d -name 'ros2_launch_*' | wc -l)
if [ "${count}" -gt 4 ]; then
  oldest=$(ls -1 "${HISTORY_LOG}" | grep 'ros2_launch_' | sort | head -n 1)
  mv "${HISTORY_LOG}/${oldest}" "${ARCHIVE_LOG}/"
fi

# Move current logs (if any) into last_five
if [ -n "$(ls -A "${CURRENT_LOG}" 2>/dev/null)" ]; then
  target="${HISTORY_LOG}/ros2_launch_${CURRENT_DATE}"
  mkdir -p "${target}"
  mv "${CURRENT_LOG}"/* "${target}/"
fi

# Source the ROS 2 workspace overlays
source "/opt/ros/humble/setup.bash"
source "${SWARM_WS}/install/setup.bash"

# Define log file paths
STDOUT_LOG="${CURRENT_LOG}/ros2_launch_${CURRENT_DATE}.log"
STDERR_LOG="${CURRENT_LOG}/ros2_launch_error_${CURRENT_DATE}.log"

# Run the ROS 2 launch (Gazebo simulation)
exec ros2 launch "${SWARM_WS}/drone_launch/drone.launch.py" \
  > "${STDOUT_LOG}" 2> "${STDERR_LOG}"
```

- Maintains up to 5 previous runs (`last_five`), archiving older logs under `old/`.
- Places all output from `ros2 launch` into timestamped files under `logs/ros/current/`.
- Sources both the base ROS 2 and your workspace’s `install/setup.bash` so the `drone_launch` package is available.

---

## Usage

### Initial Setup

1. **Clone the Repository**  
   ```bash
   git clone https://github.com/your_org/uav-formations-fyp-2024.git
   cd uav-formations-fyp-2024/simulation
   ```

2. **Ensure XQuartz Is Running**  
   - Launch XQuartz from **Applications → Utilities**.  
   - In XQuartz’s Preferences → Security, enable “Allow connections from network clients.”  
   - In a macOS Terminal:
     ```bash
     xhost + 127.0.0.1
     ```

3. **Make Scripts Executable**  
   ```bash
   chmod +x setup.sh build.sh run.sh sim_launch.bash
   ```

---

### Building the Docker Image

From the `simulation/` folder on your Mac:

```bash
./setup.sh
```

- Installs Docker Desktop and Buildx if missing (only on first run).  
- Pulls the `ros:humble` ARM64 base image.  
- Invokes:
  ```bash
  ./build.sh humble
  ```
  which uses Buildx to create `drone:humble` targeting ARM64 (Raspberry Pi).

---

### Launching the Simulation

1. **Start a Drone Container**  
   In the `simulation/` directory:
   ```bash
   ./run.sh 1
   ```
   - This sets `DRONE_ID=1` and creates a container named `drone_1_container`.  
   - Binds your local workspace into `/home/ros/drones/uav-formations-fyp-2024`.  
   - Opens an interactive shell inside the container as user `ros`.

2. **Inside the Container, Start the ROS 2 Launch**  
   In the container shell:
   ```bash
   cd /home/ros/drones/uav-formations-fyp-2024/simulation
   ./sim_launch.bash
   ```
   - Gazebo should open an X11 window on your Mac.  
   - Log files appear under `/home/ros/logs/ros/current/`.

---

### Running Multiple Drone Instances

You can run multiple simulated drones by opening new macOS Terminal tabs (or windows) and repeating:

```bash
cd ~/…/uav-formations-fyp-2024/simulation
./run.sh 2
```

- Each invocation with a different `DRONE_ID` (e.g., `2`, `3`, `A`) creates a separate container (`drone_2_container`, etc.).  
- Each container writes its own logs to `/home/ros/logs/ros` inside that container.

---

## Log Management

- **Current logs**: `/home/ros/logs/ros/current/ros2_launch_<timestamp>.log`  
- **Last five runs**: `/home/ros/logs/ros/last_five/ros2_launch_<timestamp>/`  
- **Older archives**: `/home/ros/logs/ros/old/ros2_launch_<timestamp>/`  

`sim_launch.bash` automatically rotates logs as described above.

---

## Troubleshooting

1. **Gazebo GUI Doesn’t Appear**  
   - Ensure XQuartz is running and that you’ve run `xhost + 127.0.0.1`.  
   - In the container, verify:
     ```bash
     echo $DISPLAY
     ```
     should show `host.docker.internal:0`.

2. **Permission Errors on Mounted Files**  
   - Because we built the image with `UID=$(id -u)` and `GID=$(id -g)`, files should match your macOS user.  
   - If you see `Permission denied`, check that the container’s `ros` user has correct UID/GID:
     ```bash
     id ros
     ```

3. **Buildx or Emulation Issues**  
   - Confirm Docker Desktop is updated and that Buildx supports ARM emulation.  
   - Run:
     ```bash
     docker buildx ls
     ```
     and ensure the `linux/arm64` platform is available.

4. **Container Won’t Start**  
   - Check logs:
     ```bash
     docker logs drone_<ID>_container
     ```
   - Ensure environment variables (especially `DISPLAY` and `HOST_WS_PATH`) are correctly set in `.env_drone_<ID>`.

---

## License & Acknowledgements

- This Dockerized setup was inspired by best practices for cross-platform ROS 2 simulation.  
- ROS 2 Humble, Gazebo, and Docker represent open-source components under their respective licenses.  
- Written and maintained by the Wireless Research Centre (WRC), University of Canterbury.