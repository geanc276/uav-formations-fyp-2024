

# Messaging Services Overview

This directory contains all the messaging services that run on each drone to handle status updates, mission events, API requests, and service discovery. Each component runs as a background Linux service to ensure continuous operation.

## Contents

- **status/status_publisher.py**  
- **mission/mission_status_pub.py**  
- **api/api_server.py**  
- **api/api_server.service**  
- **advertise/mdns_advertise.py**

## Setup & Directory Structure

To set up the messaging services, run:

```bash
sudo bash setup.sh
```

This script installs dependencies, creates the `/home/swarm_software` directory hierarchy, copies service and script files into place, and enables the messaging services under systemd.

After running `setup.sh`, the directory structure will look like:

```
/home/swarm_software
├── advertise
│   └── mdns_advertise.py
├── api
│   ├── api_server.py
│   └── api_server.service
├── mission
│   └── mission_status_pub.py
├── status
│   └── status_publisher.py
└── logs
```

---

## 1. Status Publisher (`status/status_publisher.py`)

**Purpose:**  
Continuously publishes drone configuration and state information over ZeroMQ.

**Key Components:**
- `Info`: stores current `config_file`, `state_file`, and `launched` flag.
- `InfoCollector`: observes file system directories (`META_DATA_DIR`, `CONFIG_DIR`, `STATE_DIR`) and a ROS 2 service to update `Info`.
- `InfoPublisher`: uses `Publisher` on port `5556` to broadcast JSON messages on topic `<hostname>/status` every 5 seconds.

**Behavior:**
1. Watches for new or changed files in metadata, config, and state directories.
2. Monitors the ROS 2 `ros2_launch` service to set `launched` state.
3. Publishes status JSON `{ config_file, state_file, launched }` to ZeroMQ subscribers.

---

## 2. Mission Status Publisher (`mission/mission_status_pub.py`)

**Purpose:**  
Bridges ROS 2 mission status messages into ZeroMQ for external monitoring.

**Key Components:**
- `StatusListenerNode`: an `rclpy` node subscribing to `<hostname>/mission/status`.
- `MissionPublisher`: wraps a ZeroMQ `Publisher` on port `5557`.

**Behavior:**
1. Receives `std_msgs/String` data from ROS 2 over the `/mission/status` topic.
2. Repacks the payload into a JSON string.
3. Publishes it on ZeroMQ topic `<hostname>/mission_status`.

---

## 3. API Server (`api/api_server.py`)

**Purpose:**  
Provides a Flask-based HTTP API for uploading files, triggering builds, launching services, and executing commands on the drone.

**Key Endpoints:**
- `POST /upload_zip` — upload and extract launch ZIP archives.
- `POST /upload` — handle JSON, executable, config, or state file uploads.
- `POST /start_executable` — run a specified executable with optional parameters.
- `POST /launch` & `/launch_target` — start or stop systemd services for core and target drone components.
- `POST /request/ros` — publish ROS 2 state commands (`START`, `STOP`, `LAND`, etc.).
- `GET /logs/<filename>` — download application log files.
- `GET /get_public_key` & `/rsync_build` — support encrypted file synchronization.
- `POST /start_build` & `GET /download_build` — trigger `colcon build` and retrieve build artifacts.

**Utilities:**
- Inotify-based file watchers (`inotify.adapters`) for efficient ZIP monitoring.
- RSA key generation and decryption for secure password handling.
- Automatic directory creation and backup logic for safe uploads.

---

## 4. API Service Definition (`api/api_server.service`)

This systemd unit file ensures the API server runs in the background and restarts on failure.

```ini
[Unit]
Description=Gunicorn API Server for File Uploads & Executable Management
After=network.target

[Service]
WorkingDirectory=/home/swarm_software/messaging_services/api
Environment="PYTHONPATH=/home/swarm_software"
ExecStart=/usr/bin/python3 -m gunicorn -w 2 -b 0.0.0.0:8001 api_server:app
Restart=always
User=root

[Install]
WantedBy=multi-user.target
```

---

## 5. MDNS Advertiser (`advertise/mdns_advertise.py`)

**Purpose:**  
Publishes the drone’s HTTP and API services via Avahi/MDNS for network discovery.

**Key Components:**
- D-Bus interface to Avahi (`org.freedesktop.Avahi`).
- TXT records including IP address, hostname, ports, and supported commands.
- Regular refresh (every 60 seconds) to update metadata based on directory contents.

---

## Linux Services and systemd

### What is a Linux Service?
A Linux service (daemon) is a background process managed by `systemd` or another init system. Services start automatically at boot, can be monitored, and can be configured to restart on failure.

### Creating and Managing a systemd Service

1. **Unit File**  
   Place your `.service` file in `/etc/systemd/system/`, e.g.:
   ```
   /etc/systemd/system/api_server.service
   ```

2. **Reload and Enable**  
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable api_server.service
   ```

3. **Start and Check Status**  
   ```bash
   sudo systemctl start api_server.service
   sudo systemctl status api_server.service
   ```

4. **View Logs**  
   ```bash
   sudo journalctl -u api_server.service -f
   ```

Use the `api_server.service` file as a template for other messaging services (status publisher, mission publisher, MDNS advertiser) by adjusting `ExecStart`, `WorkingDirectory`, and service names.

---

*End of Messaging Services Documentation*