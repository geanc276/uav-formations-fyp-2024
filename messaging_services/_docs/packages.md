# UAV Formations FYP 2024 Documentation

## Overview

This project is part of the UAV formations Final Year Project (FYP) 2024. It comprises a suite of messaging services designed for drone operations. The services include:

- **Service Advertisement:** Uses mDNS/Avahi to advertise the drone service on the local network.
- **RESTful API Server:** Manages file uploads, executable launches, service management, build synchronization, and configuration updates.
- **Mission Status Publishing:** Utilizes ROS 2 to listen for and republish mission status messages.
- **General Status Publishing:** Observes file directories and service states to publish current status information.


## Components

### 1. MDNS Advertise Service

**File:** `messaging_services/advertise/mdns_advertise.py`

- **Purpose:**  
  Advertises a service using Avahi (mDNS) to enable other network devices to discover it.
  
- **Key Functions:**
  - `create_txt_array`: Builds TXT records that include metadata (IP address, hostname, port numbers, supported commands).
  - `update_service`: Resets and adds the service to the Avahi entry group, then commits the advertisement.
  - `stop`: Resets the entry group to stop advertising the service.
  - `advertise_service`: Initializes the advertisement and maintains it in a loop until interrupted.

- **Usage:**  
  Run the script directly to begin advertising the service on your network.

### 2. API Server

**File:** `messaging_services/api/api_server.py`

- **Purpose:**  
  Provides a RESTful API for various operations such as uploading files, launching executables, managing configurations, and synchronizing builds.

- **Key Endpoints:**

  - **File Uploads:**
    - `/upload_zip`:  
      Uploads a ZIP file for drone launch; monitors file creation with inotify and extracts its contents.
    - `/upload`:  
      Handles file uploads for JSON, executables, configuration, and state files based on the provided command.
    - `/upload_any_file`:  
      Uploads arbitrary files to a designated directory.
  
  - **Executable Management:**
    - `/start_executable`:  
      Starts an executable file on the server with optional command-line parameters.
  
  - **Service Management:**
    - `/launch`:  
      Launches services, setting environment variables and interacting with systemd to start the services.
    - `/launch_target`:  
      Launches target services with specific environment configurations.
    - `/request/stop`:  
      Stops all running services managed by the server.
  
  - **Key Exchange & Synchronization:**
    - `/get_public_key`:  
      Generates and returns an RSA public key for secure communication.
    - `/rsync_build`:  
      Synchronizes build files from a remote server using rsync (with encrypted credentials).
    - `/start_build`:  
      Initiates the build process using `colcon build` in a specified workspace.
    - `/download_build`:  
      Packages the build’s install directory into a ZIP file for download.
  
  - **Additional Operations:**
    - `/connected`:  
      Restarts the status publisher service.
    - `/request/ros`:  
      Sends ROS commands (e.g., START, STOP, LAND, PAUSE, RESUME) to control drone state.
  
- **Authentication:**  
  Basic authentication is used (default credentials: username `drone`, password `pass`). Ensure you update these credentials in production.

- **Logging:**  
  Conditional logging is implemented. If enabled, logs are stored in the `LOGS_DIR` and streamed to the console.

### 3. Mission Status Publisher

**File:** `messaging_services/mission/mission_status_pub.py`

- **Purpose:**  
  Listens for mission status messages from a ROS 2 topic and republishes them using a custom publisher.

- **Key Components:**
  - `StatusListenerNode`:  
    A ROS 2 node subscribing to a topic based on the hostname (formatted as `{hostname}/mission/status`) to receive status updates.
  - `MissionPublisher`:  
    Initializes the ROS 2 node, handles message callbacks, and publishes the received messages.

- **Usage:**  
  Run this script to start the mission status listener and publisher.

### 4. Status Publisher

**File:** `messaging_services/status/status_publisher.py`

- **Purpose:**  
  Monitors configuration and state directories along with service status to publish current operational status.

- **Key Components:**
  - `Info`:  
    A simple class that holds the current configuration file, state file, and service launch status.
  - `InfoPublisher`:  
    Publishes the current status at regular intervals using a timer thread.
  - `InfoCollector`:  
    Observes the file system (directories for meta data, configuration, and state) and monitors the specified service status to update the `Info` instance.

- **Operation:**  
  A continuous loop periodically publishes updated status information to a topic derived from the hostname.

## Setup and Running the Project

### Prerequisites

- **Python 3.x** is required.
- **Dependencies:**  
  Install the following (ensure these are in your `requirements.txt`):
  - `dbus`
  - `flask`
  - `inotify` (Linux only)
  - `cryptography`
  - `rclpy` (ROS 2)
  - Other dependencies as referenced by the code.

### Installation

1. **Clone the Repository:**
   ```bash
   git clone <repository_url>
   ```

2. **Navigate to the Project Directory:**
   ```bash
   cd uav-formations-fyp-2024
   ```

3. **Install Dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

### Running the Services

- **MDNS Advertise Service:**
  ```bash
  ./messaging_services/advertise/mdns_advertise.py
  ```

- **API Server:**
  ```bash
  ./messaging_services/api/api_server.py
  ```

- **Mission Status Publisher:**
  ```bash
  ./messaging_services/mission/mission_status_pub.py
  ```

- **Status Publisher:**
  ```bash
  ./messaging_services/status/status_publisher.py
  ```

### API Usage Examples
Most of the interactions will be done via the gui with predefined interactions, but here are some terminal examples.

**File Upload Example**
```bash
curl -u drone:pass -F "file=@/path/to/your/file.json" -F "command=upload_new_json" http://<server_ip>:8001/upload
```

**Launching Services**
```bash
curl -u drone:pass -H "Content-Type: application/json" -d '{"command": "LAUNCH", "num_drones": 3}' http://<server_ip>:8001/launch
```

**Starting an Executable**
```bash
curl -u drone:pass -H "Content-Type: application/json" -d '{"executable_name": "your_executable", "params": ["arg1", "arg2"]}' http://<server_ip>:8001/start_executable
```

**Getting the RSA Public Key**
```bash
curl -u drone:pass http://<server_ip>:8001/get_public_key
```

## Logging and Monitoring

- **Log Access:** Access logs via the `/logs/<filename>` endpoint.
- **Monitoring:** Status publisher monitors key directories and service states, publishing updates at regular intervals.

## Troubleshooting

- **Authentication Issues:** Verify correct credentials.
- **Service Advertisement:** Ensure Avahi daemon is running.
- **Directory Observations:** Ensure required directories exist and have correct permissions.

