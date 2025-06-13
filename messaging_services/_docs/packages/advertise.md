# Drone Services Documentation

This documentation outlines the core services running on the drone. These include messaging services controlled by Linux systemd and a Python-based mDNS advertisement service that facilitates automatic discovery and connection to the drone.

## Messaging Services on the Drone

Each messaging service runs as a Linux systemd service, controlled by its respective `.service` file.

### Overview

- **Messaging Services:** Facilitate inter-module communication on the drone.
- **Linux Services:** Managed via systemd, using service files.

### Managing Services with `systemctl`

The `systemctl` command is used to control these services. Below are common commands and their descriptions:

- **status**  
  **Usage:** `sudo systemctl status [service_name].service`  
  **Description:** Displays the current state of the service (active, inactive, or failed), along with recent logs and metadata.

- **start**  
  **Usage:** `sudo systemctl start [service_name].service`  
  **Description:** Initiates the service immediately.

- **stop**  
  **Usage:** `sudo systemctl stop [service_name].service`  
  **Description:** Halts the service immediately.

- **restart**  
  **Usage:** `sudo systemctl restart [service_name].service`  
  **Description:** Stops and then starts the service, useful for applying configuration changes.

- **enable**  
  **Usage:** `sudo systemctl enable [service_name].service`  
  **Description:** Configures the service to start automatically at boot.

- **disable**  
  **Usage:** `sudo systemctl disable [service_name].service`  
  **Description:** Prevents the service from starting automatically at boot.

### Viewing Service Logs with `journalctl`

Logs can be viewed using `journalctl`:

- **Basic Log Viewing**  
  **Usage:** `sudo journalctl -u [service_name].service`  
  **Description:** Retrieves logs for the specified service.

- **Common Flags:**
  - **-r**  
    **Usage:** `sudo journalctl -u [service_name].service -r`  
    **Description:** Displays logs in reverse chronological order (most recent entries first).
  
  - **-u**  
    **Usage:** Part of the command as in `sudo journalctl -u [service_name].service`  
    **Description:** Specifies the unit name for which logs should be retrieved.
  
  - **Additional Examples:**
    - **-f:** Follows logs in real-time, similar to `tail -f`.  
      **Usage:** `sudo journalctl -u [service_name].service -f`
    - **-b:** Shows logs from the current boot.  
      **Usage:** `sudo journalctl -u [service_name].service -b`

### Typical `.service` File Structure

A typical service file defines how a messaging service runs. For example:

```ini
[Unit]
Description=Brief description of the messaging service
After=network.target

[Service]
Type=simple
User=your_user
ExecStart=/path/to/executable --option value
Restart=on-failure
# Additional directives:
# Environment=KEY=value
# WorkingDirectory=/path/to/working/directory

[Install]
WantedBy=multi-user.target
```

#### Key Fields Explained

- **[Unit] Section:**
  - `Description`: A short summary of the service’s purpose.
  - `After`: Specifies service dependencies (e.g., `After=network.target` ensures the network is available before starting).
  
- **[Service] Section:**
  - `Type`: Defines the process start-up type (commonly `simple`).
  - `User`: The user account under which the service runs.
  - `ExecStart`: The command to start the service.
  - `Restart`: Configures the restart policy (e.g., `on-failure` or `always`).
  - Additional directives: Such as environment variables and working directories.
  
- **[Install] Section:**
  - `WantedBy`: Associates the service with a system target (typically `multi-user.target`).

---

## mDNS Advertisement Service

A key component of the drone’s connectivity is the mDNS advertisement service, implemented as a Python package. This service uses the mDNS protocol (similar to Apple’s Bonjour) to broadcast essential connection information.

### Features

- **Python Package:**  
  Runs on the drone to enable auto-discovery on local networks.

- **mDNS (Bonjour) Protocol:**  
  Advertises the drone’s presence, enabling clients to automatically detect and connect.

### Advertised Information

The service broadcasts the following critical details:

- **IP Address:** The network IP of the drone.  
- **Hostname:** The drone’s network name.  
- **Port:** The main port used for general connections.  
- **API Port:** The port designated for API communications.  
- **Available Commands:** A list of supported commands for interacting with the drone.

### Implementation Details

- **File Location:**  
  The service is implemented in the Python file `mdns_advertise.py` (located at `/mnt/data/mdns_advertise.py`).

- **Purpose:**  
  By advertising the connection details, the mDNS service simplifies the process of auto-connecting to the drone from client devices on the same network.

---

## Conclusion

This documentation covers both the messaging services and the mDNS advertisement service running on the drone. It explains how to manage these services via systemd, view logs, understand typical service file configurations, and highlights the functionality of the mDNS service for automatic drone discovery and connection.
