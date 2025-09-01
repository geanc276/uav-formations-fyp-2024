Below is an example documentation outlining each service, its purpose, configuration details, and instructions for running them as Linux systemd services. You can adjust paths, environment variables, and user permissions as needed.

⸻

UAV Formations FYP 2024 - Service Documentation

This document provides an overview and instructions for deploying the messaging services in the UAV Formations FYP 2024 project on a Linux machine. The services are implemented in Python and are designed to run as background processes managed by systemd.

Services Overview

1. mDNS Advertiser Service (mdns_advertise.py)
	•	Purpose:
Advertises the service on the local network using Avahi (mDNS). It retrieves network details (IP, hostname) and metadata, creates TXT records, and registers the service via Avahi.
	•	Key Features:
	•	Retrieves IP, hostname, and meta data.
	•	Constructs TXT records (including supported commands and ports).
	•	Advertises and updates the service continuously.
	•	Cleans up the advertisement when stopping.

2. API Server Service (api_server.py)
	•	Purpose:
Provides a RESTful API server (using Flask) for handling file uploads, executable management, service orchestration, and additional functionalities like RSA key generation and rsync-based file synchronization.
	•	Key Features:
	•	Supports multiple endpoints for uploading files (ZIP, JSON, executables) and launching/stopping services.
	•	Implements basic authentication.
	•	Uses inotify to monitor file events for efficient file handling.
	•	Offers endpoints for starting executables, building projects, and publishing logs.

3. Mission Status Publisher (mission_status_pub.py)
	•	Purpose:
Listens for ROS2 messages related to mission status and publishes these updates using a custom messaging publisher.
	•	Key Features:
	•	Integrates with ROS2 to subscribe to mission status topics.
	•	Listens for and logs status messages.
	•	Publishes received messages over a dedicated channel using a custom Publisher.

4. Status Publisher (status_publisher.py)
	•	Purpose:
Aggregates configuration and state information from specified directories and system services, then publishes a consolidated status message.
	•	Key Features:
	•	Monitors directories for configuration, meta data, and state changes.
	•	Uses a background thread to periodically publish the current status.
	•	Observes the state of specific system services to update status information.

⸻

Running the Services as systemd Services

Each service can be deployed as a systemd service. The following example shows a sample service file, and similar templates can be adapted for each of the services.

Example Service File Template

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

Adapting the Template for Each Service

mDNS Advertiser Service
	•	Working Directory: Directory where mdns_advertise.py is located.
	•	ExecStart:

/usr/bin/python3 /home/swarm_software/messaging_services/advertise/mdns_advertise.py


	•	Description: Advertises the service using Avahi.
	•	Sample Service File:

[Unit]
Description=mDNS Advertiser Service
After=network.target

[Service]
WorkingDirectory=/home/swarm_software/messaging_services/advertise
Environment="PYTHONPATH=/home/swarm_software"
ExecStart=/usr/bin/python3 /home/swarm_software/messaging_services/advertise/mdns_advertise.py
Restart=always
User=root

[Install]
WantedBy=multi-user.target



API Server Service
	•	Working Directory: Directory containing api_server.py.
	•	ExecStart (Direct Execution):

/usr/bin/python3 /home/swarm_software/messaging_services/api/api_server.py


	•	Description: Flask-based API server for handling file uploads and service management.
	•	Sample Service File (Direct Execution):

[Unit]
Description=API Server for File Uploads & Executable Management
After=network.target

[Service]
WorkingDirectory=/home/swarm_software/messaging_services/api
Environment="PYTHONPATH=/home/swarm_software"
ExecStart=/usr/bin/python3 /home/swarm_software/messaging_services/api/api_server.py
Restart=always
User=root

[Install]
WantedBy=multi-user.target


	•	Alternative:
If using Gunicorn (as in the provided example), use the provided sample file.

Mission Status Publisher
	•	Working Directory: Directory containing mission_status_pub.py.
	•	ExecStart:

/usr/bin/python3 /home/swarm_software/messaging_services/mission/mission_status_pub.py


	•	Description: ROS2-based service to subscribe to mission status and republish messages.
	•	Sample Service File:

[Unit]
Description=Mission Status Publisher Service
After=network.target

[Service]
WorkingDirectory=/home/swarm_software/messaging_services/mission
Environment="PYTHONPATH=/home/swarm_software"
ExecStart=/usr/bin/python3 /home/swarm_software/messaging_services/mission/mission_status_pub.py
Restart=always
User=root

[Install]
WantedBy=multi-user.target



Status Publisher
	•	Working Directory: Directory containing status_publisher.py.
	•	ExecStart:

/usr/bin/python3 /home/swarm_software/messaging_services/status/status_publisher.py


	•	Description: Publishes aggregated configuration and state information.
	•	Sample Service File:

[Unit]
Description=Status Publisher Service
After=network.target

[Service]
WorkingDirectory=/home/swarm_software/messaging_services/status
Environment="PYTHONPATH=/home/swarm_software"
ExecStart=/usr/bin/python3 /home/swarm_software/messaging_services/status/status_publisher.py
Restart=always
User=root

[Install]
WantedBy=multi-user.target



⸻

Configuration and Deployment Instructions

Prerequisites
	•	Python3 installed with required modules.
	•	Network connectivity and appropriate permissions.
	•	The base directory (e.g., /home/swarm_software) added to PYTHONPATH if needed.
	•	The services’ working directories must match the file locations.

Deployment Steps
	1.	Create Service Files:
Save each service file (e.g., api_server.service) in /etc/systemd/system/.

sudo cp /path/to/api_server.service /etc/systemd/system/


	2.	Reload systemd Daemon:

sudo systemctl daemon-reload


	3.	Enable the Service:

sudo systemctl enable api_server.service


	4.	Start the Service:

sudo systemctl start api_server.service


	5.	Verify Status:

sudo systemctl status api_server.service

Repeat the above steps for each service file.

Troubleshooting
	•	Logs:
Use the journal to view logs for any service:

sudo journalctl -u <service_name>.service


	•	Permissions:
Verify that the specified user (e.g., root or another account) has access to the necessary directories and files.
	•	Environment Variables:
Ensure that PYTHONPATH and other environment variables are correctly set in the service files.
