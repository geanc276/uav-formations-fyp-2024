# Drone Swarm FYP

## Table of Contents
- [Description](#description)
- [Installation](#installation)
- [Development Environment Setup](#development-environment-setup)
- [Non-Development Setup](#non-development-setup)
- [Building Swarm_ws](#building-swarm_ws)
- [Setting up QGroundControl](#setting-up-qgroundcontrol)
- [Using DroneBridge Firmware on ESP32](#using-dronebridge-firmware-on-esp32)
  - [Flashing DroneBridge onto the ESP32](#flashing-dronebridge-onto-the-esp32)
  - [Wiring the ESP32 to the Flight Controller](#wiring-the-esp32-to-the-flight-controller)
  - [Initial Network Configuration via Web Interface](#initial-network-configuration-via-web-interface)
  - [Adding DroneBridge to QGroundControl](#adding-dronebridge-to-qgroundcontrol)
  - [Troubleshooting Tips](#troubleshooting-tips)
- [Setting Up Drone Raspberry Pi (RPI)](#setting-up-drone-raspberry-pi-rpi)
- [Setting Up Drone PX4](#setting-up-drone-px4)
- [Uploading Swarm Software to Drones](#uploading-swarm-software-to-drones)
- [Run Launch](#run-launch)

## Description
Final year project repository for students working on target tracking using drone swarms. The Wireless Research Centre (WRC) is developing harmonic radar technology for wildlife tracking, specifically native insects. Drone swarms built as part of this project will track insects and record key data.


### Requirements
- Ubuntu 22.04 LTS
- Python 3.11

### Software Used in Project
- [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
- [Gazebo Fortress](https://gazebosim.org/docs/latest/ros_installation/): the most compatible version with ROS2 Humble 
- [UXRCE_DDS](https://docs.px4.io/main/en/middleware/uxrce_dds.html)
- QGroundControl
- [DroneBridge](https://dronebridge.github.io/ESP32/install.html)

> **Note:** If not developing locally, you can use an RPi running Ubuntu connected to a Windows machine with QGroundControl.


## Installation

## Development Environment Setup

1. **Clone the repository** into your directory of choice.
2. **Pull the PX4-Autopilot submodule**:
   ```bash
   cd uav-formations
   git submodule update --init --recursive
   ```
3. **Install required packages**, including ROS 2 Humble, by running:
   ```bash
   source install.sh
   ```
   - This script will prompt for confirmation to download packages.
   - To install without prompts (untested), run:
     ```bash
     source install_auto.sh -y
     ```
4. **Restart the machine**:
   ```bash
   sudo reboot now
   ```
5. **Build the simulation**:
   ```bash
   cd uav-formations-fyp-2024/px4-autopilot
   make px4_sitl
   ```

## Non-Development Setup
If you are not developing further on the project, all that is needed is the GUI and Qgroundcontrol, and temporarily still ROS2 humble to run user input to command the drones.
1. **Clone the repository** onto the Ubuntu host (and Windows machine if using a mixed environment).
2. **Launch the GUI** to create the required virtual environment:
   ```bash
   python3 start_gui.py
   ```
3. **Ensure ROS 2 Humble is installed** on Ubuntu and build `controller_msgs` and `user_input`:
   ```bash
   cd swarm_ws
   colcon build --packages-select controller_msgs user_input
   # or
   make select controller_msgs user_input
   ```
4. **Download QGroundControl**:
   - Linux: Download the `.AppImage`, then:
     ```bash
     cd ~/Downloads
     chmod +x ./QGroundControl.AppImage
     ```
   - Windows: Follow instructions on the QGroundControl website.
   - macOS: Download the `.dmg` and move QGroundControl to Applications.

## Building Swarm_ws
Navigate to the ROS 2 workspace `swarm_ws` and run:
```bash
colcon build
```
or
```bash
make
```
> **Tip:** The first build may take time due to external packages (e.g., `px4_msgs`).  
> If your computer cannot build ROS 2 natively (e.g., macOS), use the drone_build GUI to offload the build to a Raspberry Pi.

## Setting up QGroundControl

1. **Download QGroundControl** from the official website: [http://qgroundcontrol.com/downloads/](http://qgroundcontrol.com/downloads/).
2. Follow platform-specific instructions in the **Installation** section above.
3. Refer to the [QGroundControl User Guide](https://docs.qgroundcontrol.com/) for detailed setup and configuration.

## Using DroneBridge Firmware on ESP32

**Note:** DroneBridge replaces traditional telemetry radios to support more simultaneous connections, which we had errors with current in house telemetry radios.

### DroneBridge Open Source Repository
Here DroneBridge is install via binaries, but its github contains the opensource software aswell: [Here](https://github.com/DroneBridge/DroneBridge)

### Flashing DroneBridge onto the ESP32

1. **Prerequisites:**
   - An ESP32 development board (officially supported modules recommended).
   - USB cable to connect the ESP32 to your computer.

2. **Official Online Flashing Tool (Recommended):**
   1. Go to the [DroneBridge Flash Tool for ESP32](https://github.com/DroneBridge/ESP32).
   2. Launch the “Official Online Flashing Tool” in your browser.
   3. Connect the ESP32 via USB. If flashing a different firmware, hold the BOOT button at power-up.
   4. In the tool UI, click **Connect**, select the desired DroneBridge release, and click **Flash**.
   5. After flashing, unplug and replug the ESP32 to reboot into DroneBridge firmware.

3. **Command-Line Flashing with `esptool.py`:**
   1. Install Espressif Python tools on Ubuntu 22.04:
      ```bash
      sudo apt-get update
      sudo apt-get install -y python3-pip
      pip3 install esptool
      ```
   2. Download the latest DroneBridge ESP32 `.bin` from the [GitHub Releases page](https://github.com/DroneBridge/ESP32/releases).
   3. Erase existing flash (recommended):
      ```bash
      esptool.py -p /dev/ttyUSB0 erase_flash
      ```
   4. Flash the firmware:
      ```bash
      esptool.py -p /dev/ttyUSB0 write_flash 0x1000 DroneBridge-ESP32-vX.Y.Z.bin
      ```
      Replace `vX.Y.Z` with the downloaded version.

### Wiring the ESP32 to the Flight Controller

1. **Power Supply:**
   - Ensure the ESP32 is powered at 3.3 V (some modules accept 5 V via onboard regulator).

2. **UART Connections:**
   1. Identify an unused UART on the flight controller (e.g., `/dev/ttyS1` on Raspberry Pi or TELEM2 on Pixhawk).
   2. Connect flight controller TX → ESP32 RX (e.g., GPIO3).
   3. Connect flight controller RX → ESP32 TX (e.g., GPIO1).
   4. Connect a common ground between flight controller and ESP32.

3. **Baud Rate:**
   - Configure the flight controller’s telemetry port to 57600, 115200, or 921600 baud. DroneBridge auto-detects common MAVLink rates.

### Initial Network Configuration via Web Interface

1. **Connect to the ESP32’s Wi-Fi AP:**
   - Power on ESP32. It will broadcast SSID `DroneBridge ESP32` with password `dronebridge`.
   - Join this network on your computer.

2. **Access the Web Configurator:**
   - Open a browser and go to `http://dronebridge.local` or `http://192.168.2.1`.
   - If mDNS fails, use `192.168.2.1`.

3. **Configure Mode & Encryption:**
   - **Mode:** Set to **Station (STA)** to join an existing Wi-Fi network.
   - **Encryption:** AES-GCM 256-bit enabled by default.
   - **SSID & Passphrase:** Enter your drone network’s SSID (e.g., `WorkshopNet`) and password.

4. **Set UDP Targets:**
   1. Under **Network → UDP Targets**, add ground station IP (e.g., `192.168.1.100`) and port `14550`.
   2. Save and reboot ESP32. It will connect to Wi-Fi and forward serial data to the UDP target(s).

### Adding DroneBridge to QGroundControl

1. **Ensure ESP32 is on the same LAN:**
   - Confirm ESP32’s IP via serial monitor or router DHCP leases.

2. **Create a New UDP Link in QGroundControl:**
   1. Launch QGroundControl.
   2. Open **Application Settings (gear icon)**.
   3. In **Comm Links**, click **+** to add a new link.
   4. Configure:
      - **Name:** `DroneBridge-ESP32`
      - **Type:** **UDP**
      - **Listening Port:** `14550`
      - **Target Host:** Enter drone IPs (e.g., `192.168.0.142`, `192.168.0.121`, etc.).
   5. Click **OK/Save**.

3. **Enable and Test:**
   - Check the box next to `DroneBridge-ESP32` in **Comm Links**.
   - QGroundControl sends a MAVLink heartbeat over UDP. Once telemetry is received, vehicle status displays.

### Troubleshooting Tips

- **Cannot Connect to ESP32’s AP?**
  - Disable cellular or secondary network adapters to force connection to `DroneBridge ESP32` SSID.
- **ESP32 Fails to Join Wi-Fi:**
  - Revisit `http://192.168.2.1` to verify SSID/password (case-sensitive).
  - DroneBridge does not support WPA2-Enterprise.
- **No MAVLink Packets in QGroundControl:**
  - Check ESP32 serial monitor for “Joined SSID: YourNet” and “UDP target added: 192.168.1.100:14550.”
  - On ground station, run:
    ```bash
    netstat -ulnp | grep 14550
    ```
  - Verify flight controller is sending MAVLink on the UART (e.g., use `screen` or `minicom`).

## Setting Up Drone Raspberry Pi (RPI)

1. **Ensure RPi is connected to the internet.**
2. **Clone the Repository:**
   ```bash
   git clone <repository_url>
   ```
3. **Initial Setup:**
   ```bash
   cd __new_drone_setup
   ./install_on_drone.bash
   ```
   > **Note:** As of right now, hostname of the drone must be a unique version of Drone_{number}. This is important for the automatic setup of the uxrce_dds connection

   
4. **Messaging Services Configuration:**
   ```bash
   cd messaging\ services
   ./setup.sh
   ```
5. **ROS Services Setup:**
   ```bash
   cd ros_services
   ./setup.bash
   sudo python3 setup_services.py -p
   ```
6. **Configuration Upload:**
   - Use the GUI to upload `config` and `state_config` files to the drone.
7. **Software Package Upload:**
   - In the GUI, select the drone and click **Upload Software** to transfer `swarm_ws` packages.
   - Reboot the RPi after upload.

## Setting Up Drone PX4
As part of this repository, there is a `px4-autopilot` submodule which contains a forked version of PX4 Autopilot with custom firmware changes. Follow these steps to set up PX4 with the custom firmware:

1. **Initialize and update the `px4-autopilot` submodule**  
   ```bash
   git submodule update --init --recursive px4
   ```
   If the submodule URL is missing or outdated, add it manually:
   ```bash
   git submodule add https://eng-git.canterbury.ac.nz/bir16/px4-autopilot.git px4
   git submodule update --init --recursive
   ```

3. **Build the custom PX4 firmware**  
   ```bash
   cd px4-autopilot
   make px4_fmu-v5x_default
   ```
4. **Flash the firmware onto the flight controller**  
   Connect your flight controller via USB, then run:
   ```bash
   ./install_firmware.bash
   ```
   This is a custom script on the forked repository, ensure the number you enter is the same as the drone ID on the RPI. This is important for the uxrce_dds, as discovery between client and agent will fail if they do not match.

### Notable Firmware and DDS Client Changes

The following PX4 firmware files have been updated as part of the custom integration:

1. **`ROMFS/px4fmu_common/init.d/rc.mc_apps`**  
   The DDS client startup line has been add in order to start uxrce client on the correct ports:  
   ```
   uxrce_dds_client start -p 8888 -h 10.41.10.1 -n px4_{number}
   ```

2. **`src/modules/uxrce_dds_client/dds_topics.yaml`**  
   The list of topics for PX4 messages has been updated as follows to expose previously non published messages:
   ```
     - topic: /fmu/out/telemetry_status
       type: px4_msgs::msg::TelemetryStatus

     - topic: /fmu/out/log_message
       type: px4_msgs::msg::LogMessage

     - topic: /fmu/out/mavlink_log
       type: px4_msgs::msg::MavlinkLog

     - topic: /fmu/out/offboard_control_mode
       type: px4_msgs::msg::OffboardControlMode

     - topic: /fmu/out/battery_status
       type: px4_msgs::msg::BatteryStatus

     - topic: /fmu/out/failure_detector_status
       type: px4_msgs::msg::FailureDetectorStatus

     - topic: /fmu/out/event
       type: px4_msgs::msg::Event
   ```


### Notable Errors with PX4
- **Messages are not published**:  
  By default, the uXRCE-DDS client does not publish additional PX4 topics unless they are explicitly added. To fix this:
  1. Open `src/modules/uxrce_dds_client/dds_topics.yaml`.
  2. Ensure each required topic is listed under the `topics:` key, for example:
     ```
       - topic: /fmu/out/telemetry_status
         type: px4_msgs::msg::TelemetryStatus

       - topic: /fmu/out/log_message
         type: px4_msgs::msg::LogMessage

       - topic: /fmu/out/mavlink_log
         type: px4_msgs::msg::MavlinkLog

       - topic: /fmu/out/offboard_control_mode
         type: px4_msgs::msg::OffboardControlMode

       - topic: /fmu/out/battery_status
         type: px4_msgs::msg::BatteryStatus

       - topic: /fmu/out/failure_detector_status
         type: px4_msgs::msg::FailureDetectorStatus

       - topic: /fmu/out/event
         type: px4_msgs::msg::Event
     ```
  3. Rebuild the firmware so that the DDS client reflects these changes:
     ```bash
     cd px4
     make clean
     make px4_fmu-v5_default
     ```
  4. Restart the flight controller. Verify the new topics appear by running:
     ```bash
     ros2 topic list | grep fmu/out
     ```
  This ensures that the uXRCE-DDS bridge publishes the selected PX4 messages over DDS.

- **uXRCE-DDS client startup fails**:  
  If the `uxrce_dds_client` does not start, check:
  1. The IP and port configuration in `ROMFS/px4fmu_common/init.d/rc.mc_apps`. Confirm it matches your network setup:
     ```
     uxrce_dds_client start -p 8888 -h 10.41.10.1 -n px4_{number}
     ```
  2. That the host IP (`-h`) is reachable from the flight controller. Ping the host from the onboard console.
  3. The DDS domain ID on both the ground station and PX4 side. Mismatch causes discovery failure.
  4. Review the PX4 log for errors:
     ```bash
     dfu-util -U /fs/microsd/log/ulog/*.ulg
     ```
  Correct any network or domain mismatches and rebuild if necessary.


## Notes on UXRCE_DDS
UXRCE_DDS (Micro XRCE-DDS) is the lightweight DDS middleware used to connect PX4 and ROS 2 over UDP. In this project, it allows each PX4 flight controller to publish and subscribe to ROS 2 topics via a uXRCE agent running on the ground station or companion computer.


## Uploading Swarm Software to Drones

1. **Power on all drones.**
2. **Launch the GUI** and wait for drones to appear.
3. **Select a drone** and click **Upload Software**.
4. **Monitor progress** in the GUI until transfer completes.

## Run Launch

1. **Power on the Drones.**
2. **Launch the GUI** and wait for all drones to appear on the home page.
3. **Launch QGroundControl** and confirm all drones are visible.
4. **Start the User Input Node:**
   ```bash
   ros2 run user_input user_input_node
   ```
   > Future integration into the GUI is planned; see the `user_input` branch for reference.
5. **Navigate to the Launch Page:**
   - Select desired drones.
   - Verify configuration files and upload new ones if needed.
6. **Launch the Drones:**
   - Initiate launch and wait for confirmation that drones are ready.
7. **Target Drone (Optional):**
   - On the Home page, select a target drone and click **Launch Target**.
8. **Control Commands:**
   - Type `start` to begin tracking and launch drones.
   - Type `land` to stop and land drones.
   - Type `return` to command drones to return home.
   - Type `pause` to hold drones in their current position.