# Drone Setup Procedure

In case of errors in the drones, or addition of new drones to the formation, some drones may need their hardware and software re-configured. 

## Raspberry Pi Configuration

The Raspberry Pi companion computer is reponsible for running most of the code running in this project. 

### Installing OS and packages

1. Acquire an SD card (64GB is recommended). 
2. Download and install rpi-imager software from the [Raspberry Pi Website](https://www.raspberrypi.com/software/). 
3. Insert SD card into SD card reader of a computer. 
4. Run the rpi-imager on this computer, select "Choose OS > Other general-purpose OS > Ubuntu > Ubuntu Server 20.04.5 LTS (64-bit)". 
5. It is recommended to configure the OS before flashing. Click the cog in the bottom write to open the settings menu. 
    - Set the hostname to the name of the drone (e.g. drone-1)
    - Select "Enable SSH", "Use password authentication" 
    - Set username and password to preferred (e.g. ubuntu, password)
    - Configure wireless LAN to autoconnect. Not technically necessary if using an ethernet connection but if could be helpful if using a wireless connection to install packages. 
6. Select "Write" to begin the flashing process. 
7. Remove SD card from computer and insert into the Raspberry Pi. 
8. Connect keyboard and mouse (via USB), power supply (USB-C) and monitor (via micro-HDMI cable) to the Raspberry Pi. Turn on the power supply and wait for Raspberry Pi to boot, then login with previous credentials. 
1. Assuming network is active, run `sudo apt-get update`
1. (Desktop Environment: Optional but recommended) Run `sudo apt-get install ubuntu-desktop`, then reboot. 
1. Refer to [the readme](../README.md) for instructions as to how to install the packages. Note that you do not need to install the PX4 sitl on the Raspberry Pis as they will not need to run this software. 

#### Management Interface

If you intend to transfer software using the management interface, there are a few more steps that need to be performed.

1. Install the docker engine on the system. Instructions can be found at https://docs.docker.com/engine/install/
2. Copy over ssh keys with 
```bash
ssh-copy-id -i src/2023/management_interface/backend/certs/id_rsa ubuntu@{drone_ip}
ssh-copy-id -i src/2023/management_interface/backend/certs/id_rsa root@{drone_ip}
```
3. Assign the drone a name by creating a file. `echo uav0 > drone_name`
4. Transfer the webserver to the drones, and run it with 
```bash
docker run csse-sdff.canterbury.ac.nz/web-server -p 5000:5000 --restart=always 
```

### Drone number in MAVROS

Ensure that the drone number configured matches the drone number configured in QGroundControl for the drone. In the launch file (for example [drone 1](../src/2023/launch/px4_uav1.launch)), this parameter is the value of `<arg name="tgt_system" default="1" />`. In QGroundControl, it is the value of the parameter `MAV_SYS_ID`. 

You can check the connection works by running the MAVROS node when the PX4 is connected via USB. The value of `/uav<uav_num>/mavros/state/connected` should be true; if it is false instead there is an issue. 


### Network configuration

1. Open the `/etc/hosts` file on the drone. Add the Static IP of the host computer and all the other drones, e.g. 
```
192.168.0.10    icos-m3401QA
192.168.0.16    drone-1
192.168.0.17    drone-2
192.168.0.18    drone-3
```
2. Open the `.bashrc` file for each drone, and add the hostname of the ROS master laptop (e.g. if the hostname is icos-m3401QA)
```
export ROS_MASTER_URI='http://icos-m3401QA:11311'
```

## Router configuration

TP-Link Archer AX1500 router is being used in this project, but most routers should have this option. Once the drones and laptop are connected to the router, assign them all a static IP. This ensures that the configuration set in the `/etc/hosts` file previously is persistent across reboots. 

Note that the TP-Link Archer AX1500 requires a 12V power supply. A 3S battery should be sufficient for this purpose; do not let it drop below 11.2V or connection dropout and power saving modes may be triggered. 

## Physical Hardware

Using the other drones as a reference, verify that the physical hardware is correct. If any problems occur, refer to the manuals for the drones and individual components. 
