# Simulation Procedure

1. Ensure that the virtual drone target and simulation build options are selected by navigating to the uav-formation-fyp-*year*/swarm_ws and entering the following commands:
```bash
export SIM_BUILD=1
export TARGET=virtual
```
2. Build the software from the uav-formation-fyp-*year*/swarm_ws directory using the following command `colcon build`.
3. Open multiple terminal windows.
4. Open QGroundControl, this must be running to launch the drones in simulation. This can be done by running: 
```bash
cd ~/
./QGroundControl.AppImage
```
5. Launch Gazebo simulator using the following command for each drone within the swarm. Ensure you are in the uav-formation-fyp-*year*/px4-autopilot directory, replacing *X*, *Y*, and *DRONE_NUMBER* with values and run:
```bash
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 PX4_GZ_MODEL_POSE="*X*,*Y*" ./build/px4_sitl_default/bin/px4 -i *DRONE_NUMBER*
``` 
6. Navigate into the launch directory - uav-formation-fyp-*year*/swarm_ws and run:
```bash
python3 launch_swarm_sim.py 
```
Selecting the swarm configuration to be used from the dropdown menu.


## Instructions to run simulations on a university computer 

1. Open Software Center
2. Install and open Oracle VirtualBox
3. Download Ubuntu 22.04.5 by downloading the ubuntu-22.04.5-desktop-amd64.iso 4.1G file.
4. Make a new virtual machine called 'ubuntu' and save in the C drive. It should be of type Linux and Version Ubuntu (64-bit)
5. Set RAM to 16GB. Select create a virtual hard disk now and click create. 
6. Choose VDI (VirtualBox Disk Image) for hard disk file type and click next. 
7. Choose dynamically allocated and click next.
8. Set hard-disk storage to 50GB and click create. 
9. Now navigate to settings in the newly created virtual machine... 
10. Under System tab navigate to Processor tab - set the number of processors to 6 cores. 
11. Under display set video memory ot 128MB (max)
12. Still under display 'monitor count' select 2 for double monitors 
13. Under storage, under Controller: IDE, click the 'Empty' option. For the optical drive, select the .iso file previously downloaded.
14. To enable fullscreen select Devices -> Insert Guest Additions CD Images. Then press adjust window size under view in Oracle VM viertualbox toolbar. You may have to restart the VM for this to work. 
15. Install git `sudo apt install git`
16. Install VS Code.
17. Clone repository and complete the instructions found on README.md of this repository. 
18. See simulation procedure above. 
