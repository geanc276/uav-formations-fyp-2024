# Flight Test Procedure
## Before Leaving WRC
1. Ensure the latest code is pushed to all drones with the relevant configuration json files
2. Charge drone, router, and transmitter batteries
3. Collect Hi-Vis, RC transmitters, USB telemetry radio, charged batteries, propeller bag (with correct number of white/black props and spares), battery alarm, and router. Put these items in the box
4. Take drones, table, laptops, and box out to the test site

## In the field
1. Notify security if testing on Ilam Fields
2. Put on propellers 
3. Start router and ensure WLAN is enabled (can connect from laptop)
4. Plug in batteries on drone
5. Start `roscore` on laptop
6. Start `QGroundControl` on laptop
7. Adjust drone ground positions and headings if necessary
8. SSH into each drone and run (keep this window open):
```bash
roslaunch uav-formations/src/2023/controller/launch/px4_uav<uav_number>.launch
```
9. SSH into each drone which requires automatic control. Navigate to the `uav-formations/src/2023/controller/` directory and run the following two scripts: 
```bash
python3 generatedController.py configs/<config_file>
python3 controller.py configs/<config_file>`
```
10. Start the following programs on the host laptop. 
```bash
python3 uav-formations/src/2023/controller/user_input.py
./uav-formations/src/2023/rosbag.sh
```
11. Control person vocalises that final pre-launch commands have been run
12. Control person asks each pilot to perform pre-flight checks: arm, killswitch, un-killswitch, disarm
13. Control person asks if pilots are ready, gives launch countdown. In terminal window running `user_input.py` script type `LAUNCH`

## Mission Abort Procedure
1. If mission controller or any drone pilot believes a drone or the swarm have diverged significantly from the intended mission, vocalise intention to abort mission
2. If there is immediate risk of a drone collision pilots should throw kill switch
3. If there is no immediate risk - pilots set arm switch, and toggle mode switch: position->hold->position (up->middle->up) to regain manual control and land

## After Test
1. Pack up all equipment, noting batteries that require charging/discharging, and ensuring all items are accounted for
2. If at Ilam Fields, notify security that testing has finished
3. Owner of mission control laptop responsible for uploading bag files to git; videos taken to be uploaded to one drive 

## Viewing Results

Results are automatically rendered as soon as the bag logging script is closed. However, bagfile data can also be plotted at any time using the following commnad from the uav-formation/src/2023 directory.

```bash
python3 analytics.py bagfiles/<bagfile> <ledaer_drone> <follower_drone1> ... <follower_droneN>
```
