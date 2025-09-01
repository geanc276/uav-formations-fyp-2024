# The Data Dissemination Protocol (DDP)

## Building

### First Build
After following the setup/install instructions in the main README.md run the below command once.

```
pip install setuptools==58.0.4
```

### Normal Builds
From the directory (`uav-formations-fyp-2024/swarm_ws`), run the below commands

```
colcon build
```

## Running/Launching
After building, still in the same directory (`uav-formations-fyp-2024/swarm_ws`), run the below: 

It is required to start this way as the Beaconing Protocol code requires `sudo` permissions to broadcast the packets.

```sudo bash -c "source install/setup.sh; export DRONE_NAME='uav1'; ros2 launch dcp launchDataDisseminationProtocol.xml"```

This will run the launch file. If only one terminal session is open the protocol won't do much at this stage.
To see the protocol fully working create another instance on a different computer (needs different MAC address).
If you create another instance on the same computer the packets will be ignored by the protocol and so appear to be doing nothing.
Make sure to change the `DROME_NAME` parameter or else the packets will also get ignored.

Instead of using a separate computer you can manually set the MAC via an environmental variable (similar to `DRONE_NAME`). 
This way you can create multiple sessions on 1 computer. The MAC's do not have to be valid, for example `"aa:aa:aa:aa:aa:aa"`
will work. You will have to implement this!

## Launch File
Located at `uav-formations-fyp-2024/swarm_ws/src/dcp/launch`. The launch file contains the `xml` required to launch the Data Dissemination Protocol.
Each sub-protocol currently has its own node that is run in a separate process. Currently, the SRP protocol is commented out. This is because
when both VarDis and SRP are working at the same time only the SRP protocol packets are processed/received. It is currently not known
why the SRP packets conflict with the VarDis packets.

## Colcon
Colcon is the automated build process included with ROS2 and is used to build this protocol. It uses the CMakeLists.txt file.

## ROS2 Messages
The ROS2 messages that are used by the subscribers and publishers are defined in `dcp_msgs/msg`. 
These define message formats for sending data using ROS2 publishers and subscribers. These messages are generated into C++ headers for interfacing with the fields in the message.

## Code

The `CMakeLists.txt` associated with the DDP is located at `uav-formations-fyp-2024/swarm_ws/src/dcp`. This will be referred to as the `root`.

Within the `src` folder of the `root` directory will be four folders, the folder-protocol associations are listed below.
```
Beaconing Protocol (BP): 
 - beaconing_client
State Reporting Protocol (SRP):
 - srp_client
 - neighbour_table
Variable Dissemination Protocol (VarDis):
 - variable_dissemination
```

All the header (`.h`) files are located in the `include\dcp` directory from the `root`

## Architecture

Each protocol has two main `.cpp` files. Named `<protocolName>Client` and `<protocolName>ClientNode`. The latter is what ROS2 uses to launch each
node, hence the name. This is where `timers`, `callbacks`, `subscribers & publishers` etc are located.

The former contains the code that actually performs the actions for the corresponding protocol. Such as broadcasting beacons, updating the neighbour table
or updating the real-time database. 

## Next Steps

VarDis and the State Reporting Protocol (SRP) currently do not work at the sametime. The SRP packets will be received and processed but the VarDis packets will not. It is unclear why.

Currently SRP is commented out in the launch file so you must uncomment this to see it.