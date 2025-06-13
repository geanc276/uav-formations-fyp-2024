# The Data Dissemination Protocol 

## Building

From this directroy (`uav-formations-fyp-2024/swarm_ws`), run the below commands

```colcon build```

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
will work.

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