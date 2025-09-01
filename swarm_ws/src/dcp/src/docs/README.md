# Data Dissemination Protocol

## Launching the protocol (In isolation)

1. Run the command `sudo bash ubuntu_sim_ros_noetic.sh` in a terminal from the root directory of the project
2. Next run the command `sudo su` to use the superuser terminal
3. Then navigate from the root directory to `/src/catkin_ws`
4. Run the command `catkin clean` and enter "y" when prompted
5. Run the command `catkin build` followed by `catkin build data_dissemination`
6. Run the command `catkin build data_dissemination` until the package builds successfully
7. Next run the command `source devel/setup.bash`
8. Followed by navigating to `/src/catkin_ws/src/data_dissemination/src`
9. Now run the command `DRONE_NAME=uav1 roslaunch launchDataDisseminationProtocol.launch` to start a node of the Data Dissemination Protocol. 
10. (Note) Following these steps on multiple devices on the same network will allow the protocols to communicate in isolation.
11. (Note) The `DRONE_NAME=uav1` sets a local environment variable for the drone name which the Beaconing protocol looks for on startup

## Launching the protocol (On drone platform)

1. Copy the most recent Docker image to the drone platform using the management interface.
2. Execute the image on the drone.
3. (Note) Follow the steps provided in the management interface documentation for more detail

## Beaconing Protocol

The Beaconing Protocol manages the sending and receiving of packets between drones in the network. The Beaconing Protocol (BP) sends ethernet broadcast packets at 10 Hz and listens for other drones broadcast beacons. 

## State Reporting Protocol

The State Reporting Protocol (SRP) interfaces with the BP to send the drones state data to other drones in the formation. State data refers to the position, speed, rotation and direction of a drone which is transported in BP beacons to neighbour drones.

## Neighbour Table

The Neighbour Table exists as a part of the SRP. When the BP receives a beacon with state data included, it passes it on to the SRP. The SRP then passes the data on to the Neighbour Table which stores the data in a memory mapped file. 

The memory mapped file is provided by the Boost libraries and the documentation exists [here](https://boost.org/doc/libs/1_83_0/doc/html/interprocess/sharedmemorybetween1_64_0processes.html#interprocess.sharedmemorybetweenprocesses.mapped_file).

## Robot Operating System (ROS)

The Data Dissemination Protocol is designed as a pair of ROS nodelets. See nodelet documentation [here](http://wiki.ros.org/nodelet). The nodelets are launched using the `roslaunch` command on the relevant "launchDataDisseminationProtocol.launch" file. Each nodelet is comprised on a wrapper class which takes the native C++ class and integrates it with the ROS setup process (see srpClientNodelet.cpp & beaconingClientNodelet.cpp), these wrappers are what is referenced by the .launch file while the wrapper classes reference native C++ classes (see beaconingClient.cpp & srpClient.cpp). The nodelet wrapper classes also handle the creation of ROS publishers and subscribers which the BP and SRP use for zero copy cost interprocess communication. The process followed for the development of the nodelet wrapper classes is loosely based on [this example](https://www.clearpathrobotics.com/assets/guides/melodic/ros/Nodelet%20Everything.html).

## Catkin

Catkin is the automated build process included with ROS and is used to build this protocol. The build process is defined in the CMakeLists.txt file as well as what resources to include and all external dependencies. More information about the Catkin build process can be found [here](http://wiki.ros.org/catkin/Tutorials).

## ROS Messages

ROS messages are defined in `.msg` files which are contained in the `data_dissemination_msgs` directory. These define message formats for sending data using ROS publishers and subscribers. These messages are generated into C++ headers for interfacing with the fields in the message. Currently, only the ReceivePayloadIndication, RegisterProtocolRequest and TransmitPayloadRequest messages are used and built. However, all specified BP message types are defined in the `msg` directory. More information on ROS msg can be found [here](http://wiki.ros.org/msg).

## Neighbour Table Reader

This is a python interface for the managed_mapped_file used by the Neighbour Table for storing neighbour drone entries. It can only read from the Neighbour Table and has its access controlled by the Neighbour Table. At the point this was developed, the Python Shared Memory interface was not working as expected so Python mmap was used for file access. This interface is mainly for debugging the neighbour table contents, although it was intended to provide neighbour table contents to the Python control loop.