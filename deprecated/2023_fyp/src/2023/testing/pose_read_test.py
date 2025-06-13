"""
 * File: pose_read_test.py

 Read the pose of drones and print them
"""

#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import numpy as np
from gps import GPSCoord
from dronepos import DronePos


# List of the drones
drones = ['uav0', 'uav2', 'uav3']
# List of the links between drones we are interested in
links = [('uav0', 'uav2'), ('uav0', 'uav3'), ('uav2', 'uav3')]
# Dictionary containing positions of all the drones
drone_pos = {drone: DronePos() for drone in drones}
# ROS subscribers handling the drone positions
subscribers = []


# Returns a callback function that handles a local position subscriber. 
def make_local_pose_cb(drone_name): 
    def cb(msg: PoseStamped): 
        drone_pos[drone_name].local = msg.pose

    return cb

# Returns a callback function that handles a global position subscriber. 
def make_global_pose_cb(drone_name): 
    def cb(msg: NavSatFix): 
        drone_pos[drone_name].global_pos.lat = msg.latitude
        drone_pos[drone_name].global_pos.long = msg.longitude
        drone_pos[drone_name].global_pos.alt = msg.altitude

    return cb


if __name__ == "__main__":
    rospy.init_node("pose_read")

    # Create a subscriber for the local and global position of each drone
    for drone in drones: 
        subscribers.append(rospy.Subscriber(f"/{drone}/mavros/local_position/pose", PoseStamped, callback=make_local_pose_cb(drone)))
        subscribers.append(rospy.Subscriber(f"/{drone}/mavros/global_position/global", NavSatFix, callback=make_global_pose_cb(drone)))

    
    rate = rospy.Rate(5)
    while not rospy.is_shutdown(): 
        # Print information of each drone
        for drone in drones: 
            rospy.loginfo(f"Drone {drone}\t{str(drone_pos[drone])}")

        # Print information of each link
        for link in links: 
            pos1 = drone_pos[link[0]].global_pos
            pos2 = drone_pos[link[1]].global_pos
            rospy.loginfo(f"{link[0]} to {link[1]}\tX{pos1.x_distance(pos2)}\tY{pos1.x_distance(pos2)}\tZ{pos1.z_distance(pos2)}\tR{pos1.distance(pos2)}\tB{pos1.get_heading(pos2)}")

        # Sleep until the next iteration
        rate.sleep()
