""" This file extracts relevant data from bagfiles obtained in simulation and from real life
    flight tests. Data is then plotted """

import rosbag
import numpy as np
import os
import re
import matplotlib.pyplot as plt
import argparse

from controller.gps import GPSCoord
from visualize import *


def disp_unique_topics (bag):
    """ prints a list of the unique topics being run """

    unique_topics = set()
    for topic, _, _ in bag.read_messages():
        unique_topics.add(topic)

    for topic in unique_topics:
        print(topic)


def get_initial_pos(uav_list, bag, leader):
    """ gets initial positions of all drones """

    gps_initial_pos =  {uav: None for uav in uav_list}
    global_initial_pos = {uav: ([], []) for uav in uav_list}

    for topic, msg, _ in bag.read_messages(topics=[f"/{uav}/mavros/global_position/global" for uav in uav_list]): 
        uav = topic.split('/')[1]
        if gps_initial_pos[uav] is None:
            print(f"{uav}: {msg.latitude}, {msg.longitude}, {msg.altitude}")
            uav_pos = GPSCoord(msg.latitude, msg.longitude, msg.altitude)
            gps_initial_pos[uav] = uav_pos

        if all(value is not None for value in gps_initial_pos.values()):
            break
            
    for key in global_initial_pos.keys():
        global_initial_pos[key][0].append(gps_initial_pos[leader].x_distance(gps_initial_pos[key]))
        global_initial_pos[key][1].append(gps_initial_pos[leader].y_distance(gps_initial_pos[key]))

    return global_initial_pos


def playback (filepath, uav_list):
    """ takes bag files for data from each drone and processes data """

    leader = uav_list[0]

    if not os.path.exists(filepath):
        print("Error: file not found")
        return
    
    with rosbag.Bag(filepath, 'r') as bag:
        # disp_unique_topics (bag) # uncomment to display list of all unique ROS topics 
        state_changes = []
        for topic, msg, timestamp in bag.read_messages(topics=['/user_input/state']): 
            state_changes.append((msg.data, timestamp.to_sec() - bag.get_start_time()))
    

        # get command velocities
        cmd_vels = {uav: ([], []) for uav in uav_list}
        cmd_omega = ([], []) 
        for topic, msg, timestamp in bag.read_messages(topics=[f"/{uav}/mavros/setpoint_velocity/cmd_vel" for uav in uav_list]): 
            uav = topic.split('/')[1]
            vel = np.linalg.norm([msg.twist.linear.x, msg.twist.linear.y])
            omega = msg.twist.angular.z
            cmd_vels[uav][0].append(timestamp.to_sec() - bag.get_start_time())
            cmd_vels[uav][1].append(vel)
            if uav == uav_list[0]:
                omega = msg.twist.angular.z 
                cmd_omega[0].append(timestamp.to_sec() - bag.get_start_time())
                cmd_omega[1].append(omega)
            

        # get real velociites 
        real_vels = {uav: ([], []) for uav in uav_list}
        real_omega = ([], [])
        for topic, msg, timestamp in bag.read_messages(topics=[f"/{uav}/mavros/local_position/velocity_local" for uav in uav_list]): 
            uav = topic.split('/')[1]
            vel = np.linalg.norm([msg.twist.linear.x, msg.twist.linear.y])
            real_vels[uav][0].append(timestamp.to_sec() - bag.get_start_time())
            real_vels[uav][1].append(vel)
            if uav == uav_list[0]:
                omega = msg.twist.angular.z
                real_omega[0].append(timestamp.to_sec() - bag.get_start_time())
                real_omega[1].append(omega)


        # get leader acceleration 
        acc = ([], [], [])
        for topic, msg, timestamp in bag.read_messages(topics=[f"/{leader}/mavros/imu/data"]): 
            time = timestamp.to_sec() - bag.get_start_time()
            raw_acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y])

            acc[0].append(time)
            acc[1].append(raw_acc[0])
            acc[2].append(raw_acc[1])


        # get edge errors
        topics = bag.get_type_and_topic_info()[1].keys()
        edge_error_regex = re.compile("\\/uav\\d*\\/edge_errors\\/distance.*$")
        edge_error_topics = list(filter(edge_error_regex.match, topics))
        linear_edge_errors = {edge: ([], []) for edge in edge_error_topics} 
        for topic, msg, timestamp in bag.read_messages(topics=edge_error_topics): 
            error = msg.data[0]
            linear_edge_errors[topic][0].append(timestamp.to_sec() - bag.get_start_time())
            linear_edge_errors[topic][1].append(error)

        
        # get crash avoidance
        crash_avoid_regex = re.compile("\\/uav\\d*\\/edge_errors\\/crash_avoidance.*$")
        crash_avoid_topics = list(filter(crash_avoid_regex.match, topics))
        if crash_avoid_topics == []: 
            crash_avoid_topics.append('')   # Make sure there is something in the list
        crash_avoid_errors = {uav: ([], []) for uav in crash_avoid_topics}
        for topic, msg, timestamp in bag.read_messages(topics=crash_avoid_topics): 
            error = msg.data[0]
            crash_avoid_errors[topic][0].append(timestamp.to_sec() - bag.get_start_time())
            crash_avoid_errors[topic][1].append(error)        


        # get altitudes
        altitude_regex = re.compile("\\/uav\\d*\\/edge_errors\\/altitude.*$")
        altitude_topics = list(filter(altitude_regex.match, topics))
        altitudes = {uav: ([], []) for uav in altitude_topics} 
        for topic, msg, timestamp in bag.read_messages(topics=altitude_topics): 
            error = msg.data[0]
            altitudes[topic][0].append(timestamp.to_sec() - bag.get_start_time())
            altitudes[topic][1].append(error)


        # get angular errors
        topics = bag.get_type_and_topic_info()[1].keys()
        edge_error_regex = re.compile("\\/uav\\d*\\/edge_errors\\/angular.*$")
        edge_error_topics = list(filter(edge_error_regex.match, topics))
        angular_edge_errors = {edge: ([], []) for edge in edge_error_topics} 
        try:
            for topic, msg, timestamp in bag.read_messages(topics=edge_error_topics): 
                error = msg.data[0]
                angular_edge_errors[topic][0].append(timestamp.to_sec() - bag.get_start_time())
                angular_edge_errors[topic][1].append(error)
        except:
            print("no angular constraints")
            

        # get drone paths
        initial_pos = get_initial_pos (uav_list, bag, leader) 
        initial_local_pos = {uav: None for uav in uav_list}
        path = {uav: ([], [], []) for uav in uav_list}
        for topic, msg, timestamp in bag.read_messages(topics=[f"/{uav}/mavros/local_position/pose" for uav in uav_list]): 
            uav = topic.split('/')[1]
            if initial_local_pos[uav] is None: 
                initial_local_pos[uav] = [msg.pose.position.x, msg.pose.position.y]

            x = msg.pose.position.x 
            y = msg.pose.position.y
            path[uav][0].append(timestamp.to_sec() - bag.get_start_time())
            path[uav][1].append(x+initial_pos[uav][0][0]-initial_local_pos[uav][0])
            path[uav][2].append(y+initial_pos[uav][1][0]-initial_local_pos[uav][1])



        # plot data 
        # COMMAND VELOCITY
        plt.figure()
        plot_uav_velocity(cmd_vels, "command")
        plot_state_changes(state_changes)

        # MEASURED VELOCITY
        plt.figure()
        plot_uav_velocity(real_vels, "measured")
        plot_state_changes(state_changes)

        # LEADER ANGULAR VELOCITY REAL VS MEASURED
        plt.figure ()
        plot_leader_angular_velocity (cmd_omega, real_omega)

        # LEADER ACCELERATION 
        plt.figure ()
        plot_leader_acc (acc)

        # LINEAR EDGE ERRORS
        plt.figure()
        plot_formation_errors(linear_edge_errors, "linear")
        plot_state_changes(state_changes)

        # CRASH AVOIDANCE 
        plt.figure()
        plot_crash_avoidance(crash_avoid_errors)
        plot_state_changes(state_changes)

        # ALTITUDE ERROS
        plt.figure()
        plot_formation_errors(altitudes, "altitude")
        plot_state_changes(state_changes)

        # UAV PATHS
        plt.figure()
        plot_uav_paths(path)

        # ANGULAR EDGE ERRORS
        try:
            plt.figure()
            plot_formation_errors (angular_edge_errors, "angular")
            plot_state_changes(state_changes)
        except:
            print("\n")

        plt.show()

        # ANIMATE FLIGHT PATH
        visualize_flight_2D (path)

        bag.close()


if __name__ == "__main__":
    """ exapmle use: python3 analytics.py bagfiles/2023-08-02-11-07-26.bag uav0 uav2 uav3 (first drone
        is assumed to be leader) """

    parser = argparse.ArgumentParser(description="Anaylzing bagfile from UAV flight test")
    parser.add_argument("filepath", help="Path to bag file")
    parser.add_argument("uav_list", nargs="+", help="list of uavs from flight test")

    args = parser.parse_args ()
    playback (args.filepath, args.uav_list)
        



