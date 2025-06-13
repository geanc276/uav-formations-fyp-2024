#! /usr/bin/env python
import rospy
import sys
from std_msgs.msg import Header, Float64MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Float64, Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import json
import statemachine
import numpy as np
import rospysimple
import lifecycle


class Cooperative:
    def __init__(self, drone_name, other_drones, topic_name):
        self.drone_name = drone_name
        self.other_drones = other_drones
        self.topic_name = topic_name
        self.list = {
            self.drone_name: rospysimple.PublisherSimple(f"/{self.drone_name}_life_cycle/{self.topic_name}", Bool)}
        for i in range(len(self.other_drones)):
            self.list[self.other_drones[i]] = rospysimple.SubscriberSimple(
                f"/{self.other_drones[i]}_life_cycle/{self.topic_name}", Bool)

    def synchronise(self, value):
        # First, push the local status to the cooperative
        sync = Bool()
        sync.data = value
        self.list[self.drone_name].push(sync)
        for i in range(len(self.other_drones)):
            # If one drone's local cooperative value is false, the cooperation will be blocked
            if not self.list[self.other_drones[i]].get().data:
                sync.data = False
        return sync.data


class EdgeErrorChecker:
    def __init__(self, drone_name):
        self.drone_name = drone_name
        self.convergence_timer = rospysimple.TimerSimple(3)
        self.edge_list = []
        self.edge_error_names = []

    def converged(self, distance_epsilon=0.2, altitude_epsilon=0.2, angular_epsilon=5, fixed_local_epsilon=0.1):
        if self.edge_list != []:
            for i in range(len(self.edge_list)):
                try:
                    # Infrequently an index runtime error gets thrown as a consequence of the way edge errors are generated. This try except block reports this error and allows execution to proceed
                    if 'fixed_local' in self.edge_error_names[i][0]:
                        if abs(self.edge_list[i].get().data[0]) > fixed_local_epsilon:
                            self.convergence_timer.reset()
                            return False
                    if "distance" in self.edge_error_names[i][0]:
                        if abs(self.edge_list[i].get().data[0]) > distance_epsilon:
                            self.convergence_timer.reset()
                            return False
                    if "altitude" in self.edge_error_names[i][0]:
                        if abs(self.edge_list[i].get().data[0]) > altitude_epsilon:
                            self.convergence_timer.reset() 
                            return False
                    if "angular" in self.edge_error_names[i][0]:
                        if abs(self.edge_list[i].get().data[0]) > angular_epsilon:
                            self.convergence_timer.reset()
                            return False
                except IndexError:
                    rospy.loginfo(f"{self.drone_name} index error on data: f{self.edge_list[i].get().data}")
                    return False
                
            if self.convergence_timer.stopped:
                self.convergence_timer.start()
            if self.convergence_timer.status():
                rospy.loginfo(f"{self.drone_name} CONVERGENCE")
                return True # event loop runs at 10 Hz so only move out of rendezvous state after 5 seconds 
            
        else:
            # if the list of edge errors was empty, generate it
            self.edge_list = []
            self.edge_error_names = rospy.get_published_topics(namespace=f"/{self.drone_name}/edge_errors/")
            for i in range(len(self.edge_error_names)):
                self.edge_list.append(rospysimple.SubscriberSimple(self.edge_error_names[i][0], Float64MultiArray))
        
        return False 


class DropoutChecker:
    def __init__(self, drone_name, other_drones):
        self.drone_name = drone_name
        self.other_drones = other_drones
        self.dropout_threshold = 0.2 # seconds: 0.2 equates to about 6 lost packets from MAVROS 
        self.dropout_flag = False 
        self.subscriber_topics = [rospy.Subscriber(f"/{drone_name}/mavros/local_position/pose", PoseStamped, callback=self.dropout_cb())] # also need to check own drone. 

        for drone in other_drones:
            self.subscriber_topics.append(rospy.Subscriber(f"/{drone}/mavros/local_position/pose", PoseStamped, callback=self.dropout_cb()))

    def dropout_cb (self):
        def cb (msg: PoseStamped):
            log_time = msg.header.stamp.to_sec()
            current_time = rospy.Time.now().to_sec()
            
            if (current_time - log_time) > self.dropout_threshold:
                self.dropout_flag = True # communication lost

        return cb

    def dropout (self):
        """ queries to see if drone or any neighbour drones have communication dropout """

        if self.dropout_flag:
            return True # drone has lost communication with at least one neighbour 
        
        return False