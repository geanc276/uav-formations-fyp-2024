#! /usr/bin/env python

import rospy
import rosbag
import sys
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Float64
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

from typing import List
import json
import numpy as np

from constraint import *
from dronepos import DronePos

user_input = ''


def user_input_cb(msg):
    global user_input
    user_input = msg.data


with open(sys.argv[1], 'r') as f: 
    config_data = json.load(f)
    name = config_data['name']
    rospy.init_node(f"{name}_main_ctrl")
    state_pub = rospy.Publisher(f"/{name}_main_ctrl/state", String, queue_size=10)
    rospy.wait_for_service(f"/{name}/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy(f"/{name}/mavros/set_mode", SetMode)

user_input_sub = rospy.Subscriber('/user_input/state', String, user_input_cb)


def main(): 
    while not rospy.is_shutdown():
        message = String()
        message.data = user_input
        state_pub.publish(message)

if __name__ == "__main__": 
    main()








