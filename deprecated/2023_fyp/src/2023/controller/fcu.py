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
import fcu

class fcu:
    def __init__(self, drone_name):
        self.drone_name = drone_name
        self._drone_pos = rospysimple.SubscriberSimple(f"{self.drone_name}/mavros/local_position/pose", PoseStamped)
        self._drone_dir = rospysimple.SubscriberSimple(f"{self.drone_name}/mavros/local_position/pose", PoseStamped)
        self._drone_state = rospysimple.SubscriberSimple(f"{self.drone_name}/mavros/state", State)
        self._mode = rospysimple.ServiceSimple(f"/{self.drone_name}/mavros/set_mode", SetMode)
        self._armed = rospysimple.ServiceSimple(f"/{self.drone_name}/mavros/cmd/arming", CommandBool)
        self._takeoff_height = rospysimple.ParamSimple(self.drone_name)

    @property
    def drone_pos(self):
        return self._drone_pos.get().pose.position

    @property
    def drone_dir(self):
        return self._drone_dir.get().pose.orientation

    @property
    def mode(self):
        return self._drone_state.get().mode

    # Retry on failure to set, but only once
    @mode.setter
    def mode(self, new_mode):
        if new_mode != self.mode:
            mode_cmd = SetModeRequest()
            mode_cmd.custom_mode = new_mode
            if not self._mode.call(mode_cmd).mode_sent:
                rospy.sleep(0.5)
                self._mode.call(mode_cmd)

    @property
    def armed(self):
        return self._drone_state.get().armed

    @armed.setter
    def armed(self, arming_value):
        if arming_value != self.armed:
            arm_cmd = CommandBoolRequest()
            arm_cmd.value = arming_value
            if not self._armed.call(arm_cmd).success:
                rospy.sleep(0.5)
                self._armed.call(arm_cmd)

    @property
    def takeoff_height(self):
        if self._takeoff_height.pull():
            if self._takeoff_height.get("MIS_TAKEOFF_ALT"):
                return self._takeoff_height.get("MIS_TAKEOFF_ALT")
        return False
    
    @takeoff_height.setter
    def takeoff_height(self, height):
        if self._takeoff_height.set("MIS_TAKEOFF_ALT", float(height)):
            if not self._takeoff_height.push():
                rospy.loginfo("Parameter push failed")
        


