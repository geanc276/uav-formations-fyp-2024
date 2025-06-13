#! /usr/bin/env python
import rospy
import sys
from std_msgs.msg import Header, Float64MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Float64, Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, ParamSet, ParamSetRequest, ParamGet, ParamGetRequest, ParamPull, ParamPullRequest, ParamPush, ParamPushRequest
import json
import statemachine
import numpy as np
import rospysimple


class SubscriberSimple:
    def __init__(self, topic, msg_type):
        self._value_ = msg_type()
        self._sub_ = rospy.Subscriber(topic, msg_type, callback=self._cb_)
        rospy.loginfo(f"Created subscriber for {topic}")

    def _cb_(self, msg):
        self._value_ = msg

    def get(self):
        return self._value_


class PublisherSimple:
    def __init__(self, topic, msg_type, msg_queue=5):
        self._pub_ = rospy.Publisher(topic, msg_type, queue_size=msg_queue)

    def push(self, value):
        self._pub_.publish(value)


class ServiceSimple:
    def __init__(self, topic, srv_type):
        rospy.wait_for_service(topic)
        self._client = rospy.ServiceProxy(topic, srv_type)
    
    def call(self, cmd):
        return self._client(cmd)


class TimerSimple:
    def __init__(self, period_secs_float, oneshot=True, initial_status=False):
        self.initial_status = initial_status
        self._status = self.initial_status
        self.period = period_secs_float
        self.oneshot = oneshot
        self._oneshot_flag = False
        self._last_time = 0
        self._stop_time = 0
        self.stopped = True

    def status(self):
        if (rospy.get_time() - self._last_time) > self.period and not self._oneshot_flag:
            self._status_cb()
            self._last_time = rospy.get_time()
            if self.oneshot:
                self._oneshot_flag = True
        return self._status
    
    def _status_cb(self):
        if self._status:
            self._status = False
        else:
            self._status = True

    def start(self):
        self._last_time = rospy.get_time() - self._stop_time
        self.stopped = False

    def stop(self):
        self._stop_time = rospy.get_time() - self._last_time
        self.stopped = True

    def reset(self):
        self._status = self.initial_status
        self._stop_time = 0
        self._last_time = rospy.get_time()
        self.stopped = True
        self._oneshot_flag = False


class ParamSimple:
    def __init__(self, drone_name):
        self.drone_name = drone_name
        self._pull_client = rospysimple.ServiceSimple(f"/{self.drone_name}/mavros/param/pull", ParamPull)
        self._push_client = rospysimple.ServiceSimple(f"/{self.drone_name}/mavros/param/push", ParamPush)
        self._get_client = rospysimple.ServiceSimple(f"/{self.drone_name}/mavros/param/get", ParamGet)
        self._set_client = rospysimple.ServiceSimple(f"/{self.drone_name}/mavros/param/set", ParamSet)

    # Pull function to refresh ros cache of PX4 parameters
    def pull(self):
        cmd = ParamPullRequest()
        cmd.force_pull = True
        return self._pull_client.call(cmd).success
    
    # Push function (to update PX4 parameters from ros cache)
    def push(self):
        cmd = ParamPushRequest()
        return self._push_client.call(cmd).success
    
    def set(self, param_id, value):
        cmd = ParamSetRequest()
        value_is_int = isinstance(value, int)
        if value_is_int:
            cmd.value.integer = value
            cmd.value.real = 0.0
        elif isinstance(value, float):
            cmd.value.real = value
            cmd.value.integer = 0
        else:
            rospy.loginfo("Parameters can only be set as ints or floats")
            return False
        
        cmd.param_id = param_id
        if self._set_client.call(cmd).success:
            return True
        return False
    
    def get(self, param_id):
        cmd = ParamGetRequest()
        cmd.param_id = param_id
        response = self._get_client.call(cmd)
        if not response.success:
            return False
        else:
            if response.value.integer == 0 and response.value.real == 0.0:
                return 0
            elif response.value.integer == 0:
                return response.value.real
            else:
                return response.value.integer
            