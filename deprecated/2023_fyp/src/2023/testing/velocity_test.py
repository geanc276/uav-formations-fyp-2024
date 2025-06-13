"""
 * File: velocity_test.py

 Proof-of-concept test that shows how the PX4 autopilot can be sent a velocity
 input and how the pose can be read from the PX4. 
"""

#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import numpy as np


current_state = State()
current_pose = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg


def pose_cb(msg): 
    global current_pose
    current_pose = msg




if __name__ == "__main__":
    rospy.init_node("uav1_ctrl")

    state_sub = rospy.Subscriber("/uav1/mavros/state", State, callback = state_cb)
    pose_sub = rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, callback=pose_cb)

    local_pos_pub = rospy.Publisher("/uav1/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('/uav1/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

    rospy.wait_for_service("/uav1/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("/uav1/mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/uav1/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("/uav1/mavros/set_mode", SetMode)


    rospy.loginfo("Subscribers and publishers created")
    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 4
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    for i in range(500): 
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()
                break
        local_pos_pub.publish(pose)

        rate.sleep()

    while (not rospy.is_shutdown()): 


        
        coords2d = np.array([current_pose.pose.position.x, current_pose.pose.position.y])
        rad_sp = 4
        z_sp = 2
        Krp = 1
        omega = 0.5
        rad = np.linalg.norm(coords2d)
        unitv = coords2d / rad

        tang = np.array([-coords2d[1], coords2d[0]])
        vel = Krp * (rad_sp - rad) * unitv + omega * tang
        velo_cmd = TwistStamped()
        velo_cmd.twist.linear.x = vel[0]
        velo_cmd.twist.linear.y = vel[1]
        velo_cmd.twist.linear.z = (z_sp - current_pose.pose.position.z)
        rospy.loginfo(f"{vel}\t{rad}")
        # rospy.loginfo(velo_cmd)
        # rospy.loginfo(f"Coords: {coords2d}\tRad: {rad}\tDir: {unitv}\tAlt: {current_pose.pose.position.z}\tVel: {vel}")

        local_vel_pub.publish(velo_cmd)
        rate.sleep()
        
