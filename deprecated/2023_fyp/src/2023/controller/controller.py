"""
Controller for a single drone which is executed on the Raspberry Pi. Includes code for the leader drone,
follower drone and control system initializers including reading in formation configuration files. 
"""

#! /usr/bin/env python

import rospy
import sys
from std_msgs.msg import Header, Float64MultiArray, String, Float64
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

from typing import List
import json
import numpy as np
import time

from constraint import *
from leader import *
from dronepos import DronePos

# Initialization of variables
drone_state = State()
controller_state = ''
leader_twist = TwistStamped()
drone_pos = {}
altitude_bias = {}
constraints: List[Constraint] = []
subscribers = []


# Callback for the drone state
def state_cb(msg):
    global drone_state
    drone_state = msg

def ctrl_state_cb(msg):
    global controller_state
    controller_state = msg.data

# Returns a callback function that handles a local position subscriber. 
def make_local_pose_cb(drone_name): 
    def cb(msg: PoseStamped):
        drone_pos[drone_name].local = msg.pose
        drone_pos[drone_name].local.position.z -= altitude_bias[drone_name]
    return cb


# Returns a callback function that handles a global position subscriber. 
def make_global_pose_cb(drone_name): 
    def cb(msg: NavSatFix):
        drone_pos[drone_name].global_pos.lat = msg.latitude
        drone_pos[drone_name].global_pos.long = msg.longitude
        drone_pos[drone_name].global_pos.alt = msg.altitude
    return cb


# Returns a callback function that handles a heading subscriber. 
def make_heading_cb(drone_name): 
    def cb(msg: Float64):
        drone_pos[drone_name].heading = msg.data
    return cb


# Callback function that handles a setpoint velocity subscriber. 
def setpoint_velocity_cb(msg: TwistStamped):
    global leader_twist
    leader_twist = msg


# Callback function that handles the altitude bias on takeoff for drones
def altitude_bias_cb(drone_name):
    def cb(msg: Float64):
        altitude_bias[drone_name].local.z = msg
    return cb


# Add drone to directory and make subscribers
def add_drone(drone_name): 
    if drone_name not in drone_pos:
        drone_pos[drone_name] = DronePos()
        altitude_bias[drone_name] = 0
        subscribers.append(rospy.Subscriber(f"/{drone_name}/mavros/local_position/pose", PoseStamped, callback=make_local_pose_cb(drone_name)))
        subscribers.append(rospy.Subscriber(f"/{drone_name}/mavros/global_position/global", NavSatFix, callback=make_global_pose_cb(drone_name)))
        subscribers.append(rospy.Subscriber(f"/{drone_name}/mavros/global_position/compass_hdg", Float64, callback=make_heading_cb(drone_name)))


# read config files and initialize drones to be able to enter main control loop
def init_formation (config_data):
    """ reads config files and creates all objects required by the main control loop """

    name = config_data['name']
    leader = config_data['leader']
    log_edge_error = config_data.get('edge_errors_logging', True)
    max_correction = config_data['max_correction']
    max_angular_correction = config_data['max_angular_correction']
    initial_pos = config_data['local_initial']

    # Initialize the rospy node
    rospy.init_node(f"{name}_ctrl")
    add_drone(name)

    # Add the neighbours (that are listened to by the constraints) 
    # to the list of subscribers and publishers 
    for constraint in config_data['constraints']:
        constr_object = parse_constraint(constraint)
        constraints.append(constr_object)

        # If we need to monitor a new neighbour, add it to the neighbor_pos
        # dictionary
        if 'other' in constraint:
            other = constraint['other']
            if isinstance(other, str):      # Single other drone
                add_drone(other)
            elif isinstance(other, list):
                for other_ in other:
                    add_drone(other_)

        # Create edge error logger
        if log_edge_error: 
            if 'other' in constraint: 
                other = constraint['other']
                if isinstance(other, list): 
                    others = ''.join(f"_{other_[3:]}" for other_ in other)
                else: 
                    others = f"_{other[3:]}"
            else: 
                others = ''
            base_name = f"/{name}/edge_errors/{constraint['type']}_{name[3:]}{others}"
            base_name = base_name.replace('-', '_')

            # Initialize the publisher with the topic
            rospy.loginfo(f"{name} {base_name}")
            constr_object.logger = rospy.Publisher(f"{base_name}", Float64MultiArray, queue_size=10)
            rospy.loginfo(f"Log Object: {constr_object.logger}")

    # Subscribe to the leader's velocity (only if we are not the leader)
    if not leader == name: 
        subscribers.append(rospy.Subscriber(f"/{leader}/mavros/setpoint_velocity/cmd_vel/", TwistStamped, callback=setpoint_velocity_cb))

    return name, leader, initial_pos, max_correction, max_angular_correction


# Control Functions
def clamp_output (vel, max_vel):
    """ takes in command velocity as input and returns a clamped velocity which does not
        exceed the maximum allowable velocity """
    vel_norm = np.linalg.norm(vel)
    if vel_norm > max_vel:
        vel = vel * max_vel / vel_norm

    return vel


def gain_scheduling (constraints, num):
    """ updates gains for constraints which required gain scheduling. Control gains are 
        updated depedning on the state of the drones. This increases the robustness 
        of control. """
    for constraint in constraints:
        try:
            constraint.update_gains (num)
        except Exception as e:
            pass
        

def leader_heading (leader_pos,  linear_vel, max_angular_correction):
    """ updates the leader drone angular velocity using a simple P controller to ensure 
        the leader drones heading matches the direction of the leader velocity. """
    
    current_heading = convert_heading (leader_pos.heading)   
    desired_heading = 180 / np.pi * np.arctan2(linear_vel[1], linear_vel[0])
    heading_error = clamp_heading(desired_heading - current_heading)
    angular_vel = np.array([0,0,0.1*heading_error])

    # clamp heading 
    angular_vel[2] = max_angular_correction * np.sign(angular_vel[2]) if abs(angular_vel[2]) > max_angular_correction else angular_vel[2]

    return angular_vel


def rendezvous_control (name, drone_pos, constraints, max_correction):
    """ control when leader drone is sationary to be run when the formation is being first 
        initialized in the air. The formation will rendezvous at the locatoin of the leader
        drone. 
    """

    total_correction = np.zeros((6,))

    # closed loop feedback control (angular + linear)
    for constraint in constraints:
        if not isinstance (constraint, HoldConstraint):
            correction = constraint.generate_error(drone_pos, drone_pos[name])
            total_correction += correction

    # clamp control (only velocity, not angular components)
    total_correction[:3] = clamp_output (total_correction[:3], max_correction)

    return total_correction


def return_control (name, leader, drone_pos, constraints, max_correction):
    """ control when leader drone is sationary to be run when the formation is being first 
        initialized in the air. The formation will rendezvous at the locatoin of the leader
        drone. 
    """

    total_correction = np.zeros((6,))

    # open loop linear velocity 
    if name != leader:
        linear_correction = np.array([
            leader_twist.twist.linear.x,  leader_twist.twist.linear.y,  leader_twist.twist.linear.z, 
            leader_twist.twist.angular.x, leader_twist.twist.angular.y, leader_twist.twist.angular.z, 
        ])
        total_correction += linear_correction

        # open loop angular velocity 
        r = np.array([
            drone_pos[leader].global_pos.x_distance(drone_pos[name].global_pos),
            drone_pos[leader].global_pos.y_distance(drone_pos[name].global_pos),
            0
        ])
        omega = np.array([0, 0, leader_twist.twist.angular.z])
        angular_correction = np.append(np.cross(omega, r), np.array([0,0,0]))
        total_correction += angular_correction

    # closed loop feedback control (angular + linear)
    for constraint in constraints:
        if not isinstance (constraint, HoldConstraint):
            correction = constraint.generate_error(drone_pos, drone_pos[name])
            total_correction += correction

    # clamp control (only velocity, not angular components)
    total_correction[:3] = clamp_output (total_correction[:3], max_correction)

    return total_correction



def leader_control (name, current_time, max_correction, max_angular_correction):
    """ control of leader drone once the drone formation has formed within some error tolerance
        threshold. Leader control can be manually from the ground using a transmitter. Or, 
        it's position or velocity can be controlled using pre-defined velocity functions - s(t), v(t).
        This function is currently a blackbox for the leader drone path-plannng which can be placed 
        here when it is ready. 
    """

    total_correction = np.zeros((6,))

    # leader drone manual path
    linear_vel = circular_path (current_time)
    total_correction[:3] = linear_vel 
    total_correction[3:6] = leader_heading (drone_pos[name], linear_vel, max_angular_correction) # command leader drone heading to match velocity heading

    # leader altitude constraint
    for constraint in constraints:
        if isinstance (constraint, AltitudeConstraint):
            correction = constraint.generate_error(drone_pos, drone_pos[name])
            total_correction += correction 

    # clamp control (only velocity, not angular components)
    total_correction[:3] = clamp_output (total_correction[:3], max_correction)

    return total_correction


def follower_control (name, leader, drone_pos, constraints, max_correction):
    """" control of leader drones attempting to track the leader drone while remaining in the desired
        formation with minimial error. Follower drones take in the GPS position of itself and it's 
        neighbours in addition to the leader drones motion to output a command velocity which will 
        keep the drones in the desired formation. """
    
    total_correction = np.zeros((6,))

    # open loop linear velocity 
    linear_correction = np.array([
        leader_twist.twist.linear.x,  leader_twist.twist.linear.y,  leader_twist.twist.linear.z, 
        leader_twist.twist.angular.x, leader_twist.twist.angular.y, leader_twist.twist.angular.z, 
    ])
    total_correction += linear_correction

    # open loop angular velocity 
    r = np.array([
        drone_pos[leader].global_pos.x_distance(drone_pos[name].global_pos),
        drone_pos[leader].global_pos.y_distance(drone_pos[name].global_pos),
        0
    ])
    omega = np.array([0, 0, leader_twist.twist.angular.z])
    angular_correction = np.append(np.cross(omega, r), np.array([0,0,0]))
    total_correction += angular_correction

    # closed loop feedback control (angular + linear)
    for constraint in constraints:
        correction = constraint.generate_error(drone_pos, drone_pos[name])
        total_correction += correction

    # clamp control (only velocity, not angular components)
    total_correction[:3] = clamp_output (total_correction[:3], max_correction)

    return total_correction

def hold_control (name, hold_pos, max_correction):
    """ control of leader drones when controller is in the pause state to keep drone in place using the control 
        PID loop. Follower drones still operating using the follower_control to maintain the correct formation"""

    total_correction = np.zeros((6,))

    # hold constraint
    for constraint in constraints:
        if isinstance (constraint, HoldConstraint):
            correction = constraint.generate_error(hold_pos, drone_pos[name])
            total_correction += correction 

    # clamp control (only velocity, not angular components)
    total_correction[:3] = clamp_output (total_correction[:3], max_correction)

    return total_correction


def main(): 


    # create formation
    with open(sys.argv[1], 'r') as f:
        config_data = json.load(f)
        name, leader, initial_pos, max_correction, max_angular_correction = init_formation (config_data)

    #log formation
    rospy.loginfo(constraints)
    rospy.loginfo(subscribers)
    rospy.loginfo(drone_pos)


    
    # Set up misc subscribers and publishers
    state_sub = rospy.Subscriber(f"/{name}/mavros/state", State, callback=state_cb)
    ctrl_state_sub = rospy.Subscriber(f"/{name}_ctrl/state", String, callback=ctrl_state_cb)
    altitude_bias_sub = rospy.Subscriber(f"/{name}_ctrl/altitude_bias", Float64, callback=altitude_bias_cb)
    rospy.wait_for_service(f"/{name}/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy(f"/{name}/mavros/set_mode", SetMode)
    
    local_pos_pub = rospy.Publisher(f"/{name}/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher(f'/{name}/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

    rospy.loginfo("Subscribers and publishers created")

    # wait for Flight Controller connection
    rate = rospy.Rate(15)
    while not rospy.is_shutdown() and not drone_state.connected:
        rate.sleep()

    if name == leader:
        leader_pos = (drone_pos[leader].local.position.x, drone_pos[leader].local.position.y)
        for constraint in constraints:
            if isinstance (constraint, FixedLocalConstraint):
                constraint.position = leader_pos

    start_flag = False  # initialize start time flag to ensure leader drone velocity starts at v(t=0)
    pause_flag = False
    lag_time = 0 
    gains_set = 1
    hold_pos = GPSCoord(0, 0, 0)
    # flight controller loop
    while not rospy.is_shutdown():

        t0 = time.time ()

        total_correction = np.zeros((6,))
        twist = TwistStamped()

        if controller_state == 'MOVE':
            if gains_set != 2:
                gain_scheduling (constraints, 2)
                gains_set = 2
            
            # follower control
            if name != leader:
                total_correction = follower_control(name, leader, drone_pos, constraints, max_correction)
            
            # leader control            
            else:
                if not start_flag:
                    start_flag = True
                    start_time = rospy.Time.now().to_sec()
                current_time = rospy.Time.now().to_sec() - start_time
                total_correction = leader_control (name, current_time-lag_time, max_correction,max_angular_correction)
                pause_flag = False

        elif controller_state == 'RENDEZVOUS':
            # rendezvous control 
            if gains_set != 1:
                gain_scheduling (constraints, 1)
                gains_set = 1 
            total_correction = rendezvous_control (name, drone_pos, constraints, max_correction)

        elif controller_state == "PAUSE":
            if gains_set != 1:
                gain_scheduling (constraints, 1)
                gains_set = 1 

            if name != leader:
                pause_flag = True
                total_correction = follower_control(name, leader, drone_pos, constraints, max_correction)

            elif not pause_flag and name == leader:
                pause_flag = True
                pause_time = current_time - 0.78 #offset to correct time
                hold_pos.lat = drone_pos[name].global_pos.lat
                hold_pos.long = drone_pos[name].global_pos.long
                hold_pos.alt = drone_pos[name].global_pos.alt

            elif name == leader: 
                # record time paused for smooth leader velocity 
                lag_time += rospy.Time.now ().to_sec () - start_time - pause_time
                total_correction = hold_control(name, hold_pos, max_correction)

                pause_time = rospy.Time.now ().to_sec () - start_time

            elif not pause_flag:
                # reset PID controllers (prevent integral wind-up)
                for constraint in constraints:
                    constraint.controller._integral = 0

        elif controller_state == "RETURN":
            if gains_set != 2:
                gain_scheduling (constraints, 2)
                gains_set = 2
            total_correction = return_control (name, leader, drone_pos, constraints, max_correction)

        else:
            # drones landing or taking off 
            pass


        
        # Send correction to drone
        twist.twist.linear.x = total_correction[0]
        twist.twist.linear.y = total_correction[1]
        twist.twist.linear.z = total_correction[2]
        twist.twist.angular.x = total_correction[3]
        twist.twist.angular.y = total_correction[4]
        twist.twist.angular.z = total_correction[5]
        local_vel_pub.publish(twist)
        
        t1 = time.time()
        # rospy.loginfo(f"TIME: {(t1-t0)*1e3} ms")

        # Sleep until the next iteration
        rate.sleep()


if __name__ == "__main__": 
    main()
    