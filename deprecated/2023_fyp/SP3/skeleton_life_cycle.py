#! /usr/bin/env python

import rospy
import sys
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Float64, Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

import json
import statemachine

# Sets the rate at which the eventGenerator() function gets called - generates events to cause state machine to change and perform actions
EVENT_LOOP_RATE_HZ = 10

# Definition of EFSM for drone life cycle
class LifeCycleMachine(statemachine.StateMachine):
    #
    # First block reserved for statemachine.State() declarations
    #
    
    #
    # Second block reserved for transition definitions
    #
    
    def __init__(self):
        with open(sys.argv[1], 'r') as f:
            self.config_data = json.load(f)
            self.drone_name = self.config_data['name']
        #
        # Variables that are needed 'globally' within the EFSM, subscribers/publishers/clients etc
        # should all be declared and initialised here 
        #
        super.__init__()
        
    def eventGenerator(self, state):
        # Event triggers defined in this function as part of if/elif statement
        return
    
    #
    # Rospy subscriber callback definitions placed here
    #
    
    #
    # General action (on all transitions/states) definitions here
    #
    
    #
    # Specific actions (events or states) definitions here
    #

def main():
    # Initialise the state machine, rospy node, and periodically call the eventGenerator() function
    drone_life_cycle = LifeCycleMachine()
    rospy.init_node(f"{drone_life_cycle.drone_name}/life_cycle")
    event_loop = rospy.Rate(EVENT_LOOP_RATE_HZ)
    while not rospy.is_shutdown():
        drone_life_cycle.eventGenerator()
        event_loop.sleep()

if __name__ == "__main__":
    main()