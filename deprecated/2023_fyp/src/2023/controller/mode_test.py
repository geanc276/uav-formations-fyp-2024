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


EVENT_LOOP_RATE_HZ = 100


# class that defines an example mission state machine for individual drones
# operating as a swarm or solo. Event generator is currently implemented ad hoc
# in main() - could change in future
class DroneMissionMachine(statemachine.StateMachine):
    "Simple mission for a drone (individual or swarm)"
    # Definitions of states. Be careful to avoid confusing statemachine.State() with State() (mavros_msgs) - may change
    # handling of namespaces to disambiguate in future
    wait_for_mission = statemachine.State(initial=True)
    px4_auto_launch = statemachine.State()
    ctrl_init_formation = statemachine.State()
    ctrl_manoeuvre = statemachine.State()
    px4_hold_in_sky = statemachine.State()
    px4_return_to_base = statemachine.State()

    # Definitions of transitions. Note this is using a fairly simple event = transition model
    # Library has capability to extend this greatly
    launch = wait_for_mission.to(px4_auto_launch)
    launch_done = px4_auto_launch.to(ctrl_init_formation)
    formation_good = ctrl_init_formation.to(ctrl_manoeuvre)
    pause = ctrl_manoeuvre.to(px4_hold_in_sky)
    resume = px4_hold_in_sky.to(ctrl_manoeuvre)
    finish = px4_hold_in_sky.to(px4_return_to_base)
    mission_completed = px4_return_to_base.to(wait_for_mission)

    def __init__(self):
        # Extract drone name from controller config file - may need more data from this in future
        with open(sys.argv[1], 'r') as f:
            self.config_data = json.load(f)
            self.drone_name = self.config_data['name']

        # User input from a command prompt for now, drone_state refers to the mavros/state topic
        self.drone_state = State()
        self.user_input = ''

        # Set up clients for setting PX4 arming, autopilot modes
        # Register subscribers for PX4 autopilot state, user input
        self.arm_cmd = CommandBoolRequest()
        self.mode_cmd = SetModeRequest()
        rospy.wait_for_service(f"/{self.drone_name}/mavros/set_mode")
        rospy.wait_for_service(f"/{self.drone_name}/mavros/cmd/arming")
        self.set_mode_client = rospy.ServiceProxy(f"/{self.drone_name}/mavros/set_mode", SetMode)
        self.arming_client = rospy.ServiceProxy(f"/{self.drone_name}/mavros/cmd/arming", CommandBool)
        self.state_sub = rospy.Subscriber(f"{self.drone_name}/mavros/state", State, callback=self.state_cb)
        self.user_input_sub = rospy.Subscriber('/user_input/state', String, callback=self.user_input_cb)
        self.ctrl_state_pub = rospy.Publisher(f"/{self.drone_name}_ctrl/state", String, queue_size=10)
        
        super().__init__()
    
    def eventGenerator(self):
        if self.user_input == 'LAUNCH' and self.wait_for_mission.is_active:
            self.launch()
        elif self.drone_state.mode == 'AUTO.LOITER' and self.px4_auto_launch.is_active:
            self.launch_done()
        elif self.user_input == 'FORMATION_GOOD' and self.ctrl_init_formation.is_active:
            self.formation_good()
        elif self.user_input == 'PAUSE' and self.ctrl_manoeuvre.is_active:
            self.pause()
        elif self.user_input == 'RESUME' and self.px4_hold_in_sky.is_active:
            self.resume()
        elif self.user_input == 'FINISH' and self.px4_hold_in_sky.is_active:
            self.finish()
        elif self.px4_return_to_base.is_active and not self.drone_state.armed:
            self.mission_completed()


    # Callback functions used to automatically update user_input and drone_state as they change
    def user_input_cb(self, msg):
        self.user_input = msg.data

    def state_cb(self, msg):
        self.drone_state = msg


    # Functions that actually arm and set PX4 autopilot states; currently blocking
    def arm_drone(self, arm_value):
        rate = rospy.Rate(1)
        self.arm_cmd.value = arm_value
        while not self.arming_client.call(self.arm_cmd).success and not rospy.is_shutdown():
            rate.sleep()
    
    def change_drone_state(self, mode_string):
        rate = rospy.Rate(1)
        self.mode_cmd.custom_mode = mode_string
        while not self.set_mode_client.call(self.mode_cmd).mode_sent and not rospy.is_shutdown():
            rate.sleep()

    def change_controller_state(self, state_string):
        self.ctrl_state_pub.publish(state_string)

    def start_sending_setpoints(self):
        self.change_controller_state('INIT')

    # General logging functions that get called for every transition
    def before_transition(self, event, state):
        rospy.loginfo(f"{self.drone_name} exits {state.id} on the event {event}")

    def after_transition(self, event, state):
        rospy.loginfo(f"{self.drone_name} enters {state.id} on the event {event}")

    # Functions that only get called on their respective transitions
    def on_launch(self):
        while not rospy.is_shutdown() and self.drone_state.mode != 'AUTO.TAKEOFF':
            rate = rospy.Rate(2)
            if not self.drone_state.armed:
                self.arm_drone(True)
            elif self.drone_state.mode != 'AUTO.TAKEOFF':
                self.change_drone_state('AUTO.TAKEOFF')
            else:
                print('on_launch event should not have triggered here')
            rate.sleep()

    def on_launch_done(self):
        while not rospy.is_shutdown() and self.drone_state.mode != 'OFFBOARD':
            rate = rospy.Rate(1)
            self.change_drone_state('OFFBOARD')
            self.change_controller_state('INIT')

    def on_formation_good(self):
        self.change_controller_state('MOVE')

    def on_pause(self):
        while not rospy.is_shutdown() and self.drone_state.mode != 'AUTO.LOITER':
            rate = rospy.Rate(2)
            self.change_drone_state('AUTO.LOITER')
            rate.sleep()

    def on_resume(self):
        while not rospy.is_shutdown() and self.drone_state.mode != 'OFFBOARD':
            rate = rospy.Rate(2)
            self.change_drone_state('OFFBOARD')
            rate.sleep()

    def on_finish(self):
        while not rospy.is_shutdown() and self.drone_state.mode != 'AUTO.RTL':
            rate = rospy.Rate(2)
            self.change_drone_state('AUTO.RTL')
            rate.sleep()

    def on_mission_complete(self):
        rate = rospy.Rate(2)
        while self.drone_state.mode != 'AUTO.LOITER' or self.drone_state.armed:
            if self.drone_state.mode != 'AUTO.LOITER':
                rate = rospy.Rate(2)
                self.change_drone_state('AUTO.LOITER')
                rate.sleep()
            elif self.drone_state.armed:
                self.arm_drone(False)
                rate.sleep()



# main() currently contains the event generator code, in future this will be moved elsewhere/cleaned up
def main():
    drone_life_cycle = DroneMissionMachine()
    rospy.init_node(f"{drone_life_cycle.drone_name}_life_cycle")
    event_loop = rospy.Rate(EVENT_LOOP_RATE_HZ)
    while not rospy.is_shutdown():
        drone_life_cycle.eventGenerator()
        event_loop.sleep()

if __name__ == "__main__":
    main()
