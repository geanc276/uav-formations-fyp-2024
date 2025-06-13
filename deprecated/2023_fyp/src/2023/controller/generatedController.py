#! /usr/bin/env python
import rospy
import sys
from std_msgs.msg import Header, Float64MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Float64, Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, ParamGet, ParamSet, ParamSetRequest
import json
import statemachine
import rospysimple
import lifecycle
from fcu import fcu


class DroneMissionMachine(statemachine.StateMachine):
	wait_for_mission = statemachine.State(initial=True)
	px4_auto_launch = statemachine.State()
	ctrl_init_formation = statemachine.State()
	ctrl_manoeuvre = statemachine.State()
	ctrl_hold_in_sky = statemachine.State()
	ctrl_return_to_base = statemachine.State()
	px4_auto_land = statemachine.State()
	
	launch = wait_for_mission.to(px4_auto_launch)
	launch_done = px4_auto_launch.to(ctrl_init_formation)
	formation_good = ctrl_init_formation.to(ctrl_manoeuvre)
	pause = ctrl_manoeuvre.to(ctrl_hold_in_sky)
	resume = ctrl_hold_in_sky.to(ctrl_manoeuvre)
	finish = ctrl_hold_in_sky.to(ctrl_return_to_base)
	land = ctrl_return_to_base.to(px4_auto_land)
	mission_completed = px4_auto_land.to(wait_for_mission)
	
	def eventGenerator(self):
		if self.user_input.get().data == 'LAUNCH' and self.wait_for_mission.is_active:
			self.launch()
		elif self.fcu.mode == 'AUTO.LOITER' and self.px4_auto_launch.is_active and self.is_drone_launch_complete():
			self.launch_done()
		elif self.ctrl_init_formation.is_active and self.is_formation_good():
			self.formation_good()
		elif self.ctrl_manoeuvre.is_active and self.is_data_stale ():
			rospy.loginfo("COMMUNICATION DROPOUT") #TODO handle communication dropout -> pause all drones in swarm 
		elif self.user_input.get().data == 'PAUSE' and self.ctrl_manoeuvre.is_active:
			self.pause()
		elif self.user_input.get().data == 'RESUME' and self.ctrl_hold_in_sky.is_active:
			self.resume()
		elif self.user_input.get().data == 'FINISH' and self.ctrl_hold_in_sky.is_active:
			self.finish()
		elif self.ctrl_return_to_base.is_active and self.is_return_finished():
			self.land()
		elif self.px4_auto_land.is_active and not self.fcu.armed:
			self.mission_completed()

	def __init__(self):
		rospy.loginfo('__init__ called')
		# Extract drone name from controller config file - may need more data from this in future
		with open(sys.argv[1], 'r') as f:
			self.config_data = json.load(f)
			self.drone_name = self.config_data['name']
			self.other_drones = self.config_data['other_drones']
			self.launch_height = self.config_data['local_initial'][2]

		# Register subscribers for PX4 autopilot state, user input

		self.user_input = rospysimple.SubscriberSimple('/user_input/state', String)
		self.ctrl_state = rospysimple.PublisherSimple(f"/{self.drone_name}_ctrl/state", String)
		self.altitude_bias_pub = rospysimple.PublisherSimple(f"/{self.drone_name}_ctrl/altitude_bias", Float64)
		self.launch_cooperative = lifecycle.Cooperative(self.drone_name, self.other_drones, "launch_done")
		self.formation_cooperative = lifecycle.Cooperative(self.drone_name, self.other_drones, "formation_good")
		self.return_cooperative = lifecycle.Cooperative(self.drone_name, self.other_drones, "return_good")
		self.launch_cooperative.synchronise(False)
		self.formation_cooperative.synchronise(False)
		self.return_cooperative.synchronise(False)

		# Set up clients for setting PX4 arming, autopilot modes

		self.fcu = fcu(self.drone_name)
		self.fcu.takeoff_height = self.launch_height
		self.takeoff_edges = lifecycle.EdgeErrorChecker(self.drone_name)
		self.land_edge = lifecycle.EdgeErrorChecker(self.drone_name)
		self.dropout_checker = lifecycle.DropoutChecker(self.drone_name, self.other_drones)

		super().__init__()

	def is_drone_launch_complete(self):
		return self.launch_cooperative.synchronise(self.drone_launch_complete_condition(1))

	def drone_launch_complete_condition(self, error_m):
		drone_altitude = self.fcu.drone_pos.z - self.altitude_bias
		return (self.launch_height + error_m) > drone_altitude > (self.launch_height - error_m)

	def is_formation_good(self):
		return self.formation_cooperative.synchronise(self.takeoff_edges.converged())

	def is_return_finished(self):
		return self.return_cooperative.synchronise(self.land_edge.converged())
	
	def is_data_stale (self):
		return self.formation_cooperative.synchronise(self.dropout_checker.dropout())


	# General logging functions that get called for every transition
	def before_transition(self, event, state):
		rospy.loginfo(f"{self.drone_name} exits {state.id} on the event {event}")
	
	def after_transition(self, event, state):
		rospy.loginfo(f"{self.drone_name} enters {state.id} on the event {event}")
	
	# Functions that only get called on their respective transitions
	def on_launch(self):
		self.altitude_bias = self.fcu.drone_pos.z
		self.altitude_bias_pub.push (self.altitude_bias) 
		self.ctrl_state.push('TAKEOFF')
		self.fcu.armed = True
		self.fcu.mode = 'AUTO.TAKEOFF'

	def on_launch_done(self):
		self.fcu.mode = 'OFFBOARD'
		self.ctrl_state.push('RENDEZVOUS')

	def on_formation_good(self):
		self.ctrl_state.push('MOVE')
		self.launch_cooperative.synchronise(False)
	
	def on_pause(self):
		self.ctrl_state.push('PAUSE')
		self.formation_cooperative.synchronise(False)
		self.takeoff_edges.convergence_timer.reset()
	
	def on_resume(self):
		self.ctrl_state.push('MOVE')

	def on_finish(self):
		self.ctrl_state.push('RETURN')

	def on_land(self):
		self.fcu.mode = 'AUTO.LAND'
		self.ctrl_state.push('LANDING')
	
	def on_mission_complete(self):
		self.fcu.mode = 'AUTO.LOITER'
		self.fcu.armed = False
		self.land_edge.convergence_timer.reset()
		self.return_cooperative.synchronise(False)


def main():
	with open(sys.argv[1], 'r') as f:
		config_data = json.load(f)
		rospy.init_node(f"{config_data['name']}_life_cycle")

	exampleMission = DroneMissionMachine()
	event_loop = rospy.Rate(10)
	while not rospy.is_shutdown():
		exampleMission.eventGenerator()
		event_loop.sleep()


if __name__ == "__main__":
	main()
