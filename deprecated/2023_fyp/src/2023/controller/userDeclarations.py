def __init__(self):
	# Extract drone name from controller config file - may need more data from this in future
	with open(sys.argv[1], 'r') as f:
		self.config_data = json.load(f)
		self.drone_name = self.config_data['name']

	# Register subscribers for PX4 autopilot state, user input
	self.drone_state = rospysimple.SubscriberSimple(f"{self.drone_name}/mavros/state", State)
	self.user_input = rospysimple.SubscriberSimple('/user_input/state', String)
	self.ctrl_state = rospysimple.TopicSimple(f"/{self.drone_name}_ctrl/state", String)

	# Set up clients for setting PX4 arming, autopilot modes
	self.arm_cmd = CommandBoolRequest()
	self.mode_cmd = SetModeRequest()
	rospy.wait_for_service(f"/{self.drone_name}/mavros/set_mode")
	rospy.wait_for_service(f"/{self.drone_name}/mavros/cmd/arming")
	self.set_mode_client = rospy.ServiceProxy(f"/{self.drone_name}/mavros/set_mode", SetMode)
	self.arming_client = rospy.ServiceProxy(f"/{self.drone_name}/mavros/cmd/arming", CommandBool)
	super().__init__()

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

def start_sending_setpoints(self):
	self.change_controller_state('INIT')

# General logging functions that get called for every transition
def before_transition(self, event, state):
	rospy.loginfo(f"{self.drone_name} exits {state.id} on the event {event}")

def after_transition(self, event, state):
	rospy.loginfo(f"{self.drone_name} enters {state.id} on the event {event}")

# Functions that only get called on their respective transitions
def on_launch(self):
	while not rospy.is_shutdown() and self.drone_state.get().mode != 'AUTO.TAKEOFF':
		rate = rospy.Rate(2)
		if not self.drone_state.get().armed:
			self.arm_drone(True)
		elif self.drone_state.get().mode != 'AUTO.TAKEOFF':
			self.change_drone_state('AUTO.TAKEOFF')
		else:
			print('on_launch event should not have triggered here')
		rate.sleep()

def on_launch_done(self):
	while not rospy.is_shutdown() and self.drone_state.get().mode != 'OFFBOARD':
		rate = rospy.Rate(1)
		self.change_drone_state('OFFBOARD')
		self.ctrl_state.push('INIT')

def on_formation_good(self):
	self.ctrl_state.push('MOVE')

def on_pause(self):
	while not rospy.is_shutdown() and self.drone_state.get().mode != 'AUTO.LOITER':
		rate = rospy.Rate(2)
		self.change_drone_state('AUTO.LOITER')
		rate.sleep()

def on_resume(self):
	while not rospy.is_shutdown() and self.drone_state.get().mode != 'OFFBOARD':
		rate = rospy.Rate(2)
		self.change_drone_state('OFFBOARD')
		rate.sleep()

def on_finish(self):
	while not rospy.is_shutdown() and self.drone_state.get().mode != 'AUTO.RTL':
		rate = rospy.Rate(2)
		self.change_drone_state('AUTO.RTL')
		rate.sleep()

def on_mission_complete(self):
	rate = rospy.Rate(2)
	while self.drone_state.get().mode != 'AUTO.LOITER' or self.drone_state.get().armed:
		if self.drone_state.get().mode != 'AUTO.LOITER':
			rate = rospy.Rate(2)
			self.change_drone_state('AUTO.LOITER')
			rate.sleep()
		elif self.drone_state.get().armed:
			self.arm_drone(False)
			rate.sleep()