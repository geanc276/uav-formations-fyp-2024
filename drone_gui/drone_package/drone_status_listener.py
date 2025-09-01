import zmq
import threading
import json




def remove_topic_prefix(topic_message):
    print(f"Removing topic prefix from: {topic_message}")
    return topic_message.split(' ', 1)[1]


class DroneStatus():
    def __init__(self): 
        self.config_file = "None"
        self.state_file = "None"
        self.launched = False

    def set_all(self, json_str):
        json_str = remove_topic_prefix(json_str)
        json_obj = json.loads(json_str)
        self.config_file = json_obj.get("config_file", "None")
        self.state_file = json_obj.get("state_file", "None")
        self.launched = json_obj.get("launched", False)


        
    def to_dict(self):
        return {
            "config_file": self.config_file,
            "state_file": self.state_file,
            "launched": self.launched,
        }

    def set_config_file(self, config_file):
        self.config_file = config_file

    def set_state_file(self, state_file):
        self.state_file = state_file

    def set_launched(self, launched):
        self.launched = launched

    def get_config_file(self):
        return self.config_file
    
    def get_state_file(self):
        return self.state_file
    
    def get_launched(self):
        return self.launched
    
    def __str__(self):
        return f"Config File: {self.config_file}, State File: {self.state_file}, Launched: {self.launched}"



class MissionStatus():
    def __init__(self):
        self.armed = False
        self.state = "None"
        self.battery_percent = -1
        self.in_rendezvous_position = False
        self.norm_error = -1

    def set_all(self,json_str):
        json_str = remove_topic_prefix(json_str)
        json_obj = json.loads(json_str)
        self.armed = json_obj.get("armed", False)
        self.state = json_obj.get("state", "None")
        self.battery_percent = json_obj.get("battery_percent", -1)
        self.in_rendezvous_position = json_obj.get("in_rendezvous_position", False)
        self.norm_error = json_obj.get("norm_error", -1)

    def set_armed(self, armed):
        self.armed = armed

    def set_state(self, state):
        self.state = state
    
    def set_battery_percent(self, battery_percent):
        self.battery_percent = battery_percent

    def set_in_rendezvous_position(self, in_rendezvous_position):
        self.in_rendezvous_position = in_rendezvous_position

    def set_norm_error(self, norm_error):
        self.norm_error = norm_error

    def get_armed(self):
        return self.armed
    
    def get_state(self):
        return self.state
    
    def get_battery_percent(self):
        return self.battery_percent
    
    def get_in_rendezvous_position(self):
        return self.in_rendezvous_position
    
    def get_norm_error(self):
        return self.norm_error
    
    def __str__(self):
        return f"Armed: {self.armed}, State: {self.state}, Battery: {self.battery_percent}, Rendezvous: {self.in_rendezvous_position}, Norm Error: {self.norm_error}"
    

class Subscriber:
    def __init__(self, ip, port, topic, callback):
        self.topic = topic
        self.callback = callback
        self.ip_address = ip
        self.port = port 
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)


    def connect(self):
        print(f"Connecting to {self.topic} at {self.ip_address}:{self.port}")
        self.socket.connect(f"tcp://{self.ip_address}:{self.port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, self.topic)


    def start(self):
        self.listen_thread = threading.Thread(target=self.listen)
        self.listen_thread.start()


    def listen(self):
        while True:
            print("Listening for messages...")
            message = self.socket.recv_string()
            return_msg = self.process_message(message)
            self.callback(return_msg)

    def process_message(self, message):
        print(" SUBSCRIBER \"process_message\" SHOULD BE OVERRIDDEN")
        return message




class DroneStatusListener(Subscriber):
    def __init__(self, ip, port, topic, callback):
        super().__init__(ip, port, topic, callback)
        self.connect()
    

    def process_message(self, message):
        print(message)
        drone_status = DroneStatus()
        drone_status.set_all(message)
        return drone_status


class DroneMissionListener(Subscriber):
    def __init__(self, ip, port, topic,callback):
        super().__init__(ip, port, topic, callback)
        self.connect()
    
    def process_message(self, message):
        print(message)
        mission_status = MissionStatus()
        mission_status.set_all(message)
        return mission_status
    
