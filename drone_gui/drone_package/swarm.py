from utils.helpers import parse_commands_supported  # Ensure this helper exists
from backend.commands import get_command_class  # Ensure this helper exists
import customtkinter as ctk
import logging
from frontend import CommandScrollbox  # Ensure correct import path
from frontend import CommandButton  # Ensure correct import path
from drone_package.drone_status_listener import DroneStatusListener, DroneMissionListener, DroneStatus, MissionStatus
import threading

logger = logging.getLogger(__name__)


class Drone:
    def __init__(self, info, backend_update_func):
        self.backend_update_func = backend_update_func
        self.name = info.get('name', 'Unknown')
        self.connection = DroneConnection(info)
        self.status_sub_port = 5556
        self.mission_sub_port = 5557
        status_topic = f"{self.connection.hostname}/status"
        mission_topic = f"{self.connection.hostname}/mission/status"
        self.StatusListener = DroneStatusListener(
            self.connection.ip,
            self.status_sub_port,
            status_topic,
            self.set_drone_status
        )
        self.StatusListener.start()

        self.MissionListener = DroneMissionListener(
            self.connection.ip,
            self.mission_sub_port,
            mission_topic,
            self.set_drone_mission_status
        )
        self.MissionListener.start()

        self.drone_status = DroneStatus()
        self.drone_mission_status = MissionStatus()
        self.old_status = DroneStatus()
        self.page = DronePageInfo(self, None)
        self.create_commands()
        self.launchable = True
        self.needs_update = False

    def create_commands(self):
        self.launch_command = get_command_class("launch")("launch", self.connection)
        self.stop_command = get_command_class("stop")("stop", self.connection)
        self.send_state_command = get_command_class("state")("send_state", self.connection)
        self.send_connection_command = get_command_class("connection")("send_connection", self.connection)

    def set_drone_status(self, status):
        logger.debug("Drone Status: %s", status)
        self.drone_status = status
        current_status = self.drone_status.to_dict()
        old_status = self.old_status.to_dict()

        self.needs_update = (old_status != current_status)
        logger.debug("Old Status: %s", old_status)
        logger.debug("Current Status: %s", current_status)
        logger.debug("Needs Update: %s", self.needs_update)

        if self.needs_update:
            logger.info("Drone Status Updated")
            logger.debug("backend_update_func: %s", self.backend_update_func)
            self.backend_update_func(type="status", drone=self)

        self.old_status = self.drone_status

    def set_drone_mission_status(self, mission_status):
        logger.debug("Drone Mission Status: %s", mission_status)
        update = False
        for k, v in mission_status.to_dict().items():
            if self.drone_mission_status.get(k) != v:
                logger.debug("Mission Status Changed: %s - %s", k, v)
                update = True
                self.drone_mission_status.set(k, v)
        if update:
            logger.info("Drone Mission Status Updated")
            self.backend_update_func("mission", drone=self)

    def connected(self):
        self.send_connection_command.execute()

    def add_connection(self, connection):
        self.connection = connection

    def remove_connection(self):
        self.connection = None

    def replace_connection(self, connection):
        self.connection = connection
        self.page.clear()

    def add_page(self, page):
        self.page = page

    def remove_page(self):
        self.page = None

    def launch(self, num_drones):
        if self.launchable:
            self.launch_command.set_num_drones(num_drones)
            self.launch_command.execute()

    def stop(self):
        self.stop_command.execute()

    def send_state(self, state):
        self.send_state_command.set_state(state)
        self.send_state_command.execute()

    def assign_config(self, config_path):
        if self.connection:
            self.connection.assign_config(config_path)

    def __str__(self):
        return f"{self.name}"


class DroneConnection:
    def __init__(self, info, service=None):
        self.service = service
        self.name = info['name']
        self.hostname = info['properties'].get('hostname', 'Unknown')
        self.ip = info['ip']
        self.port = info['port']
        self.api_port = info['properties'].get('api_port', '8001')
        self.properties = info['properties']
        self.commands_supported = info['commands_supported']
        self.parse_commands_supported()

    def parse_commands_supported(self):
        if not hasattr(self, 'commands_supported'):
            return
        commands_str = self.commands_supported
        commands_dict = {}
        for command in commands_str.split(','):
            if ':' in command:
                command_name, command_type = command.split(':')
                command_instance = get_command_class(command_type)(command_name, self)
                commands_dict[command_name] = command_instance
            else:
                # Handle commands without a colon.
                command_name = command
                if command_name == "build_pls":
                    command_instance = get_command_class("build_pls")(command_name, self)
                    self.build_pls_command = command_instance
        self.commands_supported = commands_dict

    def get_supported_commands(self):
        return self.commands_supported

    def assign_config(self, config_path):
        if "upload_config" in self.commands_supported:
            self.commands_supported["upload_config"].execute(config_path)


class DronePageInfo:
    def __init__(self, drone, backend):
        self.drone = drone
        self.backend = backend
        self.info_frame = None
        self.name_label = None
        self.info_grid = None
        self.properties_label = None
        self.prop_frame = None
        self.ssh_button = None
        self.built = False
        self.commands_built = False
        self.commands_label = None
        self.commands_scrollbox = None

    def clear(self):
        if self.info_frame:
            self.info_frame.grid_forget()
        if self.commands_label:
            self.commands_label.grid_forget()
        if self.commands_scrollbox:
            self.commands_scrollbox.grid_forget()
        self.built = False
        self.commands_built = False

    def make_info_frame(self, main_frame):
        self.info_frame = ctk.CTkFrame(main_frame, corner_radius=10)
        self.info_frame.grid(row=0, column=0, sticky="ew", padx=20, pady=(0, 20))
        self.info_frame.grid_columnconfigure(0, weight=1)
        logger.debug("Info frame created.")

    def make_name_label(self):
        if not self.info_frame:
            logger.error("Info frame not created.")
            return
        self.name_label = ctk.CTkLabel(
            self.info_frame,
            text=f"Drone: {self.drone.name}",
            font=ctk.CTkFont(size=24, weight="bold"),
            anchor="w"
        )
        self.name_label.grid(row=0, column=0, sticky="w", padx=10, pady=10)
        logger.debug("Name label created.")

    def make_info_grid(self):
        if not self.info_frame:
            logger.error("Info frame not created.")
            return
        self.info_grid = ctk.CTkFrame(self.info_frame)
        self.info_grid.grid(row=1, column=0, sticky="ew", padx=10, pady=(0, 10))
        self.info_grid.grid_columnconfigure(0, weight=1)
        self.info_grid.grid_columnconfigure(1, weight=3)

        info_items = [
            ("Hostname", self.drone.connection.hostname),
            ("IP Address", self.drone.connection.ip),
            ("Port", self.drone.connection.port),
        ]
        for row, (label_text, value_text) in enumerate(info_items):
            label = ctk.CTkLabel(
                self.info_grid,
                text=f"{label_text}:",
                font=ctk.CTkFont(size=16, weight="bold"),
                anchor="e"
            )
            label.grid(row=row, column=0, sticky="e", padx=(0, 10), pady=5)
            value = ctk.CTkLabel(
                self.info_grid,
                text=str(value_text),
                font=ctk.CTkFont(size=16),
                anchor="w"
            )
            value.grid(row=row, column=1, sticky="w", pady=5)
            logger.debug("Added info item: %s - %s", label_text, value_text)

    def make_properties_section(self):
        if not self.info_frame:
            logger.error("Info frame not created.")
            return
        self.properties_label = ctk.CTkLabel(
            self.info_frame,
            text="Properties:",
            font=ctk.CTkFont(size=18, weight="bold"),
            anchor="w"
        )
        self.properties_label.grid(row=2, column=0, sticky="w", padx=10, pady=(20, 10))

        self.prop_frame = ctk.CTkFrame(self.info_frame)
        self.prop_frame.grid(row=3, column=0, sticky="ew", padx=10)
        self.prop_frame.grid_columnconfigure(0, weight=1)
        self.prop_frame.grid_columnconfigure(1, weight=3)

        properties = self.drone.connection.properties.copy()
        status_properties = self.drone.drone_status.to_dict()
        properties.update(status_properties)

        for r, (key, value) in enumerate(properties.items()):
            key_label = ctk.CTkLabel(
                self.prop_frame,
                text=f"{key}:",
                font=ctk.CTkFont(size=14, weight="bold"),
                anchor="e"
            )
            key_label.grid(row=r, column=0, sticky="e", padx=(0, 10), pady=2)
            value_label = ctk.CTkLabel(
                self.prop_frame,
                text=str(value),
                font=ctk.CTkFont(size=14),
                anchor="w"
            )
            value_label.grid(row=r, column=1, sticky="w", pady=2)
            logger.debug("Added property: %s - %s", key, value)

    def make_ssh_button(self, main_frame, command_callback):
        self.ssh_button = ctk.CTkButton(
            main_frame,
            text="SSH to Drone",
            command=command_callback,
            fg_color="#32CD32",
            hover_color="#228B22",
            width=200,
            height=50,
            corner_radius=10,
            font=ctk.CTkFont(size=16, weight="bold"),
            text_color="white"
        )
        next_row = main_frame.grid_size()[1]
        self.ssh_button.grid(row=next_row, column=0, pady=(30, 20), padx=10, sticky="w")
        logger.debug("SSH button created.")

    def build_info_section(self, main_frame, command_callback):
        if self.built:
            self.info_frame.grid()
            self.ssh_button.grid()
        else:
            self.make_info_frame(main_frame)
            self.make_name_label()
            self.make_info_grid()
            self.make_properties_section()
            self.make_ssh_button(main_frame, command_callback)
            logger.info("Drone info section built successfully.")
            self.built = True

    def make_commands_label(self, main_frame):
        self.commands_label = ctk.CTkLabel(
            main_frame,
            text="Supported Commands:",
            font=ctk.CTkFont(size=18, weight="bold"),
            anchor="w"
        )
        next_row = main_frame.grid_size()[1]
        self.commands_label.grid(row=next_row, column=0, sticky="w", padx=20, pady=(10, 10))
        logger.debug("Commands label created.")

    def make_commands_scrollbox(self, main_frame):
        self.commands_scrollbox = CommandScrollbox(
            main_frame,
            width=800,
            height=200,
            max_columns=3,
            min_button_width=150,
            min_button_height=40,
            padding_x=10,
            padding_y=10
        )
        next_row = main_frame.grid_size()[1]
        self.commands_scrollbox.grid(row=next_row, column=0, pady=10, padx=20, sticky="nsew")
        main_frame.grid_columnconfigure(0, weight=1)
        main_frame.grid_rowconfigure(next_row, weight=1)
        logger.debug("Commands scrollbox created.")

    def parse_and_create_commands(self):
        for k, command_instance in self.drone.connection.get_supported_commands().items():
            command_button = CommandButton(self.commands_scrollbox.scrollable_frame, command_instance)
            self.commands_scrollbox.add_command_button(command_button)
            logger.debug("Command button created for: %s", k)

    def build_commands_section(self, main_frame):
        if self.commands_built:
            self.commands_label.grid()
            self.commands_scrollbox.grid()
        else:
            self.make_commands_label(main_frame)
            self.make_commands_scrollbox(main_frame)
            self.parse_and_create_commands()
            self.commands_built = True
            logger.info("Drone commands section built successfully.")


class Swarm:
    def __init__(self, func):
        self.update_func = func
        self.discovered_drones_dict = {}
        self.discovered_drones = []
        self.currently_launched = []

    def add_or_update_drone(self, drone_data):
        name = drone_data['name']
        if name in self.discovered_drones_dict:
            self.update_drone(drone_data, name)
            return 0
        else:
            self.add_drone(drone_data, self.update_func)
            return 1

    def add_drone(self, drone_data, update_func):
        info = self.make_drone_info(drone_data)
        drone = Drone(info, update_func)
        self.discovered_drones_dict[drone.name] = drone
        self.discovered_drones = list(self.discovered_drones_dict.values())
        drone.connected()

    def remove_drone(self, name):
        logger.debug("Removing Drone: %s", name)
        if name in self.discovered_drones_dict:
            del self.discovered_drones_dict[name]
            self.discovered_drones = list(self.discovered_drones_dict.values())
        logger.debug("Current drones: %s", self.discovered_drones)

    @classmethod
    def make_drone_info(cls, drone_data):
        properties = drone_data.get('properties', {})
        commands_supported = parse_commands_supported(properties)
        drone_info = {
            "name": drone_data['name'],
            "ip": drone_data['ip'],
            "port": drone_data['port'],
            "properties": properties,
            "commands_supported": commands_supported
        }
        return drone_info

    def update_drone(self, drone_data, name):
        if name in self.discovered_drones_dict:
            drone_connection = DroneConnection(self.make_drone_info(drone_data))
            self.discovered_drones_dict[name].replace_connection(drone_connection)

    def get_all_drones(self):
        return self.discovered_drones

    def get_currently_launched(self):
        self.currently_launched = []
        for drone in self.discovered_drones:
            if drone.drone_status.get_launched():
                self.currently_launched.append(drone)
        logger.debug("Currently Launched: %s", self.currently_launched)
        return self.currently_launched

    def update_drone_status(self, drone):
        self.get_currently_launched()

    def set_currently_launched(self, drones):
        self.currently_launched = drones

    def stop_all_launched(self):
        for drone in self.currently_launched:
            drone.stop()
        self.currently_launched = []

    def launch_select_drones(self, drones):
        def launch_drone(drone, num_drones):
            drone.launch(num_drones)

        num_drones = len(drones)
        for drone in drones:
            threading.Thread(
                target=launch_drone,
                args=(drone, num_drones),
                daemon=True
            ).start()
        self.currently_launched = drones

    def get_swarm_builders(self):
        for k, v in self.discovered_drones_dict.items():
            if hasattr(v.connection, 'build_pls_command'):
                yield v

    def print_swarm_builder(self):
        for builder in self.get_swarm_builders():
            logger.debug(builder)

    def get_launchable_drones(self):
        return self.discovered_drones
