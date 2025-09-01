# backend/backend.py
from drone_package import Swarm
import threading
import logging
import os
import shutil
import subprocess
from tkinter import messagebox
logger = logging.getLogger(__name__)

class Backend:
    def __init__(self, app):
        self.app = app
        self.swarm = Swarm(self.request_update_of_drone)
        self.currently_launched = []
        self.drone_update_status_request = []
        self.drone_update_request = []
        self.drones_to_remove = []
        self.updated_drones= 0

    def menu_action(self, page_name):
        print(f"Menu: {page_name} selected")
        self.app.reset_navigation()
        self.app.show_page(page_name, add_to_history=False)

    def navigate_to(self, page_name):
        print(f"Navigate to: {page_name}")
        self.app.navigate_to(page_name)

    def navigate_to_drone_page(self, drone):
        print(f"Navigate to drone: {drone.name}")
        self.app.drone_page.change_drone(drone)
        self.app.navigate_to("Drone")
        # Optionally pass the drone to the

    def go_back(self):
        print("Go back")
        self.app.go_back()


    def check_updates(self):
        self.run_drone_update_requests()
        self.run_drone_status_update_requests()
        if self.updated_drones:
            # self.app.update_current_page()
            self.updated_drones = 0
        self.app.after(100, self.run_drone_removal_requests)
        self.app.after(100, self.check_updates)

# ////////////////////////////////////////////
# ////////////////////////////////////////////
# ////////////////////////////////////////////
# ////////////////////////////////////////////

    def add_drone(self, drone_data):
        added = self.swarm.add_or_update_drone(drone_data)
        print(f"Drone added: {added}")
        self.app.update_current_page()
        # self.app.update_all_pages()
    

    def remove_drone(self, name):
        self.swarm.remove_drone(name)
        # self.app.update_all_pages()
        self.app.update_current_page()
        self.app.removed_drone(name)


# ////////////////////////////////////////////
# ////////////////////////////////////////////
# ////////////////////////////////////////////
# ////////////////////////////////////////////
    def request_remove_drone(self, name):
        print(f"Requesting removal of drone: {name}")
        self.drones_to_remove.append(name) 
    
    def run_drone_removal_requests(self):
        for name in self.drones_to_remove:
            self.remove_drone(name)
        self.drones_to_remove = []


    def request_update_of_drone(self, type = "service", **kwargs):
        print("Requesting update of drone")
        if type == "service":
            self.request_update_drone(kwargs["drone_data"], kwargs["name"])
        elif type == "status":
            self.request_update_drone_status(kwargs["drone"])
        elif type == "mission":
            self.request_update_drone_mission(kwargs["drone"])
        else:
            print("Invalid request type")

    def request_update_drone_mission(self, drone):
        print(f"Requesting update for drone mission: {drone.name}")
        self.drone_update_request

    def run_drone_mission_update_requests(self):
        for drone in self.drone_update_request:
            self.update_drone_mission(drone["drone_data"], drone["name"])
            self.updated_drones = 1
        self.drone_update_request = []

    def update_drone_mission(self, drone_data, name):
        print(f"Updating drone mission: {name}")
        self.app.updated_drone(name, mission=True)
        # self.app.update_all_pages()



    def request_update_drone_status(self,drone):
        print(f"Requesting update for drone: {drone.name}")
        self.drone_update_status_request.append(drone)

    def run_drone_status_update_requests(self):
        for drone in self.drone_update_status_request:
            self.update_drone_status(drone)
            self.updated_drones = 1
        self.drone_update_status_request = []

    def update_drone_status(self, drone):
        print(f"Updating drone status: {drone.name}")
        print(drone.name)
        self.swarm.update_drone_status(drone)
        self.app.updated_drone(drone.name)
        # self.app.update_all_pages()


    def request_update_drone(self, drone_data, name):
        print(f"Requesting update for drone: {name}")
        drone_info = {"drone_data": drone_data, "name": name}
        self.drone_update_request.append(drone_info)

    def run_drone_update_requests(self):
        for drone_info in self.drone_update_request:
            self.update_drone(drone_info["drone_data"], drone_info["name"])
            self.updated_drones = 1
        self.drone_update_request = []

    def update_drone(self, drone_data, name):
        self.swarm.update_drone(drone_data, name)
        self.app.updated_drone(name)
        # Optionally update the drone's button if needed
        # This depends on whether button properties need updating
    
    def get_swarm_builders(self):
        return self.swarm.get_swarm_builders()
    
    def open_ssh(self, connection):
        """
        Implement SSH connection logic here.
        """
        hostname = connection.hostname
        ip = connection.ip
        port = connection.port
        logger.info(f"Initiating SSH connection to {hostname} ({ip}:{port})")
        # Implement SSH connection using paramiko or subprocess
        # For demonstration, we'll just log the action

    def open_ssh_terminal(self):
        """
        Opens a terminal and runs the SSH command to connect to the drone.
        """
        drone_ip = self.drone.connection.ip
        if not drone_ip:
            messagebox.showerror("Error", "Drone IP address is missing.")
            return
        user_name = getattr(self.drone.connection, 'user_name', 'drone_swarm')

        ssh_command = f"ssh {user_name}@{drone_ip}"
        try:
            # Detect the operating system to open the appropriate terminal
            if os.name == 'nt':  # Windows
                subprocess.Popen(["cmd.exe", "/c", ssh_command])
            elif os.name == 'posix':  # Unix/Linux/Mac
                terminal_emulators = ['gnome-terminal', 'konsole', 'xfce4-terminal', 'xterm', 'mate-terminal', 'tilix']
                for term in terminal_emulators:
                    if shutil.which(term):
                        subprocess.Popen([term, "--", "bash", "-c", ssh_command + "; exec bash"])
                        break
                else:
                    subprocess.Popen(["xterm", "-e", ssh_command])
            else:
                messagebox.showerror("Error", "Unsupported operating system for SSH.")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to open SSH terminal: {e}")
            logger.error(f"Failed to open SSH terminal for drone {self.drone.name}: {e}")



# ////////////////////////////////////////////
# ////////////////////////////////////////////
# ///////////////////SWARM////////////////////
# ////////////////////////////////////////////
# ////////////////////////////////////////////

    def get_launchable_drones(self):
        return self.swarm.get_launchable_drones()

    def get_all_drones(self):
        return self.swarm.get_all_drones()
    

    def get_currently_launched_drones(self):
        return self.swarm.get_currently_launched()

    def get_launched_drones(self):
        return self.swarm.get_launched_drones()
    
    def launch_drones(self, drones):
        self.swarm.launch_select_drones(drones)

    def stop_drones(self, drones = None):
        #! This is a temporary solution !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        print(self.swarm.currently_launched)
        self.swarm.stop_all_launched()

    def stop_all_drones(self):
        self.stop_drones()

# ////////////////////////////////////////////
# ////////////////////////////////////////////
# ////////////////////////////////////////////
# ////////////////////////////////////////////


    def assign_config(self, drone, config_path):
        drone.assign_config(config_path)


    def open_user_input_window(self):
        num_drones = len(self.swarm.get_all_drones())
        ros_command = "cd ../swarm_ws && source install/setup.bash"
        launch_command = f"ros2 run user_input user_input_node {num_drones}"
        combined = f"echo '{ros_command} && {launch_command}' && {ros_command} && {launch_command}"
        try:
            if os.name == 'nt':  # Windows
                subprocess.Popen(["cmd.exe", "/c", combined])
            elif os.name == 'posix':  # Unix/Linux/Mac
                terminal_emulators = ['gnome-terminal', 'konsole', 'xfce4-terminal', 'xterm', 'mate-terminal', 'tilix', 'tmux']
                for term in terminal_emulators:
                    if shutil.which(term):
                        if term == 'tmux':
                            subprocess.Popen([term, "new-session", combined])
                        else:
                            subprocess.Popen([term, "--", "bash", "-c", f"{combined}; exec bash"])
                        break
                else:
                    subprocess.Popen(["xterm", "-e", combined])
            else:
                messagebox.showerror("Error", "Unsupported operating system.")
        except Exception as e:
            logger.error(f"Failed to open terminal: {e}")
            messagebox.showerror("Error", f"Failed to open terminal: {e}")


    def send_state(self, state):
        print(f"Sending state: {state}")
        print(self.swarm.currently_launched)

        if self.swarm.currently_launched != []:
            self.swarm.currently_launched[0].send_state(state)
        else:
            messagebox.showerror("Error", "No drones are currently launched.")