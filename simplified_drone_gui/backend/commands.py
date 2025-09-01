# frontend/commands.py

import customtkinter as ctk
from tkinter import Toplevel, StringVar, Label, Button, messagebox, filedialog, simpledialog
import subprocess
import json
import os
import requests
from requests.auth import HTTPBasicAuth
import logging
from config.config import BACKEND_IP, BACKEND_PORT, BACKEND_USERNAME, BACKEND_PASSWORD  # Ensure this file exists with the necessary configurations
import utils.helpers as helpers 
from utils.utils import TextInputDialog, ExecutableSelectorDialog
import socket
import threading
import base64  # Added for base64 encoding
from cryptography.hazmat.primitives import serialization, hashes  # Added for cryptography
from cryptography.hazmat.primitives.asymmetric import padding  # Added for cryptography

import time
import zipfile
import shutil

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("drone_gui.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class BaseCommand:
    """Base class for all commands."""
    def __init__(self, command_name, drone):
        self.command_name = command_name
        self.drone = drone

    def execute(self):
        """Execute the command. Must be implemented by subclasses."""
        raise NotImplementedError("Execute method must be implemented by subclasses.")

    def get_display_name(self):
        """Return the display name for the command button."""
        return self.command_name.replace('_', ' ').capitalize()

    def get_fg_color(self):
        """Return the foreground color based on command type."""
        return "lightgray"

    def get_hover_color(self):
        """Return the hover color based on command type."""
        return "gray"
    
    def handle_response_success(self, response_data):
        """Handle a successful response with standard formatting."""
        message = response_data.get('message', 'Operation completed successfully.')
        messagebox.showinfo("Success", message)
        logger.info(f"Drone {self.drone.name}: {message}")

    def handle_response_error(self, response_data):
        """Handle an error response with standard formatting."""
        message = response_data.get('message', 'An unknown error occurred.')
        messagebox.showerror("Error", message)
        logger.error(f"Drone {self.drone.name} Error: {message}")

    def handle_http_error(self, response, http_err):
        """Handle HTTP errors and try to extract meaningful message."""
        try:
            response_data = response.json()
            success = response_data.get('success', False)
            message = response_data.get('message', 'Unknown error')
            if success:
                # If success is True but status_code indicates error, still show error
                messagebox.showerror("Error", f"Unexpected error: {message}")
                logger.error(f"Drone {self.drone.name} HTTPError: {http_err}")
            else:
                self.handle_response_error(response_data)
        except ValueError:
            # Could not decode JSON
            messagebox.showerror("Error", f"Failed with HTTP error: {http_err}")
            logger.error(f"Drone {self.drone.name} HTTPError (no JSON): {http_err}")



class JSONCommand(BaseCommand):
    """Handles JSON-style commands."""
    def get_fg_color(self):
        return "blue"

    def get_hover_color(self):
        return "darkblue"

    def execute(self, json_file = None):
        api_url = f"http://{self.drone.ip}:{self.drone.api_port}/upload"
        logger.info(f"Executing JSONCommand for drone {self.drone.name} at {api_url}")

        if not json_file:
            json_file = helpers.select_json_file()
            
        file_name = os.path.basename(json_file) if json_file else "None"
        if not json_file:
            logger.info(f"JSONCommand canceled by user for drone {self.drone.name}")
            return

        try:
            with open(json_file, 'r') as f:
                json_data = json.load(f)

            response = requests.post(
                api_url,
                auth=HTTPBasicAuth(BACKEND_USERNAME, BACKEND_PASSWORD),
                data={'command': self.command_name},
                files={'file': (f'{file_name}', json.dumps(json_data), 'application/json')}
            )
            response.raise_for_status()
            response_data = response.json()
            success = response_data.get('success', False)
            if success:
                self.handle_response_success(response_data)
            else:
                self.handle_response_error(response_data)
        except requests.exceptions.HTTPError as http_err:
            self.handle_http_error(response, http_err)
        except requests.exceptions.RequestException as e:
            messagebox.showerror("Error", f"Request failed: {str(e)}")
            logger.error(f"Request failed for JSONCommand on drone {self.drone.name}: {str(e)}")


# class ObjCommand(BaseCommand):
# #     """Handles binary/object upload commands."""
# #     def get_fg_color(self):
# #         return "green"

# #     def get_hover_color(self):
# #         return "darkgreen"

# #     def execute(self):
# #         api_url = f"http://{self.drone.ip}:{self.drone.api_port}/upload"
# #         logger.info(f"Executing ObjCommand for drone {self.drone.name} at {api_url}")

# #         executable_path = filedialog.askopenfilename(
# #             title="Select Executable",
# #             filetypes=[("Executable Files", "*.exe;*.sh;*.bin"), ("All Files", "*.*")]
# #         )
# #         if not executable_path:
# #             logger.info(f"ObjCommand canceled by user for drone {self.drone.name}")
# #             return

# #         if not os.path.isfile(executable_path):
# #             messagebox.showerror("Error", f"Executable not found: {executable_path}")
# #             logger.error(f"Executable not found at {executable_path} for drone {self.drone.name}")
# #             return

# #         try:
# #             with open(executable_path, 'rb') as f:
# #                 files = {'file': (os.path.basename(executable_path), f, 'application/octet-stream')}
# #                 data = {'command': 'upload_new_executable'}
# #                 response = requests.post(
# #                     api_url,
# #                     auth=HTTPBasicAuth(BACKEND_USERNAME, BACKEND_PASSWORD),
# #                     data=data,
# #                     files=files
# #                 )
# #                 response.raise_for_status()
# #                 response_data = response.json()
# #                 success = response_data.get('success', False)
# #                 if success:
# #                     self.handle_response_success(response_data)
# #                 else:
# #                     self.handle_response_error(response_data)
# #         except requests.exceptions.HTTPError as http_err:
# #             self.handle_http_error(response, http_err)
# #         except requests.exceptions.RequestException as e:
# #             messagebox.showerror("Error", f"Request failed: {str(e)}")
# #             logger.error(f"Request failed for ObjCommand on drone {self.drone.name}: {str(e)}")


# class CommandCommand(BaseCommand):
#     """Handles sending commands to the drone."""
#     def get_fg_color(self):
#         return "red"

#     def get_hover_color(self):
#         return "darkred"

#     def execute(self):
#         predefined_executables = ["my_executable1", "my_executable2", "my_executable3"]
#         dialog = ExecutableSelectorDialog(self.drone, predefined_executables)
#         executable_name = dialog.get_selection()

#         if not executable_name:
#             logger.info(f"CommandCommand canceled by user for drone {self.drone.name}")
#             return

#         params = ["--option1", "value1"]

#         api_url = f"http://{self.drone.ip}:{self.drone.api_port}/start_executable"
#         logger.info(f"Executing CommandCommand for drone {self.drone.name} at {api_url}")

#         payload = {
#             "executable_name": executable_name,
#             "params": params
#         }

#         try:
#             response = requests.post(
#                 api_url,
#                 auth=HTTPBasicAuth(BACKEND_USERNAME, BACKEND_PASSWORD),
#                 json=payload
#             )
#             response.raise_for_status()
#             response_data = response.json()
#             success = response_data.get('success', False)
#             if success:
#                 self.handle_response_success(response_data)
#             else:
#                 self.handle_response_error(response_data)
#         except requests.exceptions.HTTPError as http_err:
#             self.handle_http_error(response, http_err)
#         except requests.exceptions.RequestException as e:
#             messagebox.showerror("Error", f"Request failed: {str(e)}")
#             logger.error(f"Request failed for CommandCommand on drone {self.drone.name}: {str(e)}")


class RequestCommand(BaseCommand):
    """Handles sending requests to retrieve files from the drone."""
    def get_fg_color(self):
        return "gold"

    def get_hover_color(self):
        return "yellow"

    def execute(self):
        try:
            api_url = f"http://{self.drone.ip}:{self.drone.api_port}/request/{self.command_name}"
            logger.info(f"Executing RequestCommand for drone {self.drone.name} at {api_url}")

            response = requests.post(
                api_url,
                headers={'Content-Type': 'application/json'},
                auth=HTTPBasicAuth(BACKEND_USERNAME, BACKEND_PASSWORD),
                data = json.dumps({'command': self.command_name})
            )
            response.raise_for_status()
            response_data = response.json()
            success = response_data.get('success', False)
            if success:
                self.handle_response_success(response_data)
            else:
                self.handle_response_error(response_data)
        except requests.exceptions.HTTPError as http_err:
            self.handle_http_error(response, http_err)


# class AnyFile(BaseCommand):
#     """Handles uploading any file type."""
#     def get_fg_color(self):
#         return "purple"

#     def get_hover_color(self):
#         return "darkblue"

#     def execute(self):
#         api_url = f"http://{self.drone.ip}:{self.drone.api_port}/upload_any_file"
#         logger.info(f"Executing AnyFile command for drone {self.drone.name} at {api_url}")

#         file_path = filedialog.askopenfilename(
#             title="Select Any File",
#             filetypes=[("All Files", "*.*")]
#         )
#         if not file_path:
#             logger.info(f"AnyFile command canceled by user for drone {self.drone.name}")
#             return

#         if not os.path.isfile(file_path):
#             messagebox.showerror("Error", f"File not found: {file_path}")
#             logger.error(f"File not found at {file_path} for drone {self.drone.name}")
#             return

#         try:
#             with open(file_path, 'rb') as f:
#                 files = {'file': (os.path.basename(file_path), f, 'application/octet-stream')}
#                 response = requests.post(
#                     api_url,
#                     auth=HTTPBasicAuth(BACKEND_USERNAME, BACKEND_PASSWORD),
#                     files=files
#                 )
#                 response.raise_for_status()
#                 response_data = response.json()
#                 success = response_data.get('success', False)
#                 if success:
#                     self.handle_response_success(response_data)
#                 else:
#                     self.handle_response_error(response_data)
#         except requests.exceptions.HTTPError as http_err:
#             self.handle_http_error(response, http_err)
#         except requests.exceptions.RequestException as e:
#             messagebox.showerror("Error", f"Request failed: {str(e)}")
#             logger.error(f"Request failed for AnyFile command on drone {self.drone.name}: {str(e)}")


class LaunchCommand(BaseCommand):
    def __init__(self, command_name, drone, num_drones=1):
        super().__init__(command_name, drone)

    def set_num_drones(self, num_drones):
        self.num_drones = num_drones

    def get_fg_color(self):
        return "orange"

    def get_hover_color(self):
        return "darkorange"

    def execute(self):
        command_to_launch = "LAUNCH"
        num_drones = 1

        if not command_to_launch:
            logger.info(f"LaunchCommand canceled by user for drone {self.drone.name}")
            return

        api_url = f"http://{self.drone.ip}:{self.drone.api_port}/{self.command_name}"
        logger.info(f"Executing LaunchCommand for drone {self.drone.name} at {api_url}")

        payload = {
            "command": command_to_launch,
            "num_drones": num_drones
        }

        try:
            response = requests.post(
                api_url,
                auth=HTTPBasicAuth(BACKEND_USERNAME, BACKEND_PASSWORD),
                json=payload
            )
            response.raise_for_status()
            response_data = response.json()
            success = response_data.get('success', False)
            if success:
                self.handle_response_success(response_data)
            else:
                self.handle_response_error(response_data)
        except requests.exceptions.HTTPError as http_err:
            self.handle_http_error(response, http_err)
        except requests.exceptions.RequestException as e:
            messagebox.showerror("Error", f"Request failed: {str(e)}")
            logger.error(f"Request failed for LaunchCommand on drone {self.drone.name}: {str(e)}")


class StopCommand(BaseCommand):
    """Handles sending stop commands to the drone."""
    def get_fg_color(self):
        return "red"

    def get_hover_color(self):
        return "darkred"

    def execute(self):
        api_url = f"http://{self.drone.ip}:{self.drone.api_port}/request/stop"
        logger.info(f"Executing StopCommand for drone {self.drone.name} at {api_url}")

        try:
            response = requests.post(
                api_url,
                auth=HTTPBasicAuth(BACKEND_USERNAME, BACKEND_PASSWORD),
                data={'command': self.command_name}
            )
            response.raise_for_status()
            response_data = response.json()
            success = response_data.get('success', False)
            if success:
                self.handle_response_success(response_data)
            else:
                self.handle_response_error(response_data)
        except requests.exceptions.HTTPError as http_err:
            self.handle_http_error(response, http_err)
        except requests.exceptions.RequestException as e:
            messagebox.showerror("Error", f"Request failed: {str(e)}")
            logger.error(f"Request failed for StopCommand on drone {self.drone.name}: {str(e)}")

# class ZipCommand(BaseCommand):
#     """Handles sending zip uploads to the drone."""
#     def get_fg_color(self):
#         return "brown"
    
#     def get_hover_color(self):
#         return "darkred"
    
#     def get_password_from_user(self):
#         password = simpledialog.askstring(
#             "SSH Password",
#             "Enter the SSH password for drone_swarm:",
#             show='*'
#         )
#         if not password:
#             messagebox.showerror("Error", "Password is required to continue.")
#             return None
#         return password

#     def execute_scp_with_password(self, zip_path, filepath, password):
        
        
#         #! ! ! ! ! ! ! ! ! MAYBE A SECURITY RISK ! ! ! ! ! ! ! ! !
        
#         ssh_command = f"scp -o StrictHostKeyChecking=no {zip_path} drone_swarm@{self.drone.ip}:{filepath}"
#         logger.info(f"Executing SCP command: {ssh_command}")

#         try:
#             result = subprocess.run(
#                 f"sshpass -p {password} {ssh_command}",
#                 shell=True,
#                 stdout=subprocess.PIPE,
#                 stderr=subprocess.PIPE,
#                 text=True
#             )

#             if result.returncode != 0:
#                 error_message = f"SCP failed: {result.stderr.strip()}"
#                 messagebox.showerror("Error", error_message)
#                 logger.error(error_message)
#                 return False

#             success_message = f"File uploaded successfully to {filepath}"
#             messagebox.showinfo("Success", success_message)
#             logger.info(success_message)
#             return True
#         except Exception as e:
#             error_message = f"Unexpected error during SCP: {str(e)}"
#             messagebox.showerror("Error", error_message)
#             logger.error(error_message, exc_info=True)
#             return False

#     def execute(self):
#         api_url = f"http://{self.drone.ip}:{self.drone.api_port}/upload_zip"
#         logger.info(f"Executing ZipCommand for drone {self.drone.name} at {api_url}")

#         dir_path = os.path.dirname(os.path.abspath(__file__))
#         parent_dir = os.path.dirname(os.path.abspath(dir_path))
#         parent_dir = os.path.dirname(os.path.abspath(parent_dir))
#         zip_path = os.path.join(parent_dir, 'swarm_ws', 'drone_launch.zip')

#         if not os.path.exists(zip_path):
#             messagebox.showerror("Error", f"Zip file not found: {zip_path}")
#             logger.error(f"Zip file not found at {zip_path} for drone {self.drone.name}")
#             return

#         try:
#             response = requests.post(
#                 api_url,
#                 auth=HTTPBasicAuth(BACKEND_USERNAME, BACKEND_PASSWORD),
#                 data={'command': self.command_name}
#             )
#             response.raise_for_status()
#             response_data = response.json()
#             success = response_data.get('success', False)
#             if not success:
#                 self.handle_response_error(response_data)
#                 return
#             filepath = response_data.get('data', {}).get('filepath')
#             logger.debug(f"Retrieved filepath from server: {filepath}")

#             password = self.get_password_from_user()
#             if not password:
#                 return

#             self.execute_scp_with_password(zip_path, filepath, password)

#         except requests.exceptions.HTTPError as http_err:
#             self.handle_http_error(response, http_err)
#         except requests.exceptions.RequestException as e:
#             messagebox.showerror("Error", f"Request failed: {str(e)}")
#             logger.error(f"Request failed for ZipCommand on drone {self.drone.name}: {str(e)}")


class UnknownCommand(BaseCommand):
    """Handles unknown command types."""
    def get_fg_color(self):
        return "gray"

    def get_hover_color(self):
        return "darkgray"

    def execute(self):
        messagebox.showwarning("Warning", f"Unknown command type for command: {self.command_name}")
        logger.warning(f"Unknown command type '{self.command_name}' for drone {self.drone.name}")




class SyncStartDownloadCommand(BaseCommand):
    """Handles the sequential syncing, building, and downloading of build artifacts."""

    def get_fg_color(self):
        return "cyan"

    def get_hover_color(self):
        return "darkcyan"

    def execute(self):
        logger.info(f"Executing SyncStartDownloadCommand for drone {self.drone.name}")
        user_name = self.get_host_user_name()
        text_input = TextInputDialog("SSH Password", f"Enter the SSH password for {user_name} to allow rsync:", private=True)
        time.sleep(1)
        password = text_input.get_input()

        if password is None:
            logger.info("Password entry canceled by user.")
            return

        def run_commands():
            ip = self.get_ip_address()
            if not ip:
                messagebox.showerror("Error", "Failed to retrieve the machine's IP address.")
                logger.error("Failed to retrieve the machine's IP address.")
                return

            logger.info(f"Machine IP address: {ip}")

            swarm_ws_path = self.find_swarm_ws_path()
            if not swarm_ws_path:
                messagebox.showerror("Error", "Could not locate 'uav-formations-fyp-2024/swarm_ws' directory.")
                logger.error("Could not locate 'uav-formations-fyp-2024/swarm_ws' directory.")
                return

            logger.info(f"Swarm workspace path: {swarm_ws_path}")

            public_key = self.get_public_key()
            if not public_key:
                messagebox.showerror("Error", "Failed to retrieve public key from the drone.")
                logger.error("Failed to retrieve public key from the drone.")
                return

            encrypted_password = self.encrypt_password(password, public_key)
            if not encrypted_password:
                messagebox.showerror("Error", "Failed to encrypt the password.")
                logger.error("Failed to encrypt the password.")
                return
            
            encrypted_password_b64 = base64.b64encode(encrypted_password).decode('utf-8')
            
            rsync_url = f"http://{self.drone.ip}:{self.drone.api_port}/rsync_build"
            rsync_payload = {
                "ip": ip,
                "path": swarm_ws_path,
                "username": user_name,
                "encrypted_password": encrypted_password_b64
            }

            logger.info(f"Sending POST request to {rsync_url} with payload: {rsync_payload}")

            try:
                rsync_response = requests.post(
                    rsync_url,
                    auth=HTTPBasicAuth(BACKEND_USERNAME, BACKEND_PASSWORD),
                    json=rsync_payload,
                    timeout=300
                )
                rsync_response.raise_for_status()
                rsync_data = rsync_response.json()
                if rsync_data.get('success', False):
                    self.handle_response_success(rsync_data)
                else:
                    self.handle_response_error(rsync_data)
                    return
            except requests.exceptions.HTTPError as http_err:
                self.handle_http_error(rsync_response, http_err)
                return
            except requests.exceptions.RequestException as e:
                messagebox.showerror("Error", f"Rsync request failed: {str(e)}")
                logger.error(f"Request exception during rsync for drone {self.drone.name}: {str(e)}")
                return

            start_build_url = f"http://{self.drone.ip}:{self.drone.api_port}/start_build"
            logger.info(f"Sending POST request to {start_build_url} to start build.")

            try:
                build_response = requests.post(
                    start_build_url,
                    auth=HTTPBasicAuth(BACKEND_USERNAME, BACKEND_PASSWORD),
                    json={},
                    timeout=600
                )
                build_response.raise_for_status()
                build_data = build_response.json()
                if build_data.get('success', False):
                    self.handle_response_success(build_data)
                else:
                    self.handle_response_error(build_data)
                    return
            except requests.exceptions.HTTPError as http_err:
                self.handle_http_error(build_response, http_err)
                return
            except requests.exceptions.RequestException as e:
                messagebox.showerror("Error", f"Build request failed: {str(e)}")
                logger.error(f"Request exception during build for drone {self.drone.name}: {str(e)}")
                return

            download_build_url = f"http://{self.drone.ip}:{self.drone.api_port}/download_build"
            logger.info(f"Sending GET request to {download_build_url} to download build artifacts.")

            try:
                download_response = requests.get(
                    download_build_url,
                    auth=HTTPBasicAuth(BACKEND_USERNAME, BACKEND_PASSWORD),
                    stream=True,
                    timeout=600
                )
                download_response.raise_for_status()
                swarm_ws = self.find_swarm_ws_path()
                save_path = filedialog.asksaveasfilename(
                    title="Save Build Artifacts",
                    defaultextension=".zip",
                    filetypes=[("ZIP Files", "*.zip"), ("All Files", "*.*")],
                    initialfile="downloaded_install.zip",
                    initialdir=f"{swarm_ws}"
                )

                if not save_path:
                    messagebox.showinfo("Info", "Download canceled by user.")
                    logger.info(f"Download canceled by user for drone {self.drone.name}")
                    return

                with open(save_path, 'wb') as f:
                    for chunk in download_response.iter_content(chunk_size=8192):
                        if chunk:
                            f.write(chunk)

                messagebox.showinfo("Success", f"Build artifacts downloaded successfully to {save_path}.")
                logger.info(f"Build artifacts downloaded successfully for drone {self.drone.name} to {save_path}")

                install_dir = os.path.join(swarm_ws,'arm_install')
                if os.path.exists(install_dir):
                    shutil.rmtree(install_dir)
                os.makedirs(install_dir, exist_ok=True)
                with zipfile.ZipFile(save_path, 'r') as zip_ref:
                    zip_ref.extractall(install_dir)
                messagebox.showinfo("Success", f"Build artifacts extracted successfully to {install_dir}.")
                logger.info(f"Build artifacts extracted successfully to {install_dir}")

                helpers.run_zip_script("arm_install")

            except requests.exceptions.HTTPError as http_err:
                self.handle_http_error(download_response, http_err)
            except requests.exceptions.RequestException as e:
                messagebox.showerror("Error", f"Download request failed: {str(e)}")
                logger.error(f"Request exception during download for drone {self.drone.name}: {str(e)}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save the file: {str(e)}")
                logger.error(f"Exception during file save for drone {self.drone.name}: {str(e)}")

        threading.Thread(target=run_commands).start()


    def get_host_user_name(self):
        sys_user = os.getlogin()
        return sys_user
    

    def get_public_key(self):
        public_key_url = f"http://{self.drone.ip}:{self.drone.api_port}/get_public_key"
        logger.info(f"Requesting public key from {public_key_url}")

        try:
            response = requests.get(
                public_key_url,
                auth=HTTPBasicAuth(BACKEND_USERNAME, BACKEND_PASSWORD),
                timeout=30
            )
            response.raise_for_status()
            data = response.json()
            pem_public_key = data.get('data', {}).get('public_key')
            if pem_public_key:
                logger.info("Public key received successfully.")
                return pem_public_key
            else:
                logger.error("Public key not found in response.")
                return None
        except requests.exceptions.RequestException as e:
            logger.error(f"Failed to get public key: {str(e)}")
            return None

    def encrypt_password(self, password, pem_public_key):
        try:
            public_key = serialization.load_pem_public_key(
                pem_public_key.encode('utf-8')
            )

            encrypted_password = public_key.encrypt(
                password.encode('utf-8'),
                padding.OAEP(
                    mgf=padding.MGF1(algorithm=hashes.SHA256()),
                    algorithm=hashes.SHA256(),
                    label=None
                )
            )
            logger.info("Password encrypted successfully.")
            return encrypted_password
        except Exception as e:
            logger.error(f"Failed to encrypt password: {str(e)}")
            return None

    def get_ip_address(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                ip = s.getsockname()[0]
            return ip
        except Exception as e:
            logger.error(f"Error retrieving IP address: {e}")
            return None

    def find_swarm_ws_path(self):
        # current_dir = os.path.abspath(os.getcwd())
        current_dir = os.path.dirname(os.path.abspath(__file__))
        target_dir_name = 'uav-formations-fyp-2024'
        swarm_ws_subdir = 'swarm_ws'

        logger.debug(f"Starting traversal from directory: {current_dir}")

        while True:
            if os.path.basename(current_dir) == target_dir_name:
                swarm_ws_path = os.path.join(current_dir, swarm_ws_subdir)
                if os.path.isdir(swarm_ws_path):
                    logger.debug(f"Found swarm_ws directory at: {swarm_ws_path}")
                    return swarm_ws_path
                else:
                    logger.error(f"swarm_ws directory does not exist in {current_dir}")
                    return None
            parent_dir = os.path.dirname(current_dir)
            if parent_dir == current_dir:
                logger.error(f"Reached root directory without finding {target_dir_name}")
                return None
            current_dir = parent_dir
            logger.debug(f"Moving up to parent directory: {current_dir}")



class SendStateCommand(BaseCommand):
    def __init__(self, command_name, drone):
        self.state = None
        super().__init__(command_name, drone)

    def set_state(self, state):
        self.state = state

    def get_fg_color(self):
        return "blue"
    
    def get_hover_color(self):
        return "darkblue"
    
    def execute(self):
        api_url = f"http://{self.drone.ip}:{self.drone.api_port}/request/ros"
        logger.info(f"Executing SendStateCommand for drone {self.drone.name} at {api_url}")
        if not self.state:
            messagebox.showerror("Error", "State is missing.")
            logger.error(f"State is missing for SendStateCommand on drone {self.drone.name}")
            return
        
        try:

            response = requests.post(
                api_url,
                headers = {'Content-Type': 'application/json'},
                auth=HTTPBasicAuth(BACKEND_USERNAME, BACKEND_PASSWORD),
                json={'command': self.state}
            )
            response.raise_for_status()
            response_data = response.json()
            success = response_data.get('success', False)
            if success:
                pass
                # self.handle_response_success(response_data)
            else:
                self.handle_response_error(response_data)
        except requests.exceptions.HTTPError as http_err:
            self.handle_http_error(response, http_err)
        except requests.exceptions.RequestException as e:
            messagebox.showerror("Error", f"Request failed: {str(e)}")
            logger.error(f"Request failed for SendStateCommand on drone {self.drone.name}: {str(e)}")

class ConnectionCommand(BaseCommand):

    def get_fg_color(self):
        return "blue"
    
    def get_hover_color(self):
        return "darkblue"
    
    def execute(self):
        if self.drone is None:
            logger.warning("ConnectionCommand called without a drone.")
            return

        logger.info(f"Executing ConnectionCommand for drone {self.drone.name}")
        if self.drone.ip.startswith("fe80"):
            logger.warning(f"Skipping link-local IPv6 {self.drone.ip}, forcing IPv4 only.")
            return

        api_url = f"http://{self.drone.ip}:{self.drone.api_port}/connected"
        try:
            response = requests.post(
                api_url,
                auth=HTTPBasicAuth(BACKEND_USERNAME, BACKEND_PASSWORD),
                timeout=30
            )
            response.raise_for_status()
            response_data = response.json()
            success = response_data.get('success', False)
            if success:
                self.handle_response_success(response_data)
            else:
                self.handle_response_error(response_data)

        except requests.exceptions.HTTPError as http_err:
            self.handle_http_error(response, http_err)
        except requests.exceptions.RequestException as e:
            messagebox.showerror("Error", f"Request failed: {str(e)}")
            logger.error(f"Request failed for ConnectionCommand on drone {self.drone.name}: {str(e)}")


def get_command_class(cmd_type):
    cmd_type = cmd_type.lower()
    if cmd_type == 'json':
        return JSONCommand
    # elif cmd_type == 'obj':
    #     return ObjCommand
    # elif cmd_type == 'command':
    #     return CommandCommand
    elif cmd_type == 'request':
        return RequestCommand
    # elif cmd_type == 'file':
    #     return AnyFile
    elif cmd_type == 'launch':
        return LaunchCommand
    elif cmd_type == 'stop':
        return StopCommand
    # elif cmd_type == 'zip':
    #     return ZipCommand
    elif cmd_type == 'build_pls':
        return SyncStartDownloadCommand
    elif cmd_type == 'state':
        return SendStateCommand
    elif cmd_type == 'connection':
        return ConnectionCommand
    else:
        return UnknownCommand
