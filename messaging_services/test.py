import os
import json
import time
from zeroconf import ServiceBrowser, Zeroconf, ServiceListener
import requests
from tkinter import Tk, filedialog
from requests.auth import HTTPBasicAuth

class DroneListener(ServiceListener):
    """Listener for mDNS service advertisements."""

    def __init__(self):
        self.discovered_drones = []

    def add_service(self, zeroconf, service_type, name):
        """Callback when a new service is discovered."""
        info = zeroconf.get_service_info(service_type, name)
        if info:
            drone_data = self.parse_service_info(info)
            if drone_data:
                self.discovered_drones.append(drone_data)
                print(f"Discovered Drone: {drone_data}")

    def remove_service(self, zeroconf, service_type, name):
        """Callback when a service is removed."""
        print(f"Service {name} removed")
        # Optionally remove from discovered_drones

    def update_service(self, zeroconf, service_type, name):
        """Callback when a service is updated."""
        print(f"Service {name} updated")
        # Optionally update the drone data in self.discovered_drones

    def parse_service_info(self, info):
        """Extract metadata from the service info."""
        try:
            metadata = {}
            metadata['name'] = info.name
            addresses = info.parsed_addresses()
            if addresses:
                metadata['ip'] = addresses[0]
            else:
                print(f"No addresses found for service {info.name}")
                return None

            metadata['port'] = info.port
            if info.properties:
                metadata['properties'] = {
                    k.decode('utf-8'): v.decode('utf-8')
                    for k, v in info.properties.items()
                }
            return metadata
        except Exception as e:
            print(f"Error parsing service info: {e}")
            return None

def select_json_file():
    """Open a Tkinter file dialog to select a JSON file."""
    try:
        root = Tk()
        root.withdraw()  # Hide the root window
        file_path = filedialog.askopenfilename(
            title="Select JSON File",
            filetypes=[("JSON Files", "*.json")]
        )
        root.destroy()
        return file_path
    except Exception as e:
        print(f"Error selecting file: {e}")
        return None

def upload_json_to_drone(drone, file_path, username, password):
    """Upload a JSON file to the drone."""
    url = f"http://{drone['ip']}:{drone['port']}/upload"
    try:
        with open(file_path, 'rb') as file:
            response = requests.post(
                url,
                files={'file': file},
                data={'command': 'upload_new_json'},
                auth=HTTPBasicAuth(username, password),
                timeout=10  # set a timeout
            )
            return response
    except Exception as e:
        print(f"Error uploading file to {url}: {e}")
        return None

def main():
    # Initialize Zeroconf to discover drones
    zeroconf = Zeroconf()
    listener = DroneListener()
    print("Listening for drone advertisements...")
    browser = ServiceBrowser(zeroconf, "_http._tcp.local.", listener)

    # Wait for drones to be discovered
    try:
        # Wait until at least one drone is discovered or timeout
        timeout = 10  # seconds
        start_time = time.time()
        while True:
            if listener.discovered_drones:
                break
            if time.time() - start_time > timeout:
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        zeroconf.close()

    if not listener.discovered_drones:
        print("No drones discovered. Exiting.")
        return

    # List discovered drones
    print("\nDiscovered Drones:")
    for i, drone in enumerate(listener.discovered_drones):
        print(f"[{i}] {drone['name']} - {drone['ip']}:{drone['port']}")

    # Select a drone
    while True:
        try:
            drone_index = int(input("\nSelect a drone by index: "))
            if 0 <= drone_index < len(listener.discovered_drones):
                selected_drone = listener.discovered_drones[drone_index]
                break
            else:
                print("Index out of range. Please try again.")
        except ValueError:
            print("Invalid input. Please enter a number.")
        except Exception as e:
            print(f"Error selecting drone: {e}")

    # Ask for username and password
    username = input("Enter username for the drone: ")
    password = input("Enter password for the drone: ")

    # Select a JSON file
    json_file = select_json_file()
    if not json_file:
        print("No file selected. Exiting.")
        return

    # Upload the JSON file
    print(f"Uploading {os.path.basename(json_file)} to {selected_drone['name']}...")
    response = upload_json_to_drone(selected_drone, json_file, username, password)
    if response:
        if response.status_code == 200:
            print("JSON file uploaded successfully!")
            try:
                print(response.json())
            except json.JSONDecodeError:
                print("Response is not JSON.")
                print(response.text)
        else:
            print(f"Failed to upload JSON file: {response.status_code}")
            print(response.text)
    else:
        print("No response from the server.")

if __name__ == "__main__":
    main()