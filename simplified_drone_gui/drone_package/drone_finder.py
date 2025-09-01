import socket
import threading
import time
import datetime
import logging
from typing import Dict
from zeroconf import ServiceBrowser, Zeroconf, ServiceInfo, ServiceListener


class DroneFinder(ServiceListener):
    def __init__(self, add_drone_func, remove_drone_func, update_drone_func, debounce_time=15.0):
        """
        Initialize the DroneFinder.

        :param add_drone_func: Function to call when a new drone is discovered.
        :param remove_drone_func: Function to call when a drone is removed.
        :param update_drone_func: Function to call when a drone's information is updated.
        :param debounce_time: Time in seconds to wait before confirming removal.
        """
        self.zeroconf = Zeroconf()
        # self.browser_drone = ServiceBrowser(self.zeroconf, "_drone._tcp.local.", self)
        self.browser_http = ServiceBrowser(self.zeroconf, "_http._tcp.local.", self)
        # self.browser_afp = ServiceBrowser(self.zeroconf, "_afpovertcp._tcp.local.", self)

        self.add_drone_func = add_drone_func
        self.remove_drone_func = remove_drone_func
        self.update_drone_func = update_drone_func
        self.known_services: Dict[str, Dict] = {}  # Key: service_name
        self.lock = threading.Lock()  # Ensure thread-safe operations
        self.pending_removals: Dict[str, threading.Timer] = {}  # Key: service_name
        self.debounce_time = debounce_time  # Debounce duration

        self.check_drones_alive_thread = threading.Thread(target=self.check_drones_alive)
        self.check_drones_alive_thread.daemon = True
        self.check_drones_alive_thread.start()
        self.logger = logging.getLogger(__name__)  # AJOUT


    def parse_service_info(self, info: ServiceInfo) -> Dict:
        """
        Parse the ServiceInfo into a dictionary containing drone data.

        :param info: ServiceInfo object from Zeroconf.
        :return: Dictionary with drone data or None if parsing fails.
        """
        try:
            print(f"{datetime.datetime.now()} - [DEBUG] Parsing service info: {info}")
            metadata = {}
            # Extract base name by removing service type and domain
            base_name = info.name.split('._http._tcp.local.')[0]
            metadata['name'] = base_name
            addresses = info.parsed_addresses()
            if addresses:
                ip_candidate = addresses[0]
                # Ignore link-local IPv6 if we have a better option
                if ip_candidate.startswith("fe80") and b'ip_address' in info.properties:
                    try:
                        ip_candidate = info.properties[b'ip_address'].decode('utf-8')
                    except:
                        pass
                metadata['ip'] = ip_candidate

            else:
                print(f"{datetime.datetime.now()} - [WARNING] No addresses found for service {info.name}")
                return None

            metadata['port'] = info.port
            metadata['api_port'] = info.properties.get(b'api_port', b'8001').decode('utf-8')
            if info.properties:
                metadata['properties'] = {
                    (k.decode('utf-8') if isinstance(k, bytes) else str(k)):
                    (v.decode('utf-8') if isinstance(v, bytes) and v is not None else str(v) if v is not None else "")
                    for k, v in info.properties.items()
                }

            return metadata
        except Exception as e:
            print(f"{datetime.datetime.now()} - [ERROR] Error parsing service info: {e}")
            return None

    def add_service(self, zeroconf, service_type, name):
        """
        Called when a new service is discovered.

        :param zeroconf: Zeroconf instance.
        :param service_type: Type of the service.
        :param name: Name of the service.
        """
        info = zeroconf.get_service_info(service_type, name)
        if not info:
            return

        # Filtrer ESP32 ou autres non-drones
        if info.port == 80 and b'commands_supported' not in info.properties:
            print(f"{datetime.datetime.now()} - [INFO] Skipping non-drone service {info.name} on port 80.")
            return

        drone_data = self.parse_service_info(info)
        if drone_data:
            service_name = info.name
            with self.lock:
                if service_name in self.pending_removals:
                    print(f"{datetime.datetime.now()} - [INFO] Drone '{service_name}' is being updated. Cancelling pending removal.")
                    timer = self.pending_removals.pop(service_name)
                    timer.cancel()
                    self.known_services[service_name] = drone_data
                    print(f"{datetime.datetime.now()} - [INFO] Updated Drone: {drone_data}")
                elif service_name in self.known_services:
                    print(f"{datetime.datetime.now()} - [INFO] Drone '{service_name}' already known. Treating as update.")
                    self.known_services[service_name] = drone_data
                    print(f"{datetime.datetime.now()} - [INFO] Updated Drone: {drone_data}")
                else:
                    self.known_services[service_name] = drone_data
                    print(f"{datetime.datetime.now()} - [INFO] Discovered New Drone: {drone_data}")

                self.add_drone_func(drone_data)


    def remove_service(self, zeroconf, service_type, name):
        """
        Called when a service is removed.

        :param zeroconf: Zeroconf instance.
        :param service_type: Type of the service.
        :param name: Name of the service.
        """
        # Extract full service name
        service_name = name  # Full service name including type and domain
        if service_name in self.known_services:
                base_name = service_name.split('._http._tcp.local.')[0]
                self.remove_drone_func(base_name)
                print(f"{datetime.datetime.now()} - [INFO] Removed Drone: {service_name}")
                self.known_services.pop(service_name)


    def update_service(self, zeroconf, service_type, name):
        """
        Called when a service is updated.

        :param zeroconf: Zeroconf instance.
        :param service_type: Type of the service.
        :param name: Name of the service.
        """
        print(f"{datetime.datetime.now()} - [DEBUG] Inside update_service callback for '{name}'")
        info = zeroconf.get_service_info(service_type, name)
        if info:
            drone_data = self.parse_service_info(info)
            if drone_data:
                service_name = info.name  # Full service name including type and domain
                with self.lock:
                    if service_name in self.known_services:
                        self.known_services[service_name] = drone_data
                        self.update_drone_func("service",drone_data =drone_data,name= drone_data['name'])
                        print(f"{datetime.datetime.now()} - [INFO] Updated Drone: {drone_data}")
                    else:
                        # If the drone wasn't previously known, treat it as a new drone
                        self.known_services[service_name] = drone_data
                        self.add_drone_func(drone_data)
                        print(f"{datetime.datetime.now()} - [INFO] Discovered Drone During Update: {drone_data}")

    def ping_drone(self, drone_data, retries=2):
        """
        Ping the drone to check if it is still available.

        :param drone_data: Dictionary containing drone data.
        :param retries: Number of retries if the connection is refused.
        """
        
        ip = drone_data['ip']
        port = 8001
        for attempt in range(retries):
            try:
                # print(f"{datetime.datetime.now()} - [DEBUG] Pinging drone at {ip}:{port}, attempt {attempt + 1}")
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(5)
                    s.connect((ip, port))
                    return True
            except socket.error as e:
                print(f"{datetime.datetime.now()} - [ERROR] Error pinging drone: {e}")
                # if e.errno == 111:  # Connection refused implies still alive
                #     return True
                time.sleep(1)  # Wait a bit before retrying
        return False


    def check_drones_alive(self):
        """
        Check if known drones are still alive.
        """
        while True:
            time.sleep(2)
            with self.lock:
                
                for service_name, drone_data in self.known_services.items():
                    if not self.ping_drone(drone_data):
                        base_name = service_name.split('._http._tcp.local.')[0]
                        self.remove_drone_func(base_name)
                        print(f"{datetime.datetime.now()} - [INFO] Removed Drone: {service_name}")
                        if service_name in self.known_services:
                            self.known_services.pop(service_name)       
                        break 

    def run(self):
        """
        Keep the DroneFinder running.
        """
        print(f"{datetime.datetime.now()} - [INFO] DroneFinder is running. Press Ctrl+C to exit.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print(f"{datetime.datetime.now()} - [INFO] KeyboardInterrupt received. Shutting down DroneFinder.")
        finally:
            # Cancel all pending removal timers
            with self.lock:
                for timer in self.pending_removals.values():
                    timer.cancel()
                self.pending_removals.clear()
            self.zeroconf.close()
            print(f"{datetime.datetime.now()} - [INFO] Zeroconf closed. DroneFinder stopped.")


def add_drone(drone_data):
    print(f"{datetime.datetime.now()} - Adding Drone: {drone_data}")


def remove_drone(service_name):
    print(f"{datetime.datetime.now()} - Removing Drone: {service_name}")


def update_drone(drone_data):
    print(f"{datetime.datetime.now()} - Updating Drone: {drone_data}")


if __name__ == "__main__":
    finder = DroneFinder(add_drone, remove_drone, update_drone, debounce_time=15.0)
    finder.run()
