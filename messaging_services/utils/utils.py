import datetime
import os
import socket
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
# Directory Paths
from messaging_services.utils.directories import EXECUTABLES_DIR, LOGS_DIR, ROS_LOGS_DIR, UORB_LOGS_DIR, ANY_FILE_DIR, LAUNCH_DIR, META_DATA_DIR, CONFIG_DIR, STATE_DIR, SW_INSTALL_DIR


import logging

import asyncio

import zmq  



logger = logging.getLogger(__name__)



def get_ip():
    """Get the IP address of the default network interface."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # This doesn't actually send data
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP


def get_hostname():
    """Get the hostname of the machine."""
    return socket.gethostname()


def get_first_file(directory):
    """Get the name of the first file in the given directory."""
    try:
        files = os.listdir(directory)
        files = [f for f in files if os.path.isfile(os.path.join(directory, f))]
        if files:
            files.sort()  # Sort the files to have consistent ordering
            return files[0]
        else:
            return 'No files'
    except Exception as e:
        return f'Error reading {directory}'


def get_first_meta_file():
    """Get the name of the first file in the meta_data directory."""
    return get_first_file(META_DATA_DIR)




import logging
import time
import os
from threading import Thread

class ServiceObserver:
    """Observes a service and updates on start and stop events by polling."""

    def __init__(self, service_name, update_callback, poll_interval=3):

        self.service_name = service_name
        self.update_callback = update_callback
        self.poll_interval = poll_interval
        self.running = False
        self.last_status = None

    def is_service_active(self):

        return os.system(f"systemctl is-active --quiet {self.service_name}") == 0

    def monitor(self):
        print(f"Starting to monitor service: {self.service_name}")
        logging.info(f"Starting to monitor service: {self.service_name}")
        self.running = True
        while self.running:
            try:
                print(f"Checking status of service: {self.service_name}")
                current_status = self.is_service_active()
                print(f"Service '{self.service_name}' is {'active' if current_status else 'inactive'}.")
                if current_status != self.last_status:
                    state = "active" if current_status else "inactive"
                    logging.info(f"Service '{self.service_name}' state changed to {state}.")
                    self.update_callback(current_status)
                    self.last_status = current_status
                time.sleep(self.poll_interval)
            except Exception as e:
                logging.error(f"Error while monitoring service '{self.service_name}': {e}")
                self.running = False

    def start(self):
        self.thread = Thread(target=self.monitor, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        logging.info(f"Stopped monitoring service: {self.service_name}")

class DirectoryObserver:
    """Manages observers for multiple directories."""

    def __init__(self, directories, update_service_func):
        """
        Initialize the DirectoryObserver.

        :param directories: A dictionary of directories to observe with descriptive names.
                            e.g., {'meta_data': '/path/to/meta_data', 'config': '/path/to/config'}
        :param update_service_func: The function to call when changes are detected.
        """
        self.directories = directories
        self.update_service_func = update_service_func
        self.observers = []

    def start(self):
        """Start observing the directories."""
        for name, path in self.directories.items():
            event_handler = GenericChangeHandler(name, self.update_service_func)
            observer = Observer()
            observer.schedule(event_handler, path, recursive=False)
            observer.start()
            self.observers.append(observer)
        print(f"{datetime.datetime.now()} - Observers started for directories:", self.directories.keys())

    def stop(self):
        """Stop all observers."""
        for observer in self.observers:
            observer.stop()
            observer.join()
        print(f"{datetime.datetime.now()} - Observers stopped.")


class GenericChangeHandler(FileSystemEventHandler):
    """Generic handler for file system changes."""

    def __init__(self, directory_name, update_service_func):
        """
        Initialize the GenericChangeHandler.

        :param directory_name: A descriptive name for the directory being observed.
        :param update_service_func: The function to call when changes are detected.
        """
        self.directory_name = directory_name
        self.update_service_func = update_service_func
        self.cool_down = 0.1  # Cool down period in seconds
        self.last_modified_times ={}
        self.modified_debounce_time = 1 # seconds

    def on_any_event(self, event):
        if( not event.is_directory):
            return
        

        current_time = time.time()
        last_modified_time = self.last_modified_times.get(event.src_path, 0)
        if current_time - last_modified_time < self.modified_debounce_time:
            return
        self.last_modified_times[event.src_path] = current_time

        print(event)
        print(f"SRC_PATH: {event.src_path}")
        print(f"TYPE: {event.event_type}")  
        print(f"{datetime.datetime.now()} - Detected change in {self.directory_name}, updating service...")
        self.update_service_func()
        time.sleep(self.cool_down)



class Publisher:
    def __init__(self,port=5556):
        self.context = zmq.Context()
        self.ip = get_ip()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://{self.ip}:{port}")
        logger.info(f'Publisher bound to {self.ip}:{port}')
        print(f'Publisher bound to {self.ip}:{port}')

    def publish(self, topic, message):
        self.socket.send_string(f'{topic} {message}')
        logger.info(f'Published: {self.socket} {topic} {message}')
        print(f'Published: {topic} {message}')