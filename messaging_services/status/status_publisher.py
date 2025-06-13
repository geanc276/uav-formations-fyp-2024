import time

from messaging_services.utils.directories import META_DATA_DIR, CONFIG_DIR, STATE_DIR
from messaging_services.utils.utils import DirectoryObserver, ServiceObserver, Publisher
from messaging_services.utils.utils import get_first_file, get_hostname, get_ip
import threading
import json

import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class Info():
    def __init__(self):
        self.config_file = ""
        self.state_file = ""
        self.launched = False

class InfoPublisher():
    def __init__(self):
        self.hostname = get_hostname()
        self.publisher = Publisher(port=5556)
        self.info = Info()
        self.info_collector = InfoCollector(self.info, self.publish)
        logger.info(f'Publisher running for {self.hostname}\t{get_ip()}')
        self.timer_thread(5)

    def publish(self):
        topic = f'{self.hostname}/status'
        logger.info(f'Publishing to {topic}:  {self.info.config_file} {self.info.state_file} {self.info.launched}')
        # message = f'{self.info.config_file} {self.info.state_file} {self.info.launched}'
        message_json = {}
        message_json['config_file'] = self.info.config_file
        message_json['state_file'] = self.info.state_file
        message_json['launched'] = self.info.launched
        message = json.dumps(message_json)

        self.publisher.publish(topic, message)
    

    def timer_thread(self, interval):
        while True:
            time.sleep(interval)
            self.publish()


class InfoCollector():
    def __init__(self, info, post_update_func):
        self.info = info
        self.post_update_func = post_update_func
        # Directories to observe
        directories = {
            'meta_data': META_DATA_DIR,
            'config': CONFIG_DIR,
            'state': STATE_DIR
        }
        self.update_info()
        self.update_launched(False)
        # Start observing the directories
        observer_manager = DirectoryObserver(directories, self.update_info)
        observer_manager.start()

        #start observing the service
        service_name = 'ros2_launch'
        # service_name = 'ros_bag_movement'
        service_observer = ServiceObserver(service_name, self.update_launched)
        service_observer.start()

        logger.info(f'Watching directories: {META_DATA_DIR} {CONFIG_DIR} {STATE_DIR}')
        logger.info(f'Watching service: {service_name}')


    def update_info(self):
        self.info.config_file = get_first_file(CONFIG_DIR)
        self.info.state_file = get_first_file(STATE_DIR)
        self.post_update_func()


    def update_launched(self, is_active):
        self.info.launched = is_active
        self.post_update_func()



if __name__ == "__main__":
    info_publisher = InfoPublisher()
    while True:
        time.sleep(1)
        pass
