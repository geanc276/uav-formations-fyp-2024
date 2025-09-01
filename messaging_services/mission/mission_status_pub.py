import rclpy
from rclpy.node import Node
from std_msgs.msg import String


from messaging_services.utils.utils import Publisher
from messaging_services.utils.utils import get_hostname, get_ip

import json
class StatusListenerNode(Node):
    def __init__(self, callback):
        super().__init__('status_listener')
        self.hostname = get_hostname()
        self.callback = callback
        self.subscription = self.create_subscription(
            String,
            f'{self.hostname}/mission/status',
            self.callback,
            10)
        self.get_logger().info(f'StatusListenerNode initialized with hostname: {self.hostname}')

class MissionPublisher:
    def __init__(self):
        self.publisher = Publisher(port=5557)
        self.hostname = get_hostname()
        print(f'Publisher running for {self.hostname}\t{get_ip()}')
        self.status_listener = StatusListenerNode(self.listener_callback)
        self.status_listener.get_logger().info("Node spinning...")
        try:
            rclpy.spin(self.status_listener)
        except Exception as e:
            print(f"Exception in rclpy.spin(): {e}")
            raise
        finally:
            rclpy.shutdown()
            print("ROS 2 node shutdown")

    def listener_callback(self, msg):
        try:
            self.status_listener.get_logger().info(f'Received message: {msg.data}')
            json_data = msg.data
            # json_obj = json.loads(json_data)
            # print(json_obj)
            topic = f'{self.hostname}/mission_status'
            self.publisher.publish(topic, json_data)
            self.status_listener.get_logger().info(f'Published message: {json_data}')
            
        except Exception as e:
            self.status_listener.get_logger().error(f"Error in listener_callback: {e}")

if __name__ == '__main__':
    try:


        rclpy.init()
        print("Starting MissionPublisher...")
        mission_publisher = MissionPublisher()
    except Exception as e:
        print(f"Unhandled exception: {e}")
