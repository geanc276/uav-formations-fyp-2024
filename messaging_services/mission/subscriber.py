# subscriber.py
import zmq
from messaging_services.utils.utils import get_hostname


def subscriber():
    # Create a ZeroMQ context
    context = zmq.Context()
    # Create a SUB socket
    socket = context.socket(zmq.SUB)
    # Connect to the publisher of drone_1 for testing
    pub_ip = "192.168.0.11"
    socket.connect(f"tcp://{pub_ip}:5557")
    print("Connected to publisher")
    # Subscribe to a specific topic
    topic = f"/status"
    # socket.setsockopt_string(zmq.SUBSCRIBE, topic)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all topics

    print(f"Subscribing to {topic}")
    print("Subscriber is running...")

    print("Press Ctrl+C to stop the subscriber.")
    try:
        while True:
            # Receive messages
            message = socket.recv_string()
            print(f"Received: {message}")
    except KeyboardInterrupt:
        print("Subscriber stopped.")
    finally:
        socket.close()
        context.term()

if __name__ == "__main__":
    subscriber()
