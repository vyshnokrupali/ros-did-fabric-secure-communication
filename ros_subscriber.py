import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import sys

DID = "did:device:subscriber001"
MAX_MESSAGES = 10  # 🔴 STOP AFTER THIS MANY

class SecureSubscriber(Node):
    def __init__(self):
        super().__init__('secure_subscriber')
        self.count = 0
        self.subscription = self.create_subscription(
            String,
            'secure_topic',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")
        self.count += 1

        if self.count >= MAX_MESSAGES:
            self.get_logger().info("Subscriber finished. Shutting down.")
            self.destroy_node()
            rclpy.shutdown()

def verify_identity():
    return subprocess.call(["python3", "identity_agent.py", DID]) == 0

def main():
    if not verify_identity():
        print("Subscriber identity invalid. Exiting.")
        sys.exit(1)

    rclpy.init()
    node = SecureSubscriber()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
