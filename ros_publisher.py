import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import sys

DID = "did:device:publisher001"
MAX_MESSAGES = 10  # 🔴 LIMIT PUBLISHING

class SecurePublisher(Node):
    def __init__(self):
        super().__init__('secure_publisher')
        self.publisher_ = self.create_publisher(String, 'secure_topic', 10)
        self.timer = self.create_timer(2.0, self.publish_message)
        self.count = 0  # 🔴 message counter

    def publish_message(self):
        if self.count >= MAX_MESSAGES:
            self.get_logger().info("Publisher finished. Shutting down.")
            self.timer.cancel()     # stop timer
            self.destroy_node()     # destroy node
            rclpy.shutdown()        # stop ROS
            return

        msg = String()
        msg.data = f"Hello from VERIFIED publisher #{self.count + 1}"
        self.publisher_.publish(msg)
        self.get_logger().info(msg.data)
        self.count += 1

def verify_identity():
    return subprocess.call(["python3", "identity_agent.py", DID]) == 0

def main():
    if not verify_identity():
        print("Publisher identity invalid. Exiting.")
        sys.exit(1)

    rclpy.init()
    node = SecurePublisher()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
