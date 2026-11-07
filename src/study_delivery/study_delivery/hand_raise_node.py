import time 

import cv2
from ultralytics import YOLO

import rclpy 
from rclpy.node import Node
from std_msgs.msg import String  # current stage simple strings for denoting someone requesting service


class HandRaiseNode(Node):
    def __init__(self):
        super().__init__("hand_raise_detector") 
        # Publisher：send /delivery/request String messages
        self.request_publisher = self.create_publisher(String, "/delivery/request", 10)
        self.declare_parameter("camera_index", 0)  # allow using ros index to choose camera
        self.get_logger().info("Ready: raise a hand for one second to publish a request.")

    def publish_request(self, request_count):
        message = String()
        message.data = f"hand_raise:{request_count}"
        self.request_publisher.publish(message)
        self.get_logger().info(f"Published {message.data} on /delivery/request")


def run_detection(node):
    # will migrate our detection logic here.
    return 

def main(args=None):
    rclpy.init(args=args)  
    node = None
    try:
        node = HandRaiseNode()
        run_detection(node)  
    except KeyboardInterrupt:
        pass  # Ctrl+C exit
    finally:
        if node is not None:
            node.destroy_node()  
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
