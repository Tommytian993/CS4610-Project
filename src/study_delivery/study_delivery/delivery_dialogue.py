import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class DeliveryDialogue(Node):
    def __init__(self):
        super().__init__("delivery_dialogue")
        self.subscription = self.create_subscription(
            String, "/delivery/request", self.on_request, 10
        )
        self.get_logger().info("Waiting for a hand-raise request.")

    def on_request(self, message):
        self.get_logger().info(f"Received delivery request: {message.data}")


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryDialogue()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
