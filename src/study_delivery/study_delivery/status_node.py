import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from study_delivery.delivery_events import format_status_event


class ProjectStatus(Node):
    def __init__(self):
        super().__init__("study_delivery_status")
        self.subscription = self.create_subscription(
            String, "/delivery/status", self.on_status, 10
        )
        self.get_logger().info("Waiting for delivery progress and outcome events.")

    def on_status(self, message):
        try:
            status = format_status_event(message.data)
        except ValueError as error:
            self.get_logger().error(f"Invalid delivery status event: {error}")
            return
        self.get_logger().info(status)


def main(args=None):
    rclpy.init(args=args)
    node = ProjectStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
