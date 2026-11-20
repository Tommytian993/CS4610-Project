import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from study_delivery.delivery_events import format_status_event, parse_status_event
from study_delivery.delivery_states import DeliveryStateTracker


class ProjectStatus(Node):
    def __init__(self):
        super().__init__("study_delivery_status")
        self.subscription = self.create_subscription(
            String, "/delivery/status", self.on_status, 10
        )
        self.state_tracker = DeliveryStateTracker()
        self.get_logger().info("Waiting for delivery progress and outcome events.")

    def on_status(self, message):
        try:
            event = parse_status_event(message.data)
            status = format_status_event(message.data)
            delivery = self.state_tracker.update(event)
        except ValueError as error:
            self.get_logger().error(f"Invalid delivery status event: {error}")
            return
        self.get_logger().info(
            f"{status} | delivery state: {delivery.phase} "
            f"(acknowledgment: {delivery.acknowledgment}, "
            f"navigation: {delivery.navigation})"
        )


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
