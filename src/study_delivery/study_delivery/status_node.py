"""Status-only node; no perception or robot motion is implemented."""
import rclpy
from rclpy.node import Node


class ProjectStatus(Node):
    def __init__(self):
        super().__init__("study_delivery_status")
        self.timer = self.create_timer(5.0, self.report)
        self.report()

    def report(self):
        self.get_logger().info("Project scaffold ready. Perception and robot control are not implemented.")


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
