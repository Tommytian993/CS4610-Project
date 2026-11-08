import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class StretchAcknowledgment(Node):
    def __init__(self):
        super().__init__("stretch_ack")
        self.client = ActionClient(
            self, FollowJointTrajectory, "/stretch_controller/follow_joint_trajectory"
        )
        self.get_logger().info("Stretch acknowledgment node started.")


def main(args=None):
    rclpy.init(args=args)
    node = StretchAcknowledgment()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
