from math import cos, sin

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from study_delivery.delivery_events import encode_status_event
from study_delivery.side_navigation import parse_navigation_request, pose_for_side


class SideNavigation(Node):
    def __init__(self):
        super().__init__("side_navigation")
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateToPose

        self.PoseStamped = PoseStamped
        self.NavigateToPose = NavigateToPose

        self.declare_parameter("navigation_action", "/navigate_to_pose")
        self.declare_parameter("navigation_frame", "map")
        self.declare_parameter("left_pose", [2.0, 0.0, 0.0])
        self.declare_parameter("right_pose", [2.0, 0.0, 0.0])

        action_name = self.get_parameter("navigation_action").value
        self.client = ActionClient(self, NavigateToPose, action_name)
        self.moving = False
        self.active_request = ""
        self.active_recipient = ""
        self.subscription = self.create_subscription(
            String, "/delivery/navigation_request", self.on_request, 10
        )
        self.status_publisher = self.create_publisher(
            String, "/delivery/status", 10
        )
        self.get_logger().info("Side navigation node started.")

    def publish_status(self, request, state, recipient=None, detail=None):
        message = String()
        message.data = encode_status_event(request, state, recipient, detail)
        self.status_publisher.publish(message)

    def on_request(self, message):
        try:
            request = parse_navigation_request(message.data)
        except ValueError as error:
            self.get_logger().error(str(error))
            self.publish_status(
                "unknown", "navigation_failed", detail=str(error)
            )
            return

        if self.moving:
            self.publish_status(
                request["request"],
                "rejected_busy",
                request["recipient"],
                "Side navigation is already in progress.",
            )
            return
        if not self.client.server_is_ready():
            self.publish_status(
                request["request"],
                "navigation_failed",
                request["recipient"],
                "Navigation action server is unavailable.",
            )
            return

        left = self.get_parameter("left_pose").value
        right = self.get_parameter("right_pose").value
        x, y, yaw = pose_for_side(request["side"], left, right)

        pose = self.PoseStamped()
        pose.header.frame_id = self.get_parameter("navigation_frame").value
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = sin(float(yaw) / 2.0)
        pose.pose.orientation.w = cos(float(yaw) / 2.0)

        goal = self.NavigateToPose.Goal()
        goal.pose = pose
        self.active_request = request["request"] or "navigation"
        self.active_recipient = request["recipient"]
        self.moving = True

        try:
            future = self.client.send_goal_async(goal)
            future.add_done_callback(self.on_goal_response)
        except Exception as error:
            self.moving = False
            self.publish_status(
                self.active_request,
                "navigation_failed",
                self.active_recipient,
                f"Could not send navigation goal: {error}",
            )

    def on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as error:
            self.moving = False
            self.publish_status(
                self.active_request,
                "navigation_failed",
                self.active_recipient,
                f"Could not track navigation goal: {error}",
            )
            return

        if not handle.accepted:
            self.moving = False
            self.publish_status(
                self.active_request,
                "navigation_failed",
                self.active_recipient,
                "Navigation rejected the delivery goal.",
            )
            return

        self.publish_status(
            self.active_request,
            "navigation_started",
            self.active_recipient,
        )
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        self.moving = False
        try:
            result = future.result()
        except Exception as error:
            self.publish_status(
                self.active_request,
                "navigation_failed",
                self.active_recipient,
                f"Could not read navigation result: {error}",
            )
            return

        if getattr(result, "status", None) == 4:
            self.publish_status(
                self.active_request,
                "navigation_succeeded",
                self.active_recipient,
            )
        else:
            self.publish_status(
                self.active_request,
                "navigation_failed",
                self.active_recipient,
                f"Navigation result status={getattr(result, 'status', 'unknown')}.",
            )


def main(args=None):
    rclpy.init(args=args)
    node = SideNavigation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
