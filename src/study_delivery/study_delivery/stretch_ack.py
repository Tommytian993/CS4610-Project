import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint


class StretchAcknowledgment(Node):
    def __init__(self):
        super().__init__("stretch_ack")
        self.client = ActionClient(
            self, FollowJointTrajectory, "/stretch_controller/follow_joint_trajectory"
        )
        self.subscription = self.create_subscription(
            String, "/delivery/request_accepted", self.on_request, 10
        )
        self.get_logger().info("Stretch acknowledgment node started.")

    def on_request(self, message):
        if not self.client.server_is_ready():
            self.get_logger().warning("Stretch action server is unavailable.")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["joint_head_pan", "joint_gripper_slide"]
        for seconds, angle, gripper in ((2, -0.60, 0.035), (4, 0.0, -0.01)):
            point = JointTrajectoryPoint()
            point.positions = [angle, gripper]
            point.time_from_start.sec = seconds
            goal.trajectory.points.append(point)

        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)
        self.get_logger().info(f"Sent head and gripper goal for {message.data}.")

    def on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as error:
            self.get_logger().error(f"Could not get Stretch goal response: {error}")
            return

        if handle.accepted:
            self.get_logger().info("Stretch accepted the head and gripper goal.")
        else:
            self.get_logger().warning("Stretch rejected the head and gripper goal.")


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
