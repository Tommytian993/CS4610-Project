import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from study_delivery.delivery_events import encode_status_event


class StretchAcknowledgment(Node):
    def __init__(self):
        super().__init__("stretch_ack")
        self.client = ActionClient(
            self, FollowJointTrajectory, "/stretch_controller/follow_joint_trajectory"
        )
        self.moving = False  
        self.active_request = None
        self.active_recipient = None
        self.subscription = self.create_subscription(
            String, "/delivery/request_accepted", self.on_request, 10
        )
        self.status_publisher = self.create_publisher(String, "/delivery/status", 10)
        self.get_logger().info("Stretch acknowledgment node started.")

    def publish_status(self, request, state, recipient=None, detail=None):
        message = String()
        message.data = encode_status_event(request, state, recipient, detail)
        self.status_publisher.publish(message)

    def on_request(self, message):
        request = message.data
        recipient = None

        if self.moving:
            self.get_logger().info(f"Ignoring {message.data}: motion in progress.")
            self.publish_status(
                request,
                "rejected_busy",
                recipient,
                "Stretch is already moving.",
            )
            return
        if not self.client.server_is_ready():
            self.get_logger().warning("Stretch action server is unavailable.")
            self.publish_status(
                request,
                "acknowledgment_failed",
                recipient,
                "Stretch action server is unavailable.",
            )
            return

        self.active_request = request
        self.active_recipient = recipient
        self.moving = True
        self.pickup_stages = iter((
            ("align", ["joint_lift", "wrist_extension", "joint_wrist_yaw", "joint_gripper_finger_left"],
             [[0.59, 0.16, 0.085, 0.04], [0.37, 0.16, 0.085, 0.04]]),
            ("close gripper", ["joint_gripper_finger_left"], [[-0.01]]),
            ("lift", ["joint_lift"], [[0.52]]),
        ))
        self.send_next_stage()

    def send_next_stage(self):
        stage, joints, positions = next(self.pickup_stages)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        for index, targets in enumerate(positions, start=1):
            point = JointTrajectoryPoint()
            point.positions = targets
            point.time_from_start.sec = 2 * index
            goal.trajectory.points.append(point)
        self.pickup_stage = stage
        try:
            future = self.client.send_goal_async(goal)
        except Exception as error:
            self.moving = False
            self.publish_status(
                self.active_request,
                "acknowledgment_failed",
                self.active_recipient,
                f"Could not send Stretch goal: {error}",
            )
            self.get_logger().error(f"Could not send Stretch goal: {error}")
            return
        future.add_done_callback(self.on_goal_response)
        self.get_logger().info(f"Sent pickup stage: {stage}.")

    def on_goal_response(self, future):
        try:
            handle = future.result()
            result_future = handle.get_result_async() if handle.accepted else None
        except Exception as error:
            self.get_logger().error(f"Could not track Stretch goal: {error}")
            self.moving = False
            self.publish_status(
                self.active_request,
                "acknowledgment_failed",
                self.active_recipient,
                f"Could not track Stretch motion: {error}",
            )
            return

        if not handle.accepted:
            self.get_logger().warning("Stretch rejected the pickup goal.")
            self.moving = False
            self.publish_status(
                self.active_request,
                "acknowledgment_failed",
                self.active_recipient,
                "Stretch rejected the acknowledgment motion.",
            )
            return

        self.get_logger().info("Stretch accepted the pickup goal.")
        self.publish_status(
            self.active_request,
            "acknowledgment_started",
            self.active_recipient,
        )
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        try:
            outcome = future.result()
            status = outcome.status
            error_code = outcome.result.error_code
        except Exception as error:
            self.get_logger().error(f"Could not read Stretch motion result: {error}")
            self.moving = False
            self.publish_status(
                self.active_request,
                "acknowledgment_failed",
                self.active_recipient,
                f"Could not read Stretch motion result: {error}",
            )
            return

        if (
            status == GoalStatus.STATUS_SUCCEEDED
            and error_code == FollowJointTrajectory.Result.SUCCESSFUL
        ):
            self.get_logger().info(f"Pickup stage succeeded: {self.pickup_stage}.")
            if self.pickup_stage != "lift":
                self.send_next_stage()
                return
            self.moving = False
            self.get_logger().info("Pickup trajectory succeeded.")
            self.publish_status(
                self.active_request,
                "acknowledgment_succeeded",
                self.active_recipient,
                "Pickup trajectory completed. Verify the block lifted in MuJoCo.",
            )
        else:
            self.moving = False
            self.get_logger().warning(
                f"Pickup stage {self.pickup_stage} failed: status={status}, error_code={error_code}."
            )
            self.publish_status(
                self.active_request,
                "acknowledgment_failed",
                self.active_recipient,
                f"Motion result status={status}, error_code={error_code}.",
            )


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
