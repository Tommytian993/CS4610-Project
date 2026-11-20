import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


READY_ARM_JOINTS = (
    "joint_lift", "wrist_extension", "joint_wrist_yaw", "joint_wrist_pitch",
    "joint_wrist_roll",
)


class PickupController(Node):

    def __init__(self):
        super().__init__("pickup_controller")

        # Safety: robot motion is disabled by default.
        self.declare_parameter("enable_motion", False)

        # Must be calibrated on the real robot.
        self.declare_parameter(
            "close_position",
            float("nan")
        )

        self.declare_parameter(
            "duration_seconds",
            3.0
        )

        self.declare_parameter(
            "joint_name",
            "joint_gripper_finger_left"
        )

        self.declare_parameter(
            "action_name",
            "/stretch_controller/follow_joint_trajectory"
        )

        self.enabled = bool(
            self.get_parameter("enable_motion").value
        )

        self.close_position = float(
            self.get_parameter("close_position").value
        )

        self.duration = float(
            self.get_parameter("duration_seconds").value
        )

        self.joint_name = str(
            self.get_parameter("joint_name").value
        )

        self.declare_parameter("hand_raise_mode", False)
        self.declare_parameter("ready_joints", [*READY_ARM_JOINTS[:3], self.joint_name])
        self.ready_joints = self.get_parameter("ready_joints").value
        self.declare_parameter("ready_position", [float("nan")] * len(self.ready_joints))
        self.declare_parameter(
            "ready_approach_position", [float("nan")] * len(self.ready_joints)
        )
        self.hand_raise_mode = self.get_parameter("hand_raise_mode").value
        self.ready_position = self.get_parameter("ready_position").value
        self.ready_approach_position = self.get_parameter("ready_approach_position").value
        self.armed = False
        self.busy = False
        self.finished = False
        self.declare_parameter("enable_release", False)
        self.declare_parameter("release_position", float("nan"))
        self.enable_release = self.get_parameter("enable_release").value
        self.release_position = self.get_parameter("release_position").value
        self.waiting_for_release = False
        if self.enabled and self.enable_release and not (
            math.isfinite(self.release_position)
            and math.isfinite(self.close_position)
            and self.release_position > self.close_position
            and math.isfinite(self.duration)
            and self.duration > 0
        ):
            raise ValueError("Configure calibrated close and release positions and a positive duration.")

        # Connect to the Stretch trajectory controller.
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("action_name").value)
        )

        # Subscribe to gesture recognition events.
        self.subscription = self.create_subscription(
            String,
            "/delivery/handover_gesture",
            self.on_gesture,
            10
        )

        if self.hand_raise_mode:
            self.hand_raise_subscription = self.create_subscription(
                String, "/delivery/request", self.on_hand_raise, 10
            )

        # Explicitly enable one pickup attempt in manual mode.
        self.arm_service = self.create_service(
            Trigger,
            "/delivery/arm_pickup",
            self.arm
        )

        self.get_logger().info(
            f"Pickup controller ready. "
            f"Motion enabled: {self.enabled}. "
            "DISARMED."
        )

    def arm(self, request, response):
        """Allow exactly one thumbs-up pickup attempt."""

        if self.hand_raise_mode:
            response.success = False
            response.message = "Raise your hand to prepare pickup."
        elif self.busy:
            response.success = False
            response.message = (
                "Motion in progress; cannot arm."
            )

        elif self.finished:
            response.success = False
            response.message = (
                "Already triggered once; restart node to reset."
            )

        elif (
            self.enabled
            and not math.isfinite(self.close_position)
        ):
            response.success = False
            response.message = (
                "Set calibrated close_position "
                "before enabling motion."
            )

        elif self.enabled and self.duration <= 0:
            response.success = False
            response.message = (
                "duration_seconds must be positive."
            )

        elif (
            self.enabled
            and not self.client.server_is_ready()
        ):
            response.success = False
            response.message = (
                "Stretch trajectory action server unavailable."
            )

        else:
            self.armed = True

            response.success = True
            response.message = (
                "Armed for ONE thumbs-up. "
                "Clear hands from gripper."
            )

        self.get_logger().info(response.message)

        return response

    def on_hand_raise(self, message):
        if not message.data.startswith("hand_raise:"):
            return
        if self.busy or self.armed or self.finished:
            return

        if not self.enabled:
            self.armed = True
            self.get_logger().info(
                "DRY RUN: Hand raise received; WOULD MOVE TO READY. Waiting for Thumb_Up."
            )
            return

        if (
            self.joint_name not in self.ready_joints
            or len(set(self.ready_joints)) != len(self.ready_joints)
            or not set(self.ready_joints).issubset((*READY_ARM_JOINTS, self.joint_name))
            or len(self.ready_position) != len(self.ready_joints)
            or not all(math.isfinite(position) for position in self.ready_position)
            or not math.isfinite(self.close_position)
            or not math.isfinite(self.duration)
            or self.duration <= 0
        ):
            self.get_logger().error(
                "Configure valid ready joints, calibrated positions, and a positive duration."
            )
            return

        if not self.client.server_is_ready():
            self.get_logger().error("Stretch action server unavailable. No motion sent.")
            return

        positions = [self.ready_position]
        if not all(math.isnan(position) for position in self.ready_approach_position):
            if len(self.ready_approach_position) != len(self.ready_joints) or not all(
                math.isfinite(position) for position in self.ready_approach_position
            ):
                self.get_logger().error("Configure one finite approach position per ready joint.")
                return
            positions.insert(0, self.ready_approach_position)
        self.send_trajectory("READY", self.ready_joints, positions)

    def on_gesture(self, message):
        """Handle a confirmed gesture event."""

        if message.data == "Thumb_Down":
            self.on_release()
            return
        if message.data != "Thumb_Up":
            return

        if not self.armed:
            self.get_logger().info(
                "Thumb_Up ignored: controller not armed."
            )
            return

        if self.busy or self.finished:
            return

        # Consume the confirmation immediately.
        self.armed = False
        self.finished = True

        if not self.enabled:
            self.waiting_for_release = self.enable_release
            self.get_logger().info(
                "DRY RUN: Thumb_Up received; "
                "WOULD CLOSE GRIPPER. No motion sent."
            )
            return

        if not self.client.server_is_ready():
            self.get_logger().error(
                "Stretch action server unavailable. "
                "No motion sent."
            )
            return

        self.send_trajectory("CLOSE", [self.joint_name], [[self.close_position]])

    def on_release(self):
        if not self.enable_release or self.busy or not self.waiting_for_release:
            return
        if not self.enabled:
            self.waiting_for_release = False
            self.get_logger().info("DRY RUN: Thumb_Down received; WOULD OPEN GRIPPER.")
            return
        if not self.client.server_is_ready():
            self.get_logger().error("Stretch action server unavailable. No release sent.")
            return
        self.send_trajectory("RELEASE", [self.joint_name], [[self.release_position]])

    def send_trajectory(self, stage, joints, positions):

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = joints

        for index, target in enumerate(positions, start=1):
            point = JointTrajectoryPoint()
            point.positions = target
            seconds, remainder = divmod(round(index * self.duration * 1_000_000_000), 1_000_000_000)
            point.time_from_start.sec = seconds
            point.time_from_start.nanosec = remainder
            goal.trajectory.points.append(point)

        self.busy = True

        self.get_logger().info(
            f"Sending pickup {stage} trajectory."
        )

        try:
            future = self.client.send_goal_async(goal)

            future.add_done_callback(
                lambda response: self.on_goal_response(response, stage)
            )

        except Exception as exc:
            self.busy = False

            self.get_logger().error(
                f"Could not send goal: {exc}"
            )

    def on_goal_response(self, future, stage):
        """Check whether Stretch accepted the trajectory."""

        try:
            handle = future.result()

            if not handle.accepted:
                self.busy = False

                self.get_logger().error(
                    f"Pickup {stage} goal rejected."
                )
                return

            self.get_logger().info(
                f"Pickup {stage} goal accepted; waiting for result."
            )

            result_future = handle.get_result_async()

            result_future.add_done_callback(
                lambda result: self.on_result(result, stage)
            )

        except Exception as exc:
            self.busy = False

            self.get_logger().error(
                f"Goal response failure: {exc}"
            )

    def on_result(self, future, stage):
        """Report final trajectory result."""

        self.busy = False

        try:
            outcome = future.result()

            result = outcome.result

            success = (
                outcome.status == GoalStatus.STATUS_SUCCEEDED
                and result.error_code
                == FollowJointTrajectory.Result.SUCCESSFUL
            )

            if success:
                if stage == "READY":
                    self.armed = True
                    self.get_logger().info("Pickup ready. Waiting for Thumb_Up.")
                elif stage == "CLOSE":
                    self.waiting_for_release = self.enable_release
                    self.get_logger().info("PICKUP CLOSE SUCCEEDED.")
                    if self.waiting_for_release:
                        self.get_logger().info("Waiting for Thumb_Down to release.")
                else:
                    self.waiting_for_release = False
                    self.get_logger().info("PICKUP RELEASE SUCCEEDED.")

            else:
                self.get_logger().error(
                    f"PICKUP {stage} FAILED: "
                    f"status={outcome.status}, "
                    f"error_code={result.error_code}, "
                    f"error_string={result.error_string!r}"
                )

        except Exception as exc:
            self.get_logger().error(
                f"Could not read action result: {exc}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = PickupController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()