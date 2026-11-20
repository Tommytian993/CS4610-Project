import time
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


THUMB_UP_SCORE = 0.75
HOLD_SECONDS = 1.0
MAX_FRAME_GAP_SECONDS = 0.5
PREVIEW_WINDOW = "Stretch Handover Gesture"


def image_to_rgb(msg, rotate_clockwise):
    """Decode a ROS color image and return contiguous RGB for MediaPipe."""
    if isinstance(msg, CompressedImage):
        encoded = np.frombuffer(msg.data, dtype=np.uint8)
        if encoded.size == 0:
            raise ValueError("Empty compressed image")

        bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError("Could not decode compressed image")

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    else:
        if msg.encoding not in ("rgb8", "bgr8"):
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")
        if msg.height <= 0 or msg.width <= 0 or msg.step < msg.width * 3:
            raise ValueError("Invalid image dimensions or row stride")

        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.step
        )
        rgb = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)

        if msg.encoding == "bgr8":
            rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)

    if rotate_clockwise:
        rgb = cv2.rotate(rgb, cv2.ROTATE_90_CLOCKWISE)

    return np.ascontiguousarray(rgb)


class HandoverGestureNode(Node):
    def __init__(self):
        super().__init__("handover_gesture")
        self.declare_parameter("image_topic", "/camera/color/image_raw/compressed")
        self.declare_parameter("model_path", "models/gesture_recognizer.task")
        self.declare_parameter("rotate_clockwise", True)
        self.declare_parameter("show_preview", False)

        self.image_topic = self.get_parameter("image_topic").value
        self.model_path = Path(self.get_parameter("model_path").value).expanduser()
        self.rotate_clockwise = self.get_parameter("rotate_clockwise").value
        self.show_preview = self.get_parameter("show_preview").value

        self.latest_frame = None
        self.last_frame_time = None
        self.last_timestamp_ms = -1
        self.thumb_up_since = None
        self.reported = False

        # Choose the ROS message type to match the selected image topic.
        image_type = (
            CompressedImage if self.image_topic.endswith("/compressed") else Image
        )

        # Keep only the newest frame instead of processing a backlog.
        self.subscription = self.create_subscription(
            image_type,
            self.image_topic,
            self.on_image,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

    def on_image(self, msg):
        self.latest_frame = (msg, time.monotonic())

    def process_latest_frame(self, recognizer):
        if self.latest_frame is None:
            return

        msg, frame_time = self.latest_frame
        self.latest_frame = None

        # Do not count a camera stall toward the gesture hold time.
        if (
            self.last_frame_time is not None
            and frame_time - self.last_frame_time > MAX_FRAME_GAP_SECONDS
        ):
            self.thumb_up_since = None
        self.last_frame_time = frame_time

        try:
            rgb = image_to_rgb(msg, self.rotate_clockwise)
        except (ValueError, cv2.error) as exc:
            self.thumb_up_since = None
            self.get_logger().warning(f"Could not read camera frame: {exc}")
            return

        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        timestamp_ms = max(int(frame_time * 1000), self.last_timestamp_ms + 1)
        self.last_timestamp_ms = timestamp_ms
        result = recognizer.recognize_for_video(image, timestamp_ms)

        top_gestures = [
            categories[0] for categories in result.gestures if categories
        ]
        thumb_up = any(
            gesture.category_name == "Thumb_Up"
            and gesture.score >= THUMB_UP_SCORE
            for gesture in top_gestures
        )

        if not thumb_up:
            self.thumb_up_since = None
            self.reported = False
            status = "Show thumbs-up"
        else:
            if self.thumb_up_since is None:
                self.thumb_up_since = frame_time

            held_seconds = frame_time - self.thumb_up_since
            if not self.reported and held_seconds >= HOLD_SECONDS:
                self.get_logger().info("Thumbs-up detected")
                self.reported = True

            if self.reported:
                status = "Detected - lower hand to reset"
            else:
                status = f"Hold: {held_seconds:.1f}/{HOLD_SECONDS:.1f} s"

        if self.show_preview:
            self.draw_preview(rgb, status, top_gestures)

    def draw_preview(self, rgb, status, top_gestures):
        """Show the same upright frame that was sent to MediaPipe."""
        preview = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        labels = [
            f"{gesture.category_name}: {gesture.score:.2f}"
            for gesture in top_gestures
        ] or ["No hand detected"]

        color = (0, 255, 0) if self.reported else (255, 255, 255)
        for index, line in enumerate([status, *labels]):
            position = (8, 22 + index * 20)
            cv2.putText(
                preview, line, position,
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA,
            )
            cv2.putText(
                preview, line, position,
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA,
            )

        cv2.imshow(PREVIEW_WINDOW, preview)
        cv2.waitKey(1)

    def run(self):
        if not self.model_path.is_file():
            raise FileNotFoundError(f"Gesture model not found: {self.model_path}")

        options = mp.tasks.vision.GestureRecognizerOptions(
            base_options=mp.tasks.BaseOptions(
                model_asset_path=str(self.model_path)
            ),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=2,
        )

        try:
            with mp.tasks.vision.GestureRecognizer.create_from_options(
                options
            ) as recognizer:
                self.get_logger().info(
                    f"Listening on {self.image_topic}. "
                    "Gesture test only. No robot motion."
                )
                while rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.2)
                    self.process_latest_frame(recognizer)
        finally:
            if self.show_preview:
                cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = HandoverGestureNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()