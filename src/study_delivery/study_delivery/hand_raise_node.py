import time 

import cv2
import numpy as np
from ultralytics import YOLO

import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String  # current stage simple strings for denoting someone requesting service


class HandRaiseNode(Node):
    def __init__(self):
        super().__init__("hand_raise_detector") 
        # Publisher：send /delivery/request String messages
        self.request_publisher = self.create_publisher(String, "/delivery/request", 10)
        self.declare_parameter("camera_index", 0)  # allow using ros index to choose camera
        self.declare_parameter("image_topic", "")
        self.get_logger().info("Ready: raise a hand for one second to publish a request.")

    def publish_request(self, request_count):
        message = String()
        message.data = f"hand_raise:{request_count}"
        self.request_publisher.publish(message)
        self.get_logger().info(f"Published {message.data} on /delivery/request")


def image_to_bgr(msg):
    """Convert a raw ROS color image, accounting for row padding."""
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"Unsupported camera encoding: {msg.encoding}")
    if msg.height <= 0 or msg.width <= 0 or msg.step < msg.width * 3:
        raise ValueError("Invalid color image dimensions or row stride")
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    frame = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    if msg.encoding == "rgb8":
        frame = frame[:, :, ::-1]
    return frame.copy()


def run_detection(node):
    model = YOLO("yolo11n-pose.pt")  # human pose model
    image_topic = node.get_parameter("image_topic").value
    camera = None
    subscription = None
    latest_image = None

    def receive_image(msg):
        nonlocal latest_image
        latest_image = msg

    raised_since = None  
    lowered_since = None 
    triggered = False  # locked in until both hands are down
    request_count = 0  
    try:
        if image_topic:
            subscription = node.create_subscription(
                Image, image_topic, receive_image,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            node.get_logger().info(f"Waiting for color images on {image_topic}")
        else:
            camera_index = node.get_parameter("camera_index").value
            camera = cv2.VideoCapture(camera_index)
            if not camera.isOpened():
                raise RuntimeError(f"Cannot open webcam at index {camera_index}.")

        while rclpy.ok():
            if subscription is not None:
                rclpy.spin_once(node, timeout_sec=0.2)
                if latest_image is None:
                    raised_since = lowered_since = None
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                    continue
                frame = image_to_bgr(latest_image)
                latest_image = None
            else:
                ok, frame = camera.read()
                if not ok:
                    print("Cannot read camera frame. Exiting.")
                    break

            # detect pose key points
            result = model.predict(
                frame, verbose=False, save=False, save_txt=False, save_crop=False
            )[0]
            annotated_frame = result.plot()  

            # first version lets only detect one person to prevent overlap
            # will show this when not detecting, detecting mutiple or keypoints unclear
            status = "SHOW ONE PERSON AND BOTH WRISTS"
            color = (0, 200, 255) # opencv -> BGR
            observation = None  # None for unclear (not hands down)
            now = time.monotonic()

            # result is this frame's yolo detection, 
            # the first selection checks there are keypoints,
            # second selection check if these belong to the same person.
            # * len(result.keypoints) is number of people not keypoints here.
            if result.keypoints is not None and len(result.keypoints) == 1:

                # xy saves each person's keypoint pixels, we only need the first at index zero
                # turn it into a numpy array of 17 (keypoints) by 2 (x,y coords).
                points = result.keypoints.xy[0].cpu().numpy()

                # confidence about the keypoint's placement
                confidence = result.keypoints.conf

                if confidence is not None:

                    # keypoint confidence array
                    scores = confidence[0].cpu().numpy()

                    # assume no hand raised
                    raised = False

                    # visible sides of arm and wrist, could be 0,1,2
                    visible_arms = 0

                    # frame.shape is the height in pixels,
                    # this is how much wrist must be higher than shoulder for the detection
                    margin = frame.shape[0] * 0.04
                    # COCO key points：left shoulder 5, right shoulder 6, left wrist 9, right wrist 10
                    for shoulder, wrist in ((5, 9), (6, 10)):

                        # for now wrist and shoulders values must be at least 0.5
                        # we could change later if hard to detect
                        if scores[shoulder] >= 0.5 and scores[wrist] >= 0.5:
                            visible_arms += 1

                            # get the y coord (0 is x), 
                            # upper-left is origin, so lower in the image higher y
                            if points[wrist, 1] < points[shoulder, 1] - margin:

                                # one side satisfies
                                raised = True
                    
                    # checked both sides, show the detection
                    if raised:
                        observation = True  # at least one hand raised
                    elif visible_arms == 2:
                        observation = False  # clear on both sides, but not raised

            # turn frame detection into one request
            if observation is True:
                lowered_since = None
                if triggered:
                    status, color = "REQUEST SENT - LOWER BOTH HANDS", (0, 255, 0)
                else:
                    if raised_since is None:
                        raised_since = now
                    elapsed = now - raised_since
                    status = f"RAISING: {min(elapsed, 1.0):.1f}/1.0s"
                    if elapsed >= 1.0:  # need to maintain raised for at least one second to trigger
                        triggered = True
                        request_count += 1
                        node.publish_request(request_count)
                        status, color = "REQUEST SENT", (0, 255, 0)
                        print(f"[Request {request_count}] Would you like me to deliver an item?",
                              flush=True)
            elif observation is False:
                raised_since = None
                if lowered_since is None:
                    lowered_since = now
                if now - lowered_since >= 0.5:  # release after both hands lowered for more than a second
                    triggered = False
                status = "LOWER BOTH HANDS TO RESET" if triggered else "IDLE - RAISE A HAND"
            else:
                # stop counting when unclear, no one or multiple people in sight
                raised_since = None
                lowered_since = None

            cv2.putText(annotated_frame, f"Requests: {request_count}", (20, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            cv2.putText(annotated_frame, status, (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
            cv2.imshow("Hand Raise Demo", annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break  
    finally:
        if camera is not None:
            camera.release()
        if subscription is not None:
            node.destroy_subscription(subscription)
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)  
    node = None
    try:
        node = HandRaiseNode()
        run_detection(node)  
    except KeyboardInterrupt:
        pass  # Ctrl+C exit
    finally:
        if node is not None:
            node.destroy_node()  
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
