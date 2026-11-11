from queue import Empty, Queue
from threading import Event, Thread

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from study_delivery.delivery_events import encode_status_event


class DeliveryDialogue(Node):
    def __init__(self):
        super().__init__("delivery_dialogue")
        self.busy = Event()  # ROS 回调和终端主线程共享的忙碌标记。
        self.pending_request = Queue(maxsize=1)  # 只把当前请求交给主线程。
        self.subscription = self.create_subscription(
            String, "/delivery/request", self.on_request, 10
        )
        self.status_publisher = self.create_publisher(String, "/delivery/status", 10)
        self.get_logger().info("[IDLE] Waiting for a hand-raise request.")

    def publish_status(self, request, state, recipient=None, detail=None):
        message = String()
        message.data = encode_status_event(request, state, recipient, detail)
        self.status_publisher.publish(message)

    def on_request(self, message):
        if self.busy.is_set():
            self.get_logger().info(f"[BUSY] Ignoring delivery request: {message.data}")
            self.publish_status(
                message.data, "rejected_busy", detail="Another request is being handled."
            )
            return

        self.busy.set()
        self.pending_request.put_nowait(message.data)
        self.publish_status(message.data, "request_received")

    def run_dialogue(self, request):
        self.get_logger().info(f"[BUSY] Handling delivery request: {request}")
        self.publish_status(request, "awaiting_confirmation")
        if ask_yes_no("Would you like me to deliver an item?"):
            self.get_logger().info("[INTENT CONFIRMED] User wants to deliver an item.")
            recipient = ask_recipient()
            self.get_logger().info(f'[REQUEST CONFIRMED] {request}: "{recipient}"')
            self.publish_status(request, "confirmed", recipient)
        else:
            self.get_logger().info("[CANCELLED] Delivery request declined.")
            self.publish_status(request, "declined")


def ask_yes_no(question):
    # omit the blank spaces and cases, repeat the same question
    while True:
        answer = input(f"{question} [yes/no] ").strip().lower()
        if answer in ("yes", "y"):
            return True
        if answer in ("no", "n"):
            return False
        print("Please enter yes or no.")


def ask_recipient():
    while True:
        description = input("Who should I deliver it to? ").strip()
        if not description:
            print("Please enter a recipient description.")
            continue

        # repeat recipient description or go back to the question
        if ask_yes_no(f'Deliver to "{description}". Is that correct?'):
            return description


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryDialogue()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    ros_thread = Thread(target=spin)
    ros_thread.start()
    try:
        while rclpy.ok():
            try:
                request = node.pending_request.get(timeout=0.2)
            except Empty:
                continue
            try:
                node.run_dialogue(request)
            finally:
                node.busy.clear()
            node.get_logger().info("[IDLE] Waiting for a hand-raise request.")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        executor.shutdown()
        ros_thread.join()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
