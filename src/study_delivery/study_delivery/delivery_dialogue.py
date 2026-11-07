import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class DeliveryDialogue(Node):
    def __init__(self):
        super().__init__("delivery_dialogue")
        self.subscription = self.create_subscription(
            String, "/delivery/request", self.on_request, 10
        )
        self.get_logger().info("Waiting for a hand-raise request.")

    def on_request(self, message):
        self.get_logger().info(f"Received delivery request: {message.data}")
        if ask_yes_no("Would you like me to deliver an item?"):
            self.get_logger().info("[INTENT CONFIRMED] User wants to deliver an item.")
            recipient = ask_recipient()
            self.get_logger().info(f'[REQUEST CONFIRMED] {message.data}: "{recipient}"')
        else:
            self.get_logger().info("[CANCELLED] Delivery request declined.")


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
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, EOFError, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
