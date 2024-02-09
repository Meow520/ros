import rclpy
from rclpy.node import Node
from datetime import datetime

from common_msgs.msg import StringStamped


class Talk2Robot(Node):
    def __init__(self):
        super().__init__("talk2robot")
        self.publisher_ = self.create_publisher(StringStamped, "/butukusa", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        publish_msg = StringStamped()
        publish_msg.header.stamp = datetime.now()
        publish_msg.data = "hello"
        self.publisher_.publish(publish_msg)
        self.get_logger().info(f"PUBLISHER: {publish_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    talk2robot = Talk2Robot()
    rclpy.spin(talk2robot)
    talk2robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
