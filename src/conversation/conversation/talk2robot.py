import rclpy
import pymongo
import json
from .talk_modules.scripts import get_script
from rclpy.node import Node

from std_msgs.msg import String


class Talk2Robot(Node):
    def __init__(self):
        super().__init__("talk2robot")
        self.publisher_ = self.create_publisher(String, "/butukusa", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        publish_msg = String()
        publish_msg.data = "hello"
        self.publisher_.publish(publish_msg)


def main(args=None):
    rclpy.init(args=args)
    talk2robot = Talk2Robot()
    rclpy.spin(talk2robot)
    talk2robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
