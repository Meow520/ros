import rclpy
import pymongo
import json
import time
from rclpy.node import Node

from std_msgs.msg import String


class RoleSetting(Node):
    def __init__(self):
        super().__init__("role_setting")
        self.publisher_ = self.create_publisher(String, "role_setting", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.recog_time = None

    def timer_callback(self):
        publish_msg = String()
        publish_msg.data = json.dumps(self.get_role())
        self.publisher_.publish(publish_msg)

    def get_role(self) -> dict:
        current_role = input()
        current_recog_time = time.time()
        return {"role": current_role, "recog_time": current_recog_time}


def main(args=None):
    rclpy.init(args=args)
    role_setting = RoleSetting()
    rclpy.spin(role_setting)
    role_setting.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
