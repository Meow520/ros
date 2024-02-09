import rclpy
import pymongo
import json
from .talk_modules.scripts import get_script
from rclpy.node import Node

from std_msgs.msg import String


class Talk(Node):
    def __init__(self, script_number:int):
        super().__init__("talk")
        self.create_subscription(String, "butukusa", self.asr_callback, 10)
        self.index = 0
        self.scripts = get_script(script_number)
        

    def asr_callback(self, msg):
        subscribe_data = json.loads(msg.data)
        self.get_script()

    def get_script(self) -> dict:
        current_script = self.scripts[self.index]
        self.get_logger().info(f"SCRIPT: {current_script}")
        if self.index < len(self.scripts) - 1:
            self.index += 1
            return current_script
        else:
            exit()
        

def main(args=None):
    rclpy.init(args=args)
    talk = Talk(1)
    rclpy.spin(talk)
    talk.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
