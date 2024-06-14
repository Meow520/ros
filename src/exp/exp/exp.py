import rclpy
import json
from rclpy.node import Node
from .talk_modules.script import get_script

from common_msgs.msg import StringStamped

class Exp(Node):
    def __init__(self, script_number:str):
        super().__init__("exp")
        self.create_subscriptions(StringStamped, "/indy2/asr/meta_data", self.callback, 10)
        self.scripts = get_script(script_number)
        self.index = 0
        self.talk_robot = True
        
    def callback(self, msg):
        subscribe_data = json.loads(msg.data)
        return subscribe_data
    
    def speech(self, data):
        if self.talk_robot:
            current_script = self.scripts[self.index]
            # self.get_logger().info(f"SCRIPT: {current_script}")
            if self.index < len(self.scripts) - 1:
                self.index += 1
                self.talk_robot = False
                return current_script
                
        else:
            if len(data["words"]) > 10:
                self.talk_robot = True
        