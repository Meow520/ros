import rclpy
import pymongo
import json
import time
from rclpy.node import Node

from std_msgs.msg import String


class AsrMemory(Node):
    def __init__(self):
        super().__init__("asr_memory")
        self.publisher_ = self.create_publisher(String, "asr_memory", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.client = pymongo.MongoClient("mongodb://i1-brain.ad180.riken.go.jp:6000/")
        self.db = self.client["sensormemory"]
        self.recog_time = None

    def timer_callback(self):
        publish_msg = String()
        """
        if self.get_memory:
            publish_msg.data = json.dumps(self.get_memory())
            self.publisher_.publish(publish_msg)
            # self.get_logger().info(f"PUBLISHER: {publish_msg.data}")
        """

        publish_msg.data = json.dumps(self.dummy_speech())
        self.publisher_.publish(publish_msg)

    def get_memory(self) -> dict:
        current_msg = self.db["asr"].find_one(sort=[("_id", pymongo.DESCENDING)])
        current_words = current_msg["words"]
        current_recog_time = current_msg["recog_time"]
        if self.recog_time != current_recog_time:
            self.recog_time = current_recog_time
            return {"words": current_words, "recog_time": current_recog_time}

    def dummy_speech(self) -> dict:
        current_words = input()
        current_recog_time = time.time()
        return {"words": current_words, "recog_time": current_recog_time}


def main(args=None):
    rclpy.init(args=args)
    asr_memory = AsrMemory()
    rclpy.spin(asr_memory)
    asr_memory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
