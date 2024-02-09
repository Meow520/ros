import rclpy
import pymongo
import json
import time
import random
from rclpy.node import Node

from std_msgs.msg import String


class PerMemory(Node):
    def __init__(self):
        super().__init__("per_memory")
        self.publisher_ = self.create_publisher(String, "per_memory", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.client = pymongo.MongoClient("mongodb://i1-brain.ad180.riken.go.jp:6000/")
        self.db = self.client["sensormemory"]
        self.recog_time = None

    def timer_callback(self):
        publish_msg = String()
        
        publish_msg.data = json.dumps(self.get_memory())
        self.publisher_.publish(publish_msg)
        self.get_logger().info(f"PUBLISHER: {publish_msg.data}")
        
        # publish_msg.data = json.dumps(self.dummy_person())
        # self.publisher_.publish(publish_msg)

    def get_memory(self) -> dict:
        current_per = self.db["per"].find_one(sort=[("_id", pymongo.DESCENDING)])
        current_recog_time = current_per["created_at_ts"]
        if self.recog_time != current_recog_time:
            self.recog_time = current_recog_time
            return current_per

    def dummy_person(self) -> dict:
        objs = [
            "cup",
            "table",
            "apple",
            "laptop",
            "phone",
            "bag",
            "umbrella",
            "pencil",
            "textbook",
            "lipstick",
        ]
        index = random.randint(0, 9)
        current_obj = objs[index]
        current_recog_time = time.time()
        return {"name": current_obj, "recog_time": current_recog_time}


def main(args=None):
    rclpy.init(args=args)
    per_memory = PerMemory()
    rclpy.spin(per_memory)
    per_memory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
