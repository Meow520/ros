import rclpy
import json
from rclpy.node import Node

from common_msgs.msg import StringStamped # type: ignore

class AsrDummy(Node):
    def __init__(self):
        super().__init__("asr_dummy")
        self.publisher_ = self.create_publisher(StringStamped, "asr_dummy", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        publish_msg = StringStamped()
        publish_msg.header.stamp = self.get_clock().now().to_msg()
        publish_msg.data = json.dumps(self.dummy_speech())
        self.get_logger().info(publish_msg.data)
        self.publisher_.publish(publish_msg)

    def dummy_speech(self) -> dict:
        current_words = input()
        current_recog_time = str(self.get_clock().now().nanoseconds * 1e-9)
        return {"words": current_words, "recog_time": current_recog_time}

def main(args=None):
    rclpy.init(args=args)
    asr_dummy = AsrDummy()
    rclpy.spin(asr_dummy)
    asr_dummy.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
