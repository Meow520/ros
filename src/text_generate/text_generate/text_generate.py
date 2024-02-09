import rclpy
import json
import os
import openai
from rclpy.node import Node
from std_msgs.msg import String
from os.path import join, dirname
from dotenv import load_dotenv

load_dotenv(verbose=True)
dotenv_path = join(dirname(__file__), ".env")
load_dotenv(dotenv_path)

text: str = None
text_recog_time: str = None
obj: str = None
per = None
talking: bool = False
role: int = "person"
messages = []


class TextGenerate(Node):
    def __init__(self, api_key: str):
        super().__init__("text_generate")
        self.create_subscription(String, "role_setting", self.role_callback, 10)
        self.create_subscription(String, "obj_memory", self.obj_callback, 10)
        self.create_subscription(String, "per_memory", self.per_callback, 10)
        self.create_subscription(String, "asr_memory", self.asr_callback, 10)
        self.__api_key = api_key

    def obj_callback(self, msg: str) -> dict:
        global obj

        subscribe_data = json.loads(msg.data)
        if subscribe_data:
            # obj = subscribe_data["name"]
            obj = "apple"
            # self.get_logger().info(f"OBJ_SUBSCRIBER: {subscribe_data}")
            # self.get_logger().info(f"OBJ_SUBSCRIBER: apple")

    def per_callback(self, msg: str) -> dict:
        global per

        subscribe_data = json.loads(msg.data)
        if subscribe_data:
            per = subscribe_data["name"]
            self.get_logger().info(f"PER_SUBSCRIBER: {subscribe_data}")

    def role_callback(self, msg: str) -> dict:
        global role

        subscribe_data = json.loads(msg.data)
        if subscribe_data:
            role = subscribe_data["role"]
            self.get_logger().info(f"ROLE_SUBSCRIBER: {subscribe_data}")
            talking = False

    def asr_callback(self, msg: str) -> dict:
        global text_recog_time
        global text
        global talking
        global messages

        subscribe_data = json.loads(msg.data)
        self.get_logger().info(f"ASR_SUBSCRIBER: {subscribe_data}")
        if subscribe_data:
            text = subscribe_data["words"]
            if text_recog_time:
                if subscribe_data["recog_time"] - text_recog_time > 60:
                    talking = False
                    messages = []
                    self.get_logger().info(f"talking is false.")
            text_recog_time = subscribe_data["recog_time"]
            self.create_response(talking)

    def create_response(self, status: bool) -> str:
        global text
        global obj
        global talking
        global role
        global messages

        openai.api_key = self.__api_key
        params = []
        params.append(
            {
                "role": "system",
                "content": "Describe only the situation what user do, using user's spoken word and object information.",
            }
        )
        if len(messages) > 0:
            params.append(
                {
                    "role": "system",
                    "content": f"Previous conversations are here: {messages}",
                }
            )

        # 話している状態でない場合（話しはじめ）→ オブジェクトの情報と音声認識情報を渡す
        if not status:
            params.append(
                {
                    "role": "system",
                    "content": f"words: {text}, object: {obj}",
                }
            )
            talking = True
        # 話している状態の場合 → 音声認識情報のみ渡す
        else:
            params.append({"role": "user", "content": f"The user said {text}."})
        response = openai.ChatCompletion.create(
            model="gpt-4-1106-preview", messages=params
        )
        situation = response.choices[0].message.content
        params = [
            {"role": "system", "content": "You are Assistant."},
            {"role": "system", "content": "You must act as a native English speaker."},
            {"role": "system", "content": "You must speak Plain English."},
            {"role": "system", "content": f"The speaker is a {role}."},
            {
                "role": "system",
                "content": "Respond to speaker NATURALLY as if we were having a conversation.",
            },
        ]
        params.append(
            {"role": "user", "content": f"The user said {text}, and {situation}."}
        )
        response = openai.ChatCompletion.create(
            model="gpt-4-1106-preview", messages=params
        )
        message = response.choices[0].message.content
        params.append({"role": "assistant", "content": message})
        if len(messages) >= 50:
            summary_params = {
                "role": "system",
                "content": f"Make a summary from {messages}",
            }
            summary_res = openai.ChatCompletion.create(
                model="gpt-4-1106-preview", messages=summary_params
            )
            summary = summary_res[0].message.content
            messages = []
            messages.append({"role": "summary", "content": {summary}})
        else:
            messages.append({"role": "Speaker", "content": {text}, "object": {obj}})
            messages.append({"role": "Assistant", "content": {message}})
        self.get_logger().info(f"ROBOT:{message}")
        return message


def main(args=None):
    rclpy.init(args=args)
    text_generate = TextGenerate(
        api_key=os.environ.get("OPENAI_API_KEY"),
    )
    rclpy.spin(text_generate)
    text_generate.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
