import openai
import os
import rclpy
import json
from dotenv import load_dotenv
from os.path import join, dirname
from typing import Tuple, Optional, List
from random import randint
from pyindy.mongo import UtteranceManager, NavigationManager, GestureManager
from rclpy.node import Node
from .talk_modules.scripts import get_script
from .talk_modules.points import get_point, get_points_file

from common_msgs.msg import StringStamped  # type: ignore

load_dotenv(verbose=True)
dotenv_path = join(dirname(__file__), ".env")
load_dotenv(dotenv_path)


class Exp(Node):
    def __init__(self, script: list, um: any, nm: any, ges: any):
        super().__init__("exp")
        self.create_subscription(StringStamped, "asr_dummy", self.asr_callback, 10)
        self.script = script
        self.um = um
        self.nm = nm
        self.ges = ges
        self.points: dict = get_points_file("src/exp/exp/talk_modules/points.json")
        self.index: int = 0
        self.correct: bool = True
        self.stop: bool = False
        self.current_pos: str = "Home"

    def asr_callback(self, msg):
        if self.stop:
            exit()
        subscribe_data = json.loads(msg.data)
        self.get_logger().info(f"USER SPEECH:{subscribe_data}")
        res_script, res_position, res_rad, res_ges = self.speech_no_feedback()
        if res_position:
            res_nav = get_point(name=res_position, points=self.points)
            self.current_pos = res_position
        elif res_rad:
            res_nav = get_point(name=self.current_pos, points=self.points, rad=res_rad)
            self.get_logger().info(f"CURRENT_POS:{self.current_pos}, RES_NAV:{res_nav}")
        else:
            res_nav = None
        self.current_pos = res_position if res_position else self.current_pos
        self.get_logger().info(f"SCRIPT:{res_script}, POSITION:{res_nav}, RAD:{res_rad}, GESTURE:{res_ges}")
        
        # send text data (robot speaking)
        self.um.insert({"text":res_script})
        
        # send position data (robot moving)
        if res_nav:
            self.nm.insert({"navigation":res_nav})
        if res_ges:
            self.ges.insert({"gesture":res_ges})
        
    def speech_no_feedback(self) -> Tuple[str, Optional[list], Optional[float], Optional[str]]:
        # put the command of repeat or skip
        cmd = input("Input your command (1: repeat, 2:skip, enter:none):")

        # repeat
        if cmd == "1":
            pass

        # skip
        elif cmd == "2":
            self.index += 4

        # none
        else:
            self.index += 2

        if self.index > len(self.script):
            self.stop = True
            self.nm.insert({"navigation":get_point(name="Home", points=self.points)})
            exit()
        
        current_script = self.script[self.index]["utterance"]
        current_position = self.script[self.index]["position"] if self.script[self.index]["position"] else None
        current_rad = self.script[self.index]["rad"] if self.script[self.index]["rad"] else None
        current_ges = self.script[self.index]["gesture"] if self.script[self.index]["gesture"] else None
        return current_script, current_position, current_rad, current_ges

    def speech(self, data) -> Tuple[str, list, Optional[float], Optional[str]]:
        # put the command of repeat or skip
        cmd = input("Input your command (1: repeat, 2:skip, enter:none):")

        # repeat
        if cmd == "1":
            current_script = self.script[self.index]["utterance"]
            current_position = self.script[self.index]["position"] if self.script[self.index]["position"] else None
            current_rad = self.script[self.index]["rad"] if self.script[self.index]["rad"] else None
            current_ges = self.script[self.index]["gesture"] if self.script[self.index]["gesture"] else None
            return current_script, None, None, current_ges

        # skip or none
        else:
            # feedback of user speech
            feedback = self.feedback(data=data)
            if feedback == "correct" or self.correct:
                if cmd == "2":
                    self.index += 4
                else:
                    self.index += 2
                    
                if self.index > len(self.script):
                    self.stop = True
                    exit()
                    
                current_script = self.script[self.index]
                current_position = self.script[self.index]["position"] if self.script[self.index]["position"] else None
                current_rad = self.script[self.index]["rad"] if self.script[self.index]["rad"] else None
                current_ges = self.script[self.index]["gesture"] if self.script[self.index]["gesture"] else None
                return current_script, current_position, current_rad, current_ges

            else:
                return feedback, None, None, None

    # user speech feedback
    def feedback(self, data) -> str:
        # 調整済み
        openai.api_key = os.environ.get("OPENAI_API_KEY")
        user_speech = data["words"]
        predicted_speech = self.script[self.index + 1]["utterance"]
        correct_examples = self.script[self.index + 1]["ex"]["correct"]
        correct_examples.append(predicted_speech)
        current_script = self.script[self.index]
        params = [
        {"role": "system", "content": "You are a peer of the user who is an English learner talking with you."},
        {
            "role": "system",
            "content": "You must evaluate whether user's speaking is lexically and grammatically correct or not.",
        },
        {
            "role": "system",
            "content": f"Here are some sample answer,  {correct_examples}",
        },
        {
            "role": "system",
            "content": "If you judjed user spoke English correctly, please output ONLY 'correct'",
        },
        {
            "role": "system",
            "content": "Otherwise, please output ONLY the modified sentence, don't output any other advices.",
        },
        {
            "role": "system",
            "content": "The answer MUST NOT include 'The user said' and 'The robot said'",
        }
        ]
        params.append(
            {
                "role": "user",
                "content": f"The robot said {current_script},  and The user said {user_speech}",
            }
        )
        response = openai.ChatCompletion.create(model="gpt-4o-mini", messages=params)
        message = response.choices[0].message.content
        if message.lower() == "correct":
            self.correct = True
            return "correct"
        elif message == user_speech:
            return "correct"
        else:
            customary_epithet = ["That means, ", "You should say, "]
            self.correct = False
            return customary_epithet[randint(0, 1)] + message


def main(args=None):
    script_number = input(
        "1文字目はタスク、2文字目はフェーズを半角数字で打ち込む 例) タスク1つ目フェーズ1であれば「11」"
    )
    script = get_script(
        number=script_number, file_path="src/exp/exp/talk_modules/scripts.json"
    )
    um = UtteranceManager("i1-brain.ad180.riken.go.jp", 9010, "indy1")
    nm = NavigationManager("i1-brain.ad180.riken.go.jp", 9010, "indy1")
    ges = GestureManager("i1-brain.ad180.riken.go.jp", 9010, "indy1")
    # robot speaks first script before user speaking
    um.insert({"text": script[0]["utterance"]})
    print(({"text": script[0]["utterance"]}))

    rclpy.init(args=args)
    exp = Exp(script=script, um=um, nm=nm, ges=ges)
    rclpy.spin(exp)
    exp.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
