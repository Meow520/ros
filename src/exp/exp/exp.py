import openai
import os
import rclpy
import json
from dotenv import load_dotenv
from os.path import join, dirname
from pyindy.mongo import UtteranceManager
from rclpy.node import Node
from .talk_modules.scripts import get_script

from common_msgs.msg import StringStamped

load_dotenv(verbose=True)
dotenv_path = join(dirname(__file__), ".env")
load_dotenv(dotenv_path)

class Exp(Node):
    def __init__(self, script:list, um:any):
        super().__init__("exp")
        self.create_subscription(StringStamped, "/asr/meta_data", self.callback, 10)
        self.script = script
        self.index = 0
        self.correct = True
        self.um = um
        self.stop = False
        
    def callback(self, msg):
        if self.stop:
            exit()
        subscribe_data = json.loads(msg.data)
        self.get_logger().info(f"USER SPEECH:{subscribe_data}")
        # res = self.speech(subscribe_data)
        res = self.dummy_speech()
        self.get_logger().info(f"SCRIPT:{res}")
        
        # send text data robot speaking
        # self.um.insert({"text":res})
        
        
    def dummy_speech(self) -> str:
        # put the command of repeat or skip
        cmd = input("Input your command (1: repeat, 2:skip, enter:none):")
        
        # repeat
        if cmd == "1":
            script = self.script[self.index]["utterance"]
        
        #skip
        elif cmd == "2":
            script = self.script[self.index + 4]["utterance"]
            self.index += 4

        # skip or none
        else:
            script = self.script[self.index + 2]["utterance"]  # next robot script is after the user predicted speech
            self.index += 2
        
        if self.index > len(self.script):
            self.stop = True
        
        return script

    
    def speech(self, data) -> str:
        # put the command of repeat or skip
        cmd = input("Input your command (1: repeat, 2:skip, enter:none):")
        
        # repeat
        if cmd == "1":
            current_script = self.script[self.index]["utterance"]
            return current_script
        
        # skip or none
        else:
            # feedback of user speech
            feedback = self.feedback(data=data)
            if feedback == "correct" or self.correct:
                if cmd == "2":
                    current_script = self.script[self.index + 4]
                    self.index += 4
                    return current_script
                next_script = self.script[self.index + 2]["utterance"]  # next robot script is after the user predicted speech
                self.index += 2
                return next_script
            
            else:
                return feedback
    
    # user speech feedback
    def feedback(self, data) -> str:
        """
        data usecase
        
        num_frame
        duration
        asr_model
        words
        scores
        recog_time
        sep_port
        """
        openai.api_key = os.environ.get("OPENAI_API_KEY")
        user_speech = data["words"]
        # predicted_speech = self.scripts[self.index + 1]
        current_script = self.script[self.index]
        params = [
            {"role":"system", "content":"You are Assistant."},
            {"role":"system", "content":"You must evaluate whether user's speaking is correct or not."},
            # {"role":"system", "content":f"We have scripts and now we predict that user's speech is going to be {predicted_speech}."},
            {"role":"system", "content":"If you judjed user spoke English correctly, please output only 'correct', otherwise, please advise to correct the mistake."}
            # 直す部分をもう少し細かくする few-shot 正解例と不正解例
        ]
        params.append({"role":"user", "content": f"The robot said {current_script},  and The user said {user_speech}"})
        response = openai.ChatCompletion.create(
            model="gpt-4-1106-preview", messages=params
        )
        message = response.choices[0].message.content
        print(message)
        if message == "correct":
            self.correct = True
            return "correct"
        else:
            self.correct = False
        return message

        
def main(args=None):
    script_number = input("1文字目はタスク、2文字目はフェーズを半角数字で打ち込む 例) タスク1つ目フェーズ1であれば「11」")
    script = get_script(script_number)
    # um = UtteranceManager("http://i1-brain.ad180.riken.go.jp", 9020, "indy2")
    um = ""
    # robot speaks first script before user speaking
    # um.insert
    print(({"text": script[0]["utterance"]}))
    
    rclpy.init(args=args)
    exp = Exp(script=script, um = um)
    rclpy.spin(exp)
    exp.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()