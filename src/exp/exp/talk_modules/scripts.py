import json
from os.path import join, dirname

file_path = join(dirname(__file__), "scripts.json")
print(file_path)


def get_script(number:str, file_path:str) -> list:
    try:
        with open(file_path) as f:
            scripts = json.load(f)
    except json.JSONDecodeError as e:
        print(f"json decode error: {e}")
        
    if type(number) != str:
        return
    if number in ["11", "12", "21", "22", "31", "32"]:
        return scripts["dialogue"+number]
    else:
        return
    