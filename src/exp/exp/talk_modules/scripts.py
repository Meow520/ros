import json

try:
    with open("./scripts.json") as f:
        scripts = json.load(f)
except json.JSONDecodeError as e:
    print(f"json decode error: {e}")

def get_script(number:str) -> list:
    if type(number) != str:
        return
    if number in ["11", "12", "21", "22", "31", "32"]:
        return scripts["dialogue"+number]
    else:
        return
    