import json
from typing import Tuple
from os.path import join, dirname

file_path = join(dirname(__file__), "points.json")
print(file_path)


def get_point(name:str, file_path:str, prev_point:str, rad: float = None) -> list:
    
    # import json file
    try:
        with open(file_path) as f:
            points = json.load(f)
    except json.JSONDecodeError as e:
        print(f"json decode error: {e}")
    
    # type error処理
    if type(name) != str:
        raise TypeError("Name must be str.")
    if type(file_path) != str:
        raise TypeError("Name must be str.")
    if type(prev_point) != str:
        raise TypeError("Name must be str.")
    if rad is not None & type(rad) != float:
        raise TypeError("rad must be float.")
    
    if name in [ "Home", "Kitchen", "Shelf", "TrushCan", "Dinig", "LookingDinig", "LookingLiving", "LookingTrushCan", "LookingTV"]:
        if name == "Home":
            if prev_point != "Kitchen":
                if rad is not None:
                    points[name]["position"][2]["rad"] = rad
                return [points[name], points["HomeViaPoint"]]
        if rad is not None:
            points[name]["position"][0]["rad"] = rad
        return [points[name]]
    else:
        return
    