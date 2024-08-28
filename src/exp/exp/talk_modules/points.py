import json
from os.path import join, dirname

file_path = join(dirname(__file__), "points.json")
print(file_path)

def get_points_file(file_path:str):
        # import json file
    try:
        with open(file_path) as f:
            points = json.load(f)
            return points
    except json.JSONDecodeError as e:
        print(f"json decode error: {e}")


def get_point(name:str, points:dict, prev_point:str, rad:float = None) -> list:
    # type error処理
    if type(name) != str:
        raise TypeError("Name must be str.")
    if type(points) != dict:
        raise TypeError("Points must be dict.")
    if type(prev_point) != str:
        raise TypeError("Prev_point must be str.")
    if rad is not None:
        if type(rad) != float:
            raise TypeError("Rad must be float.")
    
    if name in ["Home", "Kitchen", "Shelf", "TrushCan", "Dinig", "LookingDinig", "LookingLiving", "LookingTrushCan", "LookingTV"]:
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
    