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


def get_point(name:str, points:dict, rad:float = None) -> list:
    # type error処理
    if type(name) != str:
        raise TypeError("Name must be str.")
    if type(points) != dict:
        raise TypeError("Points must be dict.")
    if rad is not None:
        if type(rad) != float:
            raise TypeError("Rad must be float.")
    
    point = [0, 0, 0]
    if name in ["Home", "Kitchen", "Shelf", "TrushCan", "Dinig", "LookingDinig", "LookingLiving", "LookingTrushCan", "LookingTV"]:
        point = points[name]["position"][:]
        if rad is not None:
            point[2] = rad
        return point
    else:
        return
    