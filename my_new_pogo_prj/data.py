#import pandas as pd
#df = pd.read_feather("results/result.feather")
#print(df)

import yaml
import os

# Getting the arena surface from simple.yaml

def load_arena_surface(yaml_path: str) -> float:
    with open(yaml_path, "r") as f:
        config = yaml.safe_load(f)
    surface = float(config.get("arena_surface"))  
    print(surface)
    return surface

yaml_path = os.path.join("conf", "simple.yaml")
load_arena_surface(yaml_path)



