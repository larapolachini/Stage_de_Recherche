#import pandas as pd
#df = pd.read_feather("results/result.feather")
#print(df)

# Getting the arena surface from simple.yaml

import pandas as pd
from shapely.geometry import Polygon
import yaml
import matplotlib.pyplot as plt
from shapely import affinity


def load_arena_polygon_from_csv(arena_file_path):
    with open(arena_file_path, 'r') as f:
        lines = f.readlines()

    loops = []
    current_loop = []

    for line in lines:
        stripped = line.strip()
        if not stripped:
            if current_loop:
                loops.append(current_loop)
                current_loop = []

        else:
            x, y = map(float, stripped.split(','))
            current_loop.append((x, y))

    if current_loop:
        loops.append(current_loop)

    if len(loops) == 0:
        raise ValueError("No valid loops found in arena file")

    shell = loops[0]
    holes = loops[1:] if len(loops) > 1 else []

    polygon = Polygon(shell, holes=holes)

    if not polygon.is_valid:
        raise ValueError("Loaded polygon is invalid")
    
    return polygon


def get_bounds_from_polygon(polygon):
    minx, miny, maxx, maxy = polygon.bounds
    bounds = polygon.bounds
    return bounds

# Load YAML
with open("conf/simple.yaml", "r") as f:
    config = yaml.safe_load(f)


arena_file = config["arena_file"]
arena_surface = float(config["arena_surface"])
arena_polygon = load_arena_polygon_from_csv(arena_file)

scale_factor = (arena_surface/arena_polygon.area) ** 0.5  # Scale in x and y

minx, miny, _, _ = arena_polygon.bounds
arena_polygon = affinity.translate(arena_polygon, xoff=-minx, yoff=-miny)
arena_polygon = affinity.scale(arena_polygon, xfact=scale_factor, yfact=scale_factor, origin=(0, 0))


arena_bounds = get_bounds_from_polygon(arena_polygon)

print(f"Arena file: {arena_file}")
print(f"Arena surface (from YAML): {arena_surface} mm²")

print("Polygon bounds (minx, miny, maxx, maxy):", arena_bounds)
print("Polygon area (from CSV):", arena_polygon.area, "mm²")

# Check geometry
print("Is polygon valid?", arena_polygon.is_valid)
print("Is polygon simple?", arena_polygon.is_simple)
print("Number of points in polygon:", len(arena_polygon.exterior.coords))


x, y = arena_polygon.exterior.xy
plt.plot(x, y, color='blue')
plt.fill(x, y, alpha=0.3, color='cyan', label="Outer boundary")


# Plot holes
for interior in arena_polygon.interiors:
    xh, yh = interior.xy
    plt.plot(xh, yh, color='red')
    plt.fill(xh, yh, color='white', label='Hole')

plt.gca().set_aspect('equal')
plt.title("Scaled Arena Polygon")
plt.grid(True)
plt.legend()
plt.show()


print(arena_bounds)