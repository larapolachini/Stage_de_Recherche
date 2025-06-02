import pandas as pd
from shapely.geometry import Polygon
import yaml
import matplotlib.pyplot as plt


def load_arena_polygon_from_csv(arena_file_path):
    df = pd.read_csv(arena_file_path, header=None, names=["x", "y"])
    return Polygon(df[["x", "y"]].to_numpy())

def get_bounds_from_polygon(polygon):
    minx, miny, maxx, maxy = polygon.bounds
    return (minx, miny, maxx, maxy)


# Load YAML
with open("conf/simple.yaml", "r") as f:
    config = yaml.safe_load(f)

arena_file = config["arena_file"]
arena_surface = config["arena_surface"]
arena_polygon = load_arena_polygon_from_csv(arena_file)
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
plt.figure(figsize=(6, 6))
plt.plot(x, y, color='blue')
plt.fill(x, y, alpha=0.3, color='cyan')
plt.gca().set_aspect('equal')
plt.title("Arena Polygon")
plt.grid(True)
plt.show()
