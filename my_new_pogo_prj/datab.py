import pandas as pd
from shapely.geometry import Polygon
import yaml
import matplotlib.pyplot as plt
from shapely import affinity
import os

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
    return polygon.bounds

def process_arena(arena_file, arena_surface):
    arena_polygon = load_arena_polygon_from_csv(arena_file)

    scale_factor = (arena_surface / arena_polygon.area) ** 0.5
    minx, miny, _, _ = arena_polygon.bounds
    arena_polygon = affinity.translate(arena_polygon, xoff=-minx, yoff=-miny)
    arena_polygon = affinity.scale(arena_polygon, xfact=scale_factor, yfact=scale_factor, origin=(0, 0))

    arena_bounds = get_bounds_from_polygon(arena_polygon)

    print(f"\n=== Processing Arena: {arena_file} ===")
    print(f"Arena surface (target): {arena_surface:.2f} mm²")
    print(f"Polygon area (after scaling): {arena_polygon.area:.2f} mm²")
    print("Bounds:", arena_bounds)
    print("Is polygon valid?", arena_polygon.is_valid)
    print("Number of outer boundary points:", len(arena_polygon.exterior.coords))

    # Plot
    x, y = arena_polygon.exterior.xy
    plt.plot(x, y, color='blue')
    plt.fill(x, y, alpha=0.3, color='cyan', label="Outer boundary")

    for interior in arena_polygon.interiors:
        xh, yh = interior.xy
        plt.plot(xh, yh, color='red')
        plt.fill(xh, yh, color='white', label='Hole')

    plt.gca().set_aspect('equal')
    plt.title(f"Arena: {os.path.basename(arena_file)}")
    plt.grid(True)
    plt.legend()
    #plt.savefig(f"arena_polygon_{os.path.basename(arena_file)}.png")      # check the shape of the arena

    return arena_polygon, arena_bounds

# -----------------------------------------
# Load common values from the YAML template
# -----------------------------------------
with open("conf/simpleb.yaml", "r") as f:
    config = yaml.safe_load(f)

arena_surface = float(config["arena_surface"])
arena_files = config["arena_file"]["batch_options"]

# Process all arenas
for arena_file in arena_files:
    process_arena(arena_file, arena_surface)
