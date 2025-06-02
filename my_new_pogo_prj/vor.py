import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree, Voronoi
from shapely.geometry import Polygon, box, Point
import yaml


# ---------- Utility: Load arena info from YAML and CSV ----------

def load_arena_polygon_and_surface(yaml_path: str):
    with open(yaml_path, "r") as f:
        conf = yaml.safe_load(f)
    arena_file = conf["arena_file"]
    arena_surface = conf["arena_surface"]

    # Read CSV file - handle both comma and other separators
    try:
        df_arena = pd.read_csv(arena_file, header=None, names=["x", "y"])
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        print("Trying with different separator...")
        df_arena = pd.read_csv(arena_file, header=None, names=["x", "y"], sep=None, engine='python')
    
    print(f"Loaded {len(df_arena)} points from arena file")
    print(f"First few points:\n{df_arena.head()}")
    
    polygon = Polygon(df_arena[["x", "y"]].to_numpy())

    if not polygon.is_valid:
        print("Arena polygon is invalid, attempting to fix...")
        # Try to fix invalid polygon
        polygon = polygon.buffer(0)
        if not polygon.is_valid:
            raise ValueError("Arena polygon is invalid and cannot be fixed.")

    bounds = polygon.bounds  # (minx, miny, maxx, maxy)
    
    # Convert arena_surface to numeric type
    try:
        if isinstance(arena_surface, str):
            arena_surface = float(arena_surface)
        elif arena_surface is None:
            # Calculate area from polygon if not provided
            arena_surface = polygon.area
            print(f"Arena surface not provided, calculated from polygon: {arena_surface}")
    except (ValueError, TypeError):
        # Fallback: calculate area from polygon
        arena_surface = polygon.area
        print(f"Could not parse arena_surface, calculated from polygon: {arena_surface}")
    
    print(f"Arena bounds: {bounds}")
    print(f"Arena surface: {arena_surface}")
    
    return polygon, bounds, arena_surface


# ---------- k-NN Distance Plotting ----------

def interindividual_distance_knn_over_time_plot(
    feather_path: str, k: int = 5, communication_radius: float = 133.0, time_step: int = 100
):
    df = pd.read_feather(feather_path)
    print("Unique time values:", df["time"].unique())
    print("Max time:", df["time"].max())

    if "run" not in df.columns:
        df["run"] = 0

    all_distances = []

    for run_id, run_df in df.groupby("run"):
        time_points = sorted(df["time"].unique())
        time_distances = []

        for t in time_points:
            current_positions = run_df[run_df["time"] == t][["x", "y"]].to_numpy()

            if current_positions.shape[0] > k:
                tree = KDTree(current_positions)
                distances, _ = tree.query(current_positions, k=k+1)
                knn_distances = distances[:, 1:]
                filtered_knn = knn_distances[knn_distances <= communication_radius]

                if len(filtered_knn) > 0:
                    mean_distance = np.mean(filtered_knn)
                    std_distance = np.std(filtered_knn)
                    time_distances.append((t, mean_distance, std_distance))
                else:
                    time_distances.append((t, np.nan, np.nan))
            else:
                time_distances.append((t, np.nan, np.nan))

        if time_distances:
            df_time = pd.DataFrame(time_distances, columns=["time", "mean_distance", "std_distance"])
            df_time["run"] = run_id
            all_distances.append(df_time)

    if not all_distances:
        print("No valid k-NN distances computed.")
        return

    result_df = pd.concat(all_distances)

    # Global mean over runs
    grouped = result_df.groupby("time")
    global_mean = grouped["mean_distance"].mean()
    global_std = grouped["mean_distance"].std()

    plt.figure(figsize=(10, 6))
    plt.plot(global_mean.index, global_mean.values, label="Global mean", color="blue")
    plt.fill_between(global_mean.index,
                     global_mean - global_std,
                     global_mean + global_std,
                     color="blue", alpha=0.3, label="± 1 stddev")

    plt.xlabel("Time")
    plt.ylabel(f"Mean distance to the {k} neighbors")
    plt.title(f"Global average of k-NN metric (k={k}, radius={communication_radius})")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    return result_df


# ---------- Voronoi Metrics ----------

def compute_voronoi_metrics(df, arena_polygon, arena_bounds, arena_surface, communication_radius=133.0, time_step=100):
    bounding_box = box(*arena_bounds)
    results = []

    if "run" not in df.columns:
        df["run"] = 0

    for run_id, run_df in df.groupby("run"):
        time_points = sorted(df["time"].unique())

        for t in time_points:
            points = run_df[run_df["time"] == t][["x", "y"]].to_numpy()
            if len(points) < 4:
                continue

            vor = Voronoi(points)
            areas = []
            covered_area = 0.0

            for idx, region_index in enumerate(vor.point_region):
                region = vor.regions[region_index]
                if not region or -1 in region:
                    continue

                polygon = Polygon([vor.vertices[i] for i in region])
                if not polygon.is_valid:
                    continue

                clipped = polygon.intersection(bounding_box)
                if clipped.area > 0:
                    areas.append(clipped.area)

            # Coverage estimate using circular communication ranges
            for pt in points:
                circle = Point(pt).buffer(communication_radius)  # Fixed: Point(pt) instead of points(pt)
                covered_area += circle.intersection(arena_polygon).area

            if areas:
                mean_area = np.mean(areas)
                std_area = np.std(areas)
                var_area = np.var(areas)
                coverage_ratio = covered_area / arena_surface  # Now arena_surface is guaranteed to be numeric
                results.append((run_id, t, mean_area, std_area, var_area, len(areas), coverage_ratio))

    result_df = pd.DataFrame(results, columns=[
        "run", "time", "mean_area", "std_area", "var_area", "n_cells", "coverage_ratio"
    ])
    return result_df


def plot_voronoi_variance(df_voronoi):
    plt.figure(figsize=(10, 6))
    for run_id, group in df_voronoi.groupby("run"):
        plt.plot(group["time"], group["var_area"], label=f"Run {run_id}")
    plt.xlabel("Time")
    plt.ylabel("Voronoi Cell Area Variance")
    plt.title("Voronoi Cell Area Variance Over Time")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


def plot_voronoi_global_variance(df_voronoi):
    grouped = df_voronoi.groupby("time")
    global_mean = grouped["var_area"].mean()
    global_std = grouped["var_area"].std()

    plt.figure(figsize=(10, 6))
    plt.plot(global_mean.index, global_mean.values, label="Global mean", color="green")
    plt.fill_between(global_mean.index,
                     global_mean - global_std,
                     global_mean + global_std,
                     color="green", alpha=0.3, label="± 1 stddev")
    plt.xlabel("Time")
    plt.ylabel("Voronoi Cell Area Variance")
    plt.title("Global Average of Voronoi Cell Area Variance Over Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def plot_coverage_ratio(df_voronoi):
    grouped = df_voronoi.groupby("time")
    global_mean = grouped["coverage_ratio"].mean()
    global_std = grouped["coverage_ratio"].std()

    plt.figure(figsize=(10, 6))
    plt.plot(global_mean.index, global_mean.values, label="Coverage ratio mean", color="purple")
    plt.fill_between(global_mean.index,
                     global_mean - global_std,
                     global_mean + global_std,
                     color="purple", alpha=0.3, label="± 1 stddev")
    plt.xlabel("Time")
    plt.ylabel("Coverage Ratio")
    plt.title("Communication Coverage Over Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


# -------------------------------
# Example usage
# -------------------------------
yaml_path = "conf/simple.yaml"
arena_polygon, arena_bounds, arena_surface = load_arena_polygon_and_surface(yaml_path)

feather_path = "results/result.feather"
df = pd.read_feather(feather_path)

# k-NN plot
interindividual_distance_knn_over_time_plot(feather_path, k=3, communication_radius=133.0, time_step=100)

# Voronoi metrics with real arena
df_voronoi = compute_voronoi_metrics(df, arena_polygon, arena_bounds, arena_surface, communication_radius=133.0)

plot_voronoi_variance(df_voronoi)
plot_voronoi_global_variance(df_voronoi)
plot_coverage_ratio(df_voronoi)