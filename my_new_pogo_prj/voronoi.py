import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree, Voronoi
from shapely.geometry import Polygon, box
import yaml
import os


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


def compute_voronoi_metrics(df, arena_bounds=(0, 0, 1000, 1000), time_step=100):
    minx, miny, maxx, maxy = arena_bounds
    bounding_box = box(minx, miny, maxx, maxy)
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

            for region_index in vor.point_region:
                region = vor.regions[region_index]
                if not region or -1 in region:
                    continue

                polygon = Polygon([vor.vertices[i] for i in region])
                if not polygon.is_valid:
                    continue

                clipped = polygon.intersection(bounding_box)
                if clipped.area > 0:
                    areas.append(clipped.area)

            if areas:
                mean_area = np.mean(areas)
                std_area = np.std(areas)
                var_area = np.var(areas)
                results.append((run_id, t, mean_area, std_area, var_area, len(areas)))

    result_df = pd.DataFrame(results, columns=["run", "time", "mean_area", "std_area", "var_area", "n_cells"])
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


# -------------------------------
# Example usage
# -------------------------------
feather_path = "results/result.feather"
df = pd.read_feather(feather_path)

# k-NN metric plot
interindividual_distance_knn_over_time_plot(feather_path, k=3, communication_radius=133.0, time_step=100)

# Compute and plot Voronoi metrics
df_voronoi = compute_voronoi_metrics(df, arena_bounds=(0, 0, 1000, 1000), time_step=100)
plot_voronoi_variance(df_voronoi)
plot_voronoi_global_variance(df_voronoi)
