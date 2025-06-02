import pandas as pd
import numpy as np
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi
from shapely.geometry import Polygon, Point, box

def interindividual_distance_knn_over_time_plot(
    feather_path: str, k: int = 3, communication_radius: float = 133.0):
    df = pd.read_feather(feather_path)

    if "run" not in df.columns:
        df["run"] = 0

    all_distances = []

    for run_id, run_df in df.groupby("run"):
        time_points = run_df["time"].unique()
        time_points.sort()

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
                    time_distances.append((t, np.nan, np.nan))  # No neighbors within radius
            else:
                time_distances.append((t, np.nan, np.nan))  # There are not enough robots

        if time_distances:
            df_time = pd.DataFrame(time_distances, columns=["time", "mean_distance", "std_distance"])
            df_time["run"] = run_id
            all_distances.append(df_time)

    # Combines data from all runs
    if not all_distances:
        print("No valid metrics were calculated.")
        return

    result_df = pd.concat(all_distances)

    # Calculate global mean and standard deviation over time
    grouped = result_df.groupby("time")["mean_distance"]
    mean_over_runs = grouped.mean()
    std_over_runs = grouped.std()

    plt.figure(figsize=(10, 6))
    plt.plot(mean_over_runs.index, mean_over_runs.values, color="blue", label="Global mean")
    plt.fill_between(
        mean_over_runs.index,
        mean_over_runs - std_over_runs,
        mean_over_runs + std_over_runs,
        color="blue",
        alpha=0.3,
        label="± 1 stddev"
    )

    plt.xlabel("Time")
    plt.ylabel(f"Mean distance to the {k} neighbors")
    plt.title(f"Global average of k-NN metric (k={k}, radius={communication_radius})")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("global_knn_mean_std.png")
    


    # Graphic
    plt.figure(figsize=(10, 6))
    for run_id, group in result_df.groupby("run"):
        plt.plot(group["time"], group["mean_distance"], label=f"Run {run_id}", alpha=0.7)

        plt.fill_between(
            group["time"],
            group["mean_distance"] - group["std_distance"],
            group["mean_distance"] + group["std_distance"],
            alpha = 0.2,
            label = f"Run {run_id} ± std"
        )

    plt.xlabel("Time")
    plt.ylabel(f"Mean distance to the {k} neighbors")
    plt.ylim(0, communication_radius+5)
    plt.title(f"Evolution of the metric k-NN (k={k}, radius={communication_radius})")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("knn_distance_over_time.png")
    plt.show()

    #return result_df

interindividual_distance_knn_over_time_plot("results/result.feather", k=3, communication_radius=133.0)



def compute_voronoi_metrics(df, arena_bounds=(0, 0, 1000, 1000), time_step=100):

    minx, miny, maxx, maxy = arena_bounds
    bounding_box = box(minx, miny, maxx, maxy)

    results = []

    if "run" not in df.columns:
        df["run"] = 0

    for run_id, run_df in df.groupby("run"):
        max_time = run_df["time"].max()
        time_points = np.arange(0, max_time + 1, time_step)

        for t in time_points:
            points = run_df[run_df["time"] == t][["x", "y"]].to_numpy()

            if len(points) < 4:
                continue  # Not enough points to form Voronoi

            vor = Voronoi(points)
            areas = []

            for region_index in vor.point_region:
                region = vor.regions[region_index]
                if not region or -1 in region:
                    continue  # Infinite region

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

df = pd.read_feather("results/result.feather")
voronoi_df = compute_voronoi_metrics(df, arena_bounds=(0, 0, 1000, 1000), time_step=100)
plot_voronoi_variance(voronoi_df)
