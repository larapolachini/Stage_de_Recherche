import pandas as pd
import numpy as np
from scipy.spatial import KDTree
import matplotlib.pyplot as plt

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
                    time_distances.append((t, mean_distance))
                else:
                    time_distances.append((t, np.nan))  # No neighbors within radius
            else:
                time_distances.append((t, np.nan))  # There are not enough robots

        if time_distances:
            df_time = pd.DataFrame(time_distances, columns=["time", "mean_distance"])
            df_time["run"] = run_id
            all_distances.append(df_time)

    # Combines data from all runs
    if not all_distances:
        print("No valid metrics were calculated.")
        return

    result_df = pd.concat(all_distances)

    # Graphic
    plt.figure(figsize=(10, 6))
    for run_id, group in result_df.groupby("run"):
        plt.plot(group["time"], group["mean_distance"], label=f"Run {run_id}", alpha=0.7)

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
