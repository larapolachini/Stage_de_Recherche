import pandas as pd
import numpy as np
from scipy.spatial import KDTree

def interindividual_distance_knn_mean(feather_path: str, k: int = 3, communication_radius: float = 133.0) -> float:
    df = pd.read_feather(feather_path)

    if "run" not in df.columns:
        df["run"] = 0

    run_distances = []

    for run_id, run_df in df.groupby("run"):
        last_time = run_df["time"].max()
        last_positions = run_df[run_df["time"] == last_time][["x", "y"]].to_numpy()

        if last_positions.shape[0] > k:
            tree = KDTree(last_positions)

            distances, _ = tree.query(last_positions, k=k + 1)  
            knn_distances = distances[:, 1:]  # remove distance 0 for itself

            valid_distances = []
            for dist_array in knn_distances:
                filtered = dist_array[dist_array <= communication_radius]
                if len(filtered) > 0:
                    valid_distances.extend(filtered)

            if len(valid_distances) == 0:
                print(f"Run {run_id}: no neighbors in communication radius: {communication_radius}")
                continue

            mean_distance = np.mean(valid_distances)
            run_distances.append(mean_distance)
        else:
            print(f"Run {run_id}: insuficient number of robots to k={k}")

    if len(run_distances) == 0:
        return float('nan')

    return np.mean(run_distances)



score = interindividual_distance_knn_mean("results/result.feather", k=3, communication_radius=133.0)
print(f"Average interindividual distance to {3} nearest neighbors within radius 133 : {score}")
