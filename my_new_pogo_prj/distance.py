import pandas as pd
import numpy as np
from scipy.spatial.distance import pdist

dist = pd.read_feather("results/result.feather")
#print(dist)


def interindividual_distance_mean(feather_path: str, communication_radius: float = 80.0) -> float:  # communication radius en mm et x, y en cm?
    # Get the data
    df = pd.read_feather(feather_path)
    #print(f"{len(df)} lignes chargées depuis {feather_path}")

    # List to store average distance per run
    run_distances = []

    # Go through each run
    if "run" not in df.columns:
        #print("⚠️ Colonne 'run' absente, ajout d'un identifiant par défaut")
        df["run"] = 0  # ou un identifiant plus utile si tu veux

    for run_id, run_df in df.groupby("run"):
        # Last timestamp of the run
        last_time = run_df["time"].max()

        # Extract the (x,y) positions of the robots at this last moment
        last_positions = run_df[run_df["time"] == last_time][["x", "y"]].to_numpy()

        # If we have at least 2 robots
        if last_positions.shape[0] >= 2:
            distances = pdist(last_positions, metric = 'euclidean')
            # Takes just the robots that are neighbors
            filtered_distances = distances[distances <= communication_radius]


            if len(filtered_distances) > 0:
                # Average distances for this run
                mean_distance = np.mean(filtered_distances)
                run_distances.append(mean_distance)

            if len(filtered_distances) == 0:
                print(f"Run {run_id} : aucune distance dans le rayon {communication_radius}")


    # Average for all runs
    if len(run_distances) == 0:
        return float('nan')
    return np.mean(run_distances)

score = interindividual_distance_mean("results/result.feather", communication_radius = 80.0)
print(f"Average interindividual distance : {score}")
