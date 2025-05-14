import pandas as pd
import numpy as np
from scipy.spatial.distance import pdist

#dist = pd.read_feather("results/result.feather")
#print(dist)


def interindividual_distance_mean(feather_path: str) -> float:
    # Get the data
    df = pd.read_feather(feather_path)

    # List to store average distance per run
    run_distances = []

    # Go through each run
    for run_id, run_df in df.groupby("run"):
        # Last timestamp of the run
        last_time = run_df["time"].max()

        # Extract the (x,y) positions of the robots at this last moment
        last_positions = run_df[run_df["time"] == last_time][["x", "y"]].to_numpy()

        # If we have at least 2 robots
        if last_positions.shape[0] < 2:
            continue

        # Calculation of Euclidean distances between all pairs of agents
        distances = pdist(last_positions, metric='euclidean')
        mean_distance = distances.mean()
        run_distances.append(mean_distance)
        #print(last_positions)

    # Average distances over all runs
    return np.mean(run_distances)

score = interindividual_distance_mean("results/result.feather")
print(f"Average interindividual distance : {score}")
