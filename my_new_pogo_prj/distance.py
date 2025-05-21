import pandas as pd
import numpy as np
from scipy.spatial.distance import pdist, squareform

dist = pd.read_feather("results/result.feather")
print(dist)


def interindividual_distance_mean(feather_path: str, communication_radius: float = 80.0) -> float:
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
        if last_positions.shape[0] < 2:  #não é isso, preciso mudar para avaliar se tem um robo vizinho dentro do raio de communication_radius: 80.0 
            continue

        # Calculation of Euclidean distances between all pairs of agents
        distance_matrix = pdist(last_positions, metric = 'euclidean')

        # Filter juts the distances <= communication_radius
        filtered_distances = distance_matrix[distance_matrix <= communication_radius]

        if len(filtered_distances) == 0:
            continue  # Aucun voisin trouvé

        mean_filtered_distances = filtered_distances.mean()
        run_distances.append(mean_filtered_distances)
        #print(last_positions)

    # Average distances over all runs
    if len(run_distances) == 0:
        return  float('nan')
    return np.mean(run_distances)

score = interindividual_distance_mean("results/result.feather", communication_radius=80.0)
print(f"Average interindividual distance : {score}")
