import pandas as pd
import numpy as np
from scipy.spatial.distance import pdist

#dist = pd.read_feather("results/result.feather")
#print(dist)


def interindividual_distance_mean(feather_path: str) -> float:
    # Lire les données
    df = pd.read_feather(feather_path)

    # Liste pour stocker la distance moyenne par run
    run_distances = []

    # Parcourir chaque run
    for run_id, run_df in df.groupby("run"):
        # Dernier timestamp de la run
        last_time = run_df["time"].max()

        # Extraire les positions (x, y) des robots à ce dernier moment
        last_positions = run_df[run_df["time"] == last_time][["x", "y"]].to_numpy()

        # Si on a au moins 2 robots
        if last_positions.shape[0] < 2:
            continue

        # Calcul des distances euclidiennes entre tous les couples d'agents
        distances = pdist(last_positions, metric='euclidean')
        mean_distance = distances.mean()
        run_distances.append(mean_distance)
        #print(last_positions)

    # Moyenne des distances sur tous les runs
    return np.mean(run_distances)

score = interindividual_distance_mean("results/result.feather")
print(f"Distance interindividuelle moyenne : {score}")
