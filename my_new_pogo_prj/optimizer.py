import os
import yaml
import shutil
import tempfile
import subprocess
import numpy as np
import pandas as pd
from cma import CMAEvolutionStrategy
from scipy.spatial.distance import pdist
from distance import interindividual_distance_mean


# Configuration 
SIMULATOR_BINARY = "./examples/run_and_tumble/run_and_tumble"
BASE_CONFIG_PATH = "conf/simple.yaml"
TEMP_DIR = "tmp_cma"
N_RUNS_PER_INDIVIDUAL = 2
PARAMETER_KEYS = [
    "run_duration_min",
    "run_duration_max",
    "tumble_duration_min",
    "tumble_duration_max"
]
INITIAL_VALUES = [100, 500, 2000, 5000]
SIGMA = 300
MAX_ITER = 5


# === Créer config avec paramètres ===
def create_temp_config(base_config_path, output_dir, parameters):
    with open(base_config_path, 'r') as f:
        config = yaml.safe_load(f)

    for key, val in zip(PARAMETER_KEYS, parameters):
        config["parameters"][key] = int(val)

    config["seed"] = np.random.randint(0, 100000)
    frames_dir = os.path.join(output_dir, "frames")
    os.makedirs(frames_dir, exist_ok=True)
    config["data_filename"] = os.path.join(frames_dir, "data.feather")

    temp_config_path = os.path.join(output_dir, "simple.yaml")
    with open(temp_config_path, 'w') as f:
        yaml.safe_dump(config, f)
    return temp_config_path

# === Lancer la simulation ===
def run_simulation(config_path):
    try:
        subprocess.run(
            [SIMULATOR_BINARY, "-c", config_path, "-nr", "-g", "-q"],
            check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
    except subprocess.CalledProcessError:
        return None

    feather_path = os.path.join(os.path.dirname(config_path), "frames", "data.feather")
    return feather_path if os.path.exists(feather_path) else None

# === Fonction objectif ===
def objective_function(parameters):
    p = list(map(int, parameters))
    if not (p[0] < p[1] and p[2] < p[3]):
        return 1e6  # pénalité si violation des contraintes

    distances = []
    run_dirs = []

    for _ in range(N_RUNS_PER_INDIVIDUAL):
        run_dir = tempfile.mkdtemp(dir=TEMP_DIR)
        run_dirs.append(run_dir)
        config_path = create_temp_config(BASE_CONFIG_PATH, run_dir, p)
        feather_file = run_simulation(config_path)
        if feather_file:
            dist = interindividual_distance_mean(feather_file)
            distances.append(dist)

    # Nettoyage
    for d in run_dirs:
        shutil.rmtree(d, ignore_errors=True)

    if not distances:
        return 1e6

    return -np.mean(distances)  # Maximiser la distance → minimiser -distance

# === Optimisation CMA-ES ===
def main():
    if not os.path.exists(TEMP_DIR):
        os.makedirs(TEMP_DIR)

    es = CMAEvolutionStrategy(INITIAL_VALUES, SIGMA, {'maxiter': MAX_ITER})
    es.optimize(objective_function)


    print("Paramètres optimaux trouvés :", es.result.xbest)
    shutil.rmtree(TEMP_DIR, ignore_errors=True)

if __name__ == "__main__":
    main()
