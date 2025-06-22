import os
import yaml
import shutil
import tempfile
import subprocess
import numpy as np
import pandas as pd
import pyarrow.feather as feather
from vor import compute_voronoi_metrics
from cma import CMAEvolutionStrategy
from data import arena_polygon, arena_bounds, arena_surface
import concurrent.futures

# Force software rendering to avoid accidental GPU usage
os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

# Configuration
SIMULATOR_BINARY = "./examples/run_and_tumble/run_and_tumble"
BASE_CONFIG_PATH = "conf/simpleb.yaml"
TEMP_DIR = "tmp_cma"
N_RUNS_PER_INDIVIDUAL = 2
PARAMETER_KEYS = [
    "run_duration_min",
    "run_duration_max",
    "tumble_duration_min",
    "tumble_duration_max"
]
INITIAL_VALUES = [10, 50, 10, 50]  # x0, dx0, x1, dx1
SIGMA = 800
MAX_ITER = 2
OUTPUT_CSV = "cmaes_results.csv"

best_score = float("inf")
best_params = None

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

def run_simulation(config_path):
    try:
        subprocess.run(
            [SIMULATOR_BINARY, "-c", config_path, "-nr", "-q", "-g"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
    except subprocess.CalledProcessError as e:
        print("STDERR:", e.stderr.decode())
        print("STDOUT:", e.stdout.decode())
        return None

    feather_path = os.path.join(os.path.dirname(config_path), "frames", "data.feather")
    return feather_path if os.path.exists(feather_path) else None

def run_one_simulation(p):
    run_dir = tempfile.mkdtemp(dir=TEMP_DIR)
    config_path = create_temp_config(BASE_CONFIG_PATH, run_dir, p)
    feather_file = run_simulation(config_path)

    if feather_file:
        try:
            df = feather.read_feather(feather_file)
            metrics_df = compute_voronoi_metrics(
                df=df,
                arena_polygon=arena_polygon,
                arena_bounds=arena_bounds,
                arena_surface=arena_surface,
                communication_radius=133.0
            )
            mean_cv = metrics_df["cv_area"].mean()
            shutil.rmtree(run_dir, ignore_errors=True)
            return mean_cv
        except Exception as e:
            print(f"Error during analysis: {e}")
    shutil.rmtree(run_dir, ignore_errors=True)
    return None

def objective_function(params):

    global best_score, best_params
    
    # Reparameterize: ensure p[0] < p[1] and p[2] < p[3]
    x0, dx0, x1, dx1 = params
    p = [
        int(x0),
        int(x0 + abs(dx0)),
        int(x1),
        int(x1 + abs(dx1))
    ]

    cv_values = []
    with concurrent.futures.ProcessPoolExecutor() as executor:
        futures = [executor.submit(run_one_simulation, p) for _ in range(N_RUNS_PER_INDIVIDUAL)]
        for future in concurrent.futures.as_completed(futures):
            result = future.result()
            if result is not None:
                cv_values.append(result)
                print(f"Mean CV: {result:.4f} for parameters: {p}")
                with open(OUTPUT_CSV, 'a') as f:
                    f.write(",".join(map(str, p)) + f",{result:.6f}\n")

                # track best score
                if result < best_score:
                    best_score = result 
                    best_params = p 
            else:
                print(f"Simulation failed for: {p}")

    return -np.mean(cv_values) if cv_values else 1e6

def main():
    os.makedirs(TEMP_DIR, exist_ok=True)

    bounds = [[0, 1, 0, 1], [10000, 10000, 10000, 10000]]  # x0, dx0, x1, dx1

    es = CMAEvolutionStrategy(INITIAL_VALUES, SIGMA, {
        'bounds': bounds,
        'maxiter': MAX_ITER,
    })

    with open(OUTPUT_CSV, 'w') as f:
        f.write("run_duration_min,run_duration_max,tumble_duration_min,tumble_duration_max,mean_cv\n")

    es.optimize(objective_function)

    print("Optimal parameters found:", es.result.xbest)
    print(f"Best evaluated parameters: {best_params} with mean CV = {best_score:.6f}")
    shutil.rmtree(TEMP_DIR, ignore_errors=True)

if __name__ == "__main__":
    main()
