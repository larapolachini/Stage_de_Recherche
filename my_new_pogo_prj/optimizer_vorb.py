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
from shapely.geometry import Polygon
from shapely import affinity
from data import load_arena_polygon_from_csv
import concurrent.futures
import matplotlib.pyplot as plt
from vor import save_figure

# Force software rendering to avoid accidental GPU usage
os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

# Configuration
SIMULATOR_BINARY = "./examples/run_and_tumble/run_and_tumble"
BASE_CONFIG_PATH = "conf/simpleb.yaml"
TEMP_DIR = "tmp_cma"
N_RUNS_PER_INDIVIDUAL = 5
PARAMETER_KEYS = [
    "run_duration_min",
    "run_duration_max",
    "tumble_duration_min",
    "tumble_duration_max"
]
INITIAL_VALUES = [10, 50, 10, 50]  # x0, dx0, x1, dx1
SIGMA = 10
MAX_ITER = 20
OUTPUT_CSV = "cmaes_results.csv"

best_score = float("inf")
best_params = None
fitness_over_time = []

# Folder to save figures
figure_folder = "figures"
os.makedirs(figure_folder, exist_ok=True)


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
    cv_values = []

    with open(BASE_CONFIG_PATH, 'r') as f:
        config = yaml.safe_load(f)

    arena_files = config["arena_file"]["batch_options"]
    arena_surface = float(config["arena_surface"])

    for arena_file in arena_files:
        run_dir = tempfile.mkdtemp(dir=TEMP_DIR)
        arena_name = os.path.splitext(os.path.basename(arena_file))[0]

        with open(BASE_CONFIG_PATH, 'r') as f:
            config = yaml.safe_load(f)

        for key, val in zip(PARAMETER_KEYS, p):
            config["parameters"][key] = int(val)

        config["seed"] = np.random.randint(0, 100000)
        config["arena_file"] = arena_file
        config["data_filename"] = os.path.join(run_dir, "frames", "data.feather")

        os.makedirs(os.path.join(run_dir, "frames"), exist_ok=True)

        temp_config_path = os.path.join(run_dir, "simple.yaml")
        with open(temp_config_path, 'w') as f:
            yaml.safe_dump(config, f)

        feather_file = run_simulation(temp_config_path)

        if feather_file:
            try:
                df = feather.read_feather(feather_file)

                polygon = load_arena_polygon_from_csv(arena_file)
                scale = (arena_surface / polygon.area) ** 0.5
                minx, miny, *_ = polygon.bounds
                polygon = affinity.translate(polygon, xoff=-minx, yoff=-miny)
                polygon = affinity.scale(polygon, xfact=scale, yfact=scale, origin=(0, 0))
                bounds = polygon.bounds

                metrics_df = compute_voronoi_metrics(
                    df=df,
                    arena_polygon=polygon,
                    arena_bounds=bounds,
                    arena_surface=arena_surface,
                    communication_radius=133.0
                )
                cv = metrics_df["cv_area"].mean()
                if not np.isnan(cv):
                    cv_values.append(cv)
            except Exception as e:
                print(f"[ERROR] Arena {arena_file}: {e}")
        shutil.rmtree(run_dir, ignore_errors=True)

    return np.mean(cv_values) if cv_values else None


def objective_function(params):

    global best_score, best_params
    
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
                print(f"Avg CV across arenas: {result:.4f} for parameters: {p}")
                with open(OUTPUT_CSV, 'a') as f:
                    f.write(",".join(map(str, p)) + f",{result:.6f}\n")

                # track best score
                if result < best_score:
                    best_score = result 
                    best_params = p 

            else:
                print(f"[FAIL] Simulation failed for: {p}")

    if cv_values:
        generation_mean = np.mean(cv_values)
        fitness_over_time.append(generation_mean)
        return -generation_mean
    else:
        fitness_over_time.append(None)
        return 1e6


    return -np.mean(cv_values) if cv_values else 1e6


def main():
    os.makedirs(TEMP_DIR, exist_ok=True)

    bounds = [[0, 1, 0, 1], [10000, 10000, 10000, 10000]]

    es = CMAEvolutionStrategy(INITIAL_VALUES, SIGMA, {
        'bounds': bounds,
        'maxiter': MAX_ITER,
    })

    with open(OUTPUT_CSV, 'w') as f:
        f.write("run_duration_min,run_duration_max,tumble_duration_min,tumble_duration_max,mean_cv\n")

    es.optimize(objective_function)

    plt.figure()
    plt.plot([-v for v in fitness_over_time], marker='o')
    plt.xlabel("Generation")
    plt.ylabel("Mean CV (lower is better)")
    plt.title("CMA-ES Optimization Progress")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("optimization_curve.png", dpi=150)
    save_figure("Mean CV evolution.png", figure_folder)
    plt.show()

    print(f"Best evaluated parameters: {best_params} with mean CV = {best_score:.6f}")
    shutil.rmtree(TEMP_DIR, ignore_errors=True)


if __name__ == "__main__":
    main()
