import os
import yaml
import shutil
import tempfile
import subprocess
import numpy as np
import pandas as pd
import pyarrow.feather as feather
from vor import compute_voronoi_metrics, save_figure
from cma import CMAEvolutionStrategy
from shapely.geometry import Polygon
from shapely import affinity
from data import load_arena_polygon_from_csv
import concurrent.futures
import matplotlib.pyplot as plt

# Configuration
os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
SIMULATOR_BINARY = "./examples/run_and_tumble/run_and_tumble"
BASE_CONFIG_PATH = "conf/simpleb.yaml"
TEMP_DIR = "tmp_cma"
N_RUNS_PER_INDIVIDUAL = 5
PARAMETER_KEYS = ["run_duration_min", "run_duration_max", "tumble_duration_min", "tumble_duration_max"]
INITIAL_VALUES = [10, 50, 10, 50]  # Reparameterized
SIGMA = 10
MAX_ITER = 50
OUTPUT_CSV = "cmaes_results.csv"
FIGURE_FOLDER = "figures"
os.makedirs(FIGURE_FOLDER, exist_ok=True)

best_score = float("inf")
best_params = None
fitness_over_time = []

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

        config["parameters"].update({k: int(v) for k, v in zip(PARAMETER_KEYS, p)})
        config["seed"] = np.random.randint(0, 100000)
        config["arena_file"] = arena_file
        config["data_filename"] = os.path.join(run_dir, "frames", "data.feather")
        os.makedirs(os.path.join(run_dir, "frames"), exist_ok=True)

        config_path = os.path.join(run_dir, "simple.yaml")
        with open(config_path, "w") as f:
            yaml.safe_dump(config, f)

        feather_file = run_simulation(config_path)

        if feather_file:
            try:
                df = feather.read_feather(feather_file)
                polygon = load_arena_polygon_from_csv(arena_file)
                scale = (arena_surface / polygon.area) ** 0.5
                minx, miny, *_ = polygon.bounds
                polygon = affinity.translate(polygon, xoff=-minx, yoff=-miny)
                polygon = affinity.scale(polygon, xfact=scale, yfact=scale, origin=(0, 0))

                metrics_df = compute_voronoi_metrics(
                    df=df,
                    arena_polygon=polygon,
                    arena_bounds=polygon.bounds,
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
    """
    Fitness = mean coefficient of variation (CV) over arenas and Monte-Carlo
    replicates.  Lower = better.  Returns a large positive penalty when the
    logical constraints are violated or when all replications failed.
    """
    global best_score, best_params

    # 1. Keep the optimiser variables as floats
    p_float = list(params)

    # Hard constraints: run_min < run_max and tumble_min < tumble_max
    if p_float[0] >= p_float[1] or p_float[2] >= p_float[3]:
        return 1e6          # big penalty (positive → CMA-ES tries to avoid)

    # Cast to int only for the simulator
    p_int = [int(round(v)) for v in p_float]

    # 2. Launch Monte-Carlo replicates in parallel
    cv_values = []
    with concurrent.futures.ProcessPoolExecutor(max_workers=os.cpu_count()) as pool:
        futures = [pool.submit(run_one_simulation, p_int)
                   for _ in range(N_RUNS_PER_INDIVIDUAL)]
        for fut in concurrent.futures.as_completed(futures):
            cv = fut.result()
            if cv is not None:
                cv_values.append(cv)

    mean_cv = np.mean(cv_values) if cv_values else 1e6   # fallback penalty
    fitness_over_time.append(mean_cv)

    # 3. Book-keeping and logging
    with open(OUTPUT_CSV, "a") as f:
        f.write(",".join(map(str, p_int)) + f",{mean_cv:.6f}\n")

    if mean_cv < best_score:
        best_score, best_params = mean_cv, p_int
        print(f"[BEST] CV={best_score:.4f} @ params {best_params}")

    return mean_cv        # CMA-ES minimises this


def main():
    os.makedirs(TEMP_DIR, exist_ok=True)

    bounds = [[0, 1, 0, 1], [10000, 10000, 10000, 10000]]

    with open(OUTPUT_CSV, 'w') as f:
        f.write("run_duration_min,run_duration_max,tumble_duration_min,tumble_duration_max,mean_cv\n")

    es = CMAEvolutionStrategy(INITIAL_VALUES, SIGMA, {
        'bounds': bounds,
        'maxiter': MAX_ITER,
        'popsize': 16
    })

    es.optimize(objective_function)

    # Plotting fitness over time
    plt.figure()
    plt.plot(fitness_over_time, marker='o')
    plt.xlabel("Generation")
    plt.ylabel("Mean CV (lower is better)")
    plt.title("CMA-ES Optimization Progress")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("optimization_curve.png", dpi=150)
    save_figure("Mean CV evolution.png", FIGURE_FOLDER)
    plt.show()

    print(f"Best evaluated parameters: {best_params} with mean CV = {best_score:.6f}")
    shutil.rmtree(TEMP_DIR, ignore_errors=True)

if __name__ == "__main__":
    main()
