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
# Force software rendering to avoid any accidental GPU or /dev/dri device usage
os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"


# Configuration
SIMULATOR_BINARY = "./examples/run_and_tumble/run_and_tumble"
BASE_CONFIG_PATH = "conf/simple.yaml"
TEMP_DIR = "tmp_cma"  # Temporary directory for CMA-ES runs
N_RUNS_PER_INDIVIDUAL = 2    #10
PARAMETER_KEYS = [
    "run_duration_min",
    "run_duration_max",
    "tumble_duration_min",
    "tumble_duration_max"
]
INITIAL_VALUES = [0, 0, 0, 0] 
SIGMA = 800  # Initial step size for CMA-ES
MAX_ITER = 5  #100 # Maximum number of CMA-ES iterations

#  Create temporary configuration file with parameters
def create_temp_config(base_config_path, output_dir, parameters):

    """
    Creates a temporary configuration file with the given parameters.
    
    Args:
        base_config_path (str): Path to the base configuration template
        output_dir (str): Directory where temporary config will be created
        parameters (list): Parameter values to inject into the config
    
    Returns:
        str: Path to the created temporary configuration file
    """

    with open(base_config_path, 'r') as f:
        config = yaml.safe_load(f)

    # Update parameters with the values provided by CMA-ES
    for key, val in zip(PARAMETER_KEYS, parameters):
        config["parameters"][key] = int(val)

    # Generate a random seed for this simulation run
    config["seed"] = np.random.randint(0, 100000)

    frames_dir = os.path.join(output_dir, "frames")
    os.makedirs(frames_dir, exist_ok=True)
    data_file = os.path.join(frames_dir, "data.feather")
    config["data_filename"] = data_file

    temp_config_path = os.path.join(output_dir, "simple.yaml")
    with open(temp_config_path, 'w') as f:
        yaml.safe_dump(config, f)
    return temp_config_path

# Executes a single simulation with the given configuration file.
def run_simulation(config_path):
    try:
        result = subprocess.run(
            [SIMULATOR_BINARY, "-c", config_path, "-nr", "-q", "-g"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
    except subprocess.CalledProcessError as e:
        print("STDERR:", e.stderr.decode())
        print("STDOUT:", e.stdout.decode())
        return None

    # Check if the output file was created
    feather_path = os.path.join(os.path.dirname(config_path), "frames", "data.feather")
    if os.path.exists(feather_path):
        return feather_path

# Objective function to minimize coefficient of variation (CV) of Voronoi cell areas
def objective_function(parameters):
    p = list(map(int, parameters))

    # Validate logical constraints
    if not (p[0] < p[1] and p[2] < p[3]):
        return 1e6  # heavy penalty

    cv_values = []
    run_dirs = []

    for _ in range(N_RUNS_PER_INDIVIDUAL):
        run_dir = tempfile.mkdtemp(dir=TEMP_DIR)
        run_dirs.append(run_dir)

        config_path = create_temp_config(BASE_CONFIG_PATH, run_dir, p)
        feather_file = run_simulation(config_path)

        if feather_file:
            try:
                # Load simulation data
                df = feather.read_feather(feather_file)

                # Compute metrics (you must define arena_polygon and bounds globally or pass them)
                metrics_df = compute_voronoi_metrics(
                    df=df,
                    arena_polygon = arena_polygon,        # Must define globally
                    arena_bounds = arena_bounds,          # Must define globally
                    arena_surface = arena_surface,        # Must define globally
                    communication_radius=133.0
                )

                # Get mean CV (e.g., last frame or average over time)
                cv_series = metrics_df["cv_area"]
                if not cv_series.empty:
                    mean_cv = cv_series.mean()
                    cv_values.append(mean_cv)
                    print(f"Mean CV: {mean_cv:.4f} for parameters: {parameters}")
            except Exception as e:
                print(f"Error during analysis: {e}")
        else:
            print("Simulation failed for:", p)

    for d in run_dirs:
        shutil.rmtree(d, ignore_errors=True)

    if not cv_values:
        return 1e6
  

    return -np.mean(cv_values)  # CMA-ES minimizes: lower CV is better

# Main optimization routine 
def main():
    os.makedirs(TEMP_DIR, exist_ok=True)

    # Define parameter bounds for CMA-ES
    bounds = [[0, 0, 0, 0], [10000, 10000, 10000, 10000]]

    es = CMAEvolutionStrategy(INITIAL_VALUES, SIGMA, {
        'bounds': bounds,
        'maxiter': MAX_ITER,
    })

    es.optimize(objective_function)

    print(" Optimal parameters found :", es.result.xbest)
    shutil.rmtree(TEMP_DIR, ignore_errors=True)

if __name__ == "__main__":
    main()
