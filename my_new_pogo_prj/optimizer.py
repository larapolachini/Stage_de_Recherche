import os
import yaml
import shutil
import tempfile
import subprocess
import numpy as np
import pandas as pd
from cma import CMAEvolutionStrategy
from distance import interindividual_distance_mean

# Force software rendering to avoid any accidental GPU or /dev/dri device usage
os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"


# Configuration
SIMULATOR_BINARY = "./examples/run_and_tumble/run_and_tumble"
BASE_CONFIG_PATH = "conf/simple.yaml"
TEMP_DIR = "tmp_cma"  # Temporary directory for CMA-ES runs
N_RUNS_PER_INDIVIDUAL = 2
PARAMETER_KEYS = [
    "run_duration_min",
    "run_duration_max",
    "tumble_duration_min",
    "tumble_duration_max"
]
INITIAL_VALUES = [100, 500, 2000, 5000]
SIGMA = 300  # Initial step size for CMA-ES
MAX_ITER = 3  # Maximum number of CMA-ES iterations

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

# Objective function to minimize (maximize distance)
def objective_function(parameters):
      
    """
    Objective function for CMA-ES optimization. 
    Runs multiple simulations with given parameters and returns negative mean distance.
    
    Args:
        parameters (array): Parameter values suggested by CMA-ES
    
    Returns:
        float: Negative mean interindividual distance (to maximize distance via minimization)
    """

    p = list(map(int, parameters))
    
    # Validate logical constraints: min < max for both run and tumble durations
    if not (p[0] < p[1] and p[2] < p[3]):
        return 1e6   #  penalizes invalid constraints
    
    distances = []
    run_dirs = []

    # Run N simulations with the same parameter set
    for _ in range(N_RUNS_PER_INDIVIDUAL):
        run_dir = tempfile.mkdtemp(dir=TEMP_DIR)
        run_dirs.append(run_dir)

        # Create configuration and run simulation
        config_path = create_temp_config(BASE_CONFIG_PATH, run_dir, p)
        feather_file = run_simulation(config_path)

        # Calculate interindividual distance if simulation succeeded
        if feather_file:
            try:
                dist = interindividual_distance_mean(feather_file)
                distances.append(dist)
                print(f" Mean distance : {dist:.4f} for parameters : {parameters}")
            except Exception as e:
                print(f" Error during analysis : {e}")
        else:
            print(" Simulation failed for :", p)

    # Cleanup temporary directories
    for d in run_dirs:
        shutil.rmtree(d, ignore_errors=True)

    if not distances:
        return 1e6

    # Return negative mean distance (CMA-ES minimizes, but we want to maximize distance)
    return -np.mean(distances)

# Main optimization routine 
def main():
    os.makedirs(TEMP_DIR, exist_ok=True)

    # Define parameter bounds for CMA-ES
    bounds = [[0, 0, 0, 0], [10000, 10000, 10000, 10000]]

    es = CMAEvolutionStrategy(INITIAL_VALUES, SIGMA, {
        'bounds': bounds,
        'maxiter': MAX_ITER
    })

    es.optimize(objective_function)

    print(" Optimal parameters found :", es.result.xbest)
    shutil.rmtree(TEMP_DIR, ignore_errors=True)

if __name__ == "__main__":
    main()
