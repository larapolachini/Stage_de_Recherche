#!/usr/bin/env python3
import os
import sys
import subprocess
import tempfile
import shutil
import yaml
import pandas as pd
import argparse
import logging

import utils

# Import Pool from multiprocessing for the default backend.
from multiprocessing import Pool

class PogobotLauncher:
    def __init__(self, num_instances, base_config_path, combined_output_path, simulator_binary, temp_base_path, backend="multiprocessing", keep_temp=False):
        self.num_instances = num_instances
        self.base_config_path = base_config_path
        self.combined_output_path = combined_output_path
        self.simulator_binary = simulator_binary
        self.temp_base_path = temp_base_path
        self.backend = backend
        self.keep_temp = keep_temp
        self.temp_dirs = []
        self.dataframes = []  # Will hold DataFrames loaded from each run

    @staticmethod
    def modify_config_static(base_config_path, output_dir, seed):
        # Load the base YAML configuration.
        with open(base_config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Set a unique seed for this instance.
        config['seed'] = seed

        # Disable frame export.
        config['save_video_period'] = -1

        # Create a directory for frame files inside the temporary directory.
        frames_dir = os.path.join(output_dir, "frames")
        os.makedirs(frames_dir, exist_ok=True)

        # Update file paths so that outputs go into the temporary directory.
        if 'data_filename' in config:
            config['data_filename'] = os.path.join(frames_dir, os.path.basename(config['data_filename']))
        if 'console_filename' in config:
            config['console_filename'] = os.path.join(frames_dir, os.path.basename(config['console_filename']))
        if 'frames_name' in config:
            config['frames_name'] = os.path.join(frames_dir, os.path.basename(config['frames_name']))

        # Write the modified configuration to a new YAML file.
        new_config_path = os.path.join(output_dir, "test.yaml")
        with open(new_config_path, 'w') as f:
            yaml.safe_dump(config, f)

        return new_config_path

    @staticmethod
    def launch_simulator_static(config_path, simulator_binary):
        # Build the simulator command and run it.
        command = [simulator_binary, "-c", config_path, "-nr", "-g", "-q"]
        logging.debug(f"Executing command: {' '.join(command)}")
        subprocess.run(command, check=True)

    @staticmethod
    def worker(args):
        i, base_config_path, simulator_binary, temp_base_path = args
        # Create a temporary directory in the specified base path.
        temp_dir = tempfile.mkdtemp(prefix=f"sim_instance_{i}_", dir=temp_base_path)
        # Modify the configuration file with a unique seed and output paths.
        config_path = PogobotLauncher.modify_config_static(base_config_path, temp_dir, seed=i)
        logging.debug(f"Launching instance {i} with config {config_path} in {temp_dir}")
        # Launch the simulator and wait for it to finish.
        PogobotLauncher.launch_simulator_static(config_path, simulator_binary)

        # Load the Feather file as soon as the simulator instance finishes.
        feather_path = os.path.join(temp_dir, "frames", "data.feather")
        df = None
        if os.path.exists(feather_path):
            try:
                df = pd.read_feather(feather_path)
                # Add a column "run" corresponding to the instance number.
                df['run'] = i
                logging.debug(f"Instance {i}: Loaded data from {feather_path}")
            except Exception as e:
                logging.error(f"Instance {i}: Error reading feather file {feather_path}: {e}")
        else:
            logging.warning(f"Instance {i}: Feather file not found: {feather_path}")
        return (temp_dir, df)

    def combine_feather_files(self, dataframes):
        if dataframes:
            combined_df = pd.concat(dataframes, ignore_index=True)
            combined_df.to_feather(self.combined_output_path)
            logging.info(f"Combined data saved to {self.combined_output_path}")
        else:
            logging.error("No dataframes were loaded to combine.")

    def clean_temp_dirs(self):
        for d in self.temp_dirs:
            shutil.rmtree(d)
            logging.debug(f"Cleaned up temporary directory: {d}")

    def launch_all(self):
        # Prepare the arguments for each simulation instance.
        args_list = [
            (i, self.base_config_path, self.simulator_binary, self.temp_base_path)
            for i in range(self.num_instances)
        ]

        if self.backend == "multiprocessing":
            # Use a multiprocessing Pool.
            with Pool(processes=self.num_instances) as pool:
                results = pool.map(PogobotLauncher.worker, args_list)
        elif self.backend == "ray":
            try:
                import ray
            except ImportError:
                logging.error("Ray is not installed. Please install ray to use the 'ray' backend.")
                sys.exit(1)
            # Initialize ray.
            ray.init(ignore_reinit_error=True)
            # Convert the worker function into a Ray remote function.
            ray_worker = ray.remote(PogobotLauncher.worker)
            futures = [ray_worker.remote(args) for args in args_list]
            results = ray.get(futures)
            ray.shutdown()
        else:
            logging.error(f"Unknown backend: {self.backend}")
            sys.exit(1)

        # Separate the temporary directories and the loaded DataFrames.
        self.temp_dirs = [result[0] for result in results]
        self.dataframes = [result[1] for result in results if result[1] is not None]

        # Combine the loaded DataFrames.
        self.combine_feather_files(self.dataframes)

        if not self.keep_temp:
            self.clean_temp_dirs()
        else:
            logging.info("Keeping temporary directories:")
            for d in self.temp_dirs:
                logging.info(d)

def main():
    parser = argparse.ArgumentParser(
        description="Launch multiple simulator instances in parallel (using multiprocessing or ray), combine their data, and optionally keep temporary directories."
    )
    parser.add_argument("num_instances", type=int, help="Number of simulator instances to launch")
    parser.add_argument("base_config_path", type=str, help="Path to the base YAML configuration file")
    parser.add_argument("combined_output_path", type=str, help="Path to save the combined Feather file")
    parser.add_argument("simulator_binary", type=str, help="Path to the simulator binary")
    parser.add_argument("temp_base_path", type=str, help="Base path for temporary directories")
    parser.add_argument("--backend", choices=["multiprocessing", "ray"], default="multiprocessing", help="Parallelism backend to use (default: multiprocessing)")
    parser.add_argument("--keep-temp", action="store_true", help="Keep temporary directories after simulation")

    args = parser.parse_args()

    utils.init_logging()

    # Create the temporary base directory if it doesn't exist.
    if not os.path.exists(args.temp_base_path):
        os.makedirs(args.temp_base_path, exist_ok=True)
        logging.debug(f"Created temporary base directory: {args.temp_base_path}")

    # Ensure the directory for the combined output file exists.
    combined_output_dir = os.path.dirname(args.combined_output_path)
    if combined_output_dir and not os.path.exists(combined_output_dir):
        os.makedirs(combined_output_dir, exist_ok=True)
        logging.info(f"Created combined output directory: {combined_output_dir}")

    launcher = PogobotLauncher(
        args.num_instances,
        args.base_config_path,
        args.combined_output_path,
        args.simulator_binary,
        args.temp_base_path,
        backend=args.backend,
        keep_temp=args.keep_temp
    )
    launcher.launch_all()

if __name__ == "__main__":
    main()

