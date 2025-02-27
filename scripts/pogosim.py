#!/usr/bin/env python3
import os
import sys
import subprocess
import tempfile
import shutil
import yaml
import pandas as pd
import argparse
from multiprocessing import Pool

class PogobotLauncher:
    def __init__(self, num_instances, base_config_path, combined_output_path, simulator_binary, temp_base_path, keep_temp=False):
        self.num_instances = num_instances
        self.base_config_path = base_config_path
        self.combined_output_path = combined_output_path
        self.simulator_binary = simulator_binary
        self.temp_base_path = temp_base_path
        self.keep_temp = keep_temp
        self.temp_dirs = []
        self.dataframes = []  # Will hold the individual DataFrames loaded by each worker

    @staticmethod
    def modify_config_static(base_config_path, output_dir, seed):
        # Load the base YAML configuration.
        with open(base_config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Set a unique seed for this instance.
        config['seed'] = seed

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
        command = [simulator_binary, "-c", config_path, "-nr", "-g"]
        subprocess.run(command, check=True)

    @staticmethod
    def worker(args):
        i, base_config_path, simulator_binary, temp_base_path = args
        # Create a temporary directory in the specified base path.
        temp_dir = tempfile.mkdtemp(prefix=f"sim_instance_{i}_", dir=temp_base_path)
        # Modify the configuration file with a unique seed and output paths.
        config_path = PogobotLauncher.modify_config_static(base_config_path, temp_dir, seed=i)
        print(f"Launching instance {i} with config {config_path} in {temp_dir}")
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
                print(f"Instance {i}: Loaded data from {feather_path}")
            except Exception as e:
                print(f"Instance {i}: Error reading feather file {feather_path}: {e}")
        else:
            print(f"Instance {i}: Feather file not found: {feather_path}")
        return (temp_dir, df)

    def combine_feather_files(self, dataframes):
        if dataframes:
            combined_df = pd.concat(dataframes, ignore_index=True)
            combined_df.to_feather(self.combined_output_path)
            print(f"Combined data saved to {self.combined_output_path}")
        else:
            print("No dataframes were loaded to combine.")

    def clean_temp_dirs(self):
        for d in self.temp_dirs:
            shutil.rmtree(d)
            print(f"Cleaned up temporary directory: {d}")

    def launch_all(self):
        # Prepare the arguments for each simulation instance.
        args_list = [
            (i, self.base_config_path, self.simulator_binary, self.temp_base_path)
            for i in range(self.num_instances)
        ]
        # Use a multiprocessing Pool to launch instances in parallel.
        with Pool(processes=self.num_instances) as pool:
            results = pool.map(PogobotLauncher.worker, args_list)

        # Separate the temporary directories and the loaded DataFrames.
        self.temp_dirs = [result[0] for result in results]
        self.dataframes = [result[1] for result in results if result[1] is not None]

        # Combine the loaded DataFrames.
        self.combine_feather_files(self.dataframes)

        if not self.keep_temp:
            self.clean_temp_dirs()
        else:
            print("Keeping temporary directories:")
            for d in self.temp_dirs:
                print(d)

def main():
    parser = argparse.ArgumentParser(
        description="Launch multiple simulator instances in parallel, combine their data, and optionally keep temporary directories."
    )
    parser.add_argument("num_instances", type=int, help="Number of simulator instances to launch")
    parser.add_argument("base_config_path", type=str, help="Path to the base YAML configuration file")
    parser.add_argument("combined_output_path", type=str, help="Path to save the combined Feather file")
    parser.add_argument("simulator_binary", type=str, help="Path to the simulator binary")
    parser.add_argument("temp_base_path", type=str, help="Base path for temporary directories")
    parser.add_argument("--keep-temp", action="store_true", help="Keep temporary directories after simulation")

    args = parser.parse_args()

    # Create the temporary base directory if it doesn't exist.
    if not os.path.exists(args.temp_base_path):
        os.makedirs(args.temp_base_path, exist_ok=True)
        print(f"Created temporary base directory: {args.temp_base_path}")

    # Ensure the directory for the combined output file exists.
    combined_output_dir = os.path.dirname(args.combined_output_path)
    if combined_output_dir and not os.path.exists(combined_output_dir):
        os.makedirs(combined_output_dir, exist_ok=True)
        print(f"Created combined output directory: {combined_output_dir}")

    launcher = PogobotLauncher(
        args.num_instances,
        args.base_config_path,
        args.combined_output_path,
        args.simulator_binary,
        args.temp_base_path,
        keep_temp=args.keep_temp
    )
    launcher.launch_all()

if __name__ == "__main__":
    main()

