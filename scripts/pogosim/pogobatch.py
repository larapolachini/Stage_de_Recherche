#!/usr/bin/env python3
import os
import sys
import subprocess
import yaml
import argparse
import itertools
import tempfile
import shutil
import logging
import pandas as pd

from . import utils

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

class PogobotBatchRunner:
    """
    A reusable class to run batch simulations for every combination of parameters
    specified in a multi-value YAML configuration file. The class computes all
    combinations, writes a temporary YAML file for each combination (in a specified
    temporary base directory), computes a friendly output filename, and launches
    a PogobotLauncher for each combination.
    """
    def __init__(self, multi_config_file, runs, simulator_binary, temp_base, output_dir,
                 backend="multiprocessing", keep_temp=False, verbose=False):
        self.multi_config_file = multi_config_file
        self.runs = runs
        self.simulator_binary = simulator_binary
        self.temp_base = temp_base
        self.output_dir = output_dir
        self.backend = backend
        self.keep_temp = keep_temp
        self.verbose = verbose

        # Initialize logging via utils.
        utils.init_logging(self.verbose)

    def get_combinations(self, config):
        """
        Given a config dict where some values may be lists, return a list of dictionaries
        representing every combination.
        """
        fixed = {}
        options = {}
        for key, value in config.items():
            # Reserve the special key "result_filename_format" for output naming.
            if key == "result_filename_format":
                fixed[key] = value
            elif isinstance(value, list):
                options[key] = value
            else:
                fixed[key] = value

        if options:
            keys = list(options.keys())
            product_vals = list(itertools.product(*(options[k] for k in keys)))
            combinations = []
            for prod in product_vals:
                comb = fixed.copy()
                for i, k in enumerate(keys):
                    comb[k] = prod[i]
                combinations.append(comb)
            return combinations
        else:
            return [fixed]

    def write_temp_yaml(self, comb_config):
        """
        Write the combination configuration dictionary to a temporary YAML file in temp_base.
        Returns the path to the file.
        """
        tmp_file = tempfile.NamedTemporaryFile(
            mode="w", delete=False, suffix=".yaml", prefix="combo_", encoding="utf-8", dir=self.temp_base
        )
        yaml.safe_dump(comb_config, tmp_file)
        tmp_file.close()
        logging.debug(f"Wrote temporary YAML config: {tmp_file.name}")
        return tmp_file.name

    def compute_output_filename(self, comb_config):
        """
        If the combination config includes a key "result_filename_format", format it using the combination
        dictionary. For any string value that appears to be a file path (i.e. has a directory part),
        only its basename (without directories or extension) is used.
        If output_dir is provided and the computed filename is relative, join it with output_dir.
        """
        fmt = comb_config.get("result_filename_format")
        if not fmt:
            return os.path.join(self.output_dir, "result.feather") if self.output_dir else "result.feather"

        mod_config = {}
        for key, value in comb_config.items():
            if isinstance(value, str):
                if os.path.dirname(value):  # Assume it's a path.
                    base = os.path.basename(value)
                    base, _ = os.path.splitext(base)
                    mod_config[key] = base
                else:
                    mod_config[key] = value
            else:
                mod_config[key] = value
        try:
            filename = fmt.format(**mod_config)
        except Exception as e:
            logging.error(f"Error formatting result filename: {e}")
            filename = "result.feather"
        if self.output_dir and not os.path.isabs(filename):
            filename = os.path.join(self.output_dir, filename)
        return filename

    def run_launcher_for_combination(self, temp_config_path, output_file):
        """
        Launch a PogobotLauncher for a single configuration combination.
        """
        logging.info(f"Launching PogobotLauncher for config: {temp_config_path} with output: {output_file}")
        launcher = PogobotLauncher(
            num_instances=self.runs,
            base_config_path=temp_config_path,
            combined_output_path=output_file,
            simulator_binary=self.simulator_binary,
            temp_base_path=self.temp_base,
            backend=self.backend,
            keep_temp=self.keep_temp
        )
        launcher.launch_all()
        # Remove the temporary YAML configuration file.
        os.remove(temp_config_path)
        logging.debug(f"Removed temporary YAML config: {temp_config_path}")
        return output_file

    def run_all(self):
        """
        Load the multi-value YAML configuration, compute combinations, write temporary files,
        launch a PogobotLauncher for each combination sequentially, and return a list of output files.
        """
        # Ensure the temporary base and output directories exist.
        if not os.path.exists(self.temp_base):
            os.makedirs(self.temp_base, exist_ok=True)
            logging.info(f"Created temporary base directory: {self.temp_base}")
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir, exist_ok=True)
            logging.info(f"Created output directory: {self.output_dir}")

        # Load the multi-value configuration.
        with open(self.multi_config_file, "r") as f:
            multi_config = yaml.safe_load(f)

        combinations = self.get_combinations(multi_config)
        if not combinations:
            logging.error("No combinations found in the configuration.")
            sys.exit(1)
        logging.info(f"Found {len(combinations)} combination(s) to run.")

        tasks = []
        for comb in combinations:
            temp_yaml = self.write_temp_yaml(comb)
            output_file = self.compute_output_filename(comb)
            tasks.append((temp_yaml, output_file))
            logging.info(f"Task: Config file {temp_yaml} -> Output: {output_file}")

        results = []
        for temp_yaml, output_file in tasks:
            result = self.run_launcher_for_combination(temp_yaml, output_file)
            results.append(result)

        logging.info("Batch run completed. Generated output files:")
        for output_file in results:
            logging.info(f" - {output_file}")
        return results

def main():
    parser = argparse.ArgumentParser(
        description="Batch run multiple PogobotLauncher instances sequentially for every combination of parameters specified in a multi-value YAML config."
    )
    parser.add_argument("multi_config_file", type=str,
                        help="Path to the YAML configuration file with multiple choices (lists) for some parameters.")
    parser.add_argument("-r", "--runs", type=int, default=1,
                        help="Number of simulator runs to launch per configuration combination (default: 1).")
    parser.add_argument("-S", "--simulator-binary", type=str, required=True,
                        help="Path to the simulator binary.")
    parser.add_argument("-t", "--temp-base", type=str, required=True,
                        help="Base directory for temporary directories and YAML config files used by PogobotLauncher.")
    parser.add_argument("-o", "--output-dir", type=str, default=".",
                        help="Directory where the combined output Feather files will be saved (default: current directory).")
    parser.add_argument("--backend", choices=["multiprocessing", "ray"], default="multiprocessing",
                        help="Parallelism backend to use for launching PogobotLauncher instances (default: multiprocessing).")
    parser.add_argument("--keep-temp", action="store_true",
                        help="Keep temporary directories after simulation runs.")
    parser.add_argument("-v", "--verbose", default=False, action="store_true", help="Verbose mode")
    args = parser.parse_args()

    runner = PogobotBatchRunner(
        multi_config_file=args.multi_config_file,
        runs=args.runs,
        simulator_binary=args.simulator_binary,
        temp_base=args.temp_base,
        output_dir=args.output_dir,
        backend=args.backend,
        keep_temp=args.keep_temp,
        verbose=args.verbose
    )
    runner.run_all()

if __name__ == "__main__":
    main()

# MODELINE "{{{1
# vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
# vim:foldmethod=marker
