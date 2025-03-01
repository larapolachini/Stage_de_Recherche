#!/usr/bin/env python3
import os
import sys
import yaml
import argparse
import itertools
import tempfile
import shutil
import logging

import utils
from pogosim import PogobotLauncher


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
