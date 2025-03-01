#!/usr/bin/env python3
import os
import sys
import yaml
import argparse
import itertools
import tempfile
import shutil

# Import the PogobotLauncher class from the previous script.
try:
    from pogosim import PogobotLauncher
except ImportError:
    print("Error: Cannot import PogobotLauncher from pogosim.py. Ensure that pogosim.py is in the same directory.")
    sys.exit(1)

def get_combinations(config):
    """
    Given a config dict where some values may be lists, return a list of
    dictionaries for every combination.
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

def write_temp_yaml(comb_config):
    """
    Write the combination configuration dictionary to a temporary YAML file.
    Returns the path to the file.
    """
    tmp_file = tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".yaml", prefix="combo_", encoding="utf-8")
    yaml.safe_dump(comb_config, tmp_file)
    tmp_file.close()
    return tmp_file.name

def compute_output_filename(comb_config, output_dir):
    """
    If the combination config includes a key "result_filename_format", format it
    using the combination dictionary. For any string value that appears to be a file path 
    (i.e. has a directory part), only its basename (without directories or extension) is used.
    If output_dir is provided and the computed filename is relative, join it with output_dir.
    """
    fmt = comb_config.get("result_filename_format")
    if not fmt:
        return os.path.join(output_dir, "result.feather") if output_dir else "result.feather"

    # Create a modified dictionary for formatting.
    mod_config = {}
    for key, value in comb_config.items():
        if isinstance(value, str):
            if os.path.dirname(value):  # Assume it's a path if there is a directory component.
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
        print(f"Error formatting result filename: {e}")
        filename = "result.feather"
    if output_dir and not os.path.isabs(filename):
        filename = os.path.join(output_dir, filename)
    return filename

def run_launcher_for_combination(temp_config_path, output_file, instances_per_config,
                                   simulator_binary, temp_base_path, backend, keep_temp):
    """
    Launch a PogobotLauncher for a single combination.
    """
    print(f"Launching PogobotLauncher for config: {temp_config_path} with output: {output_file}")
    launcher = PogobotLauncher(
        num_instances=instances_per_config,
        base_config_path=temp_config_path,
        combined_output_path=output_file,
        simulator_binary=simulator_binary,
        temp_base_path=temp_base_path,
        backend=backend,
        keep_temp=keep_temp
    )
    launcher.launch_all()
    # Remove the temporary YAML configuration file.
    os.remove(temp_config_path)
    return output_file

def main():
    parser = argparse.ArgumentParser(
        description="Batch run multiple PogobotLauncher instances sequentially for every combination of parameters specified in a multi-value YAML config."
    )
    parser.add_argument("multi_config_file", type=str,
                        help="Path to the YAML configuration file with multiple choices (lists) for some parameters.")
    parser.add_argument("--instances-per-config", type=int, default=1,
                        help="Number of simulator instances to launch per configuration combination (default: 1).")
    parser.add_argument("--simulator-binary", type=str, required=True,
                        help="Path to the simulator binary.")
    parser.add_argument("--temp-base", type=str, required=True,
                        help="Base directory for temporary directories used by PogobotLauncher.")
    parser.add_argument("--output-dir", type=str, default=".",
                        help="Directory where the combined output Feather files will be saved (default: current directory).")
    parser.add_argument("--backend", choices=["multiprocessing", "ray"], default="multiprocessing",
                        help="Parallelism backend to use for launching PogobotLauncher instances (default: multiprocessing).")
    parser.add_argument("--keep-temp", action="store_true",
                        help="Keep temporary directories after simulation runs.")
    args = parser.parse_args()

    # Ensure the provided temp-base directory exists.
    if not os.path.exists(args.temp_base):
        os.makedirs(args.temp_base, exist_ok=True)
        print(f"Created temporary base directory: {args.temp_base}")

    # Ensure the output directory exists.
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir, exist_ok=True)
        print(f"Created output directory: {args.output_dir}")

    # Load the multi-value YAML configuration.
    with open(args.multi_config_file, "r") as f:
        multi_config = yaml.safe_load(f)

    # Compute all combinations.
    combinations = get_combinations(multi_config)
    if not combinations:
        print("No combinations found in the configuration.")
        sys.exit(1)
    print(f"Found {len(combinations)} combination(s) to run.")

    # For each combination, write a temporary YAML config file and compute the output file name.
    tasks = []
    for comb in combinations:
        temp_yaml = write_temp_yaml(comb)
        output_file = compute_output_filename(comb, args.output_dir)
        tasks.append((temp_yaml, output_file))
        print(f"Task: Config file {temp_yaml} -> Output: {output_file}")

    # Launch each combination sequentially.
    results = []
    for (temp_yaml, output_file) in tasks:
        result = run_launcher_for_combination(
            temp_config_path=temp_yaml,
            output_file=output_file,
            instances_per_config=args.instances_per_config,
            simulator_binary=args.simulator_binary,
            temp_base_path=args.temp_base,
            backend=args.backend,
            keep_temp=args.keep_temp
        )
        results.append(result)

    print("Batch run completed. Generated output files:")
    for output_file in results:
        print(f" - {output_file}")

if __name__ == "__main__":
    main()

