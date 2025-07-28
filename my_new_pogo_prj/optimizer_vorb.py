#!/usr/bin/env python3
"""
CMA-ES optimiser for the run-and-tumble simulator
-------------------------------------------------
• Evaluates every arena in conf/simpleb.yaml
• Spawns one persistent process pool (workers stay warm)
• Returns mean Voronoï CV across arenas & MC replicates
"""

from __future__ import annotations
import os
import sys
import shutil
import tempfile
import subprocess
import yaml
import numpy as np
import pyarrow.feather as feather
import matplotlib.pyplot as plt
import concurrent.futures as cf
from pathlib import Path
import copy

from shapely.geometry import Polygon
from shapely import affinity
from cma import CMAEvolutionStrategy

from vor import compute_voronoi_metrics, save_figure
from data import load_arena_polygon_from_csv

# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────
os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")

from pathlib import Path
SIMULATOR_BINARY = Path("./examples/run_and_tumble/run_and_tumble").resolve()

BASE_CONFIG_PATH = Path("conf/simpleb.yaml")
TEMP_DIR          = Path("tmp_cma")
FIGURES_DIR       = Path("figures")
OUTPUT_CSV        = Path("cmaes_results.csv")

N_RUNS_PER_INDIVIDUAL = 5
PARAMETER_KEYS        = ["run_duration_min", "run_duration_max",
                         "tumble_duration_min", "tumble_duration_max"]
INITIAL_VALUES = [10.0, 50.0, 10.0, 50.0]
SIGMA          = 10.0
MAX_ITER       = 50
POP_SIZE       = 16
BOUNDS         = [[0, 1, 0, 1],
                  [10_000, 10_000, 10_000, 10_000]]

FIGURES_DIR.mkdir(exist_ok=True)
TEMP_DIR.mkdir(exist_ok=True)
FORMATION_SET = [
    "disk",
    "power_lloyd",
    "random_near_walls"
]

# ─────────────────────────────────────────────────────────────────────────────
# Pre-load static YAML parts once
# ─────────────────────────────────────────────────────────────────────────────
BASE_CFG    = yaml.safe_load(BASE_CONFIG_PATH.read_text())
ARENA_FILES = BASE_CFG["arena_file"]["batch_options"]
ARENA_SURF  = float(BASE_CFG["arena_surface"])

# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────
def run_simulator(cfg_path: Path, cwd: Path) -> Path | None:
    """Return the path to frames/data.feather if the simulator ran ok."""

    env = os.environ.copy()
    env["SDL_VIDEODRIVER"] = "dummy"

    try:
        subprocess.run(
            [SIMULATOR_BINARY, "-c", str(cfg_path), "-nr", "-q", "-g"],
            check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            cwd=cwd,
            env = env,
        )
    except subprocess.CalledProcessError as err:
        print("[SIM ERR] STDERR", err.stderr.decode(), file=sys.stderr)
        print("[SIM ERR] STDOUT:", err.stdout.decode(), file=sys.stderr)

        return None

    feather_file = cwd / "frames" / "data.feather"
    return feather_file if feather_file.exists() else None


def run_one_simulation(params_int: list[int]) -> float | None:
    """ Run *every* (formation, arena) combination once and return the mean CV."""
    cv_vals: list[float] = []

    for formation in FORMATION_SET:
        for arena in ARENA_FILES:

            run_dir = Path(tempfile.mkdtemp(dir=TEMP_DIR,
                                            prefix=f"{formation}_"))
            (run_dir / "frames").mkdir(parents=True, exist_ok=True)

            # tailor YAML
            cfg = copy.deepcopy(BASE_CFG)
            cfg["parameters"].update(dict(zip(PARAMETER_KEYS, params_int)))
            cfg["seed"]             = int(np.random.randint(0, 2**31 - 1))
            cfg["initial_formation"] = formation
            cfg["arena_file"]        = arena
            cfg["data_filename"]     = str(run_dir / "frames"
                                                     / "data.feather")

            cfg_path = run_dir / "cfg.yaml"
            yaml.safe_dump(cfg, cfg_path.open("w"))

            feather_file = run_simulator(cfg_path, run_dir)
            if feather_file is None:              # simulator crashed
                shutil.rmtree(run_dir, ignore_errors=True)
                continue

            try:
                df   = feather.read_feather(feather_file)
                poly = load_arena_polygon_from_csv(arena)
                scale = (ARENA_SURF / poly.area) ** 0.5
                minx, miny, *_ = poly.bounds
                poly = affinity.translate(poly, xoff=-minx, yoff=-miny)
                poly = affinity.scale(poly, xfact=scale, yfact=scale,
                                      origin=(0, 0))

                metrics = compute_voronoi_metrics(
                    df=df, arena_polygon=poly, arena_bounds=poly.bounds,
                    arena_surface=ARENA_SURF, communication_radius=133.0,
                )
                cv_vals.append(float(metrics["cv_area"].mean()))
            except Exception as exc:               # pylint: disable=broad-except
                print(f"[ANALYSIS‑ERR] {formation} / {arena}: {exc}",
                      file=sys.stderr)
            finally:
                shutil.rmtree(run_dir, ignore_errors=True)

    return float(np.mean(cv_vals)) if cv_vals else None


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def main() -> None:
    best_score  = float("inf")
    best_params = None
    fitness_over_time: list[float] = []

    # CSV header
    OUTPUT_CSV.write_text(
        "run_duration_min,run_duration_max,tumble_duration_min,"
        "tumble_duration_max,mean_cv\n"
    )

    # persistent pool
    with cf.ProcessPoolExecutor(max_workers=os.cpu_count()) as pool:

        def objective(params_f: list[float]) -> float:
            nonlocal best_score, best_params

            # constraints
            if params_f[0] >= params_f[1] or params_f[2] >= params_f[3]:
                return 1e6

            params_int = [int(round(v)) for v in params_f]

            futures = [pool.submit(run_one_simulation, params_int)
                       for _ in range(N_RUNS_PER_INDIVIDUAL)]
            cvs = [f.result() for f in futures if f.result() is not None]
            mean_cv = float(np.mean(cvs)) if cvs else 1e6
            fitness_over_time.append(mean_cv)

            OUTPUT_CSV.open("a").write(
                ",".join(map(str, params_int)) + f",{mean_cv:.6f}\n"
            )

            if mean_cv < best_score:
                best_score, best_params = mean_cv, params_int
                print(f"[BEST] CV={best_score:.4f}  @ {best_params}")

            return mean_cv   # CMA-ES minimises

        # ── CMA-ES run ────────────────────────────────────────────────────
        es = CMAEvolutionStrategy(
            INITIAL_VALUES, SIGMA,
            {"bounds": BOUNDS, "maxiter": MAX_ITER, "popsize": POP_SIZE}
        )
        es.optimize(objective)

    # ──────────────────────────────────────────────────────────────────
    plt.figure()
    plt.plot(fitness_over_time, marker="o")
    plt.xlabel("Generation")
    plt.ylabel("Mean CV (lower is better)")
    plt.title("CMA-ES optimisation progress")
    plt.grid(True)
    plt.tight_layout()
    out_png = FIGURES_DIR / "optimization_curve.png"
    plt.savefig(out_png, dpi=150)

    try:
        save_figure(plt.gcf(), "mean_cv_evolution.png", FIGURES_DIR)  # if helper expects this
    except Exception:  # pragma: no cover
        pass

    print("== DONE ==")
    print(f"Best parameters: {best_params}")
    print(f"Best mean CV   : {best_score:.6f}")
    shutil.rmtree(TEMP_DIR, ignore_errors=True)


if __name__ == "__main__":
    main()
