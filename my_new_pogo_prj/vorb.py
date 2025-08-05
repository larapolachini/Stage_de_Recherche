# vorb.py – batch analyser for multiple arenas
# -------------------------------------------------------------
# * Uses headless Agg backend (no pop‑ups)
# * Activates vor.SAVE_FIGURES so every per‑arena call writes PNGs
#   into `figures/<arena_name>/`

import matplotlib
matplotlib.use("Agg")  # headless backend before importing pyplot

import os
import yaml
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from shapely.geometry import Polygon
from shapely import affinity

# ------------------------------------------------------------------
# Import vor *module* first, switch on its SAVE_FIGURES flag, then
# pull the required callables.
# ------------------------------------------------------------------

import vor as vor_module  # full module import
vor_module.SAVE_FIGURES = True  # enable saving inside vor

from vor import (
    compute_voronoi_metrics,
    interindividual_distance_knn_over_time_plot,
    plot_voronoi_variance,
    plot_voronoi_std,
    plot_cv,
    plot_voronoi_global_variance,
    plot_coverage_ratio,
    plot_voronoi_diagram,
    plot_voronoi_evolution,
    compute_fano_over_time_corrected,
    compute_overall_neighbor_degree_histogram,
    compute_overall_neighbor_degree_histogram_all_runs,
    compute_degree_histogram_first_last,
)

sns.set(font_scale=1.3)

# ------------------------------------------------------------------
# Helpers -----------------------------------------------------------
# ------------------------------------------------------------------


def plot_cv_mean_across_arenas(all_df_voronoi, output_path="figures/mean_cv_across_arenas.png"):
    """Draw and save the mean CV‑area curve across all arenas."""
    if all_df_voronoi.empty:
        print("No data to plot CV mean across arenas.")
        return

    plt.figure(figsize=(10, 6))

    # Per‑arena mean curves
    for arena in all_df_voronoi["arena"].unique():
        arena_df = all_df_voronoi[all_df_voronoi["arena"] == arena]
        mean_series = arena_df.groupby("time")["cv_area"].mean()
        plt.plot(mean_series.index, mean_series.values, label=arena, alpha=0.6)

    # Global mean across arenas
    global_cv = all_df_voronoi.groupby("time")["cv_area"].mean()
    plt.plot(global_cv.index, global_cv.values, label="Mean across arenas", color="black", linewidth=2.5)

    plt.xlabel("Time")
    plt.ylabel("Mean CV of Voronoi Cell Area")
    plt.title("Mean CV Area Over Time by Arena")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    save_figure(os.path.basename(output_path), os.path.dirname(output_path) or "figures")



# ------------------------------------------------------------------

def save_figure(filename: str, folder: str, dpi: int = 300) -> None:
    """Local helper for figures generated directly in this script."""
    path = os.path.join(folder, filename)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    print(f"[fig] → {path}")
    plt.savefig(path, dpi=dpi)
    plt.close()


def load_arena_polygon_and_bounds(arena_file: str, arena_surface: float):
    """Read .arena ASCII file → scaled shapely polygon + bounds."""
    with open(arena_file, "r") as f:
        lines = f.readlines()

    loops, current = [], []
    for ln in lines:
        s = ln.strip()
        if not s:
            if current:
                loops.append(current); current = []
        else:
            x, y = map(float, s.split(","))
            current.append((x, y))
    if current:
        loops.append(current)
    if not loops:
        raise ValueError("No valid loops in arena file")

    shell, holes = loops[0], loops[1:]
    poly = Polygon(shell, holes=holes)

    scale = (arena_surface / poly.area) ** 0.5
    minx, miny, *_ = poly.bounds
    poly = affinity.translate(poly, xoff=-minx, yoff=-miny)
    poly = affinity.scale(poly, xfact=scale, yfact=scale, origin=(0, 0))
    return poly, poly.bounds


# ------------------------------------------------------------------
# Per‑arena analysis ------------------------------------------------
# ------------------------------------------------------------------

def run_analysis_for_arena(arena_file: str, arena_surface: float):
    name = os.path.splitext(os.path.basename(arena_file))[0]
    feather_path = f"results/result_{name}.feather"
    print(f"\n=== Processing {name} ===")

    if not os.path.exists(feather_path):
        print("  [skip] missing", feather_path)
        return

    df = pd.read_feather(feather_path)
    poly, bounds = load_arena_polygon_and_bounds(arena_file, arena_surface)

    # Folder for this arena’s figures
    folder = os.path.join("figures", name)
    os.makedirs(folder, exist_ok=True)

    # -------- k‑NN + Voronoi metrics + plots ----------------------
    interindividual_distance_knn_over_time_plot(feather_path, folder, k=3)

    df_vor = compute_voronoi_metrics(df, poly, bounds, arena_surface, communication_radius=133.0)

    plot_voronoi_variance(df_vor, folder)
    plot_voronoi_std(df_vor, folder)
    plot_cv(df_vor, folder)
    plot_voronoi_global_variance(df_vor, folder)
    plot_coverage_ratio(df_vor, folder)

    # Single & multi‑time diagrams
    times = sorted(df["time"].unique())
    mid_time = times[len(times)//2]
    sample_times = [times[i] for i in [0, len(times)//4, len(times)//2, 3*len(times)//4, -1]]

    plot_voronoi_diagram(df, folder, poly, bounds, mid_time)
    plot_voronoi_evolution(df, folder, poly, bounds, sample_times)

    # Fano / degree stats
    compute_fano_over_time_corrected(df, folder, run_id=0)
    compute_overall_neighbor_degree_histogram(df, folder, run_id=0)
    compute_overall_neighbor_degree_histogram_all_runs(df, folder)
    compute_degree_histogram_first_last(df, folder)

    print("  ✔ saved to", folder)


# ------------------------------------------------------------------
# Main --------------------------------------------------------------
# ------------------------------------------------------------------

if __name__ == "__main__":
    with open("conf/simpleb.yaml", "r") as f:
        cfg = yaml.safe_load(f)

    arena_surface = float(cfg["arena_surface"])
    arena_files = cfg["arena_file"]["batch_options"]

    for arena in arena_files:
        run_analysis_for_arena(arena, arena_surface)

    print("\n✅ All arenas processed.")

    # ---- Cross‑arena CV violin & mean plots ----------------------
    print("\nGenerating cross‑arena CV figures…")

    combined = []
    for afile in arena_files:
        name = os.path.splitext(os.path.basename(afile))[0]
        fpath = f"results/result_{name}.feather"
        if not os.path.exists(fpath):
            print("  [skip]", fpath)
            continue
        df = pd.read_feather(fpath)
        poly, bounds = load_arena_polygon_and_bounds(afile, arena_surface)
        df_v = compute_voronoi_metrics(df, poly, bounds, arena_surface, communication_radius=133.0)
        df_v["arena"] = name
        combined.append(df_v)

    if combined:
        all_df = pd.concat(combined, ignore_index=True)
        # Mean CV over time
        plot_cv_mean_across_arenas(all_df, output_path="figures/mean_cv_across_arenas.png")

        # Violin
        plt.figure(figsize=(10, 6))
        sns.violinplot(data=all_df, x="arena", y="cv_area", inner="box", density_norm="width", cut=0)
        plt.title("Distribution of CV Area by Arena")
        plt.ylabel("Coefficient of Variation (CV) of Voronoi Area")
        plt.xlabel("Arena")
        plt.grid(True)
        plt.tight_layout()
        save_figure("violin_cv_area_by_arena.png", "figures")
    else:
        print("No data for cross‑arena plots.")
