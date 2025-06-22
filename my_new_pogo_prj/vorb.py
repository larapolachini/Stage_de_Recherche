import os
import yaml
import pandas as pd
from shapely.geometry import Polygon
from shapely import affinity
from vor import (  # Replace with your actual function imports
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
import matplotlib.pyplot as plt
import seaborn as sns

sns.set(font_scale=1.3)
figure_folder = "figures"

def save_figure(filename, dpi=300):
    global figure_folder
    path = os.path.join(figure_folder, filename)
    plt.savefig(path, dpi=dpi)
    plt.close()

def load_arena_polygon_and_bounds(arena_file, arena_surface):
    with open(arena_file, 'r') as f:
        lines = f.readlines()

    loops = []
    current_loop = []

    for line in lines:
        stripped = line.strip()
        if not stripped:
            if current_loop:
                loops.append(current_loop)
                current_loop = []
        else:
            x, y = map(float, stripped.split(','))
            current_loop.append((x, y))

    if current_loop:
        loops.append(current_loop)

    if not loops:
        raise ValueError("No valid loops found in arena file")

    shell = loops[0]
    holes = loops[1:] if len(loops) > 1 else []

    polygon = Polygon(shell, holes=holes)

    scale = (arena_surface / polygon.area) ** 0.5
    minx, miny, *_ = polygon.bounds
    polygon = affinity.translate(polygon, xoff=-minx, yoff=-miny)
    polygon = affinity.scale(polygon, xfact=scale, yfact=scale, origin=(0, 0))

    return polygon, polygon.bounds

def run_analysis_for_arena(arena_file, arena_surface):
    arena_name = os.path.splitext(os.path.basename(arena_file))[0]
    feather_path = f"results/result_{arena_name}.feather"

    print(f"\n=== Processing Arena: {arena_name} ===")
    if not os.path.exists(feather_path):
        print(f"Missing feather file: {feather_path}")
        return

    df = pd.read_feather(feather_path)
    arena_polygon, arena_bounds = load_arena_polygon_and_bounds(arena_file, arena_surface)

    global figure_folder
    figure_folder = f"figures/{arena_name}"
    os.makedirs(figure_folder, exist_ok=True)

    interindividual_distance_knn_over_time_plot(feather_path, figure_folder, k=3, communication_radius=133.0)
    df_voronoi = compute_voronoi_metrics(df, arena_polygon, arena_bounds, arena_surface, communication_radius=133.0)

    plot_voronoi_variance(df_voronoi, figure_folder)
    plot_voronoi_std(df_voronoi, figure_folder)
    plot_cv(df_voronoi, figure_folder)
    plot_voronoi_global_variance(df_voronoi, figure_folder)
    plot_coverage_ratio(df_voronoi, figure_folder)

    times = sorted(df["time"].unique())
    mid_time = times[len(times) // 2]
    sample_times = [times[i] for i in [0, len(times)//4, len(times)//2, 3*len(times)//4, -1]]

    plot_voronoi_diagram(df, figure_folder, arena_polygon, arena_bounds, mid_time, run_id=0)
    plot_voronoi_evolution(df, figure_folder, arena_polygon, arena_bounds, sample_times, run_id=0)

    compute_fano_over_time_corrected(df, figure_folder, communication_radius=133.0, run_id=0)
    compute_overall_neighbor_degree_histogram(df, figure_folder, communication_radius=133.0, run_id=0)
    compute_overall_neighbor_degree_histogram_all_runs(df, figure_folder, communication_radius=133.0)
    compute_degree_histogram_first_last(df, figure_folder, communication_radius=133.0)

    print(f" Finished: {arena_name}")

# === Main Runner ===

if __name__ == "__main__":
    with open("conf/simpleb.yaml", "r") as f:
        config = yaml.safe_load(f)

    arena_surface = float(config["arena_surface"])
    arena_files = config["arena_file"]["batch_options"]

    for arena_file in arena_files:
        run_analysis_for_arena(arena_file, arena_surface)

    print("\n✅ All arenas processed.")

    # === Violin Plot Across Arenas ===

    print("\n Generating violin plot of CV area across arenas...")

    all_data = []

    for arena_file in arena_files:
        arena_name = os.path.splitext(os.path.basename(arena_file))[0]
        feather_path = f"results/result_{arena_name}.feather"

        if not os.path.exists(feather_path):
            print(f"[Skipping] Missing: {feather_path}")
            continue

        df = pd.read_feather(feather_path)
        arena_polygon, arena_bounds = load_arena_polygon_and_bounds(arena_file, arena_surface)

        df_voronoi = compute_voronoi_metrics(
            df,
            arena_polygon=arena_polygon,
            arena_bounds=arena_bounds,
            arena_surface=arena_surface,
            communication_radius=133.0
        )
        df_voronoi["arena"] = arena_name
        all_data.append(df_voronoi)

    if all_data:
        combined_df = pd.concat(all_data, ignore_index=True)

        plt.figure(figsize=(10, 6))
        sns.violinplot(data=combined_df, x="arena", y="cv_area", inner="box", scale="width", cut=0)
        plt.title("Distribution of CV Area by Arena")
        plt.ylabel("Coefficient of Variation (CV) of Voronoi Area")
        plt.xlabel("Arena")
        plt.grid(True)
        plt.tight_layout()

        os.makedirs("figures", exist_ok=True)
        plt.savefig("figures/violin_cv_area_by_arena.png", dpi=300)
        plt.show()
    else:
        print(" No data available for violin plot.")
