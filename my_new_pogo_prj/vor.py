import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import LineCollection
from scipy.spatial import KDTree, Voronoi, voronoi_plot_2d
from shapely.geometry import Polygon, box, Point, LineString, MultiLineString
import yaml
from shapely import affinity
import math
from data import arena_polygon, arena_bounds, arena_surface
from shapely.ops import unary_union
import seaborn as sns
import os

# Folder to save figures
figure_folder = "figures"
os.makedirs(figure_folder, exist_ok=True)



# ---------- k-NN Distance Plotting ----------

def interindividual_distance_knn_over_time_plot(
    feather_path: str, figure_folder, k: int = 5, communication_radius: float = 133.0, time_step: int = 100
):
    df = pd.read_feather(feather_path)
    print("Unique time values:", df["time"].unique())
    print("Max time:", df["time"].max())

    if "run" not in df.columns:
        df["run"] = 0

    all_distances = []

    for run_id, run_df in df.groupby("run"):
        time_points = sorted(run_df["time"].unique())
        time_distances = []

        for t in time_points:
            current_positions = run_df[run_df["time"] == t][["x", "y"]].to_numpy()

            if current_positions.shape[0] > k:
                tree = KDTree(current_positions)
                distances, _ = tree.query(current_positions, k=k+1)
                knn_distances = distances[:, 1:]
                filtered_knn = knn_distances[knn_distances <= communication_radius]

                if len(filtered_knn) > 0:
                    mean_distance = np.mean(filtered_knn)
                    std_distance = np.std(filtered_knn)
                    time_distances.append((t, mean_distance, std_distance))
                else:
                    time_distances.append((t, np.nan, np.nan))
            else:
                time_distances.append((t, np.nan, np.nan))

        if time_distances:
            df_time = pd.DataFrame(time_distances, columns=["time", "mean_distance", "std_distance"])
            df_time["run"] = run_id
            all_distances.append(df_time)

    if not all_distances:
        print("No valid k-NN distances computed.")
        return

    result_df = pd.concat(all_distances)

    # Global mean over runs
    grouped = result_df.groupby("time")
    global_mean = grouped["mean_distance"].mean()
    global_std = grouped["mean_distance"].std()

    plt.figure(figsize=(10, 6))
    plt.plot(global_mean.index, global_mean.values, label="Global mean", color="blue")
    plt.fill_between(global_mean.index,
                     global_mean - global_std,
                     global_mean + global_std,
                     color="blue", alpha=0.3, label="± 1 stddev")

    plt.xlabel("Time")
    plt.ylabel(f"Mean distance to the {k} neighbors")
    plt.title(f"Global average of k-NN metric (k={k}, radius={communication_radius})")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    save_figure("knn.png", figure_folder)


    return result_df


# ---------- Voronoi Metrics ----------

def compute_voronoi_metrics(df, arena_polygon, arena_bounds, arena_surface, communication_radius=133.0, time_step=100):
    bounding_box = box(*arena_bounds)
    results = []

    if "run" not in df.columns:
        df["run"] = 0

    for run_id, run_df in df.groupby("run"):
        time_points = sorted(run_df["time"].unique())

        for t in time_points:
            points = run_df[run_df["time"] == t][["x", "y"]].to_numpy()
            if len(points) < 2:
                continue
            vor = Voronoi(points)
            areas = []
            covered_area = 0.0

            regions, vertices = voronoi_finite_polygons_2d(vor, radius=2 * max(arena_bounds[2] - arena_bounds[0],
                                                                  arena_bounds[3] - arena_bounds[1]))
            for region in regions:
                polygon = vertices[region]
                poly = Polygon(polygon)
                if not poly.is_valid or poly.is_empty:
                    continue
                clipped_poly = poly.intersection(arena_polygon)
                if not clipped_poly.is_empty:
                    areas.append(clipped_poly.area)

            # Coverage estimate using circular communication ranges
            coverage_geometries = []

            for pt in points:
                circle = Point(pt).buffer(communication_radius)        # full circle
                clipped = circle.intersection(arena_polygon)           # keep only the part inside the arena
                if not clipped.is_empty:
                    coverage_geometries.append(clipped)
                
            if coverage_geometries:
                covered_union = unary_union(coverage_geometries)       # merge all clipped circles
                covered_area = covered_union.area                      # total unique area covered
            else:
                covered_area = 0.0

            if areas:
                mean_area = np.mean(areas)
                std_area = np.std(areas)
                var_area = np.var(areas)
                cv_area = std_area/mean_area if mean_area > 0 else 0.0
                coverage_ratio = covered_area/arena_surface  
                results.append((run_id, t, mean_area, std_area, var_area, cv_area, len(areas), coverage_ratio))
                

    result_df = pd.DataFrame(results, columns=[
        "run", "time", "mean_area", "std_area", "var_area", "cv_area","n_cells", "coverage_ratio"
    ])
    return result_df

def voronoi_finite_polygons_2d(vor, radius=None):
    """
    Reconstruct infinite voronoi regions in a 2D diagram to finite
    regions.

    Parameters
    ----------
    vor : Voronoi
        Input diagram
    radius : float, optional
        Distance to 'points at infinity'.

    Returns
    -------
    regions : list of tuples
        Indices of vertices in each revised Voronoi regions.
    vertices : list of tuples
        Coordinates for revised Voronoi vertices. Same as coordinates
        of input vertices, with 'points at infinity' appended to the
        end.

    """

    if vor.points.shape[1] != 2:
        raise ValueError("Requires 2D input")

    new_regions = []
    new_vertices = vor.vertices.tolist()

    center = vor.points.mean(axis=0)
    if radius is None:
        radius = np.ptp(vor.points, axis=0).max() 

    # Construct a map containing all ridges for a given point
    all_ridges = {}
    for (p1, p2), (v1, v2) in zip(vor.ridge_points, vor.ridge_vertices):
        all_ridges.setdefault(p1, []).append((p2, v1, v2))
        all_ridges.setdefault(p2, []).append((p1, v1, v2))

    # Reconstruct infinite regions
    for p1, region in enumerate(vor.point_region):
        vertices = vor.regions[region]

        if all(v >= 0 for v in vertices):
            # finite region
            new_regions.append(vertices)
            continue

        # reconstruct a non-finite region
        ridges = all_ridges[p1]
        new_region = [v for v in vertices if v >= 0]

        for p2, v1, v2 in ridges:
            if v2 < 0:
                v1, v2 = v2, v1
            if v1 >= 0:
                # finite ridge: already in the region
                continue

            # Compute the missing endpoint of an infinite ridge

            t = vor.points[p2] - vor.points[p1] # tangent
            t /= np.linalg.norm(t)
            n = np.array([-t[1], t[0]])  # normal

            midpoint = vor.points[[p1, p2]].mean(axis=0)
            direction = np.sign(np.dot(midpoint - center, n)) * n
            far_point = vor.vertices[v2] + direction * radius

            new_region.append(len(new_vertices))
            new_vertices.append(far_point.tolist())

        # sort region counterclockwise
        vs = np.asarray([new_vertices[v] for v in new_region])
        c = vs.mean(axis=0)
        angles = np.arctan2(vs[:,1] - c[1], vs[:,0] - c[0])
        new_region = np.array(new_region)[np.argsort(angles)]

        # finish
        new_regions.append(new_region.tolist())

    return new_regions, np.asarray(new_vertices)

sns.set(font_scale = 1.3)   # Scale the font size for the entire script

def save_figure(filename, figure_folder, dpi=300):
    path = os.path.join(figure_folder, filename)
    print(f"Saving figure to: {path}")
    plt.savefig(path, dpi=dpi)
    plt.close()


def plot_voronoi_variance(df_voronoi, figure_folder):
    plt.figure(figsize=(10, 6))
    for run_id, group in df_voronoi.groupby("run"):
        plt.plot(group["time"], group["var_area"], label=f"Run {run_id}")
    plt.xlabel("Time")
    plt.ylabel("Voronoi Cell Area Variance")
    plt.title("Voronoi Cell Area Variance Over Time")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    save_figure("voronoi_variance_over_time.png", figure_folder)



def plot_voronoi_std(df_voronoi, figure_folder):
    plt.figure(figsize=(10, 6))
    for run_id, group in df_voronoi.groupby("run"):
        plt.plot(group["time"], group["std_area"], label=f"Run {run_id}")
    plt.xlabel("Time")
    plt.ylabel("Std-dev of cell area(mm²)")
    plt.title("Std-dev of cell area(mm²) Over Time")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    save_figure("voronoi_std_over_time.png", figure_folder)


def plot_cv(df_voronoi, figure_folder):
    # Example: Plot CV over time for each run
    for run_id in df_voronoi["run"].unique():
        run_df = df_voronoi[df_voronoi["run"] == run_id]
        plt.plot(run_df["time"], run_df["cv_area"], label=f"Run {run_id}")

    plt.xlabel("Time")
    plt.ylabel("Coefficient of Variation (CV)")
    plt.title("Voronoi Cell Area CV Over Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    save_figure("CV_over_time.png", figure_folder)



def plot_voronoi_global_variance(df_voronoi, figure_folder):
    grouped = df_voronoi.groupby("time")
    global_mean = grouped["var_area"].mean()
    global_std = grouped["var_area"].std()

    plt.figure(figsize=(10, 6))
    plt.plot(global_mean.index, global_mean.values, label="Global mean", color="green")
    plt.fill_between(global_mean.index,
                     global_mean - global_std,
                     global_mean + global_std,
                     color="green", alpha=0.3, label="± 1 stddev")
    plt.xlabel("Time")
    plt.ylabel("Voronoi Cell Area Variance")
    plt.title("Global Average of Voronoi Cell Area Variance Over Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    save_figure("voronoi_global_var_over_time.png", figure_folder)



def plot_coverage_ratio(df_voronoi, figure_folder):
    grouped = df_voronoi.groupby("time")
    global_mean = grouped["coverage_ratio"].mean()
    global_std = grouped["coverage_ratio"].std()

    plt.figure(figsize=(10, 6))
    plt.plot(global_mean.index, global_mean.values, label="Coverage ratio mean", color="purple")
    plt.xlabel("Time")
    y_min = df_voronoi["coverage_ratio"].min() - 0.01 * abs(df_voronoi["coverage_ratio"].min())
    y_max = df_voronoi["coverage_ratio"].max() + 0.01 * abs(df_voronoi["coverage_ratio"].max())
    plt.fill_between(global_mean.index,
                     global_mean - global_std,
                     global_mean + global_std,
                     color="purple", alpha=0.3, label="± 1 stddev")
    plt.ylim(y_min, y_max)
    plt.ylabel("Coverage Ratio")
    plt.title("Communication Coverage Over Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    save_figure("coverage_ratio.png", figure_folder)



def plot_voronoi_diagram(df, figure_folder, arena_polygon, arena_bounds, time_point,
                         run_id=0, communication_radius=133.0,
                         figsize=(10, 10)):
    """
    Plot a Voronoi diagram at a single time, clipping ALL (finite + originally
    infinite) cells to the arena polygon.
    """
    # ---------------- Data selection ----------------
    if "run" not in df.columns:
        df["run"] = 0
    run_data = df[df["run"] == run_id]
    points = run_data[run_data["time"] == time_point][["x", "y"]].to_numpy()

    if len(points) < 3:
        print(f"Not enough points at time {time_point} for run {run_id}")
        return
    
    # ---------------- Figure & arena ----------------
    fig, ax = plt.subplots(figsize=figsize)
    arena_x, arena_y = arena_polygon.exterior.xy
    ax.fill(arena_x, arena_y, color='lightgray', alpha=0.3, label='Arena')
    ax.plot(arena_x, arena_y, 'k-', linewidth=2)
    

    # ---------------- Build Voronoi -----------------
    vor = Voronoi(points)
    #voronoi_plot_2d(vor, ax) #to see the whole image
    regions, vertices = voronoi_finite_polygons_2d(vor)
    

    # ---------------- Plot cells --------------------
    sum = 0.0
    colors = plt.cm.Set3(np.linspace(0, 1, len(points)))
    for idx, (region, color) in enumerate(zip(regions, colors)):
        poly = Polygon(vertices[region])
        if not poly.is_valid or poly.is_empty:
            continue

        # clip EVERY cell (even those reconstructed from infinity)
        clipped = poly.intersection(arena_polygon)
        if clipped.is_empty:
            continue

        # Handle Polygon or MultiPolygon
        geoms = [clipped] if isinstance(clipped, Polygon) else clipped.geoms
        for i, geom in enumerate(geoms):
            if geom.is_empty or not geom.is_valid:
                continue
            x, y = geom.exterior.xy
            ax.fill(x, y, color=color, alpha=0.6,
                edgecolor='darkblue', linewidth=0.8)
            print(f"Cell {idx}: Area = {geom.area:.2f}")   
            sum += geom.area
            
    print(f"\nTotal Voronoi cell area inside arena: {sum:.2f} mm²")
    # ---------------- Communication circles ----------
    for pt in points:
        circle = Point(pt).buffer(communication_radius)
        comm_area = circle.intersection(arena_polygon)
        geoms = [comm_area] if isinstance(comm_area, Polygon) else comm_area.geoms
        for g in geoms:
            x, y = g.exterior.xy
            ax.plot(x, y, 'r--', alpha=0.5, linewidth=1)

    # ---------------- Agents -------------------------
    ax.scatter(points[:, 0], points[:, 1], c='red', s=80, zorder=10,
               edgecolors='darkred', linewidth=1,
               label=f'Agents (n={len(points)})')

    # ---------------- Formatting ---------------------
    ax.set_aspect('equal')
    ax.set_xlabel('X coordinate (mm)')
    ax.set_ylabel('Y coordinate (mm)')
    ax.set_title(f'Voronoi Diagram at Time {time_point:.2f} (Run {run_id})\n'
                 f'{len(points)} agents, Communication radius: {communication_radius} mm')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    save_figure("voronoi_diagram.png", figure_folder)


    return fig, ax



def plot_voronoi_evolution(df, figure_folder, arena_polygon, arena_bounds, time_points, run_id=0, communication_radius=133.0, figsize=(16, 12)):
    """
    Plot Voronoi diagrams at multiple time points to show evolution
    """
    from shapely.geometry import LineString
    
    n_times = len(time_points)
    cols = min(3, n_times)
    rows = (n_times + cols - 1) // cols
    
    fig, axes = plt.subplots(rows, cols, figsize=figsize)
    if n_times == 1:
        axes = [axes]
    elif rows == 1:
        axes = axes.reshape(1, -1)
    
    # Get data for the specific run
    if "run" not in df.columns:
        df["run"] = 0
    run_data = df[df["run"] == run_id]
    
    
    for i, time_point in enumerate(time_points):
        row = i // cols
        col = i % cols
        ax = axes[row, col] if rows > 1 else axes[col]
        
        # Get agent positions at this time
        points = run_data[run_data["time"] == time_point][["x", "y"]].to_numpy()
        
        if len(points) < 3:
            ax.text(0.5, 0.5, f'Not enough points\nat time {time_point:.2f}', 
                   ha='center', va='center', transform=ax.transAxes, fontsize=10)
            ax.set_title(f'Time {time_point:.2f}')
            continue
        
        # Plot arena boundary and fill
        if hasattr(arena_polygon, 'exterior'):
            arena_x, arena_y = arena_polygon.exterior.xy
            ax.fill(arena_x, arena_y, color='lightgray', alpha=0.2)
            ax.plot(arena_x, arena_y, 'k-', linewidth=1.5)
        
        # Create Voronoi diagram
        vor = Voronoi(points)
        regions, vertices = voronoi_finite_polygons_2d(vor)
        
         # ---------------- Plot cells --------------------
        colors = plt.cm.Set3(np.linspace(0, 1, len(points)))
        for idx, (region, color) in enumerate(zip(regions, colors)):
            poly = Polygon(vertices[region])
            if not poly.is_valid or poly.is_empty:
                continue

            # clip EVERY cell (even those reconstructed from infinity)
            clipped = poly.intersection(arena_polygon)
            if clipped.is_empty:
                continue

            # Handle Polygon or MultiPolygon
            geoms = [clipped] if isinstance(clipped, Polygon) else clipped.geoms
            for i, geom in enumerate(geoms):
                if geom.is_empty or not geom.is_valid:
                    continue
                x, y = geom.exterior.xy
                ax.fill(x, y, color=color, alpha=0.6,
                    edgecolor='darkblue', linewidth=0.8)
                #print(f"Agent {idx}, Cell {i}: Area = {geom.area:.2f}")
        
        
        # Plot agent positions
        ax.scatter(points[:, 0], points[:, 1], c='red', s=40, zorder=10, 
                  edgecolors='darkred', linewidth=0.8)
        
        # Set equal aspect and consistent limits
        ax.set_aspect('equal')
        ax.set_title(f'Time {time_point:.1f}\n({len(points)} agents)')
        ax.grid(True, alpha=0.3)
        
        # Only show axis labels on bottom and left edges
        if row == rows - 1:
            ax.set_xlabel('X (mm)')
        if col == 0:
            ax.set_ylabel('Y (mm)')
    
    # Hide unused subplots
    for i in range(n_times, rows * cols):
        row = i // cols
        col = i % cols
        if rows > 1:
            axes[row, col].set_visible(False)
        elif cols > 1:
            axes[col].set_visible(False)
    
    plt.suptitle(f'Voronoi Diagram Evolution (Run {run_id})\nCommunication radius: {communication_radius} mm')
    plt.tight_layout()
    save_figure("voronoi_diagram_evolution.png", figure_folder)

    
    return fig, axes

def compute_fano_over_time_corrected(df, figure_folder, communication_radius=133.0, run_id=0, plot=True):
    from scipy.spatial import KDTree

    if "run" not in df.columns:
        df["run"] = 0
    run_data = df[df["run"] == run_id]

    fano_list = []

    for t in sorted(run_data["time"].unique()):
        subset = run_data[run_data["time"] == t][["x", "y"]]
        points = subset.to_numpy()

        if len(points) < 2:
            continue

        tree = KDTree(points)
        neighbors = tree.query_ball_point(points, r=communication_radius)
        degrees = np.array([len(nlist) - 1 for nlist in neighbors])

        mu = np.mean(degrees)
        var = np.var(degrees)
        fano = var / mu if mu > 0 else np.nan

        fano_list.append((t, mu, var, fano))

    fano_df = pd.DataFrame(fano_list, columns=["time", "mean_degree", "variance", "fano_factor"])

    if plot:
        plt.figure(figsize=(10, 6))
        plt.plot(fano_df["time"], fano_df["fano_factor"], label="Fano Factor", color="darkorange")
        plt.xlabel("Time")
        plt.ylabel("Fano Factor (σ² / μ)")
        plt.title("Fano Factor of Degree Distribution Over Time")
        plt.grid(True)
        plt.tight_layout()
        save_figure("fano_factor_over_time_corrected.png", figure_folder)


    return fano_df

def compute_overall_neighbor_degree_histogram(df, figure_folder, communication_radius=133.0, run_id=0, plot=True):
    from scipy.spatial import KDTree

    if "run" not in df.columns:
        df["run"] = 0
    run_data = df[df["run"] == run_id]

    all_degrees = []

    for t in sorted(run_data["time"].unique()):
        positions = run_data[run_data["time"] == t][["x", "y"]].to_numpy()
        if len(positions) < 2:
            continue

        tree = KDTree(positions)
        neighbors = tree.query_ball_point(positions, r=communication_radius)
        degrees = np.array([len(nlist) - 1 for nlist in neighbors])  # Exclude self
        all_degrees.extend(degrees)

    if not all_degrees:
        print("No neighbor degrees found.")
        return None

    all_degrees = np.array(all_degrees)
    mean_deg = np.mean(all_degrees)
    var_deg = np.var(all_degrees)
    fano = var_deg / mean_deg if mean_deg > 0 else np.nan

    print(f"\n[Overall Degree Stats]")
    print(f"Mean degree: {mean_deg:.2f}")
    print(f"Variance: {var_deg:.2f}")
    print(f"Fano factor: {fano:.2f}")
    print(f"Total samples: {len(all_degrees)}")

    if plot:
        plt.figure(figsize=(8, 5))
        sns.histplot(all_degrees, bins=range(all_degrees.min(), all_degrees.max() + 2),
                     kde=False, color='cornflowerblue', edgecolor='black')
        plt.xlabel("Number of neighbors (degree)")
        plt.ylabel("Total agent-time samples")
        plt.title("Overall Neighbor Degree Distribution (All Time Points)")
        plt.grid(True)
        plt.tight_layout()
        save_figure("overall_neighbor_degree_histogram.png", figure_folder)


    return {
        "degrees": all_degrees,
        "mean": mean_deg,
        "variance": var_deg,
        "fano_factor": fano
    }

def compute_degree_histogram_first_last(df, figure_folder, communication_radius=133.0, plot=True):
    if "run" not in df.columns:
        df["run"] = 0

    first_degrees = []
    last_degrees = []

    for run_id, run_data in df.groupby("run"):
        times = sorted(run_data["time"].unique())
        if len(times) < 2:
            continue  # skip if there's only one time step

        t_first, t_last = times[0], times[-1]

        for t, container in [(t_first, first_degrees), (t_last, last_degrees)]:
            subset = run_data[run_data["time"] == t][["x", "y"]]
            points = subset.to_numpy()

            if len(points) < 2:
                continue

            tree = KDTree(points)
            neighbors = tree.query_ball_point(points, r=communication_radius)
            degrees = np.array([len(nlist) - 1 for nlist in neighbors])
            container.extend(degrees)

    # Convert to numpy arrays
    first_degrees = np.array(first_degrees)
    last_degrees = np.array(last_degrees)

    # Stats
    stats = {
        "first": {
            "mean": first_degrees.mean(),
            "var": first_degrees.var(),
            "fano": first_degrees.var() / first_degrees.mean() if first_degrees.mean() > 0 else np.nan,
            "samples": len(first_degrees)
        },
        "last": {
            "mean": last_degrees.mean(),
            "var": last_degrees.var(),
            "fano": last_degrees.var() / last_degrees.mean() if last_degrees.mean() > 0 else np.nan,
            "samples": len(last_degrees)
        }
    }

    if plot:
        sns.set(style="whitegrid", font_scale=1.2)
        plt.figure(figsize=(12, 5))

        for i, (label, degrees) in enumerate([("First time step", first_degrees),
                                              ("Last time step", last_degrees)]):
            plt.subplot(1, 2, i + 1)
            if len(degrees) > 0:
                bins = range(degrees.min(), degrees.max() + 2)
                sns.histplot(degrees, bins=bins, kde=False, edgecolor='black', color='steelblue')
                plt.xlabel("Number of neighbors (degree)")
                plt.ylabel("Count (across all agents & runs)")
                plt.title(label)
            else:
                plt.text(0.5, 0.5, "No data", transform=plt.gca().transAxes,
                         ha='center', va='center', fontsize=12)

        plt.tight_layout()
        plt.savefig("figures/degree_histogram_first_last.png", dpi=300)


    return stats

def compute_fano_over_time_all_runs(df, figure_folder, communication_radius=133.0, plot=True):
    from scipy.spatial import KDTree

    if "run" not in df.columns:
        df["run"] = 0

    fano_list = []

    for t in sorted(df["time"].unique()):
        degrees_all_runs = []

        for run_id in df["run"].unique():
            run_data = df[(df["run"] == run_id) & (df["time"] == t)]
            points = run_data[["x", "y"]].to_numpy()

            if len(points) < 2:
                continue

            tree = KDTree(points)
            neighbors = tree.query_ball_point(points, r=communication_radius)
            degrees = np.array([len(nlist) - 1 for nlist in neighbors])
            degrees_all_runs.extend(degrees)

        if not degrees_all_runs:
            continue

        degrees_all_runs = np.array(degrees_all_runs)
        mu = np.mean(degrees_all_runs)
        var = np.var(degrees_all_runs)
        fano = var / mu if mu > 0 else np.nan
        fano_list.append((t, mu, var, fano))

    fano_df = pd.DataFrame(fano_list, columns=["time", "mean_degree", "variance", "fano_factor"])

    if plot:
        plt.figure(figsize=(10, 6))
        plt.plot(fano_df["time"], fano_df["fano_factor"], label="Fano Factor", color="darkorange")
        plt.xlabel("Time")
        plt.ylabel("Fano Factor (σ² / μ)")
        plt.title("Fano Factor of Degree Distribution Over Time (All Runs)")
        plt.grid(True)
        plt.tight_layout()
        save_figure("fano_factor_over_time_all_runs.png", figure_folder)


    return fano_df

def compute_overall_neighbor_degree_histogram_all_runs(df, figure_folder, communication_radius=133.0, plot=True):
    from scipy.spatial import KDTree

    if "run" not in df.columns:
        df["run"] = 0

    all_degrees = []

    for (run_id, t), group in df.groupby(["run", "time"]):
        points = group[["x", "y"]].to_numpy()
        if len(points) < 2:
            continue

        tree = KDTree(points)
        neighbors = tree.query_ball_point(points, r=communication_radius)
        degrees = np.array([len(nlist) - 1 for nlist in neighbors])
        all_degrees.extend(degrees)

    if not all_degrees:
        print("No neighbor degrees found.")
        return None

    all_degrees = np.array(all_degrees)
    mu = np.mean(all_degrees)
    var = np.var(all_degrees)
    fano = var / mu if mu > 0 else np.nan

    print(f"\n[Overall Degree Stats - All Runs]")
    print(f"Mean degree: {mu:.2f}")
    print(f"Variance: {var:.2f}")
    print(f"Fano factor: {fano:.2f}")
    print(f"Total agent-time samples: {len(all_degrees)}")

    if plot:
        plt.figure(figsize=(8, 5))
        sns.histplot(all_degrees, bins=range(all_degrees.min(), all_degrees.max() + 2),
                     kde=False, color='cornflowerblue', edgecolor='black')
        plt.xlabel("Number of neighbors (degree)")
        plt.ylabel("Total agent-time samples")
        plt.title("Neighbor Degree Distribution (All Runs, All Time Points)")
        plt.grid(True)
        plt.tight_layout()
        save_figure("overall_neighbor_degree_histogram_all_runs.png", figure_folder)
        

    return {
        "degrees": all_degrees,
        "mean": mu,
        "variance": var,
        "fano_factor": fano
    }


# -------------------------------
# Example usage
# -------------------------------
#yaml_path = "conf/simple.yaml"
#arena_polygon, arena_bounds, arena_surface = load_arena_polygon_and_surface(yaml_path)

feather_path = "results/result.feather"
df = pd.read_feather(feather_path)


# k-NN plot
interindividual_distance_knn_over_time_plot(feather_path, figure_folder, k=3, communication_radius=133.0, time_step=100)

# Voronoi metrics with real arena
df_voronoi = compute_voronoi_metrics(df, arena_polygon, arena_bounds, arena_surface, communication_radius=133.0)

plot_voronoi_variance(df_voronoi, figure_folder)
plot_voronoi_std(df_voronoi, figure_folder)
plot_cv(df_voronoi, figure_folder)
plot_voronoi_global_variance(df_voronoi, figure_folder)
plot_coverage_ratio(df_voronoi, figure_folder)

# NEW: Plot Voronoi diagrams
print("\nPlotting Voronoi diagrams...")

# Plot single Voronoi diagram at a specific time
available_times = sorted(df["time"].unique())
mid_time = available_times[len(available_times)//2]  # Middle time point
plot_voronoi_diagram(df, figure_folder, arena_polygon, arena_bounds, mid_time, run_id=0, communication_radius=133.0)

# Plot evolution of Voronoi diagrams at multiple time points
sample_times = [available_times[i] for i in [0, len(available_times)//4, len(available_times)//2, 3*len(available_times)//4, -1]]
plot_voronoi_evolution(df, figure_folder, arena_polygon, arena_bounds, sample_times, run_id=0, communication_radius=133.0) 



# Compute Fano factor over time
print("\nComputing Fano factor over time...")
fano_df = compute_fano_over_time_corrected(df, figure_folder, communication_radius=133.0, run_id=0)

compute_overall_neighbor_degree_histogram(df, figure_folder, communication_radius=133.0, run_id=0)

compute_overall_neighbor_degree_histogram_all_runs(df, figure_folder, communication_radius=133.0)

stats = compute_degree_histogram_first_last(df, figure_folder, communication_radius=133.0)

