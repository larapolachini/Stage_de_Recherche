import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import LineCollection
from scipy.spatial import KDTree, Voronoi
from shapely.geometry import Polygon, box, Point, LineString, MultiLineString
import yaml
from shapely import affinity
import math
from data import arena_polygon, arena_bounds, arena_surface


# ---------- k-NN Distance Plotting ----------

def interindividual_distance_knn_over_time_plot(
    feather_path: str, k: int = 5, communication_radius: float = 133.0, time_step: int = 100
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
    plt.show()

    return result_df


# ---------- Voronoi Metrics ----------

def compute_voronoi_metrics(df, arena_polygon, arena_bounds, arena_surface, communication_radius=133.0, time_step=100):
    bounding_box = box(*arena_bounds)
    results = []

    if "run" not in df.columns:
        df["run"] = 0

    for run_id, run_df in df.groupby("run"):
        time_points = sorted(df["time"].unique())

        for t in time_points:
            points = run_df[run_df["time"] == t][["x", "y"]].to_numpy()
            if len(points) < 2:
                continue

            vor = Voronoi(points)
            areas = []
            covered_area = 0.0

            for idx, region_index in enumerate(vor.point_region):
                region = vor.regions[region_index]
                if not region or -1 in region:
                    continue

                polygon = Polygon([vor.vertices[i] for i in region])
                if not polygon.is_valid:
                    continue

                clipped = polygon.intersection(bounding_box)
                if clipped.area > 0:
                    areas.append(clipped.area)

            # Coverage estimate using circular communication ranges
            for pt in points:
                circle = Point(pt).buffer(communication_radius)  # Fixed: Point(pt) instead of points(pt)
                covered_area += circle.intersection(arena_polygon).area

            if areas:
                mean_area = np.mean(areas)
                std_area = np.std(areas)
                var_area = np.var(areas)
                coverage_ratio = covered_area / arena_surface  # Now arena_surface is guaranteed to be numeric
                results.append((run_id, t, mean_area, std_area, var_area, len(areas), coverage_ratio))

    result_df = pd.DataFrame(results, columns=[
        "run", "time", "mean_area", "std_area", "var_area", "n_cells", "coverage_ratio"
    ])
    return result_df


def plot_voronoi_variance(df_voronoi):
    plt.figure(figsize=(10, 6))
    for run_id, group in df_voronoi.groupby("run"):
        plt.plot(group["time"], group["var_area"], label=f"Run {run_id}")
    plt.xlabel("Time")
    plt.ylabel("Voronoi Cell Area Variance")
    plt.title("Voronoi Cell Area Variance Over Time")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


def plot_voronoi_global_variance(df_voronoi):
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
    plt.show()


def plot_coverage_ratio(df_voronoi):
    grouped = df_voronoi.groupby("time")
    global_mean = grouped["coverage_ratio"].mean()
    global_std = grouped["coverage_ratio"].std()

    plt.figure(figsize=(10, 6))
    plt.plot(global_mean.index, global_mean.values, label="Coverage ratio mean", color="purple")
    plt.fill_between(global_mean.index,
                     global_mean - global_std,
                     global_mean + global_std,
                     color="purple", alpha=0.3, label="± 1 stddev")
    plt.xlabel("Time")
    plt.ylabel("Coverage Ratio")
    plt.title("Communication Coverage Over Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def plot_voronoi_diagram(df, arena_polygon, arena_bounds, time_point, run_id=0, communication_radius=133.0, figsize=(10, 10)):
    """
    Plot Voronoi diagram for agents at a specific time point
    """
    
    # Get agent positions at the specified time
    if "run" not in df.columns:
        df["run"] = 0
    
    run_data = df[df["run"] == run_id]
    points = run_data[run_data["time"] == time_point][["x", "y"]].to_numpy()
    
    if len(points) < 3:
        print(f"Not enough points at time {time_point} for run {run_id}")
        return
    
    # Create Voronoi diagram
    vor = Voronoi(points)
    
    # Set up the plot with square aspect ratio
    fig, ax = plt.subplots(figsize=figsize)
    
    # Plot arena boundary (should be circular based on disk.csv)
    if hasattr(arena_polygon, 'exterior'):
        arena_x, arena_y = arena_polygon.exterior.xy
        ax.fill(arena_x, arena_y, color='lightgray', alpha=0.3, label='Arena')
        ax.plot(arena_x, arena_y, 'k-', linewidth=2, label='Arena boundary')
    
    # Create bounding box for clipping
    bounding_box = box(*arena_bounds)
    
    # Plot and fill Voronoi cells that are inside the arena
    colors = plt.cm.Set3(np.linspace(0, 1, len(points)))
    
    for idx, (point, color) in enumerate(zip(points, colors)):
        if idx < len(vor.point_region):
            region_index = vor.point_region[idx]
            region = vor.regions[region_index]
            
            if region and -1 not in region and len(region) > 2:
                # Create polygon from Voronoi vertices
                cell_vertices = [vor.vertices[i] for i in region]
                cell_polygon = Polygon(cell_vertices)
                
                if cell_polygon.is_valid:
                    # Clip to arena polygon (not just bounding box)
                    clipped = cell_polygon.intersection(arena_polygon)
                    
                    if hasattr(clipped, 'exterior') and clipped.area > 0:
                        print(f'Cell {idx} area: {clipped.area:.2f}')
                        # Plot the clipped cell
                        x_coords, y_coords = clipped.exterior.xy
                        ax.fill(x_coords, y_coords, color=color, alpha=0.6, 
                               edgecolor='darkblue', linewidth=0.8)
                    elif hasattr(clipped, 'geoms'):  # MultiPolygon case
                        for geom in clipped.geoms:
                            if hasattr(geom, 'exterior'):
                                x_coords, y_coords = geom.exterior.xy
                                ax.fill(x_coords, y_coords, color=color, alpha=0.6, 
                                       edgecolor='darkblue', linewidth=0.8)
                                
    
    # Plot Voronoi edges (only those inside the arena)
    for simplex in vor.ridge_vertices:
        if -1 not in simplex:  # Skip infinite edges
            edge_line = [vor.vertices[simplex[0]], vor.vertices[simplex[1]]]
            edge_geom = LineString(edge_line)
            
            # Only plot edges that intersect with the arena
            if edge_geom.intersects(arena_polygon):
                clipped_edge = edge_geom.intersection(arena_polygon)
                if isinstance(clipped_edge, LineString):
                    coords = list(clipped_edge.coords)
                    if len(coords) >= 2:
                        ax.plot([coords[0][0], coords[1][0]], [coords[0][1], coords[1][1]],
                            'b-', linewidth=0.5, alpha=0.7)

                elif isinstance(clipped_edge, MultiLineString):
                    for line in clipped_edge.geoms:
                        coords = list(line.coords)
                        if len(coords) >= 2:
                            ax.plot([coords[0][0], coords[1][0]], [coords[0][1], coords[1][1]],
                            'b-', linewidth=0.5, alpha=0.7)
    
    # Plot communication ranges (only show parts inside arena)
    for point in points:
        circle = Point(point).buffer(communication_radius)
        comm_area = circle.intersection(arena_polygon)
        if hasattr(comm_area, 'exterior'):
            x_coords, y_coords = comm_area.exterior.xy
            ax.plot(x_coords, y_coords, 'r--', alpha=0.5, linewidth=1)
        elif hasattr(comm_area, 'geoms'):
            for geom in comm_area.geoms:
                if hasattr(geom, 'exterior'):
                    x_coords, y_coords = geom.exterior.xy
                    ax.plot(x_coords, y_coords, 'r--', alpha=0.5, linewidth=1)
    
    # Plot agent positions
    ax.scatter(points[:, 0], points[:, 1], c='red', s=80, zorder=10, 
              edgecolors='darkred', linewidth=1, label=f'Agents (n={len(points)})')
    
    # Set equal aspect ratio and proper limits
    ax.set_aspect('equal')
    
    
    # Labels and title
    ax.set_xlabel('X coordinate (mm)', fontsize=12)
    ax.set_ylabel('Y coordinate (mm)', fontsize=12)
    ax.set_title(f'Voronoi Diagram at Time {time_point:.2f} (Run {run_id})\n'
                f'{len(points)} agents, Communication radius: {communication_radius} mm', fontsize=14)
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    return fig, ax


def plot_voronoi_evolution(df, arena_polygon, arena_bounds, time_points, run_id=0, communication_radius=133.0, figsize=(16, 12)):
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
    
    # Calculate consistent limits for all subplots
    arena_center_x = (arena_bounds[0] + arena_bounds[2]) / 2
    arena_center_y = (arena_bounds[1] + arena_bounds[3]) / 2
    arena_width = arena_bounds[2] - arena_bounds[0]
    arena_height = arena_bounds[3] - arena_bounds[1]
    max_dim = max(arena_width, arena_height)
    margin = max_dim * 0.05  # 5% margin for subplots
    
    #xlim = [arena_center_x - max_dim/2 - margin, arena_center_x + max_dim/2 + margin]
    #ylim = [arena_center_y - max_dim/2 - margin, arena_center_y + max_dim/2 + margin]
    
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
        
        # Plot Voronoi cells
        colors = plt.cm.Set3(np.linspace(0, 1, len(points)))
        
        for idx, (point, color) in enumerate(zip(points, colors)):
            if idx < len(vor.point_region):
                region_index = vor.point_region[idx]
                region = vor.regions[region_index]
                
                if region and -1 not in region and len(region) > 2:
                    cell_vertices = [vor.vertices[i] for i in region]
                    cell_polygon = Polygon(cell_vertices)
                    
                    if cell_polygon.is_valid:
                        # Clip to arena polygon
                        clipped = cell_polygon.intersection(arena_polygon)
                        
                        if hasattr(clipped, 'exterior') and clipped.area > 0:
                            x_coords, y_coords = clipped.exterior.xy
                            ax.fill(x_coords, y_coords, color=color, alpha=0.6, 
                                  edgecolor='darkblue', linewidth=0.5)
                        elif hasattr(clipped, 'geoms'):  # MultiPolygon case
                            for geom in clipped.geoms:
                                if hasattr(geom, 'exterior'):
                                    x_coords, y_coords = geom.exterior.xy
                                    ax.fill(x_coords, y_coords, color=color, alpha=0.6, 
                                           edgecolor='darkblue', linewidth=0.5)
        
        # Plot Voronoi edges (clipped to arena)
        for simplex in vor.ridge_vertices:
            if -1 not in simplex:
                edge_line = [vor.vertices[simplex[0]], vor.vertices[simplex[1]]]
                edge_geom = LineString(edge_line)
                
                if edge_geom.intersects(arena_polygon):
                    clipped_edge = edge_geom.intersection(arena_polygon)
                    if isinstance(clipped_edge, LineString):
                        coords = list(clipped_edge.coords)
                        if len(coords) >= 2:
                            ax.plot([coords[0][0], coords[1][0]], [coords[0][1], coords[1][1]],
                                'b-', linewidth=0.5, alpha=0.7)

                    elif isinstance(clipped_edge, MultiLineString):
                        for line in clipped_edge.geoms:
                            coords = list(line.coords)
                            if len(coords) >= 2:
                                ax.plot([coords[0][0], coords[1][0]], [coords[0][1], coords[1][1]],
                                    'b-', linewidth=0.5, alpha=0.7)
        
        # Plot agent positions
        ax.scatter(points[:, 0], points[:, 1], c='red', s=40, zorder=10, 
                  edgecolors='darkred', linewidth=0.8)
        
        # Set equal aspect and consistent limits
        ax.set_aspect('equal')
        #ax.set_xlim(xlim)
        #ax.set_ylim(ylim)
        ax.set_title(f'Time {time_point:.1f}\n({len(points)} agents)', fontsize=11)
        ax.grid(True, alpha=0.3)
        
        # Only show axis labels on bottom and left edges
        if row == rows - 1:
            ax.set_xlabel('X (mm)', fontsize=10)
        if col == 0:
            ax.set_ylabel('Y (mm)', fontsize=10)
    
    # Hide unused subplots
    for i in range(n_times, rows * cols):
        row = i // cols
        col = i % cols
        if rows > 1:
            axes[row, col].set_visible(False)
        elif cols > 1:
            axes[col].set_visible(False)
    
    plt.suptitle(f'Voronoi Diagram Evolution (Run {run_id})\nCommunication radius: {communication_radius} mm', 
                fontsize=14, y=0.95)
    plt.tight_layout()
    plt.show()
    
    return fig, axes


# -------------------------------
# Example usage
# -------------------------------
#yaml_path = "conf/simple.yaml"
#arena_polygon, arena_bounds, arena_surface = load_arena_polygon_and_surface(yaml_path)

feather_path = "results/result.feather"
df = pd.read_feather(feather_path)

# k-NN plot
interindividual_distance_knn_over_time_plot(feather_path, k=3, communication_radius=133.0, time_step=100)

# Voronoi metrics with real arena
df_voronoi = compute_voronoi_metrics(df, arena_polygon, arena_bounds, arena_surface, communication_radius=133.0)

plot_voronoi_variance(df_voronoi)
plot_voronoi_global_variance(df_voronoi)
plot_coverage_ratio(df_voronoi)

# NEW: Plot Voronoi diagrams
print("\nPlotting Voronoi diagrams...")

# Plot single Voronoi diagram at a specific time
available_times = sorted(df["time"].unique())
mid_time = available_times[len(available_times)//2]  # Middle time point
plot_voronoi_diagram(df, arena_polygon, arena_bounds, mid_time, run_id=0, communication_radius=133.0)

# Plot evolution of Voronoi diagrams at multiple time points
sample_times = [available_times[i] for i in [0, len(available_times)//4, len(available_times)//2, 3*len(available_times)//4, -1]]
plot_voronoi_evolution(df, arena_polygon, arena_bounds, sample_times, run_id=0, communication_radius=133.0)