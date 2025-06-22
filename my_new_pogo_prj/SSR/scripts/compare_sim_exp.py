#!/usr/bin/env python3

"""
Comparison plotting script for simulation vs experimental diffusion data.
Creates violin plots for simulations and dot plots for experiments,
comparing disk and annulus arena distributions.
"""

from __future__ import annotations
import os
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import argparse

from utils import *


############### MAIN COMPARISON PLOT FUNCTION ###############

def plot_simulation_vs_experiment_comparison(
    sim_df: pd.DataFrame,
    exp_df: pd.DataFrame,
    output_filename: str,
    icons_dir: str = None,
    last_iteration_only: bool = True,
    specific_iteration: int = None,
    ylabel: str = r"Final $\lambda_2$",
    ylim: tuple = None,
    jitter_width: float = 0.3,
    normalize: bool = False,
    title: str = None
):
    """
    Create a comparison plot grouped by arena, showing simulation vs experimental data.
    Each arena gets its own subplot with violin plots for simulations and dot plots for experiments.
    
    Parameters
    ----------
    sim_df : DataFrame
        Simulation data with 'current_it', 'avg_lambda', 'arena_file' columns
    exp_df : DataFrame  
        Experimental data with same structure
    output_filename : str
        Path for output plot file
    icons_dir : str, optional
        Directory containing arena icon files
    last_iteration_only : bool
        If True, use only the last iteration for comparison (ignored if specific_iteration is set)
    specific_iteration : int, optional
        If provided, use only this specific iteration number instead of last iteration
    ylabel : str
        Y-axis label
    ylim : tuple, optional
        Y-axis limits (min, max)
    jitter_width : float
        Width of jitter for dot plots
    normalize : bool
        If True, normalize data so disk arena mean = 1.0 for both simulations and experiments
    title : str, optional
        Title for the overall plot (default: no title)
    """
    
    # Filter for disk and annulus arenas only
    target_arenas = ['disk', 'annulus']
    sim_filtered = sim_df[sim_df['arena_file'].isin(target_arenas)].copy()
    exp_filtered = exp_df[exp_df['arena_file'].isin(target_arenas)].copy()
    
    # Use specific iteration, last iteration, or all iterations
    if specific_iteration is not None:
        print(f"Using specific iteration: {specific_iteration}")
        sim_filtered = sim_filtered[sim_filtered['current_it'] == specific_iteration]
        exp_filtered = exp_filtered[exp_filtered['current_it'] == specific_iteration]
    elif last_iteration_only:
        sim_last_it = sim_filtered['current_it'].max()
        exp_last_it = exp_filtered['current_it'].max()
        print(f"Using last iteration: sim={sim_last_it}, exp={exp_last_it}")
        sim_filtered = sim_filtered[sim_filtered['current_it'] == sim_last_it]
        exp_filtered = exp_filtered[exp_filtered['current_it'] == exp_last_it]
    else:
        print("Using all iterations")
    
    # Remove zero/invalid values
    sim_filtered = sim_filtered[sim_filtered['avg_lambda'] > 0]
    exp_filtered = exp_filtered[exp_filtered['avg_lambda'] > 0]
    
    # For simulations: aggregate by run to get mean lambda per run
    print("Aggregating simulation data by run...")
    if 'run' in sim_filtered.columns:
        sim_run_means = sim_filtered.groupby(['run', 'arena_file']).agg({
            'avg_lambda': 'mean',
            'current_it': 'first'  # Keep iteration info
        }).reset_index()
        sim_filtered = sim_run_means
        print(f"  Aggregated to {len(sim_filtered)} run-level means")
    else:
        print("  Warning: 'run' column not found in simulation data, using original data")
    
    # Apply normalization if requested
    if normalize:
        print("Applying normalization (disk mean = 1.0)...")
        
        # Calculate disk means for normalization
        sim_disk_data = sim_filtered[sim_filtered['arena_file'] == 'disk']['avg_lambda']
        exp_disk_data = exp_filtered[exp_filtered['arena_file'] == 'disk']['avg_lambda']
        
        if len(sim_disk_data) > 0:
            sim_disk_mean = sim_disk_data.mean()
            print(f"  Simulation disk mean (run-level): {sim_disk_mean:.3f}")
            # Normalize all simulation data by disk mean
            sim_filtered.loc[:, 'avg_lambda'] = sim_filtered['avg_lambda'] / sim_disk_mean
        else:
            print("  Warning: No simulation disk data found for normalization")
        
        if len(exp_disk_data) > 0:
            exp_disk_mean = exp_disk_data.mean()
            print(f"  Experimental disk mean: {exp_disk_mean:.3f}")
            # Normalize all experimental data by disk mean
            exp_filtered.loc[:, 'avg_lambda'] = exp_filtered['avg_lambda'] / exp_disk_mean
        else:
            print("  Warning: No experimental disk data found for normalization")
    
    # Check if we have data for both arenas
    sim_arenas = sim_filtered['arena_file'].unique()
    exp_arenas = exp_filtered['arena_file'].unique()
    
    if len(sim_arenas) == 0 or len(exp_arenas) == 0:
        raise ValueError("No valid data found for comparison")
    
    print(f"Simulation data: {len(sim_filtered)} points across arenas: {sim_arenas}")
    print(f"Experimental data: {len(exp_filtered)} points across arenas: {exp_arenas}")
    
    # Set up the plot - one subplot per arena
    sns.set_style("whitegrid")
    sns.set(font_scale=1.4)
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 6), sharey=True)
    arena_order = ['disk', 'annulus']
    
    # Create subplots grouped by arena
    for arena_idx, arena in enumerate(arena_order):
        ax = axes[arena_idx]
        color = colors_dict[arena]
        
        # Get data for this arena
        sim_arena_data = sim_filtered[sim_filtered['arena_file'] == arena]['avg_lambda']
        exp_arena_data = exp_filtered[exp_filtered['arena_file'] == arena]['avg_lambda']
        
        # Position 0: Simulation (violin plot + dots)
        if len(sim_arena_data) > 0:
            # Violin plot - made more transparent
            parts = ax.violinplot([sim_arena_data], positions=[0], widths=0.6, 
                                showmeans=True, showmedians=False)  # Only show mean
            
            # Color the violin with transparency
            for pc in parts['bodies']:
                pc.set_facecolor(color)
                pc.set_alpha(0.25)
            
            # Color the statistical lines
            for key in ['cmeans', 'cbars', 'cmins', 'cmaxes']:  # Removed 'cmedians'
                if key in parts:
                    parts[key].set_color('black')
                    parts[key].set_linewidth(1.5)
            
            # Add dots overlay - using darker arena color
            x_jittered = np.random.normal(0, jitter_width * 0.2, len(sim_arena_data))
            ax.scatter(x_jittered, sim_arena_data, 
                      color=dark_colors_dict[arena], alpha=0.9, s=35, edgecolors='white', linewidth=0.5)
        
        # Position 1: Experiments (violin plot + dots)
        if len(exp_arena_data) > 0:
            # Violin plot - made more transparent
            parts = ax.violinplot([exp_arena_data], positions=[1], widths=0.6, 
                                showmeans=True, showmedians=False)  # Only show mean
            
            # Color the violin with transparency
            for pc in parts['bodies']:
                pc.set_facecolor(color)
                pc.set_alpha(0.25)
            
            # Color the statistical lines
            for key in ['cmeans', 'cbars', 'cmins', 'cmaxes']:  # Removed 'cmedians'
                if key in parts:
                    parts[key].set_color('black')
                    parts[key].set_linewidth(1.5)
            
            # Add X markers - using darker arena color for visibility
            x_jittered = np.random.normal(1, jitter_width * 0.2, len(exp_arena_data))
            ax.scatter(x_jittered, exp_arena_data, 
                      color=dark_colors_dict[arena], alpha=1.0, s=60, marker='x', linewidth=3)
        
        # Customize subplot
        ax.set_xticks([0, 1])
        ax.set_xticklabels(['Simulation', 'Experiment'], rotation=0)
        ax.set_title(f'{arena.capitalize()} Arena', fontsize=16, fontweight='bold', color=color)
        
        if arena_idx == 0:  # Only label y-axis on leftmost plot
            ax.set_ylabel(ylabel)
        
        # Set x-axis limits to center the data
        ax.set_xlim(-0.5, 1.5)
    
    # Set y-limits if provided
    if ylim is not None:
        for ax in axes:
            ax.set_ylim(ylim)
    
    # Add arena icons if available
    if icons_dir and os.path.exists(icons_dir):
        for arena_idx, arena in enumerate(arena_order):
            ax = axes[arena_idx]
            icon_path = os.path.join(icons_dir, f"icon_{arena}.png")
            if os.path.exists(icon_path):
                # Load and tint icon
                arr = plt.imread(icon_path)
                if arr.dtype != np.float32 and arr.dtype != np.float64:
                    arr = arr.astype(float)
                
                # Tint the icon
                rgb = as_rgb_float(colors_dict[arena])
                if arr.shape[-1] >= 3:
                    arr[..., :3] = rgb
                
                # Create and place icon at the top of the subplot
                offset_img = OffsetImage(arr, zoom=0.3, dpi_cor=True)
                ab = AnnotationBbox(
                    offset_img,
                    (0.5, 0.95),  # Center horizontally, near top
                    xycoords="axes fraction",
                    frameon=False,
                    box_alignment=(0.5, 0.5),
                )
                ax.add_artist(ab)
    
    # Adjust layout
    plt.tight_layout()
    
    # Add overall title if provided
    if title:
        fig.suptitle(title, fontsize=18, fontweight='bold', y=0.98)
        plt.subplots_adjust(top=0.92)  # Make room for title
    
    # Save the plot
    os.makedirs(os.path.dirname(output_filename), exist_ok=True)
    plt.savefig(output_filename, dpi=300, bbox_inches='tight')
    plt.close()
    
    # Print statistics
    print("\n=== COMPARISON STATISTICS ===")
    for arena in arena_order:
        sim_data = sim_filtered[sim_filtered['arena_file'] == arena]['avg_lambda']
        exp_data = exp_filtered[exp_filtered['arena_file'] == arena]['avg_lambda']
        
        print(f"\n{arena.upper()} Arena:")
        if len(sim_data) > 0:
            print(f"  Simulations: n={len(sim_data)}, mean={sim_data.mean():.3f}, std={sim_data.std():.3f}")
        if len(exp_data) > 0:
            print(f"  Experiments: n={len(exp_data)}, mean={exp_data.mean():.3f}, std={exp_data.std():.3f}")

############### MAIN ###############

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Compare simulation and experimental data for disk vs annulus arenas",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic comparison
  %(prog)s -s simulations.feather -e experiments.csv -o comparison.pdf
  
  # With final lambdas file and icons
  %(prog)s -s sim.feather -e exp_diffusion.csv -f exp_final_lambdas.csv -o plots/comparison.pdf -a arenas/
  
  # Include all iterations (not just last)
  %(prog)s -s sim.feather -e exp.csv -o comparison.pdf --all-iterations
  
  # Use specific iteration (e.g., iteration 0)
  %(prog)s -s sim.feather -e exp.csv -o comparison.pdf --iteration 0
  
  # With normalization (disk mean = 1.0)
  %(prog)s -s sim.feather -e exp.csv -o comparison.pdf --normalize
  
  # With custom title
  %(prog)s -s sim.feather -e exp.csv -o comparison.pdf --title "Simulation vs Experiment Comparison"
        """
    )
    
    parser.add_argument('-s', '--simulation-file', required=True,
                       help='Path to simulation data file (.feather)')
    parser.add_argument('-e', '--experiment-file', required=True,
                       help='Path to experimental data file (.csv)')
    parser.add_argument('-f', '--final-lambdas-csv',
                       help='Path to final lambdas CSV file (for experimental data)')
    parser.add_argument('-o', '--output-file', required=True,
                       help='Output file path for the comparison plot')
    parser.add_argument('-a', '--arenas-dir', default="arenas",
                       help='Directory containing arena icon files')
    parser.add_argument('--all-iterations', action='store_true',
                       help='Use all iterations instead of just the last one')
    parser.add_argument('--iteration', type=int,
                       help='Use only this specific iteration (overrides --all-iterations)')
    parser.add_argument('--ylim', nargs=2, type=float,
                       help='Y-axis limits (min max)')
    parser.add_argument('--jitter-width', type=float, default=0.3,
                       help='Width of jitter for experimental dot plots')
    parser.add_argument('--normalize', action='store_true',
                       help='Normalize data so disk arena mean = 1.0 for both simulations and experiments')
    parser.add_argument('--title', type=str,
                       help='Title for the plot (default: no title)')
    
    args = parser.parse_args()
    
    # Load simulation data
    print(f"Loading simulation data from: {args.simulation_file}")
    try:
        sim_df, sim_config = load_data_feather(args.simulation_file)
        print(f"Loaded {len(sim_df)} simulation data points")
    except Exception as e:
        print(f"Error loading simulation data: {e}")
        exit(1)
    
    # Load experimental data
    print(f"Loading experimental data from: {args.experiment_file}")
    try:
        exp_df, exp_config = load_data_csv(args.experiment_file, args.final_lambdas_csv)
        print(f"Loaded {len(exp_df)} experimental data points")
    except Exception as e:
        print(f"Error loading experimental data: {e}")
        exit(1)
    
    # Create comparison plot
    print("Generating comparison plot...")
    try:
        ylabel = r"Normalized $\lambda_2$ (disk = 1.0)" if args.normalize else r"Final $\lambda_2$"
        plot_simulation_vs_experiment_comparison(
            sim_df=sim_df,
            exp_df=exp_df,
            output_filename=args.output_file,
            icons_dir=args.arenas_dir if os.path.exists(args.arenas_dir) else None,
            last_iteration_only=not args.all_iterations,
            specific_iteration=args.iteration,
            ylabel=ylabel,
            ylim=tuple(args.ylim) if args.ylim else None,
            jitter_width=args.jitter_width,
            normalize=args.normalize,
            title=args.title
        )
        print(f"Plot saved to: {args.output_file}")
    except Exception as e:
        print(f"Error generating plot: {e}")
        exit(1)
