#!/usr/bin/env python3

"""TODO"""

from __future__ import annotations
import os
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import matplotlib.colors as mcolors
from sklearn.metrics import confusion_matrix

import pyarrow as pa
import pyarrow.ipc as ipc


colors_dict = {
        'disk':     '#439494',      # Cyan
        'square':   '#bd6034',      # Orange
        'arrow2':   '#6c916c',      # Green
        'star':     '#b64949',      # Red
        'triangle': '#8b8b47',      # Gold
#        'arena8':   '#956e6e',      # Brown
#        'arena6':   '#6e6e6e',      # Grey
        'annulusBarred':   '#956e6e',      # Brown
        'annulus':  '#975497',      # Magenta
        }


def _as_rgb_float(col) -> tuple[float, float, float]:
    """Convert '#rrggbb' or 'tab:blue' → (r,g,b) floats 0-1."""
    return mcolors.to_rgb(col)

def _default_colours(n: int):
    """Fallback colour array."""
    return plt.cm.jet(np.linspace(0., 1., n))


############### LOAD DATA FILES ############### {{{1

def load_data(filename):
    # Feather V2 files are Arrow IPC “file” streams under the hood
    with ipc.open_file(filename) as reader:
        schema = reader.schema                   # pyarrow.Schema
        meta   = schema.metadata or {}           # dict-like, keys/values are bytes

        # Decode bytes → str for convenience
        decoded_meta = {k.decode(): v.decode() for k, v in meta.items()}

    # Load metadata
    config = decoded_meta.get("configuration", {})

    # Load dataframe
    df = pd.read_feather(filename)

    return df, config


############### LINE PLOTS ############### {{{1

def plot_line_all_arenas_df(df: pd.DataFrame,
                            output_filename: str,
                            error_mode: str = "sd",          #  "sd"  or  "pi"
                            colors_dict: dict[str, str] | None = None,
                            icons_dir: str | None = None,
                            exclude_arenas: list[str] | None = None,
                            max_it: int | None = None,
                            ylabel: str = r"Final $\lambda_2$",
                            ylim=None):
    """
    Draws a mean ± SD line plot for every arena in *df*.

    Parameters
    ----------
    df : DataFrame            (needs 'current_it', 'avg_lambda', 'arena_file')
    output_filename : str     (e.g. 'plots/evolution.pdf')
    error_mode:               "sd" draws mean ± SD, "pi" draws 75-/50-/25-percentile bands.
    colors_dict : {class: hex}  optional colour mapping
    icons_dir : str           folder with 'icon_<class>.png' files (optional)
    exclude_arenas : list[str] arenas to ignore (optional)
    max_it : int              truncate iterations beyond this value (optional)
    ylabel : str              y-axis label (default 'Final λ_2')
    ylim : tuple(min, max)    optional y-limits
    """
    sns.set(font_scale=1.3)
    sns.set_style("ticks")

    # ─── 1) prepare data ─────────────────────────────────────────
    df2 = df.copy()
    df2.loc[df2['avg_lambda'] == 0, 'avg_lambda'] = np.nan
    if exclude_arenas:
        df2 = df2[~df2["arena_file"].isin(exclude_arenas)]
    if max_it is not None:
        df2 = df2[df2["current_it"] <= max_it]

    arena_names = sorted(df2["arena_file"].unique())
    nb_arenas = len(arena_names)

    # colours / icons --------------------------------------------
    if colors_dict is None:
        colors_dict = {}
    colours = [colors_dict.get(a, None) for a in arena_names]
    if any(c is None for c in colours):
        colours = _default_colours(nb_arenas)

    icons = []
    if icons_dir:
        icons = [os.path.join(icons_dir, f"icon_{a}.png") for a in arena_names]

    # figure ------------------------------------------------------
    golden = 1.618
    fig, ax = plt.subplots(1, 1, figsize=(4.0 * golden, 4.0))

    # ─── 2) plot each arena ─────────────────────────────────────
    last_y_mean = []

    alpha_map = {75: .15, 50: .25, 25: .35}      # transparency for PI ribbons

    for arena, col in zip(arena_names, colours):
        part = df2[df2["arena_file"] == arena]

        # aggregate once: mean, std, and the 12.5 / 25 / 37.5 / 62.5 / 75 / 87.5 % quantiles
        stats = part.groupby("current_it")["avg_lambda"].agg(
            mean="mean",
            std="std",
            q12=lambda s: np.nanquantile(s, 0.125),
            q25=lambda s: np.nanquantile(s, 0.25),
            q37=lambda s: np.nanquantile(s, 0.375),
            q62=lambda s: np.nanquantile(s, 0.625),
            q75=lambda s: np.nanquantile(s, 0.75),
            q87=lambda s: np.nanquantile(s, 0.875),
        ).reset_index()

        x      = stats["current_it"].to_numpy()
        ymean  = stats["mean"].to_numpy()

        # always draw the mean curve once
        ax.plot(x, ymean, color=col, linewidth=1.8)

        if error_mode.lower() == "sd":
            ystd = stats["std"].to_numpy()
            ax.fill_between(x, ymean - ystd, ymean + ystd, color=col, alpha=0.30)

        else:  # "pi"  → three nested percentile-interval bands  (75 | 50 | 25)
            # 75 % PI = 12.5th–87.5th; 50 % PI = 25th–75th; 25 % PI = 37.5th–62.5th
            for perc, (low, high) in zip(
                    (75, 50, 25),
                    ((stats["q12"], stats["q87"]),
                     (stats["q25"], stats["q75"]),
                     (stats["q37"], stats["q62"]))):

                ax.fill_between(x, low, high,
                                color=col,
                                alpha=alpha_map[perc])

        # keep a valid last-iteration mean for icon placement
        last_y_mean.append(ymean[~np.isnan(ymean)][-1])

    # ─── 3) axes cosmetics ──────────────────────────────────────
    ax.set_xlabel(r"Iterations $\xi$")
    ax.set_ylabel(ylabel)
    if ylim is not None:
        ax.set_ylim(*ylim)

    # ─── 4) icon labels at the right side ───────────────────────
    if icons:
        fig = ax.figure            # make sure the renderer knows the final limits
        fig.canvas.draw()          # <- triggers a full draw, updates xlim / ylim
        x_max = ax.get_xlim()[1]   # reliable final right-hand x value
        for arena, y_val, col, icon_path in zip(arena_names, last_y_mean, colours, icons):
            if np.isnan(y_val) or not os.path.exists(icon_path):
                continue
            arr = plt.imread(icon_path)
            arr = _tint(arr, _as_rgb_float(col))
            offset_img = OffsetImage(arr, zoom=0.35, dpi_cor=True)
            xy_icon = (ax.get_xlim()[1], y_val)      # data coords (right border)
            # place icon 8 pt to the right of the last data point
            ab = AnnotationBbox(
                    offset_img,
                    (x_max, y_val),                 # anchor in DATA coords
                    xycoords="data",
                    xybox=(8, 0),                   # shift right by 8 points
                    boxcoords="offset points",
                    frameon=False,
                    box_alignment=(0.0, 0.5),
            )
            ax.add_artist(ab)
            ax.add_artist(ab)

    sns.despine(ax=ax)

    # ─── 5) dynamic layout paddings ─────────────────────────────
    bottom_pad = 0.16 + 0.01 * max(nb_arenas - 4, 0)  # grow a little w/ classes
    right_pad  = 0.97 if not icons else 0.92          # leave room for icon column
    plt.subplots_adjust(left=0.12, bottom=bottom_pad, right=right_pad, top=0.97)

    # ─── 6) export & close ──────────────────────────────────────
    os.makedirs(os.path.dirname(output_filename), exist_ok=True)
    plt.savefig(output_filename)
    plt.close()


############### CONFUSION MATRICES PLOTS ############### {{{1


def _centroids_last_iter(df_last: pd.DataFrame) -> pd.Series:
    """Mean `avg_lambda` per arena in the last iteration."""
    return df_last.groupby("arena_file")["avg_lambda"].mean()


def _nearest_centroid_labels(df_last: pd.DataFrame,
                             centroids: pd.Series) -> tuple[np.ndarray, np.ndarray]:
    """Return integer vectors (y_true, y_pred) using nearest-centroid rule."""
    arena_names = list(centroids.index)
    name_to_idx = {name: i for i, name in enumerate(arena_names)}
    y_true = df_last["arena_file"].map(name_to_idx).to_numpy()

    dists = np.abs(df_last["avg_lambda"].to_numpy()[:, None] -
                   centroids.to_numpy()[None, :])
    y_pred = dists.argmin(axis=1)
    return y_true, y_pred


def _format_annot(cm: np.ndarray) -> np.ndarray:
    """Create string array w/ max 2 decimals, trailing zeros stripped."""
    f = np.vectorize(lambda v: f"{v:.2f}".rstrip("0").rstrip("."))
    return f(cm)



def _reorder(arr, desired: list[str], current: list[str]):
    """Reorder rows/cols or list according to *desired* class order."""
    idx = [current.index(c) for c in desired if c in current]
    if isinstance(arr, np.ndarray):
        return arr[np.ix_(idx, idx)]
    return [arr[i] for i in idx]

def _tint(arr: np.ndarray, rgb: tuple[float, float, float]) -> np.ndarray:
    """Re-colour an RGBA image array *in-place* and return it."""
    if arr.dtype != np.float32 and arr.dtype != np.float64:
        arr = arr.astype(float)
    arr[..., :3] = rgb
    return arr



def confusion_mat_plot_df(df: pd.DataFrame,
                          output_dir: str,
                          colors_dict: dict[str, str] | None = None,
                          icons_dir: str | None = None,
                          ordered_class_names: list[str] | None = None,
                          fig_name: str = "confusion.pdf") -> None:
    """
    Plot a row-normalised confusion matrix for the *last* iteration inside `df`.

    Parameters
    ----------
    df : DataFrame with 'current_it', 'avg_lambda', 'arena_file'
    output_dir : directory where the PDF will be written
    colors_dict : mapping class-name → hex-colour (optional)
    icons_dir : folder containing 'icon_<class>.png' (optional)
    ordered_class_names : desired display order (optional)
    fig_name : file name for the saved figure (default 'confusion.pdf')
    """
    # ── 1) compute confusion matrix ─────────────────────────
    last_it = df["current_it"].max()
    df_last = df[df["current_it"] == last_it]

    centroids = _centroids_last_iter(df_last)
    y_true, y_pred = _nearest_centroid_labels(df_last, centroids)
    cm = confusion_matrix(y_true, y_pred, normalize="true")

    class_names = list(centroids.index)
    nb_arenas = len(class_names)

    # ── 2) build colours & icons lists ──────────────────────
    if colors_dict is None:
        colors_dict = {}
    colours = [colors_dict.get(a, None) for a in class_names]
    if any(c is None for c in colours):
        colours = _default_colours(nb_arenas)        # fallback
    icons = []
    if icons_dir:
        icons = [os.path.join(icons_dir, f"icon_{a}.png") for a in class_names]

    # reorder if user requested a specific class order
    if ordered_class_names is not None:
        cm = _reorder(cm, ordered_class_names, class_names)
        colours = _reorder(colours, ordered_class_names, class_names)
        icons = _reorder(icons, ordered_class_names, class_names)
        class_names = [c for c in ordered_class_names if c in class_names]

    # ── 3) seaborn heat-map ─────────────────────────────────
    sns.set_style("ticks")
    sns.set(font_scale=2.0 if nb_arenas == 2 else 1.6)

    annot = _format_annot(cm)
    show_labels = not icons                            # only if no icons

    heat = sns.heatmap(
        cm,
        cmap="Greens",
        xticklabels=class_names if show_labels else [],
        yticklabels=class_names if show_labels else [],
        annot=annot,
        fmt="",                 # strings pre-formatted
        linewidths=.5,
        cbar_kws={"label": "Prop. of instances"},
        vmin=0., vmax=1.0,
        annot_kws={"fontsize": 16} if nb_arenas > 2 else {}
    )

    ax = plt.gca()

    # ── 4) draw margin icons if provided ───────────────────
    if icons:
        zoom = 0.60 if nb_arenas == 2 else 0.40
        for idx, (col, icon_path) in enumerate(zip(colours, icons)):
            if not os.path.exists(icon_path):
                continue
            arr = plt.imread(icon_path).astype(float)
            rgb = _as_rgb_float(col)
            if arr.shape[-1] >= 3:
                arr[..., :3] = rgb

            offset_img = OffsetImage(arr, zoom=zoom, dpi_cor=True)

            # left (actual) margin
            ab_left = AnnotationBbox(
                offset_img,
                (-0.06, 1 - (idx + 0.5) / nb_arenas),
                xycoords="axes fraction",
                frameon=False,
                box_alignment=(0.5, 0.5),
            )
            ax.add_artist(ab_left)

            # bottom (predicted) margin
            ab_bottom = AnnotationBbox(
                offset_img,
                ((idx + 0.5) / nb_arenas, -0.06),
                xycoords="axes fraction",
                frameon=False,
                box_alignment=(0.5, 0.5),
            )
            ax.add_artist(ab_bottom)

    # ── 5) labels & layout ─────────────────────────────────
    pad = 30 + max(0, nb_arenas - 4) * 2
    plt.xlabel("Predicted classes", labelpad=pad)
    plt.ylabel("Actual classes",   labelpad=pad)

    # Dynamic paddings ----------------------------------------------------
    # Icon row needs a constant 0.05 (same we used in AnnotationBbox)
    icon_pad   = 0.05 if icons else 0.0
    # Add 0.02 for every *extra* class beyond 4, capped so it never exceeds 0.30
    bottom_pad = icon_pad + min(0.20, 0.10 + 0.02 * max(nb_arenas - 4, 0))
    # Shrink the right margin a bit as the figure gets wider; keep ≥0.82
    right_pad  = max(0.90, 0.94 - 0.02 * max(nb_arenas - 4, 0))
    plt.subplots_adjust(left=0.12,
                    bottom=bottom_pad,
                    right=right_pad,
                    top=0.97)
    #plt.subplots_adjust(left=0.12, bottom=0.18, right=0.94, top=0.98)

    # ── 6) save figure ─────────────────────────────────────
    os.makedirs(output_dir, exist_ok=True)
    plt.savefig(os.path.join(output_dir, fig_name))
    plt.close()


############### MAIN ############### {{{1

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--inputFile', type=str, default='results/result.feather', help = "Path of the input feather file")
    parser.add_argument('-o', '--outputDir', type=str, default=".", help = "Directory of the resulting plot files")
    parser.add_argument('-a', '--arenasDir', type=str, default="arenas", help = "Directory of the arenas files")
    args = parser.parse_args()

    output_dir = args.outputDir

    # Load data
    df, config = load_data(args.inputFile)
    #df = df[df['arena_file'] != "star"] # XXX

    # Find all icons
    arena_names = df['arena_file'].unique()

#    # Make all plots
    confusion_mat_plot_df(df,
                          output_dir=args.outputDir,
                          colors_dict=colors_dict,
                          icons_dir=args.arenasDir,
                          ordered_class_names=list(colors_dict.keys()))

    plot_line_all_arenas_df(
        df,
        output_filename=os.path.join(args.outputDir, "evolution.pdf"),
        error_mode="sd",
        colors_dict=colors_dict,
        icons_dir=args.arenasDir,
        max_it=None
    )

# MODELINE "{{{1
# vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
# vim:foldmethod=marker
