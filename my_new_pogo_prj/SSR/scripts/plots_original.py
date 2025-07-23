#!/usr/bin/env python3

"""
Unified plotting script for diffusion experiment data.
Supports both feather files (original format) and CSV files (from log_parser.py).
"""

from __future__ import annotations
import os
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import matplotlib.colors as mcolors
from sklearn.metrics import confusion_matrix
from scipy.signal import savgol_filter
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib import rcParams
import pathlib
import shutil
import pprint

from utils import *
import network

sns.set(font_scale=1.3)
plt.rc('text', usetex=True)
plt.rc('text.latex', preamble=r''.join([
        r'\usepackage{amsmath}',
        r"\usepackage[T1]{fontenc}",
        r"\usepackage{helvet}",
        r"\renewcommand{\familydefault}{\sfdefault}",
        r"\usepackage[helvet]{sfmath}",
        r"\everymath={\sf}",
        r'\centering',
        ]))


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
        colours = default_colours(nb_arenas)

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
        ax.plot(x, ymean, color=col, linewidth=1.8, label=arena)

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
        if len(ymean[~np.isnan(ymean)]) > 0:
            last_y_mean.append(ymean[~np.isnan(ymean)][-1])
        else:
            last_y_mean.append(0)

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
            arr = _tint(arr, as_rgb_float(col))
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
        colours = default_colours(nb_arenas)        # fallback
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
            rgb = as_rgb_float(col)
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

    # ── 6) save figure ─────────────────────────────────────
    os.makedirs(output_dir, exist_ok=True)
    plt.savefig(os.path.join(output_dir, fig_name))
    plt.close()


############### Line plots of s ############### {{{1



def categorize_by_iteration(df: pd.DataFrame, *, time_col: str = "t") -> pd.DataFrame:
    """
    1.  Create a traj_id for every (current_it, robot_id, run, arena_file)
        where current_behavior == 4.
    2.  For each (run, arena_file) block:
          • take the first iteration (smallest current_it);
          • find the *longest* trajectory length in that iteration
            → reference length;
          • any traj_id in later iterations with fewer time-steps than
            that reference is reset to -1.

    Parameters
    ----------
    df        : the dataframe (is modified in-place and returned)
    time_col  : column whose unique values count the time-steps (default "t")
    """
    # ------------------------------------------------------------------ 1️⃣  initial traj_id
    ok = df["current_behavior"].eq(4)
    df["traj_id"] = -1
    df.loc[ok, "traj_id"] = (
        df.loc[ok]
          .groupby(["current_it", "robot_id", "run", "arena_file"], sort=False)
          .ngroup()
    )

    # ------------------------------------------------------------------ 2️⃣  step count per traj_id
    n_steps = (
        df.groupby("traj_id")[time_col]          # <- YOUR requested grouping
          .nunique()
          .rename("n_steps")
    )

    meta = (
        df[["traj_id", "run", "arena_file", "current_it"]]
        .drop_duplicates(subset="traj_id")
        .set_index("traj_id")
    )

    counts = meta.join(n_steps, how="left").reset_index()

    # ------------------------------------------------------------------ 3️⃣  reference length = longest traj in *first* iteration
    #      find first iteration (smallest current_it) per (run, arena_file)
    first_it = (
        counts
        .groupby(["run", "arena_file"])["current_it"]
        .transform("min")
    )
    base = counts[counts["current_it"] == first_it]

    ref_len = (
        base.groupby(["run", "arena_file"])["n_steps"]
            .max()
            .rename("ref_steps")
            .reset_index()
    )

    counts = counts.merge(ref_len, on=["run", "arena_file"], how="left")

    # ------------------------------------------------------------------ 4️⃣  traj_ids shorter than reference → -1
    bad_traj_ids = counts.loc[counts["n_steps"] < counts["ref_steps"], "traj_id"]

    if not bad_traj_ids.empty:
        df.loc[df["traj_id"].isin(bad_traj_ids), "traj_id"] = -1

    # ------------------------------------------------------------------ 5️⃣  categorical tidy-up
    df["traj_id"] = pd.Categorical(df["traj_id"])   # keeps −1 as a category
    return df


def plot_log_abs_s(
    df: pd.DataFrame,
    current_it: int,
    run: int,
    arena_file: str,
    *,
    ylim = None,
    use_savgol: bool = True,
    window: int = 51,
    poly: int = 3,
    burn_in_seconds: float = 20.0,
    logscale: bool = True,
    show_legend: bool = False,
    show_title: bool = True,
    ax: plt.Axes | None = None,
):
    """
    Plot |s_i^n| (raw or Sav-Gol) versus diffusion step n for each robot_id
    in the chosen (current_it, run, arena_file) slice.

    Parameters
    ----------
    ...
    show_title   : bool, add/omit the title line
    """

#    # ------------------------------- 0. font prefs ---------------------------------
#    rcParams.update({
#        "font.family": "sans-serif",
#        "font.sans-serif": ["Helvetica", "Arial", "Liberation Sans", "DejaVu Sans"],
#        "axes.labelsize": 12,
#        "axes.titlesize": 13,
#        "legend.fontsize": 9,
#    })

    # ------------------------------- 1. slice data ---------------------------------
    sl = df[
        (df["current_it"] == current_it) &
        (df["run"]        == run) &
        (df["arena_file"] == arena_file) &
        (df["traj_id"]    != -1)
    ]
    if sl.empty:
        raise ValueError("No matching rows for the requested slice.")

    t0          = sl["time"].min()
    t_end_burn  = t0 + burn_in_seconds

    # ------------------------------- 2. axes prep ----------------------------------
    if ax is None:
        _, ax = plt.subplots(figsize=(8, 4.5))

    n_colors = sl["robot_id"].nunique()
    cmap     = plt.get_cmap("tab20" if n_colors <= 20 else "rainbow")
    max_n_b  = 0

    # ------------------------------- 3. plot traces --------------------------------
    for idx, (robot_id, g) in enumerate(sl.groupby("robot_id", sort=False)):
        g = g.sort_values("t")
        n = np.arange(len(g), dtype=int)                        # diffusion step
        last_burn = n[g["time"].to_numpy() <= t_end_burn].max(initial=0)
        max_n_b   = max(max_n_b, last_burn)

        y = np.abs(g["s"].to_numpy(dtype=float))
        if use_savgol and len(y) > poly + 2:
            win = min(window, len(y) if len(y) % 2 else len(y) - 1)
            win = max(win, poly + 2 | 1)
            y  = savgol_filter(y, win, poly, mode="interp")

        plot_fun = ax.semilogy if logscale else ax.plot
        plot_fun(
            n, y,
            label=f"robot {robot_id}",
            lw=1.2,
            color=cmap(idx / max(1, n_colors - 1)),
        )

    # ------------------------------- 4. burn-in zone -------------------------------
    ax.axvspan(0, max_n_b, facecolor="pink", alpha=0.25, zorder=0)

    # label at the *bottom* of the shaded area (axes coords y≈0)
    ax.text(
        0.010, 0.02,               # left, bottom in axes fraction
        r"burn\\  in",
        transform=ax.transAxes,
        fontsize=14,
        fontweight="bold",
        ha="left",
        va="bottom",
    )

    # ------------------------------- 5. cosmetics ----------------------------------
    ax.set_xlim(left=0)
    if ylim:
        ax.set_ylim(ylim)
    ax.set_xlabel(r"Diffusion step $n$")
    ax.set_ylabel(r"$| s_{i}^n |$")
    if logscale:
        ax.set_yscale("log")
    #ax.grid(True, lw=0.3, alpha=0.5, which="both")

    if show_title:
        title = (
            r"$|s_i^n|$" + (" (log-scale)" if logscale else "") +
            f" – current_it={current_it}, run={run}, arena='{arena_file}'" +
            (" – Sav-Gol" if use_savgol else " – raw")
        )
        ax.set_title(title)

    if show_legend:
        ax.legend(ncol=2, frameon=False)

    return ax


def plot_mean_log_abs_s(
    df: pd.DataFrame,
    current_it: int,
    run: int,
    arena_file: str,
    *,
    ylim=None,                     # NEW – match plot_log_abs_s
    use_savgol: bool = True,
    window: int = 51,
    poly: int = 3,
    burn_in_seconds: float = 20.0,
    logscale: bool = True,
    show_title: bool = True,
    ax: plt.Axes | None = None,
):
    """
    One black curve per (current_it, run, arena_file):
      • y = mean |s| across robots at each diffusion step n
      • dotted inside burn-in, solid afterwards
    """

    # 1️⃣  select valid rows
    sl = df[
        (df["current_it"] == current_it)
        & (df["run"] == run)
        & (df["arena_file"] == arena_file)
        & (df["traj_id"] != -1)
    ].copy()
    if sl.empty:
        raise ValueError("No matching rows for the requested slice.")

    t0, t_end_burn = sl["time"].min(), sl["time"].min() + burn_in_seconds

    # 2️⃣  diffusion index n (per robot)
    sl = sl.sort_values(["robot_id", "t"])
    sl["n"] = sl.groupby("robot_id").cumcount()

    # 3️⃣  mean |s| for every n
    mean_abs_s = (
        sl.assign(abs_s=sl["s"].abs())
          .groupby("n")["abs_s"]
          .mean()
          .sort_index()
    )
    n_vals, y_vals = mean_abs_s.index.to_numpy(), mean_abs_s.to_numpy()

    # optional Sav-Gol
    if use_savgol and len(y_vals) > poly + 2:
        win = min(window, len(y_vals) if len(y_vals) % 2 else len(y_vals) - 1)
        win = max(win, poly + 2 | 1)
        y_vals = savgol_filter(y_vals, win, poly, mode="interp")

    # 4️⃣  locate last burn-in step
    burn_mask_full = sl["time"] <= t_end_burn
    last_burn_n = sl.loc[burn_mask_full, "n"].max() if burn_mask_full.any() else -1

    # 5️⃣  plotting
    if ax is None:
        _, ax = plt.subplots(figsize=(8, 4.5))

    # background shade
    if last_burn_n >= 0:
        ax.axvspan(0, last_burn_n, facecolor="pink", alpha=0.25, zorder=0)

    plot_fun = ax.semilogy if logscale else ax.plot

    # full solid curve (ensures continuity)
    plot_fun(n_vals, y_vals, lw=1.5, color="black", zorder=1)

    # overlay dotted burn-in segment
    if last_burn_n >= 0:
        burn_mask = n_vals <= last_burn_n
        plot_fun(
            n_vals[burn_mask],
            y_vals[burn_mask],
            linestyle=":",
            lw=1.5,
            color="black",
            zorder=2,
        )

    # burn-in label (bold, two-line, bottom-left)
    if last_burn_n >= 0:
        ax.text(
            0.010, 0.02,
            r"burn\\  in",
            transform=ax.transAxes,
            fontsize=14,
            fontweight="bold",
            ha="left",
            va="bottom",
        )

    # cosmetics to match plot_log_abs_s
    ax.set_xlim(left=0)
    if ylim is not None:
        ax.set_ylim(ylim)
    ax.set_xlabel(r"Diffusion step $n$")
    ax.set_ylabel(r"$| s_{i}^n |$")
    if logscale:
        ax.set_yscale("log")
    # grid intentionally left off (same as your edited version)

    if show_title:
        ax.set_title(
            r"$|s_i^n|$ mean"
            + (" (log-scale)" if logscale else "")
            + f" – current_it={current_it}, run={run}, arena='{arena_file}'"
            + (" – Sav-Gol" if use_savgol else " – raw")
        )

    return ax


def export_random_log_abs_s(
    df: pd.DataFrame,
    k: int,
    out_dir: str,
    *,
    random_state: int | None = None,
    **plot_kwargs,                    # forwarded to BOTH plot_… functions
):
    """
    For each arena_file in *df*:
      • pick *k* random (current_it, run) pairs (or fewer if not available);
      • generate two PDFs per pair:
          1. plot_log_abs_s      (multi-robot coloured lines)
          2. plot_mean_log_abs_s (one black mean line, dotted in burn-in)

    Parameters
    ----------
    df           : DataFrame containing the data + traj_id
    k            : number of random parameter sets per arena
    out_dir      : directory to save the PDFs
    random_state : int for reproducible sampling (optional)
    **plot_kwargs: extra arguments for the plot functions
    """
    rng = np.random.default_rng(random_state)
    out_path = pathlib.Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    # ──────────────────────────────────────────────────────────────────────────
    for arena, df_a in df.groupby("arena_file"):
        unique_pairs = (
            df_a[["current_it", "run"]]
            .drop_duplicates()
            .to_numpy()
        )
        if unique_pairs.size == 0:
            continue

        n_pick  = min(k, len(unique_pairs))
        sel_idx = rng.choice(len(unique_pairs), size=n_pick, replace=False)

        for current_it, run in unique_pairs[sel_idx]:
            # 1️⃣  multi-robot plot ------------------------------------------------
            fig1, ax1 = plt.subplots(figsize=(8, 4.5))
            try:
                plot_log_abs_s(
                    df,
                    int(current_it),
                    int(run),
                    arena,
                    ax=ax1,
                    **plot_kwargs,
                )
            except ValueError as e:
                plt.close(fig1)
                print(f"Skip robots view ({arena}, it={current_it}, run={run}): {e}")
            else:
                pdf_name = (
                    f"{arena}_it{int(current_it)}_run{int(run)}_robots.pdf"
                    .replace(" ", "_")
                )
                with PdfPages(out_path / pdf_name) as pdf:
                    pdf.savefig(fig1, bbox_inches="tight")
                plt.close(fig1)
                print(f"Saved {pdf_name}")

            # 2️⃣  mean-trajectory plot ------------------------------------------
            fig2, ax2 = plt.subplots(figsize=(8, 4.5))
            try:
                plot_mean_log_abs_s(
                    df,
                    int(current_it),
                    int(run),
                    arena,
                    ax=ax2,
                    **plot_kwargs,
                )
            except ValueError as e:
                plt.close(fig2)
                print(f"Skip mean view ({arena}, it={current_it}, run={run}): {e}")
            else:
                pdf_name = (
                    f"{arena}_it{int(current_it)}_run{int(run)}_mean.pdf"
                    .replace(" ", "_")
                )
                with PdfPages(out_path / pdf_name) as pdf:
                    pdf.savefig(fig2, bbox_inches="tight")
                plt.close(fig2)
                print(f"Saved {pdf_name}")


############### MAIN ############### {{{1

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
        description="Generate plots from feather files or CSV files (from log_parser.py)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
File Format Support:
  - Feather files (.feather): Original format with embedded metadata
  - CSV files (.csv): Generated by log_parser.py
  
Examples:
  # Feather file (original usage)
  %(prog)s -i results/result.feather -o plots/ -a arenas/
  
  # CSV files from log_parser.py
  %(prog)s -i combined_diffusion_data.csv -f final_lambdas_summary.csv -o plots/
  %(prog)s -i single_file_diffusion_data.csv -o plots/  # Auto-detect final lambdas
  
  # Additional options
  %(prog)s -i data.csv --max-iterations 10 --exclude-arenas star triangle
        """
    )
    
    # Primary input (support both old and new parameter names)
    parser.add_argument('-i', '--input-file', '--inputFile', required=True,
                       help='Path to input file (.feather or .csv)')
    
    # CSV-specific parameter
    parser.add_argument('-f', '--final-lambdas-csv', 
                       help='Path to final lambdas CSV file (for CSV input only)')
    
    # Output directory (support both parameter names)
    parser.add_argument('-o', '--output-dir', '--outputDir', default="plots", 
                       help='Directory for output plot files (default: plots)')
    
    # Arenas directory (support both parameter names)
    parser.add_argument('-a', '--arenas-dir', '--arenasDir', default="arenas", 
                       help='Directory containing arena icon files (default: arenas)')
    
    # Additional options
    parser.add_argument('--max-iterations', type=int,
                       help='Maximum number of iterations to plot')
    parser.add_argument('--exclude-arenas', nargs='+',
                       help='List of arena types to exclude from plots')
    parser.add_argument('--error-mode', choices=['sd', 'pi'], default='sd',
                       help='Error representation: sd (standard deviation) or pi (percentile intervals)')
    
    args = parser.parse_args()

    # Load data (automatically detects format)
    try:
        df, config = load_data(args.input_file, args.final_lambdas_csv)
    except Exception as e:
        print(f"Error loading data: {e}")
        exit(1)
    
    print(f"Loaded {len(df)} data points")
    print(f"Arenas found: {sorted(df['arena_file'].unique())}")
    print(f"Iterations: {df['current_it'].min()} to {df['current_it'].max()}")

    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)

    # Generate confusion matrix plot
    print("Generating confusion matrix plot...")
    confusion_mat_plot_df(df,
                          output_dir=args.output_dir,
                          colors_dict=colors_dict,
                          icons_dir=args.arenas_dir,
                          ordered_class_names=list(colors_dict.keys()))

    # Generate evolution plot
    print("Generating evolution plot...")
    plot_line_all_arenas_df(
        df,
        output_filename=os.path.join(args.output_dir, "evolution.pdf"),
        error_mode=args.error_mode,
        colors_dict=colors_dict,
        icons_dir=args.arenas_dir,
        max_it=args.max_iterations,
        exclude_arenas=args.exclude_arenas
    )

    # Generate logs of abs(s)
    print("Generating log(abs(s)) lineplots...")
    categorize_by_iteration(df)
    shutil.rmtree(os.path.join(args.output_dir, "log_abs_s"), ignore_errors=True)
    export_random_log_abs_s(df, 5, out_dir=os.path.join(args.output_dir, "log_abs_s"), burn_in_seconds=15, use_savgol=False, show_title=False, ylim=[1e-4, 1])
    #shutil.rmtree(os.path.join(args.output_dir, "log_abs_s_sg"), ignore_errors=True)
    #export_random_log_abs_s(df, 5, out_dir=os.path.join(args.output_dir, "log_abs_s_sg"), burn_in_seconds=15, use_savgol=True, show_title=False)

    print("Computing network statistics...")
    stats_t0 = network.aggregate_network_statistics_by_arena(df, weight=False, directed=False, filter_it=0)
    pprint.pprint(stats_t0)
    
    print(f"Plots saved to {args.output_dir}/")
#    print("Generated files:")
#    print(f"  - {args.output_dir}/confusion.pdf")
#    print(f"  - {args.output_dir}/evolution.pdf")

# MODELINE "{{{1
# vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
# vim:foldmethod=marker