#!/usr/bin/env python3
"""Visualise Route Planner standalone eval output — with local sensor views.

Usage:
    python3 eval/plot.py [OUTPUT_DIR]

Default: /tmp/dstarlite_eval

Scenarios detected from stats.csv:
  1 (empty_diagonal):   1×2 — A* vs D* Lite side-by-side
  2 (wall_replan):      2×2 — prior map + trajectory + discovery + timing
  3 (terrain_sweep):    1×1 — terrain band heatmap + all weight paths
  4 (random_discovery): 1×2 — trajectory + sensor scans + timing

Requires: numpy, pandas, matplotlib >= 3.5
"""

import math, os, sys
import matplotlib
matplotlib.use("Agg")  # headless: save PNG instead of pop-up
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle

DEFAULT_DIR = "/tmp/dstarlite_eval"

# ── Grid config — loaded from _config.json ────────────────────────────────────
GRID_SIZE       = 100
KBandX0, KBandX1 = 35, 45
KBandY0, KBandY1 = 10, 89
KTerrainBandCost = 10.0
KSensorRange     = 10
g_obstacles      = []   # loaded from _config.json


def _load_config(out_dir):
    import json
    cfg_path = os.path.join(out_dir, "_config.json")
    if not os.path.exists(cfg_path):
        return
    try:
        with open(cfg_path) as f:
            d = json.load(f)
        global GRID_SIZE, KBandX0, KBandX1, KBandY0, KBandY1, KTerrainBandCost, KSensorRange, g_obstacles
        GRID_SIZE       = d.get("grid_size", GRID_SIZE)
        KBandX0         = d.get("band_x0", KBandX0)
        KBandX1         = d.get("band_x1", KBandX1)
        KBandY0         = d.get("band_y0", KBandY0)
        KBandY1         = d.get("band_y1", KBandY1)
        KTerrainBandCost = d.get("band_cost", KTerrainBandCost)
        KSensorRange     = d.get("sensor_range", KSensorRange)
        g_obstacles      = d.get("obstacles", [])
    except Exception:
        pass


# ── Colour palette ───────────────────────────────────────────────────────────
ALGO_COLORS = {"AStar": "#2196F3", "DStarLite": "#FF5722"}
PATH_COLORS = {
    "initial": "#1565C0", "trajectory": "#7B1FA2",
    "cumulative_replan": "#2E7D32", "replan": "#C62828",
}
SWEEP_CMAP = plt.cm.coolwarm


# ── Grid builders ────────────────────────────────────────────────────────────

def build_empty_grid():
    return np.zeros((GRID_SIZE, GRID_SIZE))


def build_terrain_band_grid():
    grid = np.zeros((GRID_SIZE, GRID_SIZE))
    grid[KBandY0:KBandY1 + 1, KBandX0:KBandX1 + 1] = KTerrainBandCost
    return grid


# ── Plot helpers ─────────────────────────────────────────────────────────────

def _add_stats_text(ax, stats_df, algo=None, label=None):
    lines = []
    subset = stats_df
    if algo is not None:
        subset = subset[subset["algo"] == algo]
    if label is not None:
        subset = subset[subset["path_type"] == label]
    for _, row in subset.iterrows():
        disc_str = ""
        if "discovery_step" in row and pd.notna(row["discovery_step"]) and int(row["discovery_step"]) >= 0:
            disc_str = f"  disc={int(row['discovery_step'])}"
        lines.append(
            f"{row['algo']} | {row['path_type']}: "
            f"len={int(row['length'])}  "
            f"{row['compute_ms']:.3f} ms{disc_str}"
        )
    if not lines:
        return
    ax.text(0.02, 0.98, "\n".join(lines), transform=ax.transAxes,
            fontsize=9, fontfamily="monospace", verticalalignment="top",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="wheat", alpha=0.85), zorder=10)


def _draw_grid(ax, grid, cmap=None, vmin=None, vmax=None):
    if cmap is None:
        cmap = LinearSegmentedColormap.from_list("obs", ["white", "black"])
        ax.imshow(grid, origin="lower", cmap=cmap, vmin=0, vmax=1,
                  extent=[0, GRID_SIZE, 0, GRID_SIZE])
    else:
        im = ax.imshow(grid, origin="lower", cmap=cmap, vmin=vmin, vmax=vmax,
                       extent=[0, GRID_SIZE, 0, GRID_SIZE])
        return im


def _draw_path(ax, df, algo, label, color, lw=2.0, ls="-", marker=None, ms=4, alpha=0.85, zorder=5):
    sub = df[(df["algo"] == algo) & (df["label"] == label)]
    if sub.empty:
        return
    sub = sub.sort_values("step")
    ax.plot(sub["x"].to_numpy(), sub["y"].to_numpy(),
            color=color, lw=lw, ls=ls, marker=marker, ms=ms, alpha=alpha, zorder=zorder)


def _mark_endpoints(ax, start, goal):
    ax.plot(start[0], start[1], "g^", ms=14, zorder=8)
    ax.plot(goal[0],  goal[1],  "r*", ms=16, zorder=8)


def _draw_obstacles(ax):
    """Draw secret obstacle cells as red blocks on the grid."""
    if not g_obstacles:
        return
    # deduplicate
    seen = set()
    xs, ys = [], []
    for ox, oy in g_obstacles:
        key = (ox, oy)
        if key not in seen:
            seen.add(key)
            xs.append(ox)
            ys.append(oy)
    ax.scatter(xs, ys, c="#FF0000", s=14, marker="s", alpha=0.8, zorder=4,
               edgecolors="#990000", linewidths=0.3, label="Secret obstacles")


def _draw_sensor_range(ax, center, range_cells, color="#00BCD4", alpha=0.15):
    rect = Rectangle((center[0] - range_cells, center[1] - range_cells),
                     2 * range_cells, 2 * range_cells,
                     linewidth=1.2, edgecolor=color, facecolor=color,
                     alpha=alpha, linestyle="--", zorder=3)
    ax.add_patch(rect)


def _stylise(ax, title):
    ax.set_xlim(-0.5, GRID_SIZE - 0.5)
    ax.set_ylim(-0.5, GRID_SIZE - 0.5)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=12)
    ax.grid(False)


# ── Per-scenario plots ───────────────────────────────────────────────────────

def plot_scenario1(paths_df, stats_df, out_dir):
    grid = build_empty_grid()
    fig, (ax_a, ax_d) = plt.subplots(1, 2, figsize=(14, 7))
    fig.suptitle("Scenario 1: Empty Grid Diagonal — A* vs D* Lite", fontsize=14, fontweight="bold")
    for ax, algo in [(ax_a, "AStar"), (ax_d, "DStarLite")]:
        _draw_grid(ax, grid)
        _draw_path(ax, paths_df, algo, "initial", ALGO_COLORS[algo], lw=1.5)
        _mark_endpoints(ax, (0, 0), (GRID_SIZE - 1, GRID_SIZE - 1))
        _stylise(ax, algo)
        _add_stats_text(ax, stats_df, algo=algo)
    plt.savefig(os.path.join(out_dir, "scenario1_empty_diagonal.png"), dpi=150, bbox_inches="tight")
    plt.close()


def plot_scenario2(paths_df, stats_df, out_dir):
    """Local sensor replan — map + timing comparison."""
    fig = plt.figure(figsize=(14, 12))
    gs = fig.add_gridspec(2, 1, height_ratios=[3, 1], hspace=0.25)
    fig.suptitle("Scenario 2: Robot discovers obstacles mid-journey (local sensor)",
                 fontsize=14, fontweight="bold")

    # ── 上半：地图 + 路径 ──
    ax_map = fig.add_subplot(gs[0])
    empty = build_empty_grid()
    _draw_grid(ax_map, empty)
    _draw_obstacles(ax_map)
    _draw_path(ax_map, paths_df, "DStarLite", "initial", PATH_COLORS["initial"],
               lw=2.5, ls="--", zorder=5)
    _draw_path(ax_map, paths_df, "DStarLite", "trajectory", PATH_COLORS["trajectory"],
               lw=3.0, ls="-", zorder=7)
    _draw_path(ax_map, paths_df, "AStar", "initial_w_obstacles", ALGO_COLORS["AStar"],
               lw=1.5, ls=":", alpha=0.6, zorder=4)
    _mark_endpoints(ax_map, (0, 0), (GRID_SIZE - 1, GRID_SIZE - 1))

    legend_items = [
        Line2D([0], [0], color=PATH_COLORS["initial"], lw=2.5, ls="--",
               label="D* Lite: initial plan (empty prior map)"),
        Line2D([0], [0], color=PATH_COLORS["trajectory"], lw=3.0, ls="-",
               label="D* Lite: actual trajectory (local sensor replan)"),
        Line2D([0], [0], color=ALGO_COLORS["AStar"], lw=1.5, ls=":",
               label="A*: full-map reference (God's eye view)"),
        Line2D([0], [0], marker="s", color="w", markerfacecolor="#FF0000",
               markersize=8, label="Obstacles (appear mid-journey)"),
    ]
    ax_map.legend(handles=legend_items, fontsize=11, loc="lower right", framealpha=0.9)
    _stylise(ax_map, "")

    # ── 下半：时间对比柱状图 ──
    ax_bar = fig.add_subplot(gs[1])

    # 从 stats 提取时间数据
    dsl_init = stats_df[(stats_df["algo"] == "DStarLite") & (stats_df["path_type"] == "initial")]
    dsl_cum  = stats_df[(stats_df["algo"] == "DStarLite") & (stats_df["path_type"] == "cumulative_replan")]
    astar_empty = stats_df[(stats_df["algo"] == "AStar") & (stats_df["path_type"] == "empty_map_baseline")]
    astar_full  = stats_df[(stats_df["algo"] == "AStar") & (stats_df["path_type"] == "initial_w_obstacles")]

    categories = []
    times = []
    colors = []

    # 1. 基准：两者搜同一张空地图
    if not dsl_init.empty:
        categories.append("D* Lite\n(empty map)")
        times.append(dsl_init["compute_ms"].iloc[0])
        colors.append(ALGO_COLORS["DStarLite"])

    if not astar_empty.empty:
        categories.append("A*\n(empty map)")
        times.append(astar_empty["compute_ms"].iloc[0])
        colors.append(ALGO_COLORS["AStar"])

    # 2. D* Lite: 全程增量 replan
    if not dsl_cum.empty:
        times.append(dsl_cum["compute_ms"].iloc[0])
        colors.append("#7B1FA2")
        n_disc = int(dsl_cum["discovery_step"].iloc[0])
        traj = paths_df[(paths_df["algo"] == "DStarLite") & (paths_df["label"] == "trajectory")]
        total_scans = len(traj) // 2 if not traj.empty else 50
        free_scans = total_scans - n_disc
        categories.append(f"D* Lite\n{n_disc} discoveries +\n{free_scans} free scans")

    # 3. A*: 每次发现都需全量重搜 (有障碍物的地图)
    if not astar_full.empty:
        n_disc = int(dsl_cum["discovery_step"].iloc[0]) if not dsl_cum.empty else 3
        per_search = astar_full["compute_ms"].iloc[0]
        categories.append(f"A*\n({n_disc} full searches)")
        times.append(per_search * n_disc)
        colors.append("#64B5F6")

    bars = ax_bar.bar(range(len(categories)), times, color=colors, edgecolor="white", width=0.6)
    ax_bar.set_xticks(range(len(categories)))
    ax_bar.set_xticklabels(categories, fontsize=10)
    ax_bar.set_ylabel("Compute time (ms)", fontsize=11)
    ax_bar.set_title("Timing Comparison: D* Lite incremental vs A* full re-search", fontsize=12)

    # 数值标注
    for bar, val in zip(bars, times):
        ax_bar.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(times)*0.03,
                   f"{val:.3f} ms", ha="center", fontsize=10, fontweight="bold")

    plt.savefig(os.path.join(out_dir, "scenario2_wall_replan.png"), dpi=150, bbox_inches="tight")
    plt.close()


def plot_scenario3(paths_df, stats_df, out_dir):
    grid = build_terrain_band_grid()
    fig, (ax_grid, ax_time) = plt.subplots(1, 2, figsize=(16, 7))
    fig.suptitle("Scenario 3: Terrain Band Weight Sweep", fontsize=14, fontweight="bold")

    im = _draw_grid(ax_grid, grid, cmap=plt.cm.YlOrRd, vmin=0, vmax=KTerrainBandCost)
    cbar = plt.colorbar(im, ax=ax_grid, shrink=0.75, label="Terrain cost (raw)")
    cbar.ax.yaxis.label.set_size(11)

    labels = stats_df["path_type"].unique()
    n_labels = len(labels)
    for i, label in enumerate(labels):
        color = SWEEP_CMAP(i / max(n_labels - 1, 1))
        sub = stats_df[stats_df["path_type"] == label]
        algo = sub["algo"].iloc[0] if len(sub) > 0 else "?"
        _draw_path(ax_grid, paths_df, algo, label, color, lw=1.5, ls="-", ms=2)

    _mark_endpoints(ax_grid, (0, 50), (99, 50))
    _stylise(ax_grid, "Paths by terrain_weight")
    legend_handles = [
        Line2D([0], [0], color=SWEEP_CMAP(i / max(n_labels - 1, 1)), lw=2, label=label)
        for i, label in enumerate(labels)
    ]
    ax_grid.legend(handles=legend_handles, fontsize=10, loc="lower left",
                   title="terrain_weight", title_fontsize=11)

    stats_dsl = stats_df[stats_df["algo"] == "DStarLite"]
    if not stats_dsl.empty:
        tw_vals = [float(l.replace("tw=", "")) for l in stats_dsl["path_type"]]
        times = stats_dsl["compute_ms"].values
        lengths = stats_dsl["length"].values
        ax_time.plot(tw_vals, times, "o-", color="#FF5722", lw=1.5, ms=6, label="D* Lite time")
        ax_time.set_xlabel("terrain_weight", fontsize=11)
        ax_time.set_ylabel("compute time [ms]", fontsize=11, color="#FF5722")
        ax_time.set_xscale("log")
        ax_time.grid(True, alpha=0.3)
        ax_len = ax_time.twinx()
        ax_len.plot(tw_vals, lengths, "s--", color="#2196F3", lw=1.5, ms=6, label="path length")
        ax_len.set_ylabel("path length [cells]", fontsize=11, color="#2196F3")
        lines1, labels1 = ax_time.get_legend_handles_labels()
        lines2, labels2 = ax_len.get_legend_handles_labels()
        ax_time.legend(lines1 + lines2, labels1 + labels2, fontsize=10, loc="upper left")
    ax_time.set_title("Compute time & path length vs terrain_weight", fontsize=12)
    plt.savefig(os.path.join(out_dir, "scenario3_terrain_sweep.png"), dpi=150, bbox_inches="tight")
    plt.close()


def plot_scenario4(paths_df, stats_df, out_dir):
    """Random obstacles — single panel: trajectory + obstacles."""
    fig, ax = plt.subplots(1, 1, figsize=(12, 12))
    fig.suptitle("Scenario 4: Random Unknown Obstacles — Continuous Discovery",
                 fontsize=14, fontweight="bold")

    empty = build_empty_grid()
    _draw_grid(ax, empty)
    _draw_obstacles(ax)
    _draw_path(ax, paths_df, "DStarLite", "initial", PATH_COLORS["initial"],
               lw=2.0, ls="--", zorder=4)
    _draw_path(ax, paths_df, "DStarLite", "trajectory", PATH_COLORS["trajectory"],
               lw=2.5, ls="-", zorder=6)
    _mark_endpoints(ax, (0, 0), (GRID_SIZE - 1, GRID_SIZE - 1))

    legend_items = [
        Line2D([0], [0], color=PATH_COLORS["initial"], lw=2, ls="--",
               label="D* Lite: initial plan (empty prior map)"),
        Line2D([0], [0], color=PATH_COLORS["trajectory"], lw=2.5, ls="-",
               label="D* Lite: actual trajectory (local sensor)"),
        Line2D([0], [0], marker="s", color="w", markerfacecolor="#FF0000",
               markersize=8, label="Obstacles (appear mid-journey)"),
    ]
    ax.legend(handles=legend_items, fontsize=11, loc="lower right")
    _stylise(ax, "")
    _add_stats_text(ax, stats_df)

    plt.savefig(os.path.join(out_dir, "scenario4_random_discovery.png"), dpi=150, bbox_inches="tight")
    plt.close()


# ── Entry point ──────────────────────────────────────────────────────────────

def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_DIR
    paths_path = os.path.join(out_dir, "paths.csv")
    stats_path = os.path.join(out_dir, "stats.csv")

    if not os.path.exists(paths_path) or not os.path.exists(stats_path):
        print(f"[ERROR] CSV files not found in: {out_dir}")
        sys.exit(1)

    _load_config(out_dir)
    paths_df = pd.read_csv(paths_path)
    stats_df = pd.read_csv(stats_path)

    scenario = stats_df["scenario"].iloc[0] if len(stats_df) > 0 else "unknown"
    print(f"[plot]  Scenario: {scenario}  |  {len(paths_df)} waypoints  |  {len(stats_df)} stats")

    if scenario == "empty_diagonal":
        plot_scenario1(paths_df, stats_df, out_dir)
    elif scenario == "wall_replan":
        plot_scenario2(paths_df, stats_df, out_dir)
    elif scenario == "terrain_sweep":
        plot_scenario3(paths_df, stats_df, out_dir)
    elif scenario == "random_discovery":
        plot_scenario4(paths_df, stats_df, out_dir)
    else:
        print(f"[ERROR] Unknown scenario: {scenario}")
        sys.exit(1)


if __name__ == "__main__":
    main()
