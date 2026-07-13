#!/usr/bin/env python3
"""Visualise FineNavLocalizer scenario-test output.

Usage:
    python3 eval/plot.py [CSV_PATH] [--no-anim]

Default CSV: /tmp/finenav_localizer_test.csv   (mirrors cfg::kCsvPath)

Outputs
    <csv>.png            static summary figure
    <csv>.mp4 / .gif     trajectory + covariance animation (unless --no-anim)

Extended CSV schema produced by test_finenav_localizer:
  t | gt_{x,y,yaw,vx,vy,wz} | est_{x,y,yaw,vx,vy,wz}
  | est_{z,vz,wx,wy} | cov_{pz,rx,ry,vz,wx,wy}
  | cov_{xx,xy,yy,yaw,min_eig}
  | obs_{x,y,yaw,vx_body,wz_body} | has_pose_obs | has_twist_obs | scenario

  cov_{xx,xy,yy} : position (x,y) covariance block → uncertainty ellipse.
  cov_yaw        : yaw (δθ_z) variance.
  cov_min_eig    : smallest eigenvalue of the full 12×12 P (PD-health monitor).

Static panel layout (4 rows × 3 cols):
  Row 0: 2-D trajectory + covariance ellipses | x(t) | y(t)
  Row 1: yaw(t)                               | vx(t)| vy(t)
  Row 2: wz(t)                                | non-2D covariance diagonals (2 cols)
  Row 3: PD health — min eigenvalue of P over time (3 cols)

The cov_* columns are optional: on an older CSV without them, ellipses / the PD
panel are silently skipped.

Requires: numpy, pandas, matplotlib ≥ 3.5
"""

#  Copyright (c) 2026.
#  IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
#  All rights reserved.

import math
import os
import sys

import matplotlib.animation as manim
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.lines import Line2D

# ── Default CSV path (mirrors cfg::kCsvPath in eval_config.hpp) ───────────────
DEFAULT_CSV = "/tmp/finenav_localizer_test.csv"

# Confidence level for covariance ellipses (2σ ≈ 95 % for a 2-D Gaussian).
N_STD = 2.0

# Ellipse density — one ellipse roughly every N seconds along the trajectory.
ELLIPSE_INTERVAL_SEC = 0.2

# ── Scenario → colour (tab10).  Add new labels here when more scenarios land. ─
_KNOWN_SCENARIOS = [
    "CircleIdeal",                                            # Scenario 1
    "Normal", "LowFreq", "HighFreq", "ObsJump", "PoseJump",  # Scenario 2 phases
    "HighDynIdeal",                                           # Scenario 3
]
_PAL = plt.cm.tab10(np.linspace(0, 0.9, len(_KNOWN_SCENARIOS)))
_SC_COLOR = {sc: tuple(_PAL[i]) for i, sc in enumerate(_KNOWN_SCENARIOS)}

_EST_COLOR = "#1f77b4"   # estimate / ellipse blue (matches "Est" line)


def _c(sc: str) -> tuple:
    """Colour for a scenario label (grey fallback for unknown labels)."""
    return _SC_COLOR.get(sc, (0.45, 0.45, 0.45, 1.0))


def _wrap(series):
    """Wrap angle series to (−π, π]."""
    return series.apply(lambda a: (a + math.pi) % (2 * math.pi) - math.pi)


def _ellipse_axes(cxx, cxy, cyy, n_std=N_STD):
    """Return (width, height, angle_deg) of an n_std covariance ellipse for the
    2×2 block [[cxx,cxy],[cxy,cyy]], or None if the block is not positive-definite
    (a non-PD block is itself a red flag and has no real ellipse)."""
    cov = np.array([[cxx, cxy], [cxy, cyy]], dtype=float)
    evals, evecs = np.linalg.eigh(cov)          # ascending eigenvalues
    if not np.all(np.isfinite(evals)) or evals[0] <= 0.0:
        return None
    vx, vy = evecs[:, 1]                          # major-axis eigenvector
    angle = math.degrees(math.atan2(vy, vx))
    width  = 2.0 * n_std * math.sqrt(evals[1])
    height = 2.0 * n_std * math.sqrt(evals[0])
    return width, height, angle


def _add_cov_ellipse(ax, cx, cy, cxx, cxy, cyy, n_std=N_STD, **kw):
    """Draw an n_std covariance ellipse at (cx,cy).  Returns the patch, or None if
    the covariance block is not positive-definite."""
    axes = _ellipse_axes(cxx, cxy, cyy, n_std)
    if axes is None:
        return None
    w, h, angle = axes
    e = mpatches.Ellipse((cx, cy), w, h, angle=angle, **kw)
    ax.add_patch(e)
    return e


# ─────────────────────────────────────────────────────────────────────────────
def _shade_phases(ax, df: pd.DataFrame, scenarios: list) -> None:
    """Draw semi-transparent phase bands and phase-boundary tick lines."""
    for sc in scenarios:
        m = df["scenario"] == sc
        if not m.any():
            continue
        t0 = df.loc[m, "t"].min()
        t1 = df.loc[m, "t"].max()
        ax.axvspan(t0, t1, alpha=0.08, color=_c(sc), zorder=0)
        ax.axvline(t0, color=_c(sc), lw=0.7, ls=":", alpha=0.6, zorder=1)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("Time [s]", fontsize=8)


# ─────────────────────────────────────────────────────────────────────────────
def _ts(ax, t, gt_vals, est_vals, ylabel: str, title: str,
        obs_t=None, obs_vals=None, obs_label: str = "Obs",
        scenarios=None, df=None) -> None:
    """Standard time-series panel: GT line, estimate line, optional obs scatter."""
    ax.plot(t, gt_vals,  "k-",  lw=1.8, label="GT",  zorder=3)
    ax.plot(t, est_vals, "b--", lw=1.3, label="Est", zorder=4)
    if obs_t is not None and obs_vals is not None and len(obs_t) > 0:
        ax.scatter(obs_t, obs_vals,
                   color="tomato", s=16, zorder=6, alpha=0.80, label=obs_label)
    ax.set_ylabel(ylabel, fontsize=8)
    ax.set_title(title, fontsize=9)
    ax.legend(fontsize=6.5, loc="upper right")
    if scenarios is not None and df is not None:
        _shade_phases(ax, df, scenarios)


# ─────────────────────────────────────────────────────────────────────────────
def _draw_static(df, csv_path, scenarios, has_cov, has_pd):
    t = df["t"].to_numpy()

    pose_m  = df["has_pose_obs"]  & df["obs_x"].notna()
    twist_m = df["has_twist_obs"] & df["obs_wz_body"].notna()
    pose_t  = df.loc[pose_m,  "t"].to_numpy()
    twist_t = df.loc[twist_m, "t"].to_numpy()

    n_rows = 4 if has_pd else 3
    fig = plt.figure(figsize=(18, 11 + (3.0 if has_pd else 0.0)))
    fig.suptitle(
        f"FineNavLocalizer  ▸  {os.path.basename(csv_path)}",
        fontsize=13, fontweight="bold", y=0.998)

    gs = gridspec.GridSpec(
        n_rows, 3, figure=fig,
        hspace=0.6, wspace=0.38,
        left=0.055, right=0.975,
        top=0.955,  bottom=0.05)

    ax_traj = fig.add_subplot(gs[0, 0])
    ax_x    = fig.add_subplot(gs[0, 1])
    ax_y    = fig.add_subplot(gs[0, 2])
    ax_yaw  = fig.add_subplot(gs[1, 0])
    ax_vx   = fig.add_subplot(gs[1, 1])
    ax_vy   = fig.add_subplot(gs[1, 2])
    ax_wz   = fig.add_subplot(gs[2, 0])
    ax_cov  = fig.add_subplot(gs[2, 1:])
    ax_pd   = fig.add_subplot(gs[3, :]) if has_pd else None

    # ── Panel 1: RMSE over time (position + yaw) ──────────────────────────────
    err_pos = np.sqrt((df["gt_x"] - df["est_x"])**2 + (df["gt_y"] - df["est_y"])**2)
    err_yaw = np.abs(_wrap(df["est_yaw"] - df["gt_yaw"])) * 180.0 / np.pi
    ax_rmse = ax_traj  # reuse the axes
    ax_rmse.plot(t, err_pos, color="#d62728", lw=1.3, zorder=3, label="pos err [m]")
    ax_rmse.set_ylabel("Pos err [m]", fontsize=8, color="#d62728")
    ax2 = ax_rmse.twinx()
    ax2.plot(t, err_yaw, color="#9467bd", lw=1.0, alpha=0.85, zorder=2, label="yaw err [°]")
    ax2.set_ylabel("Yaw err [°]", fontsize=8, color="#9467bd")
    ax_rmse.set_title(
        f"RMSE over time  (pos={err_pos.mean():.3f} m  yaw={err_yaw.mean():.1f}°)",
        fontsize=9)
    ax_rmse.grid(True, alpha=0.3)
    ax2.legend(fontsize=7, loc="upper right")
    ax_rmse.legend(fontsize=7, loc="upper left")
    for sc in scenarios:
        _shade_phases(ax_rmse, df, scenarios)

    # ── Panels 2-7: state time series ────────────────────────────────────────
    _ts(ax_x, t, df["gt_x"].to_numpy(), df["est_x"].to_numpy(),
        "x [m]", "x  (world frame)",
        obs_t=pose_t, obs_vals=df.loc[pose_m, "obs_x"].to_numpy(),
        obs_label="Pose obs", scenarios=scenarios, df=df)
    _ts(ax_y, t, df["gt_y"].to_numpy(), df["est_y"].to_numpy(),
        "y [m]", "y  (world frame)",
        obs_t=pose_t, obs_vals=df.loc[pose_m, "obs_y"].to_numpy(),
        obs_label="Pose obs", scenarios=scenarios, df=df)
    _ts(ax_yaw, t, _wrap(df["gt_yaw"]).to_numpy(), _wrap(df["est_yaw"]).to_numpy(),
        "yaw [rad]", "yaw  (world frame)",
        obs_t=pose_t if df["has_pose_obs"].any() and df["obs_yaw"].notna().any() else None,
        obs_vals=_wrap(df.loc[pose_m, "obs_yaw"]).to_numpy() if pose_m.any() else None,
        obs_label="Pose obs", scenarios=scenarios, df=df)
    _ts(ax_vx, t, df["gt_vx"].to_numpy(), df["est_vx"].to_numpy(),
        "vx [m/s]", "vx  (world frame)", scenarios=scenarios, df=df)
    _ts(ax_vy, t, df["gt_vy"].to_numpy(), df["est_vy"].to_numpy(),
        "vy [m/s]", "vy  (world frame)", scenarios=scenarios, df=df)
    _ts(ax_wz, t, df["gt_wz"].to_numpy(), df["est_wz"].to_numpy(),
        "ωz [rad/s]", "ωz  (body frame)",
        obs_t=twist_t if twist_m.any() else None,
        obs_vals=df.loc[twist_m, "obs_wz_body"].to_numpy() if twist_m.any() else None,
        obs_label="Twist obs", scenarios=scenarios, df=df)

    # ── Panel 8: Non-2D covariance diagonals ─────────────────────────────────
    COV_SPEC = [
        ("cov_pz", "p_z   (P[2,2])",   "#1f77b4"),
        ("cov_rx", "θ_x   (P[3,3])",   "#ff7f0e"),
        ("cov_ry", "θ_y   (P[4,4])",   "#2ca02c"),
        ("cov_vz", "v_z   (P[8,8])",   "#d62728"),
        ("cov_wx", "ω_x   (P[9,9])",   "#9467bd"),
        ("cov_wy", "ω_y  (P[10,10])",  "#8c564b"),
    ]
    for col, label, color in COV_SPEC:
        vals = np.maximum(df[col].to_numpy(), 1e-15)
        ax_cov.plot(t, vals, label=label, color=color, lw=1.25, alpha=0.90)
    ax_cov.set_yscale("log")
    ax_cov.set_ylabel("Covariance  [unit²]", fontsize=8)
    ax_cov.set_title(
        "Non-2D error-state covariance diagonals — runaway growth = divergence",
        fontsize=9)
    ax_cov.legend(fontsize=7.5, loc="upper right", ncol=2)
    _shade_phases(ax_cov, df, scenarios)

    # ── Panel 9: PD health — smallest eigenvalue of P ────────────────────────
    if has_pd:
        me = df["cov_min_eig"].to_numpy()
        ax_pd.plot(t, me, color="#2ca02c", lw=1.3, zorder=3, label="min eig(P)")
        ax_pd.axhline(0.0, color="k", lw=0.9, zorder=2)
        bad = me <= 0.0
        if bad.any():
            ax_pd.fill_between(t, me, 0.0, where=bad, color="red", alpha=0.35,
                               zorder=1, label="non-PD (min eig ≤ 0)")
        # symlog shows tiny-positive values and any dip below zero at once.
        ax_pd.set_yscale("symlog", linthresh=1e-6)
        ax_pd.set_ylabel("min eig(P)", fontsize=8)
        ax_pd.set_title(
            "Covariance PD health — smallest eigenvalue of P  "
            "(reaching ≤ 0 ⇒ lost positive-definiteness)", fontsize=9)
        worst = float(np.min(me))
        ax_pd.text(0.008, 0.06, f"min over run: {worst:.2e}",
                   transform=ax_pd.transAxes, fontsize=8, va="bottom",
                   color=("red" if worst <= 0 else "#2ca02c"))
        ax_pd.legend(fontsize=7.5, loc="upper right")
        _shade_phases(ax_pd, df, scenarios)

    out_png = csv_path.replace(".csv", ".png")
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"[plot_localizer]  static  {len(df)} rows  →  {out_png}")
    return fig


# ─────────────────────────────────────────────────────────────────────────────
def _draw_traj_standalone(df, csv_path, scenarios):
    """Standalone interactive trajectory + covariance ellipse figure (big, zoomable)."""
    fig, ax = plt.subplots(figsize=(12, 10))
    fig.suptitle(f"FineNavLocalizer — Trajectory  ▸  {os.path.basename(csv_path)}",
                 fontsize=13, fontweight="bold")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")

    ax.plot(df["gt_x"],  df["gt_y"],  "k-",  lw=2.0, zorder=3, label="GT")
    ax.plot(df["est_x"], df["est_y"], "b--", lw=1.5, zorder=4, label="Est")
    ax.plot(df["gt_x"].iloc[0], df["gt_y"].iloc[0], "k^", ms=9, zorder=7, clip_on=False)

    for sc in scenarios:
        m = df["scenario"] == sc
        ax.plot(df.loc[m, "est_x"], df.loc[m, "est_y"],
                color=_c(sc), lw=2.2, alpha=0.80, zorder=5)

    n_bad = 0
    n = len(df)
    t_vals = df["t"].to_numpy()
    total_t = t_vals[-1] - t_vals[0] if n > 1 else 1.0
    k = max(1, int(n * ELLIPSE_INTERVAL_SEC / total_t))  # time-based density
    ex = df["est_x"].to_numpy(); ey = df["est_y"].to_numpy()
    cxx = df["cov_xx"].to_numpy(); cxy = df["cov_xy"].to_numpy(); cyy = df["cov_yy"].to_numpy()
    for i in range(0, n, k):
        e = _add_cov_ellipse(ax, ex[i], ey[i], cxx[i], cxy[i], cyy[i],
                             fill=False, edgecolor=_EST_COLOR, lw=0.9, alpha=0.35, zorder=4.5)
        if e is None:
            n_bad += 1
            ax.plot(ex[i], ey[i], "x", color="red", ms=7, mew=1.6, zorder=8)

    handles = [
        Line2D([0], [0], color="k", lw=2.0,            label="GT"),
        Line2D([0], [0], color="b", lw=1.5, ls="--",   label="Est"),
        mpatches.Patch(facecolor="none", edgecolor=_EST_COLOR, label=f"{N_STD}σ cov ellipse"),
    ]
    if n_bad:
        handles.append(Line2D([0], [0], color="red", marker="x", ls="none", label=f"non-PD ({n_bad})"))
    for sc in scenarios:
        handles.append(mpatches.Patch(color=_c(sc), alpha=0.75, label=sc))
    ax.legend(handles=handles, fontsize=8, loc="best")

    out = csv_path.replace(".csv", "_traj.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(f"[plot_localizer]  traj     {len(df)} rows  →  {out}")

# ─────────────────────────────────────────────────────────────────────────────
def _make_animation(df, csv_path, scenarios, has_pd):
    """Animate the trajectory, estimate, current covariance ellipse and heading."""
    t     = df["t"].to_numpy()
    gx    = df["gt_x"].to_numpy();  gy    = df["gt_y"].to_numpy()
    gyaw  = df["gt_yaw"].to_numpy()
    ex    = df["est_x"].to_numpy(); ey    = df["est_y"].to_numpy()
    eyaw  = df["est_yaw"].to_numpy()
    cxx = df["cov_xx"].to_numpy(); cxy = df["cov_xy"].to_numpy()
    cyy = df["cov_yy"].to_numpy()
    me  = df["cov_min_eig"].to_numpy() if has_pd else None

    n = len(df)
    step = max(1, n // 180)                        # ~180 frames
    frames = list(range(0, n, step))
    if frames[-1] != n - 1:
        frames.append(n - 1)

    fig, ax = plt.subplots(figsize=(9, 8))
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")

    # Axis limits from both GT and estimate, with a margin for the ellipse.
    allx = np.concatenate([gx, ex]); ally = np.concatenate([gy, ey])
    padx = 0.12 * (allx.max() - allx.min() + 1e-6)
    pady = 0.12 * (ally.max() - ally.min() + 1e-6)
    ax.set_xlim(allx.min() - padx, allx.max() + padx)
    ax.set_ylim(ally.min() - pady, ally.max() + pady)
    head_len = 0.06 * max(allx.max() - allx.min(), ally.max() - ally.min(), 1e-6)

    ax.plot(gx, gy, color="0.65", lw=1.2, zorder=1, label="GT (full path)")
    est_line, = ax.plot([], [], "b--", lw=1.4, zorder=3, label="Est (so far)")
    gt_pt,    = ax.plot([], [], "k^", ms=11, zorder=6, label="GT now")
    gt_head,  = ax.plot([], [], "-", color="black", lw=2.2, zorder=6)
    est_pt,   = ax.plot([], [], "o", color=_EST_COLOR, ms=6, zorder=7, label="Est now")
    est_head, = ax.plot([], [], "-", color=_EST_COLOR, lw=2.0, zorder=6)
    ell = mpatches.Ellipse((0, 0), 0, 0, angle=0.0, fill=False,
                           edgecolor=_EST_COLOR, lw=1.6, alpha=0.85, zorder=5)
    ax.add_patch(ell)
    hud = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", ha="left",
                  fontsize=9, family="monospace",
                  bbox=dict(boxstyle="round", fc="white", ec="0.6", alpha=0.85))
    ax.legend(fontsize=8, loc="lower right")

    def update(i):
        est_line.set_data(ex[:i + 1], ey[:i + 1])
        gt_pt.set_data([gx[i]], [gy[i]])
        if not np.isnan(gyaw[i]):
            gt_head.set_data([gx[i], gx[i] + head_len * math.cos(gyaw[i])],
                             [gy[i], gy[i] + head_len * math.sin(gyaw[i])])
        else:
            gt_head.set_data([], [])
        est_pt.set_data([ex[i]], [ey[i]])
        est_head.set_data([ex[i], ex[i] + head_len * math.cos(eyaw[i])],
                         [ey[i], ey[i] + head_len * math.sin(eyaw[i])])

        axes = _ellipse_axes(cxx[i], cxy[i], cyy[i])
        pd_ok = True
        if axes is None:
            ell.set_visible(False)
            est_pt.set_color("red")
            pd_ok = False
        else:
            w, h, angle = axes
            ell.set_center((ex[i], ey[i]))
            ell.set_width(w); ell.set_height(h); ell.set_angle(angle)
            ell.set_visible(True)
            est_pt.set_color(_EST_COLOR)

        err = math.hypot(gx[i] - ex[i], gy[i] - ey[i])
        txt = f"t        = {t[i]:6.2f} s\npos err  = {err:6.3f} m"
        if has_pd:
            ok = me[i] > 0
            pd_ok = pd_ok and ok
            txt += f"\nmin eig  = {me[i]:9.2e}  [{'PD' if ok else 'NON-PD'}]"
        hud.set_text(txt)
        hud.get_bbox_patch().set_edgecolor("red" if not pd_ok else "0.6")
        ax.set_title(f"FineNavLocalizer  ▸  {df['scenario'].iloc[i]}", fontsize=11)
        return est_line, gt_pt, gt_head, est_pt, est_head, ell, hud

    anim = manim.FuncAnimation(fig, update, frames=frames, interval=50, blit=False)

    base = csv_path.rsplit(".", 1)[0]
    try:
        if manim.writers.is_available("ffmpeg"):
            out = base + ".mp4"
            anim.save(out, writer=manim.FFMpegWriter(fps=20, bitrate=2400))
        else:
            raise RuntimeError("ffmpeg unavailable")
    except Exception:
        out = base + ".gif"
        anim.save(out, writer=manim.PillowWriter(fps=20))
    print(f"[plot_localizer]  anim    {len(frames)} frames  →  {out}")
    plt.close(fig)


# ─────────────────────────────────────────────────────────────────────────────
def plot_csv(csv_path: str, do_anim: bool = True) -> None:
    df = pd.read_csv(csv_path)

    for col in ["obs_x", "obs_y", "obs_yaw", "obs_vx_body", "obs_wz_body"]:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    df["has_pose_obs"]  = df["has_pose_obs"].astype(bool)
    df["has_twist_obs"] = df["has_twist_obs"].astype(bool)

    scenarios = list(df["scenario"].unique())
    has_cov = {"cov_xx", "cov_xy", "cov_yy"}.issubset(df.columns)
    has_pd  = "cov_min_eig" in df.columns
    if not has_cov:
        print("[plot_localizer]  note: CSV has no cov_xx/xy/yy — "
              "ellipses and animation skipped (regenerate with the updated eval).")

    _draw_static(df, csv_path, scenarios, has_cov, has_pd)

    # ── Standalone interactive trajectory figure (enlarge covariates + GT/Est) ─
    if has_cov:
        _draw_traj_standalone(df, csv_path, scenarios)

    if do_anim and has_cov:
        _make_animation(df, csv_path, scenarios, has_pd)

    plt.show()


# ── Entry point ───────────────────────────────────────────────────────────────
def main() -> None:
    args = sys.argv[1:]
    do_anim = "--no-anim" not in args
    positional = [a for a in args if not a.startswith("--")]
    path = positional[0] if positional else DEFAULT_CSV
    if not os.path.exists(path):
        print(f"[ERROR] CSV not found: {path}")
        print("  Run the test first, e.g.:")
        print("    ros2 run finenav_localizer finenav_localizer_eval")
        sys.exit(1)
    plot_csv(path, do_anim=do_anim)


if __name__ == "__main__":
    main()
