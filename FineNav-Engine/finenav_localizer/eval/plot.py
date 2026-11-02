#!/usr/bin/env python3
"""Visualise FineNavLocalizer eval CSV output.

Usage:
    python3 eval/plot.py <csv_path> [--no-anim]

Outputs: <csv>.png, <csv>.mp4
"""

import math, os, sys
import matplotlib.animation as anim
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def _wrap(s):
    return s.apply(lambda a: (a + math.pi) % (2 * math.pi) - math.pi)


def plot_csv(csv_path, do_anim=True):
    df = pd.read_csv(csv_path)
    t = df["t"].to_numpy() - df["t"].iloc[0]
    has_gt = not df["gt_x"].isna().all()
    name = os.path.basename(csv_path)

    # ── Load pose observations if available ──────────────────────────────
    pose_path = csv_path.replace(".csv", "_pose.csv")
    pose_df = pd.read_csv(pose_path) if os.path.exists(pose_path) else None
    if pose_df is not None:
        pose_df["t"] -= df["t"].iloc[0]

    # ── Static figure ───────────────────────────────────────────────────
    fig = plt.figure(figsize=(20, 12))
    fig.suptitle(f"FineNavLocalizer  ▸  {name}", fontsize=13, fontweight="bold")
    gs = fig.add_gridspec(4, 2, hspace=0.5, wspace=0.3,
                          left=0.05, right=0.98, top=0.93, bottom=0.05,
                          height_ratios=[1, 1, 1.2, 1])

    for i, (col, label) in enumerate([("est_x", "x [m]"), ("est_y", "y [m]")]):
        ax = fig.add_subplot(gs[0, i])
        if has_gt: ax.plot(t, df["gt_" + col[4:]], "k-", lw=2.0, label="GT")
        ax.plot(t, df["odom_" + col[4:]], "C1-", lw=3.0, alpha=0.25, label="Odom")
        ax.plot(t, df[col], "b-", lw=1.2, label="Est")
        if pose_df is not None:
            ax.scatter(pose_df["t"], pose_df[col[4:]], c="C2", s=8, alpha=0.6, label="Pose obs")
        ax.set_ylabel(label); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    for i, (col, label) in enumerate([("est_yaw", "yaw [rad]"), ("est_wz", "ωz [rad/s]")]):
        ax = fig.add_subplot(gs[1, i])
        if col == "est_yaw" and has_gt: ax.plot(t, _wrap(df["gt_yaw"]), "k-", lw=2.0, label="GT")
        if col == "est_yaw":
            ax.plot(t, _wrap(df["odom_yaw"]), "C1-", lw=3.0, alpha=0.25, label="Odom")
        else:
            ax.plot(t, df["odom_wz"], "C1-", lw=3.0, alpha=0.25, label="Odom")
        vals = df[col] if col != "est_yaw" else _wrap(df[col])
        ax.plot(t, vals, "b-", lw=1.2, label="Est")
        if col == "est_yaw" and pose_df is not None:
            ax.scatter(pose_df["t"], pose_df["yaw"].apply(lambda a: (a + math.pi) % (2*math.pi) - math.pi),
                       c="C2", s=8, alpha=0.6, label="Pose obs")
        ax.set_ylabel(label); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    for i, (col, label) in enumerate([("est_vx", "vx [m/s]"), ("est_vy", "vy [m/s]")]):
        ax = fig.add_subplot(gs[2, i])
        ax.plot(t, df["odom_" + col[4:]], "C1-", lw=3.0, alpha=0.25, label="Odom")
        ax.plot(t, df[col], "b-", lw=1.2, label="Est")
        ax.set_ylabel(label); ax.grid(True, alpha=0.3)

    ax_traj = fig.add_subplot(gs[3, 0])
    ax_traj.set_aspect("equal")
    if has_gt: ax_traj.plot(df["gt_x"], df["gt_y"], "k-", lw=2.0, label="GT")
    ax_traj.plot(df["est_x"], df["est_y"], "b-", lw=1.2, label="Est")
    if pose_df is not None:
        ax_traj.scatter(pose_df["x"], pose_df["y"], c="C2", s=10, alpha=0.6, label="Pose obs")
    ax_traj.set_xlabel("X [m]"); ax_traj.set_ylabel("Y [m]")
    ax_traj.legend(fontsize=7); ax_traj.grid(True, alpha=0.3); ax_traj.set_title("Trajectory", fontsize=10)

    ax_rmse = fig.add_subplot(gs[3, 1])
    if has_gt:
        err_pos = np.sqrt((df["gt_x"] - df["est_x"])**2 + (df["gt_y"] - df["est_y"])**2)
        ax_rmse.plot(t, err_pos, "r-", lw=1.2, label=f"pos err (mean={err_pos.mean():.3f} m)")
        err_yaw = np.abs(_wrap(df["est_yaw"] - df["gt_yaw"])) * 180 / np.pi
        ax2 = ax_rmse.twinx()
        ax2.plot(t, err_yaw, "purple", lw=1.0, alpha=0.8, label=f"yaw err (mean={err_yaw.mean():.1f}°)")
        ax2.legend(fontsize=7, loc="upper right")
    ax_rmse.set_ylabel("Pos err [m]"); ax_rmse.legend(fontsize=7, loc="upper left")
    ax_rmse.grid(True, alpha=0.3); ax_rmse.set_title("RMSE over time", fontsize=10)
    ax_rmse.set_xlabel("Time [s]")

    png = csv_path.replace(".csv", ".png")
    fig.savefig(png, dpi=150, bbox_inches="tight")
    print(f"[plot] {len(df)} rows → {png}")

    # ── Animation ───────────────────────────────────────────────────────
    if do_anim:
        _anim(csv_path, df, has_gt)

    plt.show()


def _anim(csv_path, df, has_gt):
    t = df["t"].to_numpy() - df["t"].iloc[0]
    gx = df["gt_x"].to_numpy() if has_gt else None
    gy = df["gt_y"].to_numpy() if has_gt else None
    ex, ey = df["est_x"].to_numpy(), df["est_y"].to_numpy()

    fig, ax = plt.subplots(figsize=(9, 8))
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
    ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")
    if has_gt: ax.plot(gx, gy, "0.65", lw=2.5, zorder=1, label="GT (full)")
    est_line, = ax.plot([], [], "b-", lw=1.5, zorder=3)
    pt, = ax.plot([], [], "bo", ms=6, zorder=5)
    title = ax.set_title("")

    n = len(df)
    step = max(1, n // 300)
    frames = list(range(0, n, step))
    if frames[-1] != n - 1:
        frames.append(n - 1)

    margin = (ex.max() - ex.min() + ey.max() - ey.min()) * 0.15
    xlim = (ex.min() - margin, ex.max() + margin)
    ylim = (ey.min() - margin, ey.max() + margin)
    ax.set_xlim(xlim); ax.set_ylim(ylim)

    def update(i):
        est_line.set_data(ex[:i+1], ey[:i+1])
        pt.set_data([ex[i]], [ey[i]])
        title.set_text(f"t = {t[i]:.1f}s")
        return est_line, pt, title

    ani = anim.FuncAnimation(fig, update, frames=frames, interval=30, blit=True)
    mp4 = csv_path.replace(".csv", ".mp4")
    try:
        ani.save(mp4, writer=anim.FFMpegWriter(fps=30, bitrate=3000))
    except Exception:
        mp4 = csv_path.replace(".csv", ".gif")
        ani.save(mp4, writer=anim.PillowWriter(fps=20))
    print(f"[plot] anim → {mp4}")
    plt.close(fig)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    do_anim = "--no-anim" not in sys.argv
    path = args[0] if args else "/tmp/localizer_test_bag1.csv"
    if not os.path.exists(path):
        print(f"[ERROR] CSV not found: {path}")
        sys.exit(1)
    plot_csv(path, do_anim)


if __name__ == "__main__":
    main()
