#!/usr/bin/env python3
"""Plot one or more force_insertion runs (CSV logs from ~/.ros/rh_cell_logs).

    ros2 run rh_cell_tasks plot_run.py                      # latest run
    ros2 run rh_cell_tasks plot_run.py run_a.csv run_b.csv -o docs/compare.png
"""
import argparse
import glob
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="*", help="log files (default: latest in ~/.ros/rh_cell_logs)")
    ap.add_argument("-o", "--out", default=None, help="output PNG (default: next to the first CSV)")
    ap.add_argument("--limit", type=float, default=20.0, help="force guard limit to draw [N]")
    args = ap.parse_args()

    files = args.csv
    if not files:
        logs = sorted(glob.glob(os.path.expanduser("~/.ros/rh_cell_logs/run_*.csv")), key=os.path.getmtime)
        if not logs:
            sys.exit("No logs in ~/.ros/rh_cell_logs. Run force_insertion first.")
        files = [logs[-1]]

    fig, (ax_d, ax_f) = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True)
    for f in files:
        df = pd.read_csv(f)
        name = os.path.basename(f).replace("run_", "").rsplit("_", 2)[0]
        ax_d.plot(df.t, df.depth * 1000, label=f"{name}: measured depth")
        ax_d.plot(df.t, df.s_des * 1000, "--", lw=0.8, label=f"{name}: reference")
        ax_f.plot(df.t, -df.f_axial, label=f"{name}: axial contact force")
        trip = df[df.guard == 1]
        if len(trip):
            ax_f.axvline(trip.t.iloc[0], color="r", lw=0.8, ls=":")

    ax_d.axhline(150, color="g", lw=0.8, ls=":", label="seat face (150 mm)")
    ax_d.set_ylabel("depth along port axis [mm]")
    ax_d.legend(fontsize=8, loc="lower right")
    ax_d.grid(alpha=0.3)
    ax_f.axhline(args.limit, color="r", lw=0.8, ls="--", label=f"force guard limit ({args.limit:.0f} N)")
    ax_f.axhline(5, color="g", lw=0.8, ls=":", label="seating target ~ K x 10 mm = 5 N")
    ax_f.set_ylabel("contact force [N]")
    ax_f.set_xlabel("time [s] (sim)")
    ax_f.legend(fontsize=8, loc="upper left")
    ax_f.grid(alpha=0.3)
    fig.suptitle("rh-cell force-controlled insertion")
    fig.tight_layout()

    out = args.out or os.path.splitext(files[0])[0] + ".png"
    fig.savefig(out, dpi=130)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
