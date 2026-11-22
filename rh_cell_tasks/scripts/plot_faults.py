#!/usr/bin/env python3
"""One panel per run (nominal + injected faults): axial contact force and depth, with the outcome.

    ros2 run rh_cell_tasks plot_faults.py run_none_*.csv run_misaligned_*.csv run_stiff_*.csv \
        run_sensor_dropout_*.csv -o docs/fault_injection.png
"""
import argparse
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402

TITLES = {
    "none": "Nominal: admittance insertion, seats at 4.5 N",
    "misaligned": "Port 15 mm off: cassette hits the rigid wall",
    "stiff": "Admittance OFF (pure position control)",
    "sensor_dropout": "F/T signal lost at t = 4 s",
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    ap.add_argument("-o", "--out", default="fault_injection.png")
    ap.add_argument("--limit", type=float, default=20.0)
    a = ap.parse_args()

    n = len(a.csv)
    fig, axes = plt.subplots(n, 1, figsize=(9, 2.3 * n + 0.6), squeeze=False)
    for ax, f in zip(axes[:, 0], a.csv):
        df = pd.read_csv(f)
        kind = os.path.basename(f).replace("run_", "").rsplit("_", 2)[0]
        ins = df[df.phase != "RETRACT"]
        force = -df.f_axial
        ax.plot(df.t, force, color="#b5533c", lw=1.2, label="axial contact force [N]")
        ax.axhline(a.limit, color="r", lw=0.8, ls="--", label=f"guard {a.limit:.0f} N")
        ax.set_ylabel("force [N]")
        peak = df.f_mag.max()
        if peak > 4 * a.limit:  # rigid impact: keep the guard line readable, print the peak
            ax.set_ylim(-5, 3 * a.limit)
            t_pk = df.t[df.f_mag.idxmax()]
            ax.annotate(f"peak {peak:.0f} N (off scale)", (t_pk, 3 * a.limit * 0.95), fontsize=8,
                        color="#b5533c", ha="left", va="top")
        trip = df[df.guard == 1]
        if len(trip):
            ax.axvline(trip.t.iloc[0], color="r", lw=0.8, ls=":")
        if not df.ft_alive.all():
            t_lost = df.t[df.ft_alive == 0].iloc[0]
            ax.axvline(t_lost, color="k", lw=0.8, ls=":")
        a2 = ax.twinx()
        a2.plot(df.t, df.depth * 1000, color="#3b6ea5", lw=1.2, label="depth [mm]")
        a2.set_ylabel("depth [mm]", color="#3b6ea5")
        a2.set_ylim(-5, 175)
        end = ins.phase.iloc[-1] if len(ins) else "?"
        ax.set_title(f"{TITLES.get(kind, kind)}  ->  last phase before retract: {end}", fontsize=9, loc="left")
        ax.grid(alpha=0.3)
    axes[-1, 0].set_xlabel("time [s] (sim)")
    h1, l1 = axes[0, 0].get_legend_handles_labels()
    fig.legend(h1 + [plt.Line2D([], [], color="#3b6ea5")], l1 + ["depth [mm]"], loc="lower center", ncol=3, fontsize=8)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(a.out, dpi=130)
    print(f"saved {a.out}")


if __name__ == "__main__":
    main()
