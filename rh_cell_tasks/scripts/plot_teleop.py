#!/usr/bin/env python3
"""Compare teleoperation runs with and without the virtual fixture (logs from teleop_fixture).

    ros2 run rh_cell_tasks plot_teleop.py teleop_fixture_*.csv teleop_free_*.csv -o docs/teleop_fixture.png
"""
import argparse
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as patches  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402

# cell geometry (same as rh_cell_tasks/cell.hpp), metres, tool0 -> cassette front = 0.125
WALL_X0, WALL_X1, PORT_Y, PORT_HALF, CAS_HALF, TOOL_TO_FRONT = 0.59, 0.61, -0.25, 0.045, 0.035, 0.125


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    ap.add_argument("-o", "--out", default="teleop_fixture.png")
    a = ap.parse_args()

    free_peak = None
    fig, (ax_p, ax_f) = plt.subplots(1, 2, figsize=(11, 4.4), gridspec_kw={"width_ratios": [1.2, 1]})
    colors = {"fixture": "#2f8f4e", "free": "#b5533c"}
    for f in a.csv:
        df = pd.read_csv(f)
        kind = "fixture" if "fixture" in os.path.basename(f) else "free"
        label = "virtual fixture ON" if kind == "fixture" else "no fixture (same commands)"
        # plot the cassette FRONT face centre in top view
        ax_p.plot(df.x + TOOL_TO_FRONT, df.y, color=colors[kind], lw=1.6, label=label)
        if kind == "fixture":
            ax_f.plot(df.t, df.push, color=colors[kind], lw=1.6, label=f"{label}: peak {df.push.max():.0f} N")
        else:
            free_peak = df.push.max()

    # wall with port (top view)
    for y0, y1 in ((PORT_Y + PORT_HALF, PORT_Y + 0.2), (PORT_Y - 0.2, PORT_Y - PORT_HALF)):
        ax_p.add_patch(patches.Rectangle((WALL_X0, y0), WALL_X1 - WALL_X0, y1 - y0, color="0.55"))
    ax_p.axhline(PORT_Y, color="0.6", lw=0.8, ls=":")
    ax_p.axvline(0.71, color="#3b6ea5", lw=1, ls="--")
    ax_p.text(0.712, PORT_Y + 0.05, "seat", color="#3b6ea5", fontsize=8)
    ax_p.text(WALL_X1 + 0.003, PORT_Y + 0.065, "wall", fontsize=8, color="0.35")
    ax_p.set_xlim(0.42, 0.75)
    ax_p.set_ylim(PORT_Y - 0.08, PORT_Y + 0.08)
    ax_p.set_xlabel("x [m] (insertion direction)")
    ax_p.set_ylabel("y [m]")
    ax_p.set_title("Cassette front face, top view (90 mm port, 70 mm cassette)", fontsize=9)
    ax_p.legend(fontsize=8, loc="lower left")
    ax_p.grid(alpha=0.3)

    ax_f.axhline(8, color="r", lw=0.8, ls="--", label="teleop push limit 8 N")
    ax_f.set_ylim(-1, 20)
    if free_peak is not None:
        ax_f.text(0.02, 0.95, f"no fixture: the cassette jams against the wall,\nforce peaks at {free_peak:.0f} N (not drawn)",
                  transform=ax_f.transAxes, ha="left", va="top", fontsize=8, color=colors["free"])
    ax_f.set_xlabel("time [s] (sim)")
    ax_f.set_ylabel("axial push on the environment [N]")
    ax_f.legend(fontsize=8, loc="center left")
    ax_f.set_title("Contact force at the seat (fixture run)", fontsize=9)
    ax_f.grid(alpha=0.3)
    fig.suptitle("Teleoperation: same sloppy operator, with and without the guidance fixture", fontsize=11)
    fig.tight_layout()
    fig.savefig(a.out, dpi=130)
    print(f"saved {a.out}")


if __name__ == "__main__":
    main()
