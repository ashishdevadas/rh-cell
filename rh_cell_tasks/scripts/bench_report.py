#!/usr/bin/env python3
"""Benchmark report (RH Cell 2.0): success rate and peak force vs misalignment, one curve per controller.

    ros2 run rh_cell_tasks bench_report.py ~/.ros/rh_cell_logs/bench_*.csv -o docs/bench_baseline.png

Success = the cassette seated. Error bars on the success rate are 95 % Wilson score intervals
(well behaved at 0 % and 100 %, unlike +-1.96 sqrt(p(1-p)/n)). Shaded bands: the regimes of the
tight port for an offset along y or z (2 mm clearance, 5 mm chamfer); along a diagonal the
boundaries move out by up to sqrt(2).
"""
import argparse
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402

COLORS = {"position": "#b5533c", "axial": "#3c6fb5", "cartesian": "#3c9b5f", "adaptive": "#7a4bb5"}
LABELS = {"position": "B1 position only", "axial": "B2 axial admittance",
          "cartesian": "B3 Cartesian admittance", "adaptive": "Proposed"}


def wilson(k, n, z=1.96):
    if n == 0:
        return float("nan"), float("nan"), float("nan")
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return p, max(0.0, c - h), min(1.0, c + h)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    ap.add_argument("-o", "--out", default="bench.png")
    ap.add_argument("--clearance", type=float, default=2.0)
    ap.add_argument("--chamfer", type=float, default=5.0)
    a = ap.parse_args()

    df = pd.concat([pd.read_csv(f) for f in a.csv], ignore_index=True)
    df["ok"] = df.result == "SEATED"
    rows = []
    for (ctrl, off), g in df.groupby(["controller", "offset_mm"]):
        p, lo, hi = wilson(int(g.ok.sum()), len(g))
        rows.append(dict(controller=ctrl, offset_mm=off, n=len(g), seated=int(g.ok.sum()), rate=p, lo=lo, hi=hi,
                         peak_med=g.peak_N.median(), peak_max=g.peak_N.max(),
                         time_med=g[g.ok].time_s.median() if g.ok.any() else float("nan"),
                         failures=", ".join(f"{r}x{c}" for r, c in g[~g.ok].result.value_counts().items())))
    t = pd.DataFrame(rows).sort_values(["controller", "offset_mm"])
    pd.set_option("display.width", 160)
    print(t.to_string(index=False, float_format=lambda v: f"{v:.2f}"))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    c, ch = a.clearance, a.clearance + a.chamfer
    xmax = df.offset_mm.max() + 0.5
    for ax in (ax1, ax2):
        ax.axvspan(-0.5, c, color="#e8f3e8", zorder=0)
        ax.axvspan(c, ch, color="#fdf3dc", zorder=0)
        ax.axvspan(ch, xmax, color="#f6e3e3", zorder=0)
        ax.grid(alpha=0.3)
    for x, txt in ((c / 2 - 0.25, "free"), ((c + ch) / 2, "chamfer"), ((ch + xmax) / 2, "wall face")):
        ax1.text(x, 1.07, txt, ha="center", fontsize=9, color="#555")
    ctrls = list(t.controller.unique())
    for i, (ctrl, g) in enumerate(t.groupby("controller")):
        col = COLORS.get(ctrl, "k")
        dx = 0.08 * (i - (len(ctrls) - 1) / 2)  # small sideways shift so overlapping curves stay visible
        ax1.errorbar(g.offset_mm + dx, g.rate, yerr=[g.rate - g.lo, g.hi - g.rate], color=col, marker="o", capsize=3,
                     label=f"{LABELS.get(ctrl, ctrl)} (n = {int(g.n.min())}-{int(g.n.max())} per offset)")
        raw = df[df.controller == ctrl]
        ax2.scatter(raw.offset_mm + dx, raw.peak_N, color=col, s=14, alpha=0.5)
        ax2.plot(g.offset_mm + dx, g.peak_med, color=col, marker="o", label=f"{LABELS.get(ctrl, ctrl)} (median)")
    ax1.set(ylabel="success rate (seated)", ylim=(-0.05, 1.15), xlim=(-0.5, xmax))
    ax1.legend(loc="center right", fontsize=8)
    ax2.set(xlabel="lateral misalignment of the believed port [mm] (random direction)", ylabel="peak |F| [N]")
    ax2.set_yscale("symlog", linthresh=10)
    ax2.set_ylim(bottom=0)
    ax2.legend(loc="upper left", fontsize=8)
    fig.suptitle("Insertion into a 74 mm port (2 mm clearance, 5 mm chamfer) under misalignment", fontsize=11)
    fig.tight_layout()
    fig.savefig(a.out, dpi=130)
    print("saved", a.out)


if __name__ == "__main__":
    main()
