"""Scripted operator for the Phase 2 demo: e-stop in the middle of a force-controlled insertion.
Talks only Channel Access (the same PVs as the Phoebus screen), logs them, plots the result.

    ros2 launch rh_cell_bringup sim.launch.py ic:=true
    ros2 run rh_cell_tasks force_insertion --ros-args -p require_permit:=true
    ros2 run rh_cell_ic estop_demo                       # within 60 s of starting force_insertion

Sequence: RESET (permit ON) -> wait until 60 mm deep -> E-STOP -> 3 s frozen -> release + RESET
-> the task node backs the cassette out and returns to ready.
"""
import argparse
import csv
import os
import time

from caproto.threading.client import Context

PVS = ("PERMIT", "FIRST_OUT", "FIRST_OUT_STR", "STATE", "DEPTH", "FZ")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--prefix", default="RH:")
    ap.add_argument("--estop-depth", type=float, default=60.0, help="mm")
    ap.add_argument("--hold", type=float, default=3.0, help="s frozen before release")
    ap.add_argument("--out", default=os.path.expanduser("~/.ros/rh_cell_logs/estop_demo.csv"))
    ap.add_argument("--replot", metavar="CSV", help="only redraw the plot from an earlier run")
    a = ap.parse_args()
    if a.replot:
        replot(a.replot)
        return

    ctx = Context()
    pv = dict(zip(PVS, ctx.get_pvs(*[a.prefix + n for n in PVS], timeout=5.0)))
    op = dict(zip(("ESTOP_OK", "DOOR_CLOSED", "RESET"),
                  ctx.get_pvs(*[a.prefix + n for n in ("ESTOP_OK", "DOOR_CLOSED", "RESET")], timeout=5.0)))
    for p in list(pv.values()) + list(op.values()):
        p.wait_for_connection(timeout=5.0)

    def val(name):
        d = pv[name].read(timeout=2.0).data
        x = d[0]
        return x.decode() if isinstance(x, bytes) else x

    rows, t0, events = [], time.time(), []

    def sample():
        rows.append([round(time.time() - t0, 3)] + [val(n) for n in PVS])

    def put(name, v, label):
        op[name].write([v], wait=True, timeout=2.0)
        events.append((round(time.time() - t0, 3), label))
        print(f"[{time.time() - t0:6.2f} s] {label}", flush=True)

    def wait_until(cond, timeout, period=0.05):
        t_end = time.time() + timeout
        while time.time() < t_end:
            sample()
            if cond():
                return True
            time.sleep(period)
        return False

    put("ESTOP_OK", 1, "e-stop released, door closed")
    op["DOOR_CLOSED"].write([1], wait=True)
    if not wait_until(lambda: val("STATE") == "WAIT_PERMIT", 60):
        raise SystemExit("force_insertion is not waiting for a permit (started with require_permit:=true?)")
    time.sleep(0.5)
    put("RESET", 1, "RESET")
    if not wait_until(lambda: val("PERMIT") == 1, 5):
        raise SystemExit(f"No permit after reset, first-out: {val('FIRST_OUT_STR')}")
    if not wait_until(lambda: val("STATE") == "INSERTING" and val("DEPTH") > a.estop_depth, 120):
        raise SystemExit("Insertion never reached the e-stop depth")
    put("ESTOP_OK", 0, "E-STOP pressed")
    wait_until(lambda: False, a.hold)
    put("ESTOP_OK", 1, "e-stop released")
    wait_until(lambda: False, 1.0)
    put("RESET", 1, "RESET")
    ok = wait_until(lambda: val("STATE") == "FAULT_INTERLOCK", 120)
    wait_until(lambda: False, 2.0)

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t"] + list(PVS))
        w.writerows(rows)
    with open(a.out.replace(".csv", "_events.csv"), "w", newline="") as f:
        csv.writer(f).writerows([("t", "event")] + events)
    print(("DONE" if ok else "INCOMPLETE") + f": {len(rows)} samples -> {a.out}")
    plot(rows, events, a.out.replace(".csv", ".png"))


def plot(rows, events, path):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    t = [float(r[0]) for r in rows]
    fig, ax = plt.subplots(2, 1, figsize=(9, 5.5), sharex=True, gridspec_kw={"height_ratios": [3, 2]})
    ax[0].plot(t, [float(r[5]) for r in rows], color="#3b6ea5", lw=1.6)
    ax[0].set_ylabel("insertion depth [mm]")
    ax[1].step(t, [int(r[1]) for r in rows], where="post", color="#2f8f4e", lw=1.6, label="permit (1 = ON)")
    ax[1].step(t, [int(r[2]) for r in rows], where="post", color="#c48a1a", lw=1.2,
               label="first-out (1 E-STOP, 5 heartbeat)")
    ax[1].set_ylabel("PLC")
    ax[1].set_yticks(range(0, 6))
    ax[1].legend(loc="upper left", fontsize=8)
    ax[1].set_xlabel("time [s]")
    top = max(float(r[5]) for r in rows) * 1.25 + 1
    ax[0].set_ylim(-2, top)
    for k, (te, label) in enumerate(events):
        te = float(te)
        for a_ in ax:
            a_.axvline(te, color="0.65", lw=0.8, ls="--")
        ax[0].text(te + 0.3, top * (0.92 - 0.12 * (k % 3)), label, fontsize=8, color="0.25")
    for a_ in ax:
        a_.grid(alpha=0.3)
    fig.suptitle("E-stop during insertion: freeze, first-out E-STOP, reset, retreat", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f"plot -> {path}")


def replot(csv_path):
    """python -m rh_cell_ic.estop_demo --replot file.csv"""
    with open(csv_path) as f:
        rows = list(csv.reader(f))[1:]
    with open(csv_path.replace(".csv", "_events.csv")) as f:
        events = list(csv.reader(f))[1:]
    plot(rows, events, csv_path.replace(".csv", ".png"))


if __name__ == "__main__":
    main()
