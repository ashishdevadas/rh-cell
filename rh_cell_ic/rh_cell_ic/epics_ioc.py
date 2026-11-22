"""EPICS IOC for the remote-handling cell (Channel Access, pure Python with caproto),
bridged to ROS 2. The Phoebus operator screen (phoebus/rh_cell.bob) talks only to these PVs.

    ros2 run rh_cell_ic epics_ioc                 # PV prefix RH:
    caproto-get RH:PERMIT RH:FIRST_OUT_STR        # from any terminal (or EPICS base caget)
    caproto-put RH:ESTOP_OK 0                     # press the e-stop
    caproto-put RH:ESTOP_OK 1; caproto-put RH:RESET 1   # release it and reset

Operator PVs (writable)          ->  ROS topics, re-published at 10 Hz (a dead IOC = stale = trip)
  RH:ESTOP_OK      1 = released      /rh/operator/estop_ok
  RH:DOOR_CLOSED   1 = closed        /rh/operator/door_closed
  RH:RESET         write 1, momentary: held 0.3 s then back to 0   /rh/operator/reset

Status PVs (read-only)           <-  ROS topics
  RH:PERMIT, RH:FIRST_OUT, RH:FIRST_OUT_STR      /rh/permit, /rh/first_out
  RH:IN_ESTOP .. RH:IN_HEARTBEAT (1 = healthy)  /rh/interlock_inputs
  RH:STATE                                       /rh/state
  RH:DEPTH (mm), RH:FZ (N), RH:FMAG (N)          /rh/depth, /rh/ft_filtered
"""
import math
import threading
import time

from caproto import ChannelType
from caproto.server import PVGroup, pvproperty

from .interlock import FIRST_OUT_NAMES

INPUT_PVS = ("in_estop", "in_door", "in_guard", "in_ft", "in_heartbeat")
RESET_PULSE_S = 0.3


class Shared:
    """Thread-safe values exchanged between the ROS thread and the IOC (asyncio) thread."""

    def __init__(self):
        self._lock = threading.Lock()
        self.operator = {"estop_ok": True, "door_closed": True, "reset": False}
        self.reset_until = 0.0
        self.status = {
            "permit": 0, "first_out": 0, "inputs": [0, 0, 0, 0, 0],
            "state": "IDLE", "depth": 0.0, "fz": 0.0, "fmag": 0.0,
        }

    def set_operator(self, key, value):
        with self._lock:
            self.operator[key] = value
            if key == "reset" and value:
                self.reset_until = time.monotonic() + RESET_PULSE_S

    def operator_snapshot(self):
        with self._lock:
            if self.operator["reset"] and time.monotonic() > self.reset_until:
                self.operator["reset"] = False
            return dict(self.operator)

    def set_status(self, **kw):
        with self._lock:
            self.status.update(kw)

    def status_snapshot(self):
        with self._lock:
            return {k: (list(v) if isinstance(v, list) else v) for k, v in self.status.items()}


class RhCellIOC(PVGroup):
    """PV definitions. `shared` is the only link to the outside world, so this class can be
    tested without ROS."""

    def __init__(self, *args, shared: Shared, **kwargs):
        self.shared = shared
        super().__init__(*args, **kwargs)

    # ---------------------------------------------------------------- operator (writable)
    estop_ok = pvproperty(value=1, name="ESTOP_OK", doc="1 = e-stop released")
    door_closed = pvproperty(value=1, name="DOOR_CLOSED", doc="1 = cell door closed")
    reset = pvproperty(value=0, name="RESET", doc="write 1 to reset the interlock")

    @estop_ok.putter
    async def estop_ok(self, instance, value):
        self.shared.set_operator("estop_ok", bool(value))
        return int(bool(value))

    @door_closed.putter
    async def door_closed(self, instance, value):
        self.shared.set_operator("door_closed", bool(value))
        return int(bool(value))

    @reset.putter
    async def reset(self, instance, value):
        if value:
            self.shared.set_operator("reset", True)
        return int(bool(value))

    # ---------------------------------------------------------------- status (read-only)
    permit = pvproperty(value=0, name="PERMIT", read_only=True)
    first_out = pvproperty(value=0, name="FIRST_OUT", read_only=True)
    first_out_str = pvproperty(value="NONE", name="FIRST_OUT_STR", dtype=ChannelType.STRING, read_only=True)
    in_estop = pvproperty(value=0, name="IN_ESTOP", read_only=True)
    in_door = pvproperty(value=0, name="IN_DOOR", read_only=True)
    in_guard = pvproperty(value=0, name="IN_GUARD", read_only=True)
    in_ft = pvproperty(value=0, name="IN_FT", read_only=True)
    in_heartbeat = pvproperty(value=0, name="IN_HEARTBEAT", read_only=True)
    state = pvproperty(value="IDLE", name="STATE", dtype=ChannelType.STRING, read_only=True)
    depth = pvproperty(value=0.0, name="DEPTH", units="mm", precision=1, read_only=True)
    fz = pvproperty(value=0.0, name="FZ", units="N", precision=2, read_only=True)
    fmag = pvproperty(value=0.0, name="FMAG", units="N", precision=2, read_only=True)

    # One 10 Hz scan copies everything from `shared` into the PVs (only on change).
    heartbeat = pvproperty(value=0, name="IOC_HEARTBEAT", read_only=True, doc="counts up at 10 Hz")

    @heartbeat.scan(period=0.1)
    async def heartbeat(self, instance, async_lib):
        await instance.write((instance.value + 1) % 100000)
        s = self.shared.status_snapshot()
        fo = int(s["first_out"])
        await self._update(self.permit, int(s["permit"]))
        await self._update(self.first_out, fo)
        await self._update(self.first_out_str, FIRST_OUT_NAMES.get(fo, str(fo))[:39])
        for name, v in zip(INPUT_PVS, s["inputs"]):
            await self._update(getattr(self, name), int(v))
        await self._update(self.state, str(s["state"])[:39])
        await self._update(self.depth, round(float(s["depth"]), 1))
        await self._update(self.fz, round(float(s["fz"]), 2))
        await self._update(self.fmag, round(float(s["fmag"]), 2))
        # momentary reset: put the PV back to 0 once the pulse is over
        if self.reset.value and not self.shared.operator_snapshot()["reset"]:
            await self.reset.write(0)

    @staticmethod
    async def _update(pv, value):
        if pv.value != value:
            await pv.write(value)


# ==================================================================== ROS side
def _ros_thread(shared: Shared, ready: threading.Event, stop: threading.Event):
    import rclpy
    from geometry_msgs.msg import WrenchStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import Bool, Float64, Int32, Int32MultiArray, String

    node = rclpy.create_node("rh_epics_ioc")
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    node.create_subscription(Bool, "/rh/permit", lambda m: shared.set_status(permit=int(m.data)), latched)
    node.create_subscription(Int32, "/rh/first_out", lambda m: shared.set_status(first_out=m.data), latched)
    node.create_subscription(
        Int32MultiArray, "/rh/interlock_inputs", lambda m: shared.set_status(inputs=list(m.data)[:5]), 10
    )
    node.create_subscription(String, "/rh/state", lambda m: shared.set_status(state=m.data), latched)
    node.create_subscription(Float64, "/rh/depth", lambda m: shared.set_status(depth=m.data * 1000.0), 10)

    def on_ft(m):
        f = m.wrench.force
        shared.set_status(fz=f.z, fmag=math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z))

    node.create_subscription(WrenchStamped, "/rh/ft_filtered", on_ft, 10)

    pubs = {k: node.create_publisher(Bool, f"/rh/operator/{k}", 10) for k in ("estop_ok", "door_closed", "reset")}

    def publish_operator():
        for k, v in shared.operator_snapshot().items():
            pubs[k].publish(Bool(data=bool(v)))

    node.create_timer(0.1, publish_operator)

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ready.set()
    while rclpy.ok() and not stop.is_set():
        ex.spin_once(timeout_sec=0.1)
    node.destroy_node()


def main():
    import argparse
    import sys

    import rclpy
    from caproto.server import run

    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--prefix", default="RH:")
    ap.add_argument("--interface", default="0.0.0.0")
    opt = ap.parse_args(args)

    rclpy.init()
    shared = Shared()
    ready, stop = threading.Event(), threading.Event()
    t = threading.Thread(target=_ros_thread, args=(shared, ready, stop), daemon=True)
    t.start()
    ready.wait(5.0)

    ioc = RhCellIOC(prefix=opt.prefix, shared=shared)
    print(f"rh_cell EPICS IOC: {len(ioc.pvdb)} PVs with prefix {opt.prefix}", flush=True)
    try:
        run(ioc.pvdb, interfaces=[opt.interface], log_pv_names=True)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        t.join(1.0)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
