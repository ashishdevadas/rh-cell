"""The IOC alone (no ROS): operator PVs reach `shared`, status in `shared` reaches the PVs,
RESET is momentary. Runs a real Channel Access server on localhost."""
import os
import socket
import subprocess
import sys
import textwrap
import time

import pytest

pytest.importorskip("caproto")


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


SERVER = textwrap.dedent(
    """
    import sys, threading, time
    from caproto.server import run
    from rh_cell_ic.epics_ioc import RhCellIOC, Shared
    shared = Shared()
    def fake_ros():
        # pretend the PLC bridge reports: permit follows the operator's e-stop
        while True:
            op = shared.operator_snapshot()
            shared.set_status(permit=int(op["estop_ok"]), first_out=0 if op["estop_ok"] else 1,
                              inputs=[int(op["estop_ok"]), 1, 1, 1, 1], state="TEST",
                              depth=12.3, fz=-4.5, fmag=4.5)
            time.sleep(0.05)
    threading.Thread(target=fake_ros, daemon=True).start()
    ioc = RhCellIOC(prefix="TST:", shared=shared)
    run(ioc.pvdb, interfaces=["127.0.0.1"])
    """
)


@pytest.fixture
def ca_env():
    port = free_port()
    env = dict(os.environ)
    env.update(
        EPICS_CA_SERVER_PORT=str(port),
        EPICS_CAS_SERVER_PORT=str(port),
        EPICS_CA_ADDR_LIST="127.0.0.1",
        EPICS_CA_AUTO_ADDR_LIST="NO",
        EPICS_CAS_INTF_ADDR_LIST="127.0.0.1",
        PYTHONPATH=os.pathsep.join([os.path.join(os.path.dirname(__file__), ".."), env.get("PYTHONPATH", "")]),
    )
    proc = subprocess.Popen([sys.executable, "-c", SERVER], env=env, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, text=True)
    # wait until the IOC answers (startup time varies, e.g. while colcon builds other packages)
    t_end = time.time() + 30.0
    while time.time() < t_end:
        try:
            r = subprocess.run([sys.executable, "-m", "caproto.commandline.get", "--timeout", "1", "TST:PERMIT"],
                               env=env, capture_output=True, text=True, timeout=10)
            if r.returncode == 0:
                break
        except subprocess.TimeoutExpired:
            pass
        time.sleep(0.5)
    yield env
    proc.terminate()
    try:
        proc.wait(5)
    except subprocess.TimeoutExpired:
        proc.kill()


def ca(env, tool, *args):
    out = subprocess.run([sys.executable, "-m", f"caproto.commandline.{tool}", *args], env=env,
                         capture_output=True, text=True, timeout=15)
    assert out.returncode == 0, out.stderr
    return out.stdout


def value(env, pv):
    return ca(env, "get", "--format", "{response.data[0]}", pv).strip()


def test_status_pvs_follow_shared_state(ca_env):
    assert value(ca_env, "TST:PERMIT") == "1"
    assert value(ca_env, "TST:STATE") in ("TEST", "b'TEST'")
    assert float(value(ca_env, "TST:FZ")) == pytest.approx(-4.5)


def test_estop_put_reaches_ros_side_and_comes_back(ca_env):
    ca(ca_env, "put", "TST:ESTOP_OK", "0")
    time.sleep(0.5)
    assert value(ca_env, "TST:PERMIT") == "0"
    assert value(ca_env, "TST:FIRST_OUT_STR") in ("E-STOP", "b'E-STOP'")
    ca(ca_env, "put", "TST:ESTOP_OK", "1")
    time.sleep(0.5)
    assert value(ca_env, "TST:PERMIT") == "1"


def test_reset_is_momentary(ca_env):
    ca(ca_env, "put", "TST:RESET", "1")
    time.sleep(0.8)
    assert value(ca_env, "TST:RESET") == "0"
