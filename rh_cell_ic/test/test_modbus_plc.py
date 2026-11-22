"""ModbusPlc against a fake OpenPLC: a minimal Modbus TCP server (function codes 1, 3, 16)
whose 'PLC program' is interlock.py, scanned every 20 ms.

Checks the register map end to end (addresses, coil vs register, watchdog counter) without an
OpenPLC install, plus the fail-safe when the PLC is unreachable. The server is written by hand
(standard library only) so the test does not depend on pymodbus server internals.
"""
import socket
import socketserver
import struct
import threading
import time

import pytest

pytest.importorskip("pymodbus")

from rh_cell_ic.interlock import COMMS_LOST, Inputs, Interlock  # noqa: E402
from rh_cell_ic.plc import ModbusPlc  # noqa: E402


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


class FakeOpenPlc:
    def __init__(self, port):
        self.hr = [0] * 1100       # holding registers (OpenPLC: %QW from 0, %MW from 1024)
        self.coils = [False] * 16  # %QX0.0 ..
        self.lock = threading.Lock()
        self.il = Interlock()
        plc = self

        class Handler(socketserver.BaseRequestHandler):
            def handle(self):
                self.request.settimeout(0.5)  # never block forever on a dead client
                while plc.running:
                    try:
                        head = self.request.recv(7)  # MBAP: tid, pid, len, unit
                    except socket.timeout:
                        continue
                    except OSError:
                        return
                    if len(head) < 7:
                        return
                    tid, pid, length, unit = struct.unpack(">HHHB", head)
                    pdu = self.request.recv(length - 1)
                    resp = plc.process(pdu)
                    self.request.sendall(struct.pack(">HHHB", tid, pid, len(resp) + 1, unit) + resp)

        class Server(socketserver.ThreadingTCPServer):
            allow_reuse_address = True
            daemon_threads = True      # handler threads never keep the test process alive
            block_on_close = False     # server_close() does not wait for handler threads

        self.server = Server(("127.0.0.1", port), Handler)
        self.running = True

    def process(self, pdu):
        fc = pdu[0]
        with self.lock:
            if fc == 1:  # read coils
                addr, n = struct.unpack(">HH", pdu[1:5])
                bits = self.coils[addr:addr + n]
                packed = bytearray((n + 7) // 8)
                for k, b in enumerate(bits):
                    if b:
                        packed[k // 8] |= 1 << (k % 8)
                return bytes([1, len(packed)]) + bytes(packed)
            if fc == 3:  # read holding registers
                addr, n = struct.unpack(">HH", pdu[1:5])
                regs = self.hr[addr:addr + n]
                return bytes([3, 2 * n]) + struct.pack(">" + "H" * n, *regs)
            if fc == 16:  # write multiple registers
                addr, n, _ = struct.unpack(">HHB", pdu[1:6])
                self.hr[addr:addr + n] = struct.unpack(">" + "H" * n, pdu[6:6 + 2 * n])
                return struct.pack(">BHH", 16, addr, n)
        return bytes([fc | 0x80, 1])  # illegal function

    def scan_loop(self):
        while self.running:
            with self.lock:
                permit = self.il.scan(Inputs(*[bool(x) for x in self.hr[1024:1030]]))
                self.coils[0], self.coils[1] = permit, not permit
                self.hr[0] = self.il.first_out
            time.sleep(0.02)

    def start(self):
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        threading.Thread(target=self.scan_loop, daemon=True).start()

    def stop(self):
        self.running = False
        self.server.shutdown()
        self.server.server_close()


@pytest.fixture
def plc():
    port = free_port()
    fake = FakeOpenPlc(port)
    fake.start()
    client = ModbusPlc("127.0.0.1", port)
    yield fake, client
    client.close()
    fake.stop()


def scan_for(client, inputs, seconds=0.12):
    out = None
    t_end = time.time() + seconds
    while time.time() < t_end:
        out = client.scan(inputs)
        time.sleep(0.02)
    return out


HEALTHY = dict(estop_ok=True, door_closed=True, guard_ok=True, ft_alive=True, heartbeat_ok=True)


def test_register_map_round_trip(plc):
    fake, client = plc
    assert scan_for(client, Inputs(**HEALTHY))[0] is False            # no reset yet
    scan_for(client, Inputs(**HEALTHY, reset=True))
    assert scan_for(client, Inputs(**HEALTHY)) == (True, 0)           # permit ON
    assert fake.hr[1024:1029] == [1, 1, 1, 1, 1]                      # %MW0..%MW4
    c1 = fake.hr[1030]
    client.scan(Inputs(**HEALTHY))
    assert fake.hr[1030] != c1                                         # %MW6 watchdog counts
    assert scan_for(client, Inputs(**dict(HEALTHY, door_closed=False))) == (False, 2)  # DOOR OPEN


def test_unreachable_plc_means_no_permit():
    client = ModbusPlc("127.0.0.1", free_port(), timeout=0.1)
    permit, first_out = client.scan(Inputs(True, True, True, True, True, True))
    assert permit is False and first_out == COMMS_LOST
