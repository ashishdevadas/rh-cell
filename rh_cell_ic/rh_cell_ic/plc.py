"""PLC back-ends with one interface: scan(Inputs) -> (permit, first_out).

  SoftPlc    the Python copy of the interlock (interlock.py), runs inside the bridge
  ModbusPlc  the real IEC 61131-3 program running in the OpenPLC runtime, over Modbus TCP

No ROS here, so both can be unit-tested. The register map is in plc/modbus_map.md.
"""
from .interlock import COMMS_LOST, Inputs, Interlock

# OpenPLC Modbus addressing: %MW0.. = holding registers from 1024, %QX0.0.. = coils from 0,
# %QW0.. = holding registers from 0.
HR_INPUTS = 1024   # %MW0..%MW6  estop, door, guard, ft, heartbeat, reset, bridge watchdog counter
COIL_PERMIT = 0    # %QX0.0      (coil 1 = %QX0.1 fault, not needed: fault = not permit)
HR_FIRST_OUT = 0   # %QW0


def input_words(i: Inputs):
    return [int(i.estop_ok), int(i.door_closed), int(i.guard_ok), int(i.ft_alive), int(i.heartbeat_ok), int(i.reset)]


class SoftPlc:
    name = "soft PLC (interlock.py)"

    def __init__(self):
        self._il = Interlock()

    def scan(self, i: Inputs):
        permit = self._il.scan(i)
        return permit, self._il.first_out

    def close(self):
        pass


class ModbusPlc:
    """Writes the inputs to OpenPLC and reads back what the PLC program decided.

    Fail-safe on both sides:
      * here, any communication error returns permit=False (first-out PLC COMMS LOST);
      * in the PLC, the last written inputs would stay "healthy" forever if this bridge died,
        so every scan also writes an incrementing counter to %MW6. The PLC program drops the
        permit when the counter stops changing for 500 ms (see plc/rh_cell_interlock.st).
    """

    name = "OpenPLC over Modbus TCP"

    def __init__(self, host="127.0.0.1", port=502, timeout=0.2):
        from pymodbus.client import ModbusTcpClient  # imported here so sim mode needs no pymodbus

        self._client = ModbusTcpClient(host, port=port, timeout=timeout)
        self.host, self.port = host, port
        self.connected = False
        self.last_error = ""
        self._wd = 0

    def scan(self, i: Inputs):
        try:
            if not self._client.connected and not self._client.connect():
                raise ConnectionError(f"cannot connect to {self.host}:{self.port}")
            self._wd = (self._wd + 1) % 30000
            w = self._client.write_registers(HR_INPUTS, input_words(i) + [self._wd])
            c = self._client.read_coils(COIL_PERMIT, count=1)
            r = self._client.read_holding_registers(HR_FIRST_OUT, count=1)
            for resp in (w, c, r):
                if resp.isError():
                    raise IOError(str(resp))
            self.connected = True
            return bool(c.bits[0]), int(r.registers[0])
        except Exception as e:  # noqa: BLE001  any failure = no permit
            self.connected = False
            self.last_error = str(e)
            self._client.close()
            return False, COMMS_LOST

    def close(self):
        self._client.close()
