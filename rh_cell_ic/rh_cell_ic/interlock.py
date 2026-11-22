"""Motion-permit interlock: the same logic as plc/rh_cell_interlock.st, in Python.

Used as the built-in soft PLC (plc_mode:=sim) and as the reference in the unit tests.
Keep the two in step: any change here must be made in the .st file too (test_interlock.py checks
the first-out codes). The .st file adds a watchdog on the bridge itself (code 6), which the soft
PLC does not need because it runs inside the bridge.

Rules (the way a safety relay behaves):
  * The permit is a latch. It is SET only by a RESET rising edge while every input is healthy.
  * It is CLEARED at once when any input becomes unhealthy, and stays off until the next RESET.
    There is no automatic restart when the fault goes away.
  * First-out annunciation: the first input that dropped is remembered, so the operator sees
    the cause and not the cascade (an e-stop also makes the guard and heartbeat look bad).

Inputs are True when HEALTHY (fail-safe convention: a lost signal reads False = trip).
"""
from dataclasses import dataclass

# Order = first-out priority when several inputs drop in the same scan. Codes match the .st file.
INPUTS = ("estop_ok", "door_closed", "guard_ok", "ft_alive", "heartbeat_ok")
FIRST_OUT_NAMES = {
    0: "NONE",
    1: "E-STOP",
    2: "DOOR OPEN",
    3: "FORCE GUARD",
    4: "F/T SENSOR LOST",
    5: "CONTROLLER HEARTBEAT LOST",
    6: "COMMS LOST",  # PLC <-> bridge link (OpenPLC mode only): set by the bridge or the PLC watchdog
}
COMMS_LOST = 6


@dataclass
class Inputs:
    estop_ok: bool = False
    door_closed: bool = False
    guard_ok: bool = False
    ft_alive: bool = False
    heartbeat_ok: bool = False
    reset: bool = False


class Interlock:
    def __init__(self):
        self.permit = False
        self.first_out = 0
        self._reset_prev = False

    @staticmethod
    def healthy(i: Inputs) -> bool:
        return all(getattr(i, n) for n in INPUTS)

    def scan(self, i: Inputs) -> bool:
        """One PLC scan. Returns the motion permit."""
        reset_edge = i.reset and not self._reset_prev
        self._reset_prev = i.reset

        if not self.healthy(i):
            if self.first_out == 0:
                # record the first cause only (first input in priority order that is bad)
                for code, name in enumerate(INPUTS, start=1):
                    if not getattr(i, name):
                        self.first_out = code
                        break
            self.permit = False
        elif reset_edge:
            self.permit = True
            self.first_out = 0
        return self.permit

    @property
    def first_out_name(self) -> str:
        return FIRST_OUT_NAMES[self.first_out]
