# Modbus TCP map — ROS bridge ↔ OpenPLC

OpenPLC exposes located variables over Modbus TCP (port 502):
`%MW` = holding registers starting at 1024, `%QX` = coils starting at 0, `%QW` = holding registers starting at 0.

| PLC variable | Modbus | Direction | Meaning (1 = healthy) |
|---|---|---|---|
| `%MW0` | HR 1024 | bridge → PLC | E-stop chain closed |
| `%MW1` | HR 1025 | bridge → PLC | Cell door closed |
| `%MW2` | HR 1026 | bridge → PLC | Force guard not tripped |
| `%MW3` | HR 1027 | bridge → PLC | F/T sensor data arriving |
| `%MW4` | HR 1028 | bridge → PLC | Controller heartbeat arriving |
| `%MW5` | HR 1029 | bridge → PLC | Reset push-button (momentary) |
| `%MW6` | HR 1030 | bridge → PLC | Watchdog counter, incremented every scan |
| `%QX0.0` | coil 0 | PLC → bridge | Motion permit |
| `%QX0.1` | coil 1 | PLC → bridge | Fault (= not permit) |
| `%QW0` | HR 0 | PLC → bridge | First-out code |

First-out codes: 0 none · 1 E-STOP · 2 DOOR OPEN · 3 FORCE GUARD · 4 F/T SENSOR LOST · 5 CONTROLLER HEARTBEAT LOST · 6 COMMS LOST.

**Why the watchdog counter.** Holding registers keep their last value. If the bridge crashed after writing "all healthy", the PLC would keep the permit on forever. The bridge increments `%MW6` every 20 ms scan; the PLC drops the permit (first-out 6) when it has not changed for 500 ms. The bridge does the same in the other direction: a failed Modbus transaction is treated as permit OFF.

**Limits.** This is a functional demo of interlock logic, not a safety-rated design. A real permit would run on a safety PLC (SIL/PL rated), with hard-wired e-stop and door channels, not over Modbus TCP.
