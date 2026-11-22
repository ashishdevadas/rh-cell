# FMEA — rh-cell insertion task

Failure Mode and Effects Analysis for the remote-handling cell: picking the cassette up, inserting it through the 90 mm port, seating it, extracting it, under teleoperation or automatic control, supervised by the interlock.

Ratings are qualitative (1 = best, 10 = worst) for this simulated cell, not for a real installation.
**S** severity · **O** occurrence · **D** detection (10 = undetectable) · **RPN** = S × O × D.
"Evidence" points to the run or unit test in this repository that exercises the line.

| # | Function | Failure mode | Effect | Cause | Detection | Mitigation in the cell | S | O | D | RPN | Evidence |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | Align cassette with port | Port is not where the robot believes (error larger than the 10 mm clearance) | Cassette hits the wall face; very high impact force | Calibration error, arm deflection under load, wrong cell model | Force guard (5 samples > 20 N); insertion monitor (contact before the expected depth) | Slow approach through the port mouth; automatic retreat. **Impact peak not bounded by software** (see residual risk) | 8 | 4 | 3 | 96 | `fault:=misaligned`: stopped at 30.0 mm, peak 438 N (263–465 N across runs) |
| 2 | Bound contact force at the seat | Controller pushes too hard | Damage to seat or cassette | Admittance disabled or badly tuned, wrong reference | Force guard | Admittance (K = 500 N/m) with a reference only 10 mm behind the seat; spring-loaded seat | 6 | 3 | 2 | 36 | `fault:=stiff`: tripped at 20.1 N, 155.1 mm deep |
| 3 | Sense contact forces | F/T signal lost | Controller blind: force guard and admittance stop working | Cable, bus, driver, process crash | 50 ms watchdog in the task node; raw-topic freshness in the PLC bridge (interlock input 4) | Stop and retreat; PLC drops the permit | 8 | 3 | 2 | 48 | `fault:=sensor_dropout`: stopped at 30.9 mm; `ft_monitor` watchdog test |
| 4 | Sense contact forces | F/T signal frozen or biased (still arriving, wrong value) | Guard does not trip on real contact, or trips in free space | Sensor drift, wrong tare, stuck driver | Partly: insertion monitor (contact at an unexpected depth). A small bias or a frozen value is **not** detected | Tare at rest before every insertion | 7 | 3 | 7 | **147** | Not injected yet (open action) |
| 5 | Detect contact | False contact in free space | Insertion aborted for nothing (availability loss) | Joint jitter shaking the 0.5 kg cassette (single samples up to ~10 N seen in simulation) | — | 10-sample debounce (40 ms) on first contact; 20 Hz low-pass; guard needs 5 consecutive samples | 3 | 4 | 2 | 24 | Unit tests `JitterSpikesInFreeSpaceAreNotContact`, `SustainedEarlyContactIsStillCaught` |
| 6 | Stop on operator demand | E-stop pressed during insertion | Must freeze; no retreat until reset | Operator decision | Interlock input 1; first-out E-STOP on the operator screen | Latched permit; task holds position; MoveIt trajectories cancelled; retreat only after release **and** reset | 9 | 3 | 1 | 27 | `estop_demo`: frozen at ~60 mm, retreat after reset (`docs/estop_interlock.png`) |
| 7 | Keep people out | Door opened during motion | Person in the cell with a moving robot | Access during operation | Interlock input 2 | Same as e-stop | 10 | 2 | 1 | 20 | `test_interlock.py` (unit tests) |
| 8 | Supervise the controller | Task node hangs or crashes | Robot keeps its last command, nobody supervises the force | Software fault | Heartbeat (input 5, 0.5 s timeout) | Permit drops, first-out HEARTBEAT LOST | 7 | 3 | 2 | 42 | End of every interlock run: permit drops when the node exits |
| 9 | PLC link (OpenPLC mode) | Bridge dies after writing "all healthy" | PLC keeps the permit on | Bridge crash | Watchdog counter `%MW6`, 500 ms | PLC drops the permit, first-out COMMS LOST; the bridge treats any Modbus error as no permit | 8 | 2 | 2 | 32 | `test_modbus_plc.py` (counter, unreachable PLC). PLC side not run on the real OpenPLC runtime |
| 10 | Restart after a fault | Permit returns by itself when the fault clears | Unexpected motion | Non-latching logic | — | Permit is a latch: RESET rising edge with all inputs healthy; holding RESET does not re-arm | 9 | 1 | 1 | 9 | `test_holding_reset_does_not_rearm_after_trip`; live check step 5 (released, still no permit) |
| 11 | Teleoperation | Operator drives the cassette into the wall | Collision | Operator error, poor depth perception | Virtual fixture (radial error); push limit | Fixture keeps the tool on the port axis and blocks forward motion when off-axis; push limit 8 N; dead-man 0.3 s | 8 | 4 | 2 | 64 | `teleop_demo.py` with / without fixture: reaches the seat at 8 N vs jammed at 703 N |
| 12 | Interlock availability | Nuisance trip: "F/T sensor lost" while the sensor is fine | Cycle aborted for nothing | Gazebo stream paused > 100 ms during a controller switch on a laptop | Seen in testing | PLC-side freshness check relaxed to 300 ms (wall clock); the 50 ms sim-time watchdog in the task node is unchanged | 3 | 3 | 2 | 18 | Found while testing the e-stop demo |
| 13 | Planned motion (MoveIt) | Chosen arm configuration cannot make the following straight move | Cycle aborted ("Straight-line path only 27.5% feasible") | A pose goal lets MoveIt pick any of the UR's inverse-kinematics solutions | Cartesian path fraction < 100% | Taught joint configurations for the approach points (`kPreGraspQ`, `kPreInsertQ`) | 4 | 2 | 1 | 8 | Phase 1.1 simulation tests: 1 failure in 4 runs before, 5/5 after |
| 14 | Planned motion (MoveIt) | Trajectory rejected at start | Cycle aborted ("Execute request aborted") | Arm not settled after the previous move (start-state tolerance 0.01 rad), more frequent when the simulation runs slower than real time | `move_group` start-state check | Straight moves wait 0.3 s and retry from the current state | 3 | 3 | 1 | 9 | Phase 1.1 on the laptop (real-time factor ~60–77%) |

## Residual risk and what it teaches

**Line 1 is the important one.** The force guard and the insertion monitor detect a misaligned port at the same place every time (30.0 mm, the wall face), but the impact peak varies from 263 to 465 N between runs of the same code, and slowing the approach from 5 to 1 mm/s barely changes it. Against a rigid obstacle the first impact is set by contact and joint stiffness, not by the control loop: software detection limits how long the overload lasts, not how high it goes.

The mitigations that bound the peak are outside the force loop:

1. **Passive compliance** between tool and part (a remote-centre-compliance wrist, or a compliant docking interface like the spring-loaded seat already in this cell).
2. **Alignment check before contact**: measure the port position (vision, touch probing) and refuse to start the insertion if the error exceeds the clearance. ITER's remote-handling tests use vision for this.
3. **Knowledge of the arm**: a deflection / calibration model so the believed pose matches the real one.

**Line 4 has the highest RPN (147)** because a frozen or biased sensor is the failure the current design is worst at detecting: the watchdog only notices data that stops arriving, not data that is wrong.

## Open actions

| Action | Addresses |
|---|---|
| Inject a frozen / biased F/T signal and add a plausibility check (expected contact force vs commanded motion, or two independent sensors) | 4 |
| Add a compliant wrist (prismatic joint with spring) to the Gazebo model and repeat `fault:=misaligned` | 1 |
| Pre-insertion alignment check: simulated port measurement with noise, abort if the error exceeds 8 mm | 1 |
| Run `rh_cell_interlock.st` in the real OpenPLC runtime and repeat the e-stop demo with `plc_mode:=openplc` | 9 |
