(*
  rh-cell motion-permit interlock, IEC 61131-3 Structured Text, for the OpenPLC runtime.
  Same logic as rh_cell_ic/interlock.py (the soft PLC used when OpenPLC is not running),
  plus a watchdog on the ROS bridge, which only matters when the PLC is a separate process.

  Modbus TCP map (OpenPLC addressing, see plc/modbus_map.md)
    inputs written by the ROS bridge  -> holding registers %MW0..%MW6 (Modbus 1024..1030)
    outputs read by the ROS bridge    <- coil %QX0.0 permit, coil %QX0.1 fault, register %QW0 first-out

  All inputs are 1 when HEALTHY (fail-safe convention).
  The permit latches ON only on a RESET rising edge with every input healthy, and drops at once
  on any bad input. No automatic restart. First-out code = first input that dropped.
    1 E-STOP, 2 DOOR OPEN, 3 FORCE GUARD, 4 F/T SENSOR LOST, 5 CONTROLLER HEARTBEAT LOST,
    6 COMMS LOST (the bridge stopped updating its watchdog counter %MW6 for 500 ms)
*)
PROGRAM rh_cell_interlock
  VAR
    in_estop_ok     AT %MW0 : INT := 0;
    in_door_closed  AT %MW1 : INT := 0;
    in_guard_ok     AT %MW2 : INT := 0;
    in_ft_alive     AT %MW3 : INT := 0;
    in_heartbeat_ok AT %MW4 : INT := 0;
    in_reset        AT %MW5 : INT := 0;
    in_wd_counter   AT %MW6 : INT := 0;

    out_permit      AT %QX0.0 : BOOL := FALSE;
    out_fault       AT %QX0.1 : BOOL := FALSE;
    out_first_out   AT %QW0 : INT := 0;
  END_VAR
  VAR
    permit       : BOOL := FALSE;
    first_out    : INT := 0;
    healthy      : BOOL := FALSE;
    reset_now    : BOOL := FALSE;
    reset_prev   : BOOL := FALSE;
    reset_edge   : BOOL := FALSE;
    wd_prev      : INT := 0;
    wd_changed   : BOOL := FALSE;
    wd_timer     : TON;
    bridge_alive : BOOL := FALSE;
  END_VAR

  (* Bridge watchdog: the counter must change at least every 500 ms *)
  wd_changed := (in_wd_counter <> wd_prev);
  wd_prev := in_wd_counter;
  wd_timer(IN := NOT wd_changed, PT := T#500ms);
  bridge_alive := NOT wd_timer.Q;

  healthy := bridge_alive AND (in_estop_ok = 1) AND (in_door_closed = 1) AND (in_guard_ok = 1)
             AND (in_ft_alive = 1) AND (in_heartbeat_ok = 1);

  reset_now := (in_reset = 1);
  reset_edge := reset_now AND NOT reset_prev;
  reset_prev := reset_now;

  IF NOT healthy THEN
    IF first_out = 0 THEN
      IF NOT bridge_alive THEN
        first_out := 6;
      ELSIF in_estop_ok <> 1 THEN
        first_out := 1;
      ELSIF in_door_closed <> 1 THEN
        first_out := 2;
      ELSIF in_guard_ok <> 1 THEN
        first_out := 3;
      ELSIF in_ft_alive <> 1 THEN
        first_out := 4;
      ELSE
        first_out := 5;
      END_IF;
    END_IF;
    permit := FALSE;
  ELSIF reset_edge THEN
    permit := TRUE;
    first_out := 0;
  END_IF;

  out_permit := permit;
  out_fault := NOT permit;
  out_first_out := first_out;
END_PROGRAM


CONFIGURATION Config0
  RESOURCE Res0 ON PLC
    TASK task0(INTERVAL := T#20ms, PRIORITY := 0);
    PROGRAM instance0 WITH task0 : rh_cell_interlock;
  END_RESOURCE
END_CONFIGURATION
