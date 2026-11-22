"""ROS 2 <-> PLC bridge: collects the five interlock inputs, runs one PLC scan every 20 ms,
publishes the motion permit.

    ros2 run rh_cell_ic plc_bridge                                   # soft PLC (default)
    ros2 run rh_cell_ic plc_bridge --ros-args -p plc_mode:=openplc -p modbus_host:=127.0.0.1

Inputs (True = healthy). A signal that stops arriving counts as unhealthy (fail-safe).
  estop_ok      /rh/operator/estop_ok    (Bool, from the operator screen via the EPICS IOC)
  door_closed   /rh/operator/door_closed (Bool, same)
  guard_ok      NOT /rh/guard_tripped    (Bool, from the task node's force guard)
  ft_alive      /rh/ft (raw, from the Gazebo bridge) arriving       AND the task node does not report the sensor lost
  heartbeat_ok  /rh/heartbeat arriving   (Bool, 10 Hz from the task node)
  reset         /rh/operator/reset       (Bool, momentary)

Outputs
  /rh/permit            Bool        motion permit
  /rh/first_out         Int32       code of the first input that dropped (interlock.py)
  /rh/interlock_inputs  Int32MultiArray [estop, door, guard, ft, heartbeat] (1 = healthy)
"""
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, Int32, Int32MultiArray

from .interlock import FIRST_OUT_NAMES, Inputs
from .plc import ModbusPlc, SoftPlc


class Latest:
    """Last value of a topic and when it arrived (node clock)."""

    def __init__(self):
        self.value = None
        self.stamp = None

    def set(self, value, now):
        self.value = value
        self.stamp = now

    def fresh(self, now, timeout):
        return self.stamp is not None and (now - self.stamp).nanoseconds * 1e-9 <= timeout


class PlcBridge(Node):
    def __init__(self):
        super().__init__("rh_plc_bridge")
        self.declare_parameter("plc_mode", "sim")
        self.declare_parameter("modbus_host", "127.0.0.1")
        self.declare_parameter("modbus_port", 502)
        self.declare_parameter("scan_period", 0.02)      # s, same as the OpenPLC task
        self.declare_parameter("operator_timeout", 0.5)  # s
        self.declare_parameter("ft_timeout", 0.3)        # s, wall clock: backup to the 50 ms watchdog in the task node
        self.declare_parameter("heartbeat_timeout", 0.5)  # s

        mode = self.get_parameter("plc_mode").value
        if mode == "openplc":
            self.plc = ModbusPlc(self.get_parameter("modbus_host").value, self.get_parameter("modbus_port").value)
        else:
            self.plc = SoftPlc()
        self.get_logger().info(f"PLC back-end: {self.plc.name}")

        self.t = {k: Latest() for k in ("estop", "door", "reset", "guard", "ft_raw", "ft_report", "heartbeat")}
        sub = self.create_subscription
        sub(Bool, "/rh/operator/estop_ok", lambda m: self._set("estop", m.data), 10)
        sub(Bool, "/rh/operator/door_closed", lambda m: self._set("door", m.data), 10)
        sub(Bool, "/rh/operator/reset", lambda m: self._set("reset", m.data), 10)
        sub(Bool, "/rh/guard_tripped", lambda m: self._set("guard", m.data), 10)
        sub(Bool, "/rh/ft_alive", lambda m: self._set("ft_report", m.data), 10)
        sub(Bool, "/rh/heartbeat", lambda m: self._set("heartbeat", m.data), 10)
        sub(WrenchStamped, "/rh/ft", lambda m: self._set("ft_raw", True), qos_profile_sensor_data)

        latched = QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.permit_pub = self.create_publisher(Bool, "/rh/permit", latched)
        self.first_out_pub = self.create_publisher(Int32, "/rh/first_out", latched)
        self.inputs_pub = self.create_publisher(Int32MultiArray, "/rh/interlock_inputs", 10)

        self.permit = None
        self.first_out = None
        self.create_timer(self.get_parameter("scan_period").value, self.scan)

    def _set(self, key, value):
        self.t[key].set(value, self.get_clock().now())

    def read_inputs(self) -> Inputs:
        now = self.get_clock().now()
        op_to = self.get_parameter("operator_timeout").value
        t = self.t

        def op(key):  # operator signal: must be fresh AND true
            return t[key].fresh(now, op_to) and bool(t[key].value)

        # Guard and the task node's sensor report only exist while a task runs. When they are
        # silent there is no motion to guard; the heartbeat input covers "is a controller there".
        guard_ok = not (t["guard"].fresh(now, op_to) and t["guard"].value)
        ft_reported_lost = t["ft_report"].fresh(now, op_to) and not t["ft_report"].value
        ft_alive = t["ft_raw"].fresh(now, self.get_parameter("ft_timeout").value) and not ft_reported_lost
        heartbeat_ok = t["heartbeat"].fresh(now, self.get_parameter("heartbeat_timeout").value)

        return Inputs(
            estop_ok=op("estop"),
            door_closed=op("door"),
            guard_ok=guard_ok,
            ft_alive=ft_alive,
            heartbeat_ok=heartbeat_ok,
            reset=op("reset"),
        )

    def scan(self):
        i = self.read_inputs()
        permit, first_out = self.plc.scan(i)

        self.permit_pub.publish(Bool(data=permit))
        self.first_out_pub.publish(Int32(data=first_out))
        self.inputs_pub.publish(
            Int32MultiArray(data=[int(i.estop_ok), int(i.door_closed), int(i.guard_ok), int(i.ft_alive), int(i.heartbeat_ok)])
        )

        if permit != self.permit or first_out != self.first_out:
            name = FIRST_OUT_NAMES.get(first_out, str(first_out))
            if permit:
                self.get_logger().info("PERMIT ON (reset accepted)")
            elif self.permit:
                self.get_logger().warn(f"PERMIT OFF, first-out: {name}")
            else:
                self.get_logger().info(f"Permit off, first-out: {name}")
            self.permit, self.first_out = permit, first_out

    def destroy_node(self):
        self.plc.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = PlcBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
