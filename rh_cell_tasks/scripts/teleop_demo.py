#!/usr/bin/env python3
"""Scripted 'operator' for teleop_fixture: the same stick commands every time, so runs with and
without the virtual fixture can be compared.

    ros2 run rh_cell_tasks teleop_fixture                                  # terminal 2
    ros2 run rh_cell_tasks teleop_demo.py                                  # terminal 3

The operator is deliberately sloppy: the tool starts 20 mm to the side of the port axis and the
operator pushes straight in, then drifts sideways while inserting, then keeps pushing at the seat.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter

# (duration s, x, y, z) in stick units (-1..1); teleop_fixture scales by max_speed (0.03 m/s)
SCRIPT = [
    (1.0, 0.0, 0.0, 0.0),
    (4.0, 1.0, 0.0, 0.0),    # straight in, while 20 mm off-axis
    (4.0, 1.0, 0.3, 0.0),    # keep going in with a sideways drift
    (3.0, 1.0, -0.3, 0.2),   # drift the other way and up
    (4.0, 0.6, 0.0, 0.0),    # push on into the seat (force limit must stop it)
    (1.0, 0.0, 0.0, 0.0),
    (6.0, -1.0, 0.0, 0.0),   # pull out
    (1.0, 0.0, 0.0, 0.0),
]


class Demo(Node):
    def __init__(self):
        # Sim time, so the script lasts the same in simulated seconds whatever the real-time factor.
        super().__init__("rh_teleop_demo", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(Twist, "/rh/teleop_cmd", 10)
        self.t = 0.0
        self.k = 0
        self.seg_t = 0.0
        self.create_timer(0.02, self.tick)
        self.get_logger().info("Scripted operator starting")

    def tick(self):
        if self.k >= len(SCRIPT):
            self.get_logger().info("Script done")
            raise SystemExit
        dur, x, y, z = SCRIPT[self.k]
        m = Twist()
        m.linear.x, m.linear.y, m.linear.z = x, y, z
        self.pub.publish(m)
        self.seg_t += 0.02
        if self.seg_t >= dur:
            self.k += 1
            self.seg_t = 0.0


def main():
    rclpy.init()
    node = Demo()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
