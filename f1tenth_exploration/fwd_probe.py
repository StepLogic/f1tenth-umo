#!/usr/bin/env python3
"""Drive straight at SPEED for SECS, logging every stage of the command chain
and the lidar range dead ahead.  fwd_probe.py [speed] [secs]"""
import math, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped

SPEED = float(sys.argv[1]) if len(sys.argv) > 1 else 0.3
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 1.5
rclpy.init()
n = Node("fwd_probe")
log = {"mux": [], "erpm_cmd": [], "erpm": [], "current": [], "volts": [], "front": []}
def scan(m):
    a = np.arange(len(m.ranges)) * m.angle_increment + m.angle_min
    r = np.array(m.ranges)
    k = (np.abs(a) < math.radians(5)) & np.isfinite(r) & (r > 0.1)
    if k.any():
        log["front"].append((time.time(), float(np.median(r[k]))))
n.create_subscription(LaserScan, "/scan", scan, qos_profile_sensor_data)
n.create_subscription(AckermannDriveStamped, "/ackermann_cmd", lambda m: log["mux"].append(m.drive.speed), 10)
n.create_subscription(Float64, "/commands/motor/speed", lambda m: log["erpm_cmd"].append(m.data), 10)
def core(m):
    log["erpm"].append(m.state.speed); log["current"].append(m.state.current_motor); log["volts"].append(m.state.voltage_input)
n.create_subscription(VescStateStamped, "/sensors/core", core, 10)
pub = n.create_publisher(AckermannDriveStamped, "/drive", 10)
def spin(t):
    end = time.time() + t
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.01)
def send(v, t):
    end = time.time() + t
    while time.time() < end:
        msg = AckermannDriveStamped(); msg.drive.speed = v; msg.drive.steering_angle = 0.0
        pub.publish(msg); spin(0.05)
spin(4.0)                                   # discovery + baseline scans
before = np.median([f for _, f in log["front"][-20:]]) if log["front"] else float("nan")
for k in log: 
    if k != "front": log[k].clear()
t0 = time.time()
send(SPEED, SECS)
send(0.0, 1.0)
spin(1.0)
after = np.median([f for t, f in log["front"] if t > time.time() - 0.5]) if log["front"] else float("nan")
fmt = lambda v: f"min {min(v):+.2f} max {max(v):+.2f} (n={len(v)})" if v else "NO MESSAGES"
print(f"battery        {fmt(log['volts'])}")
print(f"mux output     {fmt(log['mux'])}  m/s")
print(f"erpm command   {fmt(log['erpm_cmd'])}")
print(f"motor erpm     {fmt(log['erpm'])}")
print(f"motor current  {fmt(log['current'])}  A")
d = before - after
print(f"front range    {before:.3f} -> {after:.3f} m: "
      + ("FORWARD" if d > 0.05 else "BACKWARD" if d < -0.05 else "did not move") + f" ({d:+.2f} m)")
n.destroy_node(); rclpy.shutdown()
