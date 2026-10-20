#!/usr/bin/env python3
"""Bridge Nav2's /cmd_vel (Twist) to the car's /drive (AckermannDriveStamped).

Nav2 speaks the differential-drive language of linear.x + angular.z. The car
steers, so angular velocity has to become a steering angle through the bicycle
model:

    delta = atan(wheelbase * omega / v)

This node also publishes CONTINUOUSLY at `publish_rate`, holding the last
command, rather than only on each /cmd_vel. That is deliberate and does three
jobs at once:

  1. Primes odometry. vesc_to_odom only emits /odom and the odom->base_link TF
     while servo commands are flowing, because it integrates the commanded
     servo position. With nothing on /drive the whole chain goes quiet, the TF
     disappears, and Nav2's costmaps come up blind — no TF means no scan
     raytracing, which means no obstacles, which means the car drives into
     walls. Publishing a stationary command from startup keeps the TF alive
     before Nav2 has said anything. (This replaces the older separate
     prime_odom.py, which raced the bridge for ownership of /drive.)

  2. Feeds the mux. ackermann_mux drops an input that goes silent for 0.2 s.

  3. Acts as a watchdog. If Nav2 stops publishing (crash, killed node, goal
     cancelled), commands go stale after `cmd_timeout` and this node commands
     zero speed. Without that the VESC would happily hold the last non-zero
     speed forever and the car would drive away on its own.

Steering limits come from the servo calibration in vesc.yaml:
    servo = -1.2135 * delta + 0.5304,  servo clamped to [0.15, 0.85]
which yields a usable steering range of about -0.263 .. +0.313 rad. The default
max_steering_angle of 0.26 rad is the symmetric limit inside that range, giving
a minimum turning radius of wheelbase / tan(0.26) ~= 0.94 m.
"""
import math
import threading

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelToAckermann(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_ackermann")

        self.declare_parameter("wheelbase", 0.25)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("max_steering_angle", 0.26)  # rad, see docstring
        self.declare_parameter("max_speed", 0.6)            # m/s, hard cap
        # Below this the motor will not start: a VESC cannot reliably start a
        # sensorless motor at very low eRPM. MEASURED on this car with
        # speed_threshold.py: 0.15 m/s (692 eRPM) produced no motion at all,
        # 0.20 m/s (923 eRPM) turned the wheels. That was measured while the
        # motor direction was inverted (the car was reversing). Re-measured
        # FORWARD on 2026-10-17 with fwd_probe.py on the floor: 0.3 m/s stalled
        # at 19 A, 0.4 m/s crept 0.13 m in 1 s, 0.5 m/s moved 0.52 m. 0.45 it is.
        # Any non-zero request below the floor is raised to it, so "go" always
        # means the car actually goes. RE-MEASURE for a different car, motor,
        # gearing or floor surface — do not inherit this number.
        self.declare_parameter("min_move_speed", 0.45)      # m/s
        self.declare_parameter("publish_rate", 20.0)        # Hz
        self.declare_parameter("cmd_timeout", 0.5)          # s before zeroing

        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.max_steer = float(self.get_parameter("max_steering_angle").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.min_move = float(self.get_parameter("min_move_speed").value)
        rate = float(self.get_parameter("publish_rate").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        self.lock = threading.Lock()
        self.speed = 0.0
        self.steer = 0.0
        self.last_cmd_time = None

        self.pub = self.create_publisher(
            AckermannDriveStamped, self.get_parameter("drive_topic").value, 10)
        self.sub = self.create_subscription(
            Twist, self.get_parameter("cmd_vel_topic").value, self.cmd_cb, 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            "cmd_vel -> ackermann bridge up: wheelbase=%.3f m, max_steer=%.3f rad, "
            "speed %.2f..%.2f m/s, %.0f Hz. Holding a stationary command until Nav2 drives."
            % (self.wheelbase, self.max_steer, self.min_move, self.max_speed, rate))

    def cmd_cb(self, msg: Twist):
        v = msg.linear.x
        omega = msg.angular.z

        # Below a threshold the bicycle model is ill-conditioned (dividing by a
        # near-zero speed sends delta to +-90 deg), so steer straight instead.
        if abs(v) < 1e-3:
            delta = 0.0
        else:
            delta = math.atan(self.wheelbase * omega / abs(v))

        delta = max(-self.max_steer, min(self.max_steer, delta))

        # Raise a non-zero request up to the motor's starting threshold. Zero
        # stays exactly zero — this must never turn a stop command into motion.
        if 0.0 < abs(v) < self.min_move:
            v = math.copysign(self.min_move, v)
        v = max(-self.max_speed, min(self.max_speed, v))

        with self.lock:
            self.speed = v
            self.steer = delta
            self.last_cmd_time = self.get_clock().now()

    def tick(self):
        with self.lock:
            speed, steer, last = self.speed, self.steer, self.last_cmd_time

        if last is None:
            # Nav2 has not spoken yet: hold the car still, but keep /drive alive
            # so odom and the odom->base_link TF exist before Nav2 starts.
            speed, steer = 0.0, 0.0
        else:
            age = (self.get_clock().now() - last).nanoseconds * 1e-9
            if age > self.cmd_timeout:
                if speed != 0.0:
                    self.get_logger().warn(
                        "No /cmd_vel for %.2f s — commanding zero speed." % age)
                with self.lock:
                    self.speed = 0.0
                    self.steer = 0.0
                speed, steer = 0.0, 0.0

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAckermann()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best effort: leave the car stopped rather than holding the last speed.
        try:
            stop = AckermannDriveStamped()
            stop.header.stamp = node.get_clock().now().to_msg()
            stop.header.frame_id = "base_link"
            node.pub.publish(stop)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
