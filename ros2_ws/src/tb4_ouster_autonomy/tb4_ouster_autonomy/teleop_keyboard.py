#!/usr/bin/env python3
"""Interactive keyboard teleoperation node for tb4_ouster_autonomy.

Publishes continuous 10 Hz velocity commands to `/teleop/cmd_vel` and
heartbeat signals to `/teleop/deadman`. Eliminates the need for multiple
terminal tabs for deadman heartbeat and fixes single-keystroke timeout issues.

Controls:
  w / i / Up-Arrow    : drive forward
  s / , / Down-Arrow  : drive backward
  a / j / Left-Arrow  : turn left on the spot
  d / l / Right-Arrow : turn right on the spot
  u / o               : curve forward left / right
  m / .               : curve backward left / right
  Space / k           : STOP (zero velocity)
  q / z               : increase / decrease linear speed (+/- 0.02 m/s)
  e / c               : increase / decrease angular speed (+/- 0.05 rad/s)
  Ctrl+C / Esc        : exit cleanly
"""

import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

HELP_TEXT = """
===========================================================
  tb4_ouster_autonomy Interactive Keyboard Teleoperation
===========================================================
Moving around:
        w / i
   a/j  s / ,  d/l
        
   u : forward left     o : forward right
   m : backward left    . : backward right

   Spacebar / k : EMERGENCY STOP (0 velocity)

Speed tuning:
   q / z : increase / decrease max linear speed (+/- 0.02 m/s)
   e / c : increase / decrease max angular speed (+/- 0.05 rad/s)

   Ctrl+C or Esc to exit
===========================================================
"""


class TeleopKeyboardNode(Node):

    def __init__(self):
        super().__init__("tb4_teleop_keyboard")

        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("default_linear_speed", 0.12)
        self.declare_parameter("default_angular_speed", 0.35)
        self.declare_parameter("max_linear_speed", 0.15)
        self.declare_parameter("max_angular_speed", 0.50)

        self.linear_speed = float(self.get_parameter("default_linear_speed").value)
        self.angular_speed = float(self.get_parameter("default_angular_speed").value)
        self.max_linear = float(self.get_parameter("max_linear_speed").value)
        self.max_angular = float(self.get_parameter("max_angular_speed").value)

        self.pub_cmd_vel = self.create_publisher(Twist, "/teleop/cmd_vel", 10)
        self.pub_deadman = self.create_publisher(Bool, "/teleop/deadman", 10)

        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.deadman_active = True

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

    def timer_callback(self):
        # 1. Publish deadman heartbeat
        deadman_msg = Bool()
        deadman_msg.data = self.deadman_active
        self.pub_deadman.publish(deadman_msg)

        # 2. Publish velocity command
        twist = Twist()
        twist.linear.x = float(self.target_linear_x)
        twist.angular.z = float(self.target_angular_z)
        self.pub_cmd_vel.publish(twist)

    def stop_robot(self):
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0


def get_key(settings, timeout=0.08) -> str:
    if select.select([sys.stdin], [], [], timeout)[0]:
        key = sys.stdin.read(1)
        if key == "\x1b":  # Escape sequence for arrow keys
            extra = sys.stdin.read(2)
            if extra == "[A":
                return "up"
            elif extra == "[B":
                return "down"
            elif extra == "[C":
                return "right"
            elif extra == "[D":
                return "left"
            return "esc"
        return key
    return ""


def main():
    if not sys.stdin.isatty():
        print("Error: teleop must be run in an interactive terminal.")
        return

    old_settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopKeyboardNode()

    print(HELP_TEXT)
    print(f"Initial: Linear = {node.linear_speed:.2f} m/s | Angular = {node.angular_speed:.2f} rad/s\n")

    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(old_settings, timeout=0.05)

            if not key:
                continue

            # Check directional commands
            if key in ("w", "i", "up"):
                node.target_linear_x = node.linear_speed
                node.target_angular_z = 0.0
            elif key in ("s", ",", "down"):
                node.target_linear_x = -node.linear_speed
                node.target_angular_z = 0.0
            elif key in ("a", "j", "left"):
                node.target_linear_x = 0.0
                node.target_angular_z = node.angular_speed
            elif key in ("d", "l", "right"):
                node.target_linear_x = 0.0
                node.target_angular_z = -node.angular_speed
            elif key == "u":
                node.target_linear_x = node.linear_speed
                node.target_angular_z = node.angular_speed
            elif key == "o":
                node.target_linear_x = node.linear_speed
                node.target_angular_z = -node.angular_speed
            elif key == "m":
                node.target_linear_x = -node.linear_speed
                node.target_angular_z = -node.angular_speed
            elif key == ".":
                node.target_linear_x = -node.linear_speed
                node.target_angular_z = node.angular_speed
            elif key in (" ", "k"):
                node.stop_robot()
            elif key == "q":
                node.linear_speed = min(node.max_linear, node.linear_speed + 0.02)
                if node.target_linear_x > 0:
                    node.target_linear_x = node.linear_speed
                elif node.target_linear_x < 0:
                    node.target_linear_x = -node.linear_speed
            elif key == "z":
                node.linear_speed = max(0.04, node.linear_speed - 0.02)
                if node.target_linear_x > 0:
                    node.target_linear_x = node.linear_speed
                elif node.target_linear_x < 0:
                    node.target_linear_x = -node.linear_speed
            elif key == "e":
                node.angular_speed = min(node.max_angular, node.angular_speed + 0.05)
                if node.target_angular_z > 0:
                    node.target_angular_z = node.angular_speed
                elif node.target_angular_z < 0:
                    node.target_angular_z = -node.angular_speed
            elif key == "c":
                node.angular_speed = max(0.10, node.angular_speed - 0.05)
                if node.target_angular_z > 0:
                    node.target_angular_z = node.angular_speed
                elif node.target_angular_z < 0:
                    node.target_angular_z = -node.angular_speed
            elif key in ("\x03", "esc"):  # Ctrl+C or Esc
                break

            # Print current state
            status_line = (
                f"\rSpeed: [lin: {node.target_linear_x:+.2f} m/s (cap: {node.linear_speed:.2f}) | "
                f"ang: {node.target_angular_z:+.2f} rad/s (cap: {node.angular_speed:.2f})]    "
            )
            sys.stdout.write(status_line)
            sys.stdout.flush()

    except Exception as e:
        print(f"\nTeleop error: {e}")
    finally:
        # Restore terminal before anything else
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        # Send zero velocity and deadman False
        node.deadman_active = False
        node.stop_robot()
        for _ in range(3):
            node.timer_callback()
            time.sleep(0.02)
        node.destroy_node()
        rclpy.shutdown()
        print("\nTeleop stopped cleanly.")


if __name__ == "__main__":
    main()
