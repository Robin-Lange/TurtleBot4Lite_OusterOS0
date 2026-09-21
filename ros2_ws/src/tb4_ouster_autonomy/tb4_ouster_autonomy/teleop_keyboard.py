#!/usr/bin/env python3
"""Standard ROS 2 keyboard teleoperation node for tb4_ouster_autonomy.

Kinematics: Non-holonomic (iRobot Create 3 differential drive).
Controls linear.x (forward/backward) and angular.z (rotation); linear.y is strictly 0.

Behavior:
Follows the standard ROS 2 tutorial conventions (turtlesim turtle_teleop_key
and teleop_twist_keyboard). Single keypress sets direction; commands are published
continuously at 10 Hz until a stop key (Spacebar, 'k') or opposite direction is pressed.

Controls:
  Arrow Keys:
    [Up Arrow]    : drive forward
    [Down Arrow]  : drive backward
    [Left Arrow]  : turn left on the spot
    [Right Arrow] : turn right on the spot

  Standard letter keys:
    w / i         : drive forward
    s / ,         : drive backward
    a / j         : turn left on the spot
    d / l         : turn right on the spot
    u             : curve forward left
    o             : curve forward right
    m             : curve backward left
    .             : curve backward right

  Stop:
    Spacebar / k  : STOP (set velocity to 0)

  Speed adjustments:
    q / z         : increase / decrease linear speed (+/- 10%)
    e / c         : increase / decrease angular speed (+/- 10%)

  Quit:
    Ctrl+C or Esc : exit cleanly (sends 0 velocity and restores terminal)
"""

import select
import sys
import termios
import threading
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

HELP_TEXT = """
===========================================================
  tb4_ouster_autonomy Interactive Keyboard Teleoperation
  Standard ROS 2 Tutorial Controls (Create 3 Non-holonomic)
===========================================================
Drive controls:
   Arrow Keys:
          [Up] Forward
   [Left] Turn L   [Right] Turn R
         [Down] Backward

   Standard letter keys:
         w / i
    a/j  s / ,  d/l

    u : curve forward left      o : curve forward right
    m : curve backward left     . : curve backward right

    Spacebar / k : STOP (zero velocity)

Speed adjustments:
    q / z : increase / decrease linear speed (+/- 10%)
    e / c : increase / decrease angular speed (+/- 10%)

    Ctrl+C or Esc to exit
===========================================================
"""

MOVE_BINDINGS = {
    "up": (1.0, 0.0),
    "down": (-1.0, 0.0),
    "left": (0.0, 1.0),
    "right": (0.0, -1.0),
    "i": (1.0, 0.0),
    "w": (1.0, 0.0),
    ",": (-1.0, 0.0),
    "s": (-1.0, 0.0),
    "j": (0.0, 1.0),
    "a": (0.0, 1.0),
    "l": (0.0, -1.0),
    "d": (0.0, -1.0),
    "u": (1.0, 1.0),
    "o": (1.0, -1.0),
    "m": (-1.0, -1.0),
    ".": (-1.0, 1.0),
}


class TeleopKeyboardNode(Node):
    """Publishes continuous non-holonomic velocity commands and deadman status."""

    def __init__(self):
        super().__init__("tb4_teleop_keyboard")

        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("default_linear_speed", 0.12)
        self.declare_parameter("default_angular_speed", 0.40)
        self.declare_parameter("max_linear_speed", 0.15)
        self.declare_parameter("max_angular_speed", 0.50)

        self.linear_speed = float(self.get_parameter("default_linear_speed").value)
        self.angular_speed = float(self.get_parameter("default_angular_speed").value)
        self.max_linear = float(self.get_parameter("max_linear_speed").value)
        self.max_angular = float(self.get_parameter("max_angular_speed").value)

        self.pub_cmd_vel = self.create_publisher(Twist, "/teleop/cmd_vel", 10)
        self.pub_deadman = self.create_publisher(Bool, "/teleop/deadman", 10)

        self.target_x = 0.0
        self.target_z = 0.0
        self.deadman_active = True

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

    def timer_callback(self):
        # 1. Publish deadman heartbeat
        deadman_msg = Bool()
        deadman_msg.data = self.deadman_active
        self.pub_deadman.publish(deadman_msg)

        # 2. Publish non-holonomic velocity command (vx, wz only; vy=0)
        twist = Twist()
        twist.linear.x = float(self.target_x * self.linear_speed)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(self.target_z * self.angular_speed)
        self.pub_cmd_vel.publish(twist)

    def stop_robot(self):
        self.target_x = 0.0
        self.target_z = 0.0


def get_key(settings, timeout=0.05) -> str:
    """Read a single key or ANSI arrow key escape sequence."""
    if select.select([sys.stdin], [], [], timeout)[0]:
        key = sys.stdin.read(1)
        if key == "\x1b":
            # Check for multi-byte escape sequence (arrow keys)
            if select.select([sys.stdin], [], [], 0.05)[0]:
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

    # Spin the ROS node in a background thread so callbacks and timer continue reliably
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(HELP_TEXT)
    print(
        f"Initial speeds: Linear = {node.linear_speed:.2f} m/s (cap: {node.max_linear:.2f}) | "
        f"Angular = {node.angular_speed:.2f} rad/s (cap: {node.max_angular:.2f})\n"
    )

    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            key = get_key(old_settings, timeout=0.08)

            if not key:
                continue

            if key in MOVE_BINDINGS:
                node.target_x, node.target_z = MOVE_BINDINGS[key]
            elif key in (" ", "k"):
                node.stop_robot()
            elif key == "q":
                node.linear_speed = min(node.max_linear, round(node.linear_speed * 1.10, 3))
            elif key == "z":
                node.linear_speed = max(0.04, round(node.linear_speed * 0.90, 3))
            elif key == "e":
                node.angular_speed = min(node.max_angular, round(node.angular_speed * 1.10, 3))
            elif key == "c":
                node.angular_speed = max(0.10, round(node.angular_speed * 0.90, 3))
            elif key in ("\x03", "esc"):  # Ctrl+C or Esc
                break

            # Print current state on a single line
            current_vx = node.target_x * node.linear_speed
            current_wz = node.target_z * node.angular_speed
            status_line = (
                f"\rSpeed: [vx: {current_vx:+.2f} m/s (cap {node.linear_speed:.2f}) | "
                f"wz: {current_wz:+.2f} rad/s (cap {node.angular_speed:.2f})]    "
            )
            sys.stdout.write(status_line)
            sys.stdout.flush()

    except Exception as e:
        print(f"\nTeleop exception: {e}")
    finally:
        # Restore terminal settings immediately
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        # Send zero velocity and halt
        node.deadman_active = False
        node.stop_robot()
        for _ in range(5):
            node.timer_callback()
            time.sleep(0.02)
        node.destroy_node()
        rclpy.shutdown()
        print("\nTeleop stopped cleanly.")


if __name__ == "__main__":
    main()
