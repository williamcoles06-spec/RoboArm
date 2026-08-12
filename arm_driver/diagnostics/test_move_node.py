#!/usr/bin/env python3
"""
Standalone single-node move test (motor degrees, NOT joint radians).
Bypasses the bridge entirely - isolates whether ENABLE + a position command
produces real motion, independent of ROS/MoveIt.

Ctrl+C the bridge first if it's running - only one process can hold the
serial port at a time.

Usage: python3 diagnostics/test_move_node.py <node_id> <delta_deg> [speed_deg_s]
Example: python3 diagnostics/test_move_node.py 4 10      # move node 4 by +10 motor-deg
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from canbus_stepper import CANBus

SERIAL_PORT = "/dev/ttyACM0"
DEFAULT_SPEED = 50.0  # deg/s, slow and safe


def main():
    if len(sys.argv) < 3:
        print("Usage: python3 diagnostics/test_move_node.py <node_id> <delta_deg> [speed_deg_s]")
        sys.exit(1)
    node_id = int(sys.argv[1])
    delta = float(sys.argv[2])
    speed = float(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_SPEED

    bus = CANBus(SERIAL_PORT)
    node = bus.node(node_id)

    start = node.get_angle()
    print(f"Starting angle: {start:.2f} deg")

    print("Commanding zero-error hold at current position, then enabling...")
    node.set_position(start)
    node.enable()
    time.sleep(0.5)

    target = start + delta
    print(f"Moving to {target:.2f} deg at {speed} deg/s - WATCH/LISTEN to the joint now.")
    node.set_position_speed(speed)
    node.set_position(target)

    wait_s = abs(delta) / speed + 1.0
    time.sleep(wait_s)

    end = node.get_angle()
    print(f"Ending angle:   {end:.2f} deg  (moved {end - start:.2f} deg)")

    bus.close()


if __name__ == "__main__":
    main()
