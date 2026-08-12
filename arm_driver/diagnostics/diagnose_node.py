#!/usr/bin/env python3
"""
Read-only diagnostic for a single CANBUS stepper node.

Run this INSTEAD of demo.launch.py - only one process can hold the serial
port at a time, so Ctrl+C the bridge first if it's running.

Usage: python3 diagnostics/diagnose_node.py <node_id>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from canbus_stepper import CANBus, MsgType

SERIAL_PORT = "/dev/ttyACM0"


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 diagnostics/diagnose_node.py <node_id>")
        sys.exit(1)
    node_id = int(sys.argv[1])

    bus = CANBus(SERIAL_PORT)
    node = bus.node(node_id)

    print(f"Querying node {node_id}...\n")

    try:
        angle = node.get_angle()
        print(f"Encoder angle:     {angle:.2f} deg")
    except TimeoutError:
        print("Encoder angle:     TIMED OUT - node is not responding on the CAN bus at all.")

    try:
        voltage = node.get_voltage()
        print(f"Bus voltage:       {voltage:.2f} V")
    except TimeoutError:
        print("Bus voltage:       TIMED OUT")

    try:
        current = node.get_current()
        print(f"Motor current:     {current}%  (should be 60%, per ROBOT_ARM_HANDOFF.md S3)")
    except TimeoutError:
        print("Motor current:     TIMED OUT")

    try:
        temp = node.get_board_temperature()
        print(f"Board temperature: {temp:.1f} C")
    except TimeoutError:
        print("Board temperature: TIMED OUT")

    try:
        payload = node._request(MsgType.FAULT_CODE)
        print(f"Fault code (raw bytes, format undocumented): {payload.hex()}")
    except TimeoutError:
        print("Fault code:        TIMED OUT")

    bus.close()


if __name__ == "__main__":
    main()
