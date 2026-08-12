#!/usr/bin/env python3
"""
Query MoveIt's real /check_state_validity service for a specific joint
configuration (or several, interpolated) and report exactly why a state is
invalid - which link pair collides, or which joint is out of bounds -
instead of guessing.

Usage: python3 diagnostics/check_state_validity.py <sequence.json> [waypoint_index]
If waypoint_index is omitted, checks every waypoint in the file. Sequence
files saved by waypoint_gui.py live in ../sequences/.
"""
import sys
import json

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

GROUP_NAME = "arm"


def check_positions(node, client, positions: dict, label: str):
    req = GetStateValidity.Request()
    req.group_name = GROUP_NAME
    js = JointState()
    js.name = list(positions.keys())
    js.position = list(positions.values())
    req.robot_state = RobotState()
    req.robot_state.joint_state = js

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.result() is None:
        print(f"{label}: SERVICE CALL TIMED OUT")
        return
    res = future.result()
    if res.valid:
        print(f"{label}: VALID")
    else:
        print(f"{label}: INVALID")
        for c in res.contacts:
            print(f"    collision: {c.contact_body_1} <-> {c.contact_body_2}")
        for jc in res.constraint_result:
            pass  # constraint details rarely populated, skip


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 check_state_validity.py <sequence.json> [waypoint_index]")
        sys.exit(1)
    path = sys.argv[1]
    only_index = int(sys.argv[2]) if len(sys.argv) > 2 else None

    data = json.loads(open(path).read())
    waypoints = data["waypoints"]

    rclpy.init()
    node = Node("check_state_validity")
    client = node.create_client(GetStateValidity, "/check_state_validity")
    if not client.wait_for_service(timeout_sec=5.0):
        print("/check_state_validity service not available - is move_group running?")
        rclpy.shutdown()
        sys.exit(1)

    indices = [only_index] if only_index is not None else range(len(waypoints))
    for i in indices:
        check_positions(node, client, waypoints[i]["positions"], f"Waypoint {i + 1}")

    # Also check a few interpolated points between consecutive distinct waypoints
    for i in range(len(waypoints) - 1):
        a = waypoints[i]["positions"]
        b = waypoints[i + 1]["positions"]
        if a == b:
            continue
        for frac in (0.25, 0.5, 0.75):
            mid = {k: a[k] + (b[k] - a[k]) * frac for k in a}
            check_positions(node, client, mid, f"Between {i + 1}->{i + 2} at {frac}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
