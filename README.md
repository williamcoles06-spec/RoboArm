# RoboArm

A 6-DOF robot arm I designed and built from scratch, running on a ROS 2 / MoveIt 2 stack.

## What this is

Six joints, each driven by a stepper motor with a custom gearbox: 27:1 cycloidal drives for the base, shoulder, and elbow, a 5:1 planetary drive for the forearm roll, and a 6:1 belt-driven differential for the wrist pitch and roll. The structure is 3D printed in PLA, designed in Fusion 360.

Motion planning goes through MoveIt 2, but the hardware interface underneath it is custom. The stepper controllers (CANBUS Stepper boards, closed loop, daisy chained over CAN) don't fit cleanly into `ros2_control` on this setup, so I wrote a standalone ROS 2 node that acts as the `FollowJointTrajectory` action server MoveIt talks to directly. It turns planned trajectories into CAN bus position commands, verifies the arm actually reached each target using live encoder feedback, and publishes `/joint_states` back to MoveIt.

## Highlights

- Full 6-DOF planning through MoveIt 2, including a differential wrist where two motors jointly produce pitch and roll (no 1:1 joint-to-motor mapping)
- A custom hardware driver that bypasses `ros2_control` entirely, with arrival verification instead of just trusting planned timing
- A companion desktop app for building and running multi-waypoint motion sequences: drag the arm into a pose in RViz, capture it as a waypoint, set a speed and pause time for that segment, and save the whole sequence under a name to reuse later
- One click to start everything: attach the hardware, launch RViz, and open the sequencer together

## Repo layout

- `robo_arm_simplified_for_urdf_v2_1/` - URDF and mesh files, exported from Onshape
- `robo_arm_moveit_config/` - MoveIt 2 configuration: kinematics, joint limits, planning pipeline, launch files
- `arm_driver/` - the hardware driver (`canbus_bridge.py`), the CANBUS Stepper protocol library, the waypoint sequencer GUI, and one-off hardware diagnostic scripts under `diagnostics/`
- `Robo Arm Assembly.step` - the full CAD assembly

## Hardware

| Joint | Function | Reduction | Motor |
|---|---|---|---|
| J1 | Base rotation | 27:1 cycloidal | NEMA 17 |
| J2 | Shoulder | 27:1 cycloidal | NEMA 17 |
| J3 | Elbow | 27:1 cycloidal | NEMA 17 |
| J4 | Forearm roll | 5:1 planetary | NEMA 17 |
| J5 / J6 | Wrist pitch / roll | 6:1 differential | pancake stepper x2 |

Each joint runs on a [CANBUS Stepper](https://thingsbyjosh.com) controller board (ESP32-S3, TMC2209 driver, closed-loop magnetic encoder), all chained on a single CAN bus reachable over one USB-C connection.

## Software

ROS 2 Jazzy and MoveIt 2. No `ros2_control` - `arm_driver/canbus_bridge.py` talks directly to MoveIt as the trajectory execution interface.

## Status

Fully working end to end: homing and calibration on all 6 joints, MoveIt Plan and Execute, and multi-waypoint sequences through the companion GUI.
