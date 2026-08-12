#!/usr/bin/env python3
"""
CANBUS Stepper <-> ROS 2 bridge and robot driver.

This node replaces ros2_control entirely (see ROBOT_ARM_HANDOFF.md §7): it is
a FollowJointTrajectory action server that MoveIt talks to directly, plus a
/joint_states publisher built from live encoder telemetry. No ros2_control,
no hardware plugin, no controller spawners.

Five parts, in the order they appear below:
 1. CONFIG            - per-joint node id, gear ratio, direction sign, home
                         encoder angle, and joint limit (§3). Edit values
                         here, never the logic below.
 2. CONVERSIONS        - joint-radians <-> motor-degrees math (§6), shared by
                         the startup sequence, the state publisher, and the
                         action server.
 3. STARTUP SEQUENCE  - §5: the zero-error handoff from "limp at an unknown
                         pose" to "enabled and sitting at true home".
 4. STATE PATH        - a 30 Hz timer reads every board's encoder and
                         publishes /joint_states in joint-space radians.
 5. ACTION SERVER     - FollowJointTrajectory, named to match
                         moveit_controllers.yaml. This is the only interface
                         MoveIt needs; it does not require ros2_control.
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

from canbus_stepper import CANBus

SERIAL_PORT = "/dev/ttyACM0"
ACTION_NAME = "arm_controller"   # must match moveit_controllers.yaml
STATE_RATE_HZ = 30.0

# Homing creep (§5 step 6) must be slow; flashed operating speed (§3) is not
# safe for an unsupervised move of unknown distance. Restored after homing.
HOMING_SPEED_ARM_DEG_S = 100.0
HOMING_SPEED_WRIST_DEG_S = 100.0
NORMAL_SPEED_ARM_DEG_S = 1600.0   # must match the flashed value in §3
NORMAL_SPEED_WRIST_DEG_S = 6400.0
HOMING_SETTLE_S = 3.0             # crude wait for the homing creep to finish

# ---------------------------------------------------------------------------
# 1. CONFIG - the only place signs/offsets/limits should be edited.
# ---------------------------------------------------------------------------

# J1-J4: simple geared joints. motor_deg = joint_deg * ratio * dir + home_deg
# All values from ROBOT_ARM_HANDOFF.md §3.
JOINTS = {
    # dir verified 2026-08-11: physical moved opposite RViz at dir=-1, flipped to +1.
    "revolute_1": {"node_id": 1, "ratio": 27.0, "dir": 1,
                   "home_deg": -52.65, "min_rad": math.radians(-180), "max_rad": math.radians(180)},
    # dir verified 2026-08-11: physical moved opposite RViz at dir=-1, flipped to +1.
    "revolute_2": {"node_id": 2, "ratio": 27.0, "dir": 1,
                   "home_deg": 236.81, "min_rad": math.radians(-90), "max_rad": math.radians(90)},
    # dir verified 2026-08-11: physical moved opposite RViz at dir=-1, flipped to +1.
    # limits verified 2026-08-11: physical range is -45deg to +405deg, not symmetric about home.
    "revolute_3": {"node_id": 3, "ratio": 27.0, "dir": 1,
                   "home_deg": 180.22, "min_rad": math.radians(-45), "max_rad": math.radians(405)},
    # dir verified 2026-08-11: matched RViz at dir=+1, no change needed.
    "revolute_4": {"node_id": 4, "ratio": 5.0, "dir": 1,
                   "home_deg": 35.44, "min_rad": math.radians(-180), "max_rad": math.radians(180)},
}

# J5/J6: differential wrist, no 1:1 joint mapping - see §6.
# revolute_5 = pitch (joint radians), revolute_6 = roll (joint radians).
# s5/s6 are placeholders - NOT yet verified, see §8.
WRIST = {
    "node5_id": 5, "node6_id": 6,
    "home5_deg": 138.83, "home6_deg": -35.89,
    "R": 6.0,
    "s5": 1, "s6": -1,  # verified 2026-08-11: same-direction motor motion = roll, not pitch
    "pitch_limit_rad": math.radians(90),
    "roll_limit_rad": math.radians(180),
}

ALL_JOINTS = ["revolute_1", "revolute_2", "revolute_3",
              "revolute_4", "revolute_5", "revolute_6"]


# ---------------------------------------------------------------------------
# 2. CONVERSIONS
# ---------------------------------------------------------------------------

def wrap_deg(delta_deg):
    """Normalize a degree difference to the shortest path across the ±360 wrap."""
    return ((delta_deg + 180) % 360) - 180


def joint_to_motor_deg(cfg, joint_rad):
    return math.degrees(joint_rad) * cfg["ratio"] * cfg["dir"] + cfg["home_deg"]


def motor_to_joint_rad(cfg, motor_deg):
    delta = wrap_deg(motor_deg - cfg["home_deg"])
    return math.radians(delta / (cfg["ratio"] * cfg["dir"]))


def wrist_forward(m5_deg, m6_deg):
    """Motor degrees -> (pitch_rad, roll_rad)."""
    d5 = wrap_deg(m5_deg - WRIST["home5_deg"]) * WRIST["s5"]
    d6 = wrap_deg(m6_deg - WRIST["home6_deg"]) * WRIST["s6"]
    pitch = math.radians((d5 + d6) / (2 * WRIST["R"]))
    roll = math.radians((d5 - d6) / (2 * WRIST["R"]))
    return pitch, roll


def wrist_inverse(pitch_rad, roll_rad):
    """(pitch_rad, roll_rad) -> (m5_deg, m6_deg)."""
    pitch_deg = math.degrees(pitch_rad)
    roll_deg = math.degrees(roll_rad)
    m5 = WRIST["home5_deg"] + (pitch_deg + roll_deg) * WRIST["R"] / WRIST["s5"]
    m6 = WRIST["home6_deg"] + (pitch_deg - roll_deg) * WRIST["R"] / WRIST["s6"]
    return m5, m6


class CanbusBridge(Node):
    def __init__(self):
        super().__init__("canbus_bridge")
        self.bus = CANBus(SERIAL_PORT)
        self.bus_lock = threading.Lock()

        self.nodes = {name: self.bus.node(cfg["node_id"]) for name, cfg in JOINTS.items()}
        self.node5 = self.bus.node(WRIST["node5_id"])
        self.node6 = self.bus.node(WRIST["node6_id"])

        self.run_startup_sequence()

        # /joint_states and the action server run on separate threads so a
        # long trajectory execution doesn't freeze the live position feed.
        state_cb_group = MutuallyExclusiveCallbackGroup()
        action_cb_group = ReentrantCallbackGroup()

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(
            1.0 / STATE_RATE_HZ, self.publish_states, callback_group=state_cb_group)

        self._action_server = ActionServer(
            self, FollowJointTrajectory, ACTION_NAME,
            execute_callback=self.execute_trajectory,
            goal_callback=self.handle_goal,
            cancel_callback=self.handle_cancel,
            callback_group=action_cb_group,
        )
        self.get_logger().info(
            f"canbus_bridge ready: '{ACTION_NAME}' action server + /joint_states")

    # ---------- 3. STARTUP SEQUENCE (§5) ----------
    def run_startup_sequence(self):
        self.get_logger().info("Startup: reading current encoder positions...")
        with self.bus_lock:
            current_deg = {name: self.nodes[name].get_angle() for name in JOINTS}
            m5_now = self.node5.get_angle()
            m6_now = self.node6.get_angle()

        self.get_logger().info("Startup: commanding zero-error hold at current pose...")
        with self.bus_lock:
            for name in JOINTS:
                self.nodes[name].set_position(current_deg[name])
            self.node5.set_position(m5_now)
            self.node6.set_position(m6_now)

        self.get_logger().info("Startup: enabling all joints (should not move)...")
        with self.bus_lock:
            for name in JOINTS:
                self.nodes[name].enable()
            self.node5.enable()
            self.node6.enable()
        time.sleep(0.5)  # let closed-loop hold settle before commanding motion

        self.get_logger().info("Startup: creeping to true home...")
        with self.bus_lock:
            for name, cfg in JOINTS.items():
                self.nodes[name].set_position_speed(HOMING_SPEED_ARM_DEG_S)
                self.nodes[name].set_position(cfg["home_deg"])
            self.node5.set_position_speed(HOMING_SPEED_WRIST_DEG_S)
            self.node6.set_position_speed(HOMING_SPEED_WRIST_DEG_S)
            self.node5.set_position(WRIST["home5_deg"])
            self.node6.set_position(WRIST["home6_deg"])
        time.sleep(HOMING_SETTLE_S)

        self.get_logger().info("Startup: restoring normal speed. Home reached, joint-space zero.")
        with self.bus_lock:
            for name in JOINTS:
                self.nodes[name].set_position_speed(NORMAL_SPEED_ARM_DEG_S)
            self.node5.set_position_speed(NORMAL_SPEED_WRIST_DEG_S)
            self.node6.set_position_speed(NORMAL_SPEED_WRIST_DEG_S)

    # ---------- 4. STATE PATH: motor degrees -> joint radians ----------
    def publish_states(self):
        try:
            with self.bus_lock:
                angles = {name: self.nodes[name].get_angle() for name in JOINTS}
                m5 = self.node5.get_angle()
                m6 = self.node6.get_angle()
        except TimeoutError as e:
            self.get_logger().warn(f"encoder read timed out: {e}", throttle_duration_sec=2.0)
            return

        positions = [motor_to_joint_rad(JOINTS[name], angles[name])
                     for name in ALL_JOINTS[:4]]
        pitch, roll = wrist_forward(m5, m6)
        positions += [pitch, roll]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ALL_JOINTS)
        msg.position = positions
        self.pub.publish(msg)

    # ---------- 5. ACTION SERVER: FollowJointTrajectory ----------
    def within_limits(self, positions):
        for name, cfg in JOINTS.items():
            if name in positions and not (cfg["min_rad"] <= positions[name] <= cfg["max_rad"]):
                return False
        if "revolute_5" in positions and abs(positions["revolute_5"]) > WRIST["pitch_limit_rad"]:
            return False
        if "revolute_6" in positions and abs(positions["revolute_6"]) > WRIST["roll_limit_rad"]:
            return False
        return True

    def handle_goal(self, goal_request):
        names = goal_request.trajectory.joint_names
        if not set(names) <= set(ALL_JOINTS):
            self.get_logger().warn(f"goal rejected: unknown joint(s) in {names}")
            return GoalResponse.REJECT
        for point in goal_request.trajectory.points:
            if not self.within_limits(dict(zip(names, point.positions))):
                self.get_logger().warn(f"goal rejected: waypoint exceeds joint limits")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def handle_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def send_joint_targets(self, positions):
        """positions: dict of joint name -> radians, for whichever joints are present."""
        with self.bus_lock:
            for name, cfg in JOINTS.items():
                if name in positions:
                    self.nodes[name].set_position(joint_to_motor_deg(cfg, positions[name]))
            if "revolute_5" in positions and "revolute_6" in positions:
                m5, m6 = wrist_inverse(positions["revolute_5"], positions["revolute_6"])
                self.node5.set_position(m5)
                self.node6.set_position(m6)

    def execute_trajectory(self, goal_handle):
        traj = goal_handle.request.trajectory
        names = traj.joint_names
        result = FollowJointTrajectory.Result()

        start_time = time.monotonic()
        for point in traj.points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_string = "canceled by client"
                return result

            target_t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            remaining = target_t - (time.monotonic() - start_time)
            if remaining > 0:
                time.sleep(remaining)

            try:
                self.send_joint_targets(dict(zip(names, point.positions)))
            except Exception as e:
                self.get_logger().error(f"trajectory execution failed: {e}")
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = str(e)
                return result

            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = names
            feedback.desired = point
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


def main():
    rclpy.init()
    node = CanbusBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.get_logger().info("Shutting down: disabling all joints...")
        with node.bus_lock:
            for n in node.nodes.values():
                n.disable()
            node.node5.disable()
            node.node6.disable()
        node.bus.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
