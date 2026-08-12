#!/usr/bin/env python3
"""
Companion waypoint-sequencer GUI for the robot arm.

Reuses real MoveIt/RViz infrastructure instead of reimplementing anything:
 - Capture: RViz's MotionPlanning panel already publishes /display_planned_path
   every time you click "Plan" there (after dragging the marker). This window
   just listens for that and grabs the final joint values as a waypoint.
 - Run: each waypoint is sent to move_group's own /move_action action (the
   exact same interface RViz's "Plan & Execute" button uses) with plan_only
   set to False and a per-waypoint velocity scaling, so it plans fresh from
   wherever the arm actually is and executes through the normal pipeline -
   same collision checking and joint limits as any other Plan+Execute.

Run this in its own terminal (or launched automatically by demo.launch.py)
alongside RViz - it does not replace it.
"""
import json
import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, simpledialog, ttk

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, DisplayTrajectory, JointConstraint

GROUP_NAME = "arm"
SEQUENCES_DIR = Path.home() / "ros2_ws/src/arm_driver/sequences"
JOINT_TOLERANCE = 0.001
DEFAULT_PLANNING_TIME = 5.0
DUPLICATE_CAPTURE_TOLERANCE = 1e-4
MAX_ATTEMPTS_PER_WAYPOINT = 3  # independent MoveGroup calls - OMPL is randomized,
                                # so a flaky failure is likely to clear on retry
RETRY_DELAY_S = 3.0  # give a still-in-flight previous attempt time to actually finish
                      # before retrying, so we don't hammer a driver that's still busy


def _positions_equal(a: dict, b: dict) -> bool:
    if a.keys() != b.keys():
        return False
    return all(abs(a[k] - b[k]) < DUPLICATE_CAPTURE_TOLERANCE for k in a)


# ---------------------------------------------------------------------------
# ROS side: capture the latest planned goal, run/cancel sequences via
# move_group's own MoveGroup action - no custom motion logic of our own.
# ---------------------------------------------------------------------------
class WaypointGuiNode(Node):
    def __init__(self, event_queue: queue.Queue):
        super().__init__("waypoint_gui")
        self.event_queue = event_queue
        self.latest_plan = None  # dict: joint_name -> position (radians)
        self._goal_handle = None
        self._goal_lock = threading.Lock()

        sub_cb_group = MutuallyExclusiveCallbackGroup()
        action_cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            DisplayTrajectory, "/display_planned_path",
            self._on_display_trajectory, 10, callback_group=sub_cb_group)

        self._action_client = ActionClient(
            self, MoveGroup, "/move_action", callback_group=action_cb_group)

    def _on_display_trajectory(self, msg: DisplayTrajectory):
        if not msg.trajectory:
            return
        joint_traj = msg.trajectory[0].joint_trajectory
        if not joint_traj.points:
            return
        last_point = joint_traj.points[-1]
        self.latest_plan = dict(zip(joint_traj.joint_names, last_point.positions))
        self.event_queue.put(("plan_captured", dict(self.latest_plan)))

    def run_waypoint(self, positions: dict, velocity_scaling: float) -> bool:
        """Blocking: send one waypoint through MoveGroup (plan+execute), wait
        for the result. Safe to call from a worker thread. Returns success."""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = GROUP_NAME
        goal_msg.request.num_planning_attempts = 3
        goal_msg.request.allowed_planning_time = DEFAULT_PLANNING_TIME
        goal_msg.request.max_velocity_scaling_factor = velocity_scaling
        goal_msg.request.max_acceleration_scaling_factor = velocity_scaling

        constraints = Constraints()
        for name, value in positions.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = JOINT_TOLERANCE
            jc.tolerance_below = JOINT_TOLERANCE
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal_msg.request.goal_constraints = [constraints]
        goal_msg.planning_options.plan_only = False

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.event_queue.put(("log", "move_group action server not available."))
            return False

        goal_accepted = threading.Event()
        result_ready = threading.Event()
        outcome = {"accepted": False, "success": False, "error": ""}

        def goal_response_cb(future):
            handle = future.result()
            outcome["accepted"] = handle.accepted
            with self._goal_lock:
                self._goal_handle = handle if handle.accepted else None
            goal_accepted.set()
            if not handle.accepted:
                result_ready.set()

        def result_cb(future):
            result = future.result().result
            outcome["success"] = (result.error_code.val == 1)  # SUCCESS = 1
            outcome["error"] = "" if outcome["success"] else f"error_code={result.error_code.val}"
            result_ready.set()

        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(goal_response_cb)

        if not goal_accepted.wait(timeout=10.0):
            self.event_queue.put(("log", "Timed out waiting for goal acceptance."))
            return False
        if not outcome["accepted"]:
            self.event_queue.put(("log", "Goal rejected by move_group."))
            return False

        with self._goal_lock:
            handle = self._goal_handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(result_cb)

        result_ready.wait()
        with self._goal_lock:
            self._goal_handle = None

        if not outcome["success"]:
            self.event_queue.put(("log", f"Waypoint failed: {outcome['error']}"))
        return outcome["success"]

    def cancel_current_goal(self):
        with self._goal_lock:
            handle = self._goal_handle
        if handle is not None:
            handle.cancel_goal_async()
            self.event_queue.put(("log", "Cancel requested."))


def ros_thread_main(node, executor):
    executor.add_node(node)
    try:
        executor.spin()
    except Exception:
        pass


# ---------------------------------------------------------------------------
# GUI side
# ---------------------------------------------------------------------------
class WaypointGuiApp:
    def __init__(self, root, node: WaypointGuiNode, event_queue: queue.Queue):
        self.root = root
        self.node = node
        self.event_queue = event_queue
        self.current_sequence = []  # list of dicts: positions, velocity_scaling, wait_after
        self.latest_captured = None
        self.running = False

        root.title("Robot Arm Waypoint Sequencer")
        self._build_ui()
        self._refresh_saved_sequences()
        self.root.after(100, self._poll_queue)

    # ---------- UI construction ----------
    def _build_ui(self):
        main = ttk.Frame(self.root, padding=8)
        main.grid(sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        # --- Current sequence table ---
        seq_frame = ttk.LabelFrame(main, text="Current Sequence", padding=6)
        seq_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 8))

        self.tree = ttk.Treeview(
            seq_frame, columns=("num", "vel", "wait"), show="headings", height=10)
        self.tree.heading("num", text="#")
        self.tree.heading("vel", text="Velocity scaling")
        self.tree.heading("wait", text="Wait after (s)")
        self.tree.column("num", width=40, anchor="center")
        self.tree.column("vel", width=110, anchor="center")
        self.tree.column("wait", width=100, anchor="center")
        self.tree.grid(row=0, column=0, columnspan=4, sticky="nsew")

        capture_frame = ttk.Frame(seq_frame)
        capture_frame.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(6, 0))

        self.vel_var = tk.DoubleVar(value=0.5)
        self.wait_var = tk.DoubleVar(value=1.0)
        ttk.Label(capture_frame, text="Velocity:").grid(row=0, column=0)
        ttk.Spinbox(capture_frame, from_=0.05, to=1.0, increment=0.05,
                    textvariable=self.vel_var, width=6).grid(row=0, column=1, padx=(2, 10))
        ttk.Label(capture_frame, text="Wait after (s):").grid(row=0, column=2)
        ttk.Spinbox(capture_frame, from_=0.0, to=60.0, increment=0.5,
                    textvariable=self.wait_var, width=6).grid(row=0, column=3, padx=(2, 10))

        ttk.Button(seq_frame, text="Capture Last Plan as Waypoint",
                   command=self._capture_waypoint).grid(row=2, column=0, columnspan=4, sticky="ew", pady=(6, 0))

        self.captured_label = ttk.Label(seq_frame, text="Last captured plan: none yet")
        self.captured_label.grid(row=3, column=0, columnspan=4, sticky="w", pady=(4, 0))

        btn_row = ttk.Frame(seq_frame)
        btn_row.grid(row=4, column=0, columnspan=4, sticky="ew", pady=(6, 0))
        ttk.Button(btn_row, text="Delete Selected", command=self._delete_selected).pack(side="left")
        ttk.Button(btn_row, text="Move Up", command=lambda: self._move_selected(-1)).pack(side="left", padx=4)
        ttk.Button(btn_row, text="Move Down", command=lambda: self._move_selected(1)).pack(side="left")
        ttk.Button(btn_row, text="Clear All", command=self._clear_sequence).pack(side="left", padx=4)

        run_row = ttk.Frame(seq_frame)
        run_row.grid(row=5, column=0, columnspan=4, sticky="ew", pady=(10, 0))
        self.run_button = ttk.Button(run_row, text="Run Sequence", command=self._run_sequence)
        self.run_button.pack(side="left", fill="x", expand=True)
        ttk.Button(run_row, text="STOP", command=self._stop).pack(side="left", padx=(6, 0))

        # --- Saved sequences ---
        saved_frame = ttk.LabelFrame(main, text="Saved Sequences", padding=6)
        saved_frame.grid(row=0, column=1, sticky="nsew")

        self.saved_listbox = tk.Listbox(saved_frame, height=12, exportselection=False)
        self.saved_listbox.grid(row=0, column=0, columnspan=2, sticky="nsew")

        ttk.Button(saved_frame, text="Load Selected",
                   command=self._load_selected_sequence).grid(row=1, column=0, sticky="ew", pady=(6, 0))
        ttk.Button(saved_frame, text="Delete Selected",
                   command=self._delete_saved_sequence).grid(row=1, column=1, sticky="ew", pady=(6, 0))
        ttk.Button(saved_frame, text="Save Current As...",
                   command=self._save_sequence_as).grid(row=2, column=0, columnspan=2, sticky="ew", pady=(4, 0))

        # --- Status log ---
        log_frame = ttk.LabelFrame(main, text="Status", padding=6)
        log_frame.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(8, 0))
        self.log_text = tk.Text(log_frame, height=8, state="disabled", wrap="word")
        self.log_text.grid(row=0, column=0, sticky="nsew")
        log_frame.columnconfigure(0, weight=1)

        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=0)
        main.rowconfigure(1, weight=1)
        seq_frame.rowconfigure(0, weight=1)

    # ---------- logging ----------
    def _log(self, text):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", text + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    # ---------- queue polling (ROS thread -> GUI thread) ----------
    def _poll_queue(self):
        try:
            while True:
                kind, payload = self.event_queue.get_nowait()
                if kind == "plan_captured":
                    self.latest_captured = payload
                    joints_str = ", ".join(f"{k}={v:.3f}" for k, v in payload.items())
                    self.captured_label.configure(text=f"Last captured plan: {joints_str}")
                    self._log(f"[{time.strftime('%H:%M:%S')}] New plan received from RViz: {joints_str}")
                elif kind == "log":
                    self._log(payload)
                elif kind == "sequence_done":
                    self._on_sequence_done(payload)
        except queue.Empty:
            pass
        self.root.after(100, self._poll_queue)

    # ---------- sequence editing ----------
    def _capture_waypoint(self):
        if self.latest_captured is None:
            messagebox.showwarning("No plan yet", "Drag the marker and click Plan in RViz first.")
            return
        if self.current_sequence and _positions_equal(
                self.current_sequence[-1]["positions"], self.latest_captured):
            if not messagebox.askyesno(
                    "Same as last waypoint",
                    "This capture is identical to the previous waypoint - did you forget to "
                    "drag the marker and click Plan again in RViz before capturing?\n\n"
                    "Add it anyway?"):
                return
        wp = {
            "positions": dict(self.latest_captured),
            "velocity_scaling": round(self.vel_var.get(), 2),
            "wait_after": round(self.wait_var.get(), 2),
        }
        self.current_sequence.append(wp)
        self._refresh_tree()
        joints_str = ", ".join(f"{k}={v:.3f}" for k, v in wp["positions"].items())
        self._log(f"[{time.strftime('%H:%M:%S')}] Captured as waypoint "
                   f"{len(self.current_sequence)}: {joints_str}")

    def _refresh_tree(self):
        self.tree.delete(*self.tree.get_children())
        for i, wp in enumerate(self.current_sequence):
            self.tree.insert("", "end", iid=str(i),
                              values=(i + 1, wp["velocity_scaling"], wp["wait_after"]))

    def _selected_index(self):
        sel = self.tree.selection()
        if not sel:
            return None
        return int(sel[0])

    def _delete_selected(self):
        idx = self._selected_index()
        if idx is None:
            return
        del self.current_sequence[idx]
        self._refresh_tree()

    def _move_selected(self, direction):
        idx = self._selected_index()
        if idx is None:
            return
        new_idx = idx + direction
        if not (0 <= new_idx < len(self.current_sequence)):
            return
        seq = self.current_sequence
        seq[idx], seq[new_idx] = seq[new_idx], seq[idx]
        self._refresh_tree()
        self.tree.selection_set(str(new_idx))

    def _clear_sequence(self):
        if self.current_sequence and not messagebox.askyesno(
                "Clear sequence", "Clear all waypoints in the current sequence?"):
            return
        self.current_sequence = []
        self._refresh_tree()

    # ---------- saved sequences ----------
    def _refresh_saved_sequences(self):
        SEQUENCES_DIR.mkdir(parents=True, exist_ok=True)
        self.saved_listbox.delete(0, "end")
        self._saved_files = {}
        for path in sorted(SEQUENCES_DIR.glob("*.json")):
            try:
                data = json.loads(path.read_text())
            except (json.JSONDecodeError, OSError):
                continue
            name = data.get("name", path.stem)
            self.saved_listbox.insert("end", name)
            self._saved_files[name] = path

    def _save_sequence_as(self):
        if not self.current_sequence:
            messagebox.showwarning("Nothing to save", "Capture at least one waypoint first.")
            return
        name = simpledialog.askstring("Save Sequence", "Name this sequence:")
        if not name:
            return
        safe_name = "".join(c if c.isalnum() or c in " _-" else "_" for c in name).strip()
        path = SEQUENCES_DIR / f"{safe_name}.json"
        data = {"name": name, "waypoints": self.current_sequence}
        SEQUENCES_DIR.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, indent=2))
        self._log(f"Saved sequence '{name}'.")
        self._refresh_saved_sequences()

    def _load_selected_sequence(self):
        sel = self.saved_listbox.curselection()
        if not sel:
            return
        name = self.saved_listbox.get(sel[0])
        path = self._saved_files.get(name)
        if not path:
            return
        data = json.loads(path.read_text())
        self.current_sequence = data.get("waypoints", [])
        self._refresh_tree()
        self._log(f"Loaded sequence '{name}' ({len(self.current_sequence)} waypoints).")

    def _delete_saved_sequence(self):
        sel = self.saved_listbox.curselection()
        if not sel:
            return
        name = self.saved_listbox.get(sel[0])
        path = self._saved_files.get(name)
        if not path:
            return
        if not messagebox.askyesno("Delete", f"Delete saved sequence '{name}'?"):
            return
        path.unlink(missing_ok=True)
        self._refresh_saved_sequences()

    # ---------- running ----------
    def _run_sequence(self):
        if self.running:
            return
        if not self.current_sequence:
            messagebox.showwarning("Empty sequence", "Add at least one waypoint first.")
            return
        if not messagebox.askyesno(
                "Run Sequence",
                f"Run this sequence of {len(self.current_sequence)} waypoints on the real robot now?"):
            return
        self.running = True
        self.run_button.configure(state="disabled")
        self._log(f"Running sequence of {len(self.current_sequence)} waypoints...")
        threading.Thread(target=self._run_sequence_worker, daemon=True).start()

    def _run_sequence_worker(self):
        sequence = list(self.current_sequence)
        for i, wp in enumerate(sequence):
            if not self.running:
                self.event_queue.put(("log", "Sequence stopped."))
                break
            success = False
            for attempt in range(1, MAX_ATTEMPTS_PER_WAYPOINT + 1):
                if not self.running:
                    break
                if attempt > 1:
                    time.sleep(RETRY_DELAY_S)
                suffix = "" if attempt == 1 else f" (retry {attempt - 1}/{MAX_ATTEMPTS_PER_WAYPOINT - 1})"
                joints_str = ", ".join(f"{k}={v:.3f}" for k, v in wp["positions"].items())
                self.event_queue.put(
                    ("log", f"Planning + executing waypoint {i + 1}/{len(sequence)}...{suffix} "
                            f"target: {joints_str}"))
                success = self.node.run_waypoint(wp["positions"], wp["velocity_scaling"])
                if success:
                    break
            if not success:
                self.event_queue.put(
                    ("log", f"Waypoint {i + 1} failed after {MAX_ATTEMPTS_PER_WAYPOINT} attempts - "
                            f"stopping sequence."))
                break
            if not self.running:
                break
            wait_s = wp["wait_after"]
            if wait_s > 0:
                self.event_queue.put(("log", f"Waiting {wait_s}s..."))
                time.sleep(wait_s)
        else:
            self.event_queue.put(("log", "Sequence complete."))
        self.event_queue.put(("sequence_done", None))

    def _on_sequence_done(self, _payload):
        self.running = False
        self.run_button.configure(state="normal")

    def _stop(self):
        if self.running:
            self.running = False
        self.node.cancel_current_goal()
        self._log("Stop requested.")


def main():
    rclpy.init()
    event_queue = queue.Queue()
    node = WaypointGuiNode(event_queue)
    executor = MultiThreadedExecutor(num_threads=4)

    ros_thread = threading.Thread(target=ros_thread_main, args=(node, executor), daemon=True)
    ros_thread.start()

    root = tk.Tk()
    app = WaypointGuiApp(root, node, event_queue)
    try:
        root.mainloop()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
