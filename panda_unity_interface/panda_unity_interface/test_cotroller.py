#!/usr/bin/env python3
"""
Franka/MoveIt2 Test Panel (ROS 2 + Tkinter UI)

This GUI publishes commands to your controller topics:
  - /safety_zone        (std_msgs/Int32) enum: 0 CLEAR, 1 WARNING, 2 DANGER
  - /emergency_stop     (std_msgs/Bool)
  - teach_mode          (std_msgs/Bool)
  - move_to_home        (std_msgs/Bool)
  - vr_controller_delta (geometry_msgs/Twist)
  - mission_command     (panda_unity_interface_msgs/Mission) with Waypoint(s)

It also subscribes to:
  - mission_execution   (panda_unity_interface_msgs/MissionStatus)

Run:
  1) Source your ROS 2 + workspace (so msgs are available)
  2) chmod +x franka_test_panel.py
  3) ./franka_test_panel.py

Notes:
  - Uses Tkinter only (stdlib) for UI.
  - Runs ROS spinning in a background thread.
"""

import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Pose, Twist

# Your custom msgs
from panda_unity_interface_msgs.msg import Mission, Waypoint, MissionStatus


ZONE_CLEAR = 0
ZONE_WARNING = 1
ZONE_DANGER = 2


def _safe_float(s: str, default: float = 0.0) -> float:
    try:
        return float(s)
    except Exception:
        return default


class RosBridge(Node):
    """ROS node that provides pubs/subs for the UI."""

    def __init__(self):
        super().__init__("franka_test_panel")

        # Publishers
        self.pub_safety_zone = self.create_publisher(Int32, "/safety_zone", 10)
        self.pub_emergency_stop = self.create_publisher(Bool, "/emergency_stop", 10)
        self.pub_teach_mode = self.create_publisher(Bool, "teach_mode", 10)
        self.pub_move_to_home = self.create_publisher(Bool, "move_to_home", 10)
        self.pub_delta = self.create_publisher(Twist, "vr_controller_delta", 10)
        self.pub_mission = self.create_publisher(Mission, "mission_command", 10)

        # Latest status snapshot
        self._status_lock = threading.Lock()
        self.latest_status = {
            "movement_started": False,
            "movement_completed": False,
            "movement_failed": False,
            "gripper_open_started": False,
            "gripper_open_completed": False,
            "gripper_close_started": False,
            "gripper_close_completed": False,
            "stamp": time.time(),
        }

        # Subscriber
        self.sub_status = self.create_subscription(
            MissionStatus, "mission_execution", self._status_cb, 10
        )

    def _status_cb(self, msg: MissionStatus):
        with self._status_lock:
            self.latest_status = {
                "movement_started": bool(msg.movement_started.data),
                "movement_completed": bool(msg.movement_completed.data),
                "movement_failed": bool(msg.movement_failed.data),
                "gripper_open_started": bool(msg.gripper_open_started.data),
                "gripper_open_completed": bool(msg.gripper_open_completed.data),
                "gripper_close_started": bool(msg.gripper_close_started.data),
                "gripper_close_completed": bool(msg.gripper_close_completed.data),
                "stamp": time.time(),
            }

    # ---- publish helpers ----
    def set_safety_zone(self, zone: int):
        m = Int32()
        m.data = int(zone)
        self.pub_safety_zone.publish(m)

    def set_emergency_stop(self, active: bool):
        m = Bool()
        m.data = bool(active)
        self.pub_emergency_stop.publish(m)

    def set_teach_mode(self, active: bool):
        m = Bool()
        m.data = bool(active)
        self.pub_teach_mode.publish(m)

    def trigger_move_to_home(self):
        m = Bool()
        m.data = True
        self.pub_move_to_home.publish(m)

    def send_twist(self, lx, ly, lz, ax, ay, az):
        t = Twist()
        t.linear.x = float(lx)
        t.linear.y = float(ly)
        t.linear.z = float(lz)
        t.angular.x = float(ax)
        t.angular.y = float(ay)
        t.angular.z = float(az)
        self.pub_delta.publish(t)

    def send_mission(self, waypoints):
        """waypoints: list[Waypoint]"""
        mission = Mission()
        mission.waypoints = waypoints
        self.pub_mission.publish(mission)

    def make_waypoint_pose(self, wp_type: int, pose: Pose) -> Waypoint:
        wp = Waypoint()
        wp.type = int(wp_type)
        wp.pose = pose
        return wp

    def make_waypoint_joints(self, joints) -> Waypoint:
        wp = Waypoint()
        wp.type = 4
        # waypoint.joints likely supports list/array assignment
        wp.joints = list(joints)
        return wp


class App(tk.Tk):
    def __init__(self, ros: RosBridge):
        super().__init__()
        self.title("Franka/MoveIt2 Test Panel")
        self.geometry("980x680")
        self.ros = ros

        # UI state
        self.emergency_var = tk.BooleanVar(value=False)
        self.teach_var = tk.BooleanVar(value=False)
        self.stream_twist_var = tk.BooleanVar(value=False)

        self._build_ui()

        # periodic UI refresh
        self.after(100, self._refresh_status)
        self.after(50, self._stream_twist_tick)

        # close handler
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        pad = {"padx": 10, "pady": 8}

        # ---- top row: safety + e-stop + teach + home ----
        top = ttk.Frame(self)
        top.pack(fill="x", **pad)

        safety_box = ttk.LabelFrame(top, text="Safety Zone (/safety_zone Int32 enum)")
        safety_box.pack(side="left", fill="both", expand=True, padx=6, pady=6)

        ttk.Button(
            safety_box, text="CLEAR (0)", command=lambda: self.ros.set_safety_zone(ZONE_CLEAR)
        ).grid(row=0, column=0, padx=6, pady=6, sticky="ew")
        ttk.Button(
            safety_box,
            text="WARNING (1)",
            command=lambda: self.ros.set_safety_zone(ZONE_WARNING),
        ).grid(row=0, column=1, padx=6, pady=6, sticky="ew")
        ttk.Button(
            safety_box,
            text="DANGER (2) STOP",
            command=lambda: self.ros.set_safety_zone(ZONE_DANGER),
        ).grid(row=0, column=2, padx=6, pady=6, sticky="ew")

        for c in range(3):
            safety_box.grid_columnconfigure(c, weight=1)

        ctrl_box = ttk.LabelFrame(top, text="Controller Toggles")
        ctrl_box.pack(side="left", fill="both", expand=True, padx=6, pady=6)

        ttk.Checkbutton(
            ctrl_box,
            text="Emergency Stop (/emergency_stop)",
            variable=self.emergency_var,
            command=self._toggle_emergency,
        ).grid(row=0, column=0, padx=6, pady=6, sticky="w")

        ttk.Checkbutton(
            ctrl_box,
            text="Teach Mode (servo) (teach_mode)",
            variable=self.teach_var,
            command=self._toggle_teach,
        ).grid(row=1, column=0, padx=6, pady=6, sticky="w")

        ttk.Button(
            ctrl_box, text="Move to Home (move_to_home)", command=self.ros.trigger_move_to_home
        ).grid(row=2, column=0, padx=6, pady=6, sticky="ew")

        ctrl_box.grid_columnconfigure(0, weight=1)

        # ---- middle row: mission commands ----
        mid = ttk.Frame(self)
        mid.pack(fill="both", expand=True, **pad)

        # Pose command panel
        pose_box = ttk.LabelFrame(mid, text="Mission: MoveJ/MoveL to Pose (mission_command)")
        pose_box.pack(side="left", fill="both", expand=True, padx=6, pady=6)

        self.pose_entries = {}
        fields = [
            ("x", "0.40"), ("y", "0.00"), ("z", "0.40"),
            ("qx", "1.0"), ("qy", "0.0"), ("qz", "0.0"), ("qw", "0.0"),
        ]
        for i, (k, default) in enumerate(fields):
            ttk.Label(pose_box, text=k).grid(row=i, column=0, padx=6, pady=4, sticky="e")
            e = ttk.Entry(pose_box, width=12)
            e.insert(0, default)
            e.grid(row=i, column=1, padx=6, pady=4, sticky="w")
            self.pose_entries[k] = e

        ttk.Button(pose_box, text="Send MoveJ (type 0)", command=self._send_movej).grid(
            row=0, column=2, padx=10, pady=6, sticky="ew"
        )
        ttk.Button(pose_box, text="Send MoveL (type 1)", command=self._send_movel).grid(
            row=1, column=2, padx=10, pady=6, sticky="ew"
        )

        ttk.Separator(pose_box, orient="horizontal").grid(
            row=7, column=0, columnspan=3, sticky="ew", padx=6, pady=10
        )

        ttk.Button(pose_box, text="Gripper OPEN (type 2)", command=self._send_open).grid(
            row=8, column=0, columnspan=3, padx=10, pady=6, sticky="ew"
        )
        ttk.Button(pose_box, text="Gripper CLOSE (type 3)", command=self._send_close).grid(
            row=9, column=0, columnspan=3, padx=10, pady=6, sticky="ew"
        )

        pose_box.grid_columnconfigure(2, weight=1)

        # Joint command panel
        joints_box = ttk.LabelFrame(mid, text="Mission: JointMove (type 4)")
        joints_box.pack(side="left", fill="both", expand=True, padx=6, pady=6)

        self.joint_entries = []
        default_j = ["0.0", "-0.785", "0.0", "-2.356", "0.0", "1.571", "0.785"]
        for i in range(7):
            ttk.Label(joints_box, text=f"j{i+1}").grid(row=i, column=0, padx=6, pady=4, sticky="e")
            e = ttk.Entry(joints_box, width=14)
            e.insert(0, default_j[i])
            e.grid(row=i, column=1, padx=6, pady=4, sticky="w")
            self.joint_entries.append(e)

        ttk.Button(joints_box, text="Send JointMove (type 4)", command=self._send_jointmove).grid(
            row=7, column=0, columnspan=2, padx=10, pady=10, sticky="ew"
        )

        # Twist/VR panel
        twist_box = ttk.LabelFrame(mid, text="VR Delta Twist (vr_controller_delta) (works in Teach Mode)")
        twist_box.pack(side="left", fill="both", expand=True, padx=6, pady=6)

        self.twist_entries = {}
        tfields = [
            ("lx", "0.0"), ("ly", "0.0"), ("lz", "0.0"),
            ("ax", "0.0"), ("ay", "0.0"), ("az", "0.0"),
        ]
        for i, (k, default) in enumerate(tfields):
            ttk.Label(twist_box, text=k).grid(row=i, column=0, padx=6, pady=4, sticky="e")
            e = ttk.Entry(twist_box, width=12)
            e.insert(0, default)
            e.grid(row=i, column=1, padx=6, pady=4, sticky="w")
            self.twist_entries[k] = e

        ttk.Button(twist_box, text="Send Twist Once", command=self._send_twist_once).grid(
            row=0, column=2, padx=10, pady=6, sticky="ew"
        )

        ttk.Checkbutton(
            twist_box, text="Stream Twist @20Hz", variable=self.stream_twist_var
        ).grid(row=1, column=2, padx=10, pady=6, sticky="w")

        ttk.Label(
            twist_box,
            text="Tip: Turn Teach Mode ON first; streaming will stop when unchecked.",
            wraplength=220,
        ).grid(row=2, column=2, padx=10, pady=6, sticky="w")

        twist_box.grid_columnconfigure(2, weight=1)

        # ---- bottom row: status ----
        bottom = ttk.Frame(self)
        bottom.pack(fill="x", **pad)

        status_box = ttk.LabelFrame(bottom, text="Mission Status (mission_execution)")
        status_box.pack(fill="x", expand=True, padx=6, pady=6)

        self.status_labels = {}
        keys = [
            "movement_started", "movement_completed", "movement_failed",
            "gripper_open_started", "gripper_open_completed",
            "gripper_close_started", "gripper_close_completed",
        ]

        for i, k in enumerate(keys):
            ttk.Label(status_box, text=k + ":").grid(row=0, column=2*i, padx=6, pady=6, sticky="e")
            v = ttk.Label(status_box, text="—", width=5)
            v.grid(row=0, column=2*i + 1, padx=6, pady=6, sticky="w")
            self.status_labels[k] = v

        self.status_age = ttk.Label(status_box, text="last update: —")
        self.status_age.grid(row=1, column=0, columnspan=14, padx=6, pady=6, sticky="w")

    # ---- UI actions ----
    def _toggle_emergency(self):
        active = bool(self.emergency_var.get())
        self.ros.set_emergency_stop(active)

    def _toggle_teach(self):
        active = bool(self.teach_var.get())
        self.ros.set_teach_mode(active)

    def _pose_from_entries(self) -> Pose:
        p = Pose()
        p.position.x = _safe_float(self.pose_entries["x"].get())
        p.position.y = _safe_float(self.pose_entries["y"].get())
        p.position.z = _safe_float(self.pose_entries["z"].get())
        p.orientation.x = _safe_float(self.pose_entries["qx"].get())
        p.orientation.y = _safe_float(self.pose_entries["qy"].get())
        p.orientation.z = _safe_float(self.pose_entries["qz"].get())
        p.orientation.w = _safe_float(self.pose_entries["qw"].get())
        return p

    def _send_movej(self):
        pose = self._pose_from_entries()
        wp = self.ros.make_waypoint_pose(0, pose)
        self.ros.send_mission([wp])

    def _send_movel(self):
        pose = self._pose_from_entries()
        wp = self.ros.make_waypoint_pose(1, pose)
        self.ros.send_mission([wp])

    def _send_open(self):
        wp = Waypoint()
        wp.type = 2
        self.ros.send_mission([wp])

    def _send_close(self):
        wp = Waypoint()
        wp.type = 3
        self.ros.send_mission([wp])

    def _send_jointmove(self):
        joints = [_safe_float(e.get()) for e in self.joint_entries]
        if len(joints) != 7:
            messagebox.showerror("JointMove", "Expected exactly 7 joints.")
            return
        wp = self.ros.make_waypoint_joints(joints)
        self.ros.send_mission([wp])

    def _twist_vals(self):
        lx = _safe_float(self.twist_entries["lx"].get())
        ly = _safe_float(self.twist_entries["ly"].get())
        lz = _safe_float(self.twist_entries["lz"].get())
        ax = _safe_float(self.twist_entries["ax"].get())
        ay = _safe_float(self.twist_entries["ay"].get())
        az = _safe_float(self.twist_entries["az"].get())
        return lx, ly, lz, ax, ay, az

    def _send_twist_once(self):
        self.ros.send_twist(*self._twist_vals())

    # ---- periodic loops ----
    def _refresh_status(self):
        with self.ros._status_lock:
            st = dict(self.ros.latest_status)

        for k, lab in self.status_labels.items():
            lab.config(text="1" if st.get(k, False) else "0")

        age = time.time() - float(st.get("stamp", time.time()))
        self.status_age.config(text=f"last update: {age:.2f}s ago")

        self.after(100, self._refresh_status)

    def _stream_twist_tick(self):
        if self.stream_twist_var.get():
            self.ros.send_twist(*self._twist_vals())
        self.after(50, self._stream_twist_tick)  # 20 Hz

    def _on_close(self):
        # Ensure streaming is off and teach is off (optional)
        try:
            self.stream_twist_var.set(False)
            # Don't force change emergency stop; user may want it ON
            self.teach_var.set(False)
            self.ros.set_teach_mode(False)
        except Exception:
            pass
        self.destroy()


def main():
    rclpy.init()

    ros = RosBridge()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(ros)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = App(ros)
    try:
        app.mainloop()
    finally:
        executor.shutdown()
        ros.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
