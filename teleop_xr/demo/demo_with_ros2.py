"""
Modified demo/__main__.py with explicit ROS2 JointTrajectory publishing.

This version:
1. Keeps the original TUI and IK solving
2. Adds explicit ROS2 node for /joint_trajectory publishing
3. Publishes IK-solved angles directly to ROS2
4. NEW: TRIGGER button maps to GRIPPER open/close

Run: python -m teleop_xr.demo.demo_with_ros2 --mode ik --robot-class openarm

Trigger controls:
- Trigger pressed  = Gripper OPEN  (position 0.03)
- Trigger released = Gripper CLOSE (position 0.0)
"""

import logging
import time
import asyncio
import threading
import json
import sys
import os
import select
import numpy as np
import cv2
import tyro
from loguru import logger as loguru_logger
from dataclasses import dataclass, field
from typing import Any, Deque, Optional, Union, Dict, Literal, TYPE_CHECKING, cast
from collections import deque

from rich.console import Console, Group
from rich.live import Live
from rich.layout import Layout
from rich.panel import Panel
from rich.table import Table
from rich import box
from rich.text import Text

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from control_msgs.action import GripperCommand
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool
    from std_msgs.msg import Float64  # GRIPPER INTENT ECHO: publish the commanded gripper position so the recorder can log true intent (0.0 close / 0.12 open) instead of measured jaw gap
    from geometry_msgs.msg import PointStamped  # DEBUG: publish raw XR controller position so the debug recorder can see if the controller kept moving while the arm froze
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

from teleop_xr import Teleop
from teleop_xr.config import TeleopSettings
from teleop_xr.common_cli import CommonCLI
from teleop_xr.messages import XRState
from teleop_xr.camera_views import build_camera_views_config
from teleop_xr.ik_utils import ensure_ik_dependencies, list_robots_or_exit
from teleop_xr.events import (
    EventProcessor,
    ButtonEvent,
    ButtonEventType,
    XRButton,
)

if sys.platform != "win32":
    import termios as _termios
    import tty as _tty
else:
    _termios = None
    _tty = None

if TYPE_CHECKING:
    from teleop_xr.ik.robot import BaseRobot
    from teleop_xr.ik.controller import IKController


MAX_EVENT_LOG_SIZE = 10

# Gripper position constants
GRIPPER_OPEN_POSITION = 0.12   # meters # this has been changing ;
GRIPPER_CLOSE_POSITION = 0.0   # meters
GRIPPER_MAX_EFFORT = 5.0       # Newtons


# ==================== ROS2 PUBLISHER NODE ====================

class IKJointTrajectoryPublisher(Node):
    """ROS2 node that publishes IK-solved joint trajectories AND gripper commands"""
    
    def __init__(self):
        super().__init__('ik_joint_trajectory_publisher')
        
        # Joint trajectory publisher
        self.joint_traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            10
        )
        
        # Gripper action clients
        self.left_gripper_client = ActionClient(
            self,
            GripperCommand,
            '/left_gripper_controller/gripper_cmd'
        )
        self.right_gripper_client = ActionClient(
            self,
            GripperCommand,
            '/right_gripper_controller/gripper_cmd'
        )
        
        # GRIPPER INTENT ECHO publishers.
        # ROS2 action goals travel over a service (_action/send_goal), NOT a topic,
        # so the lerobot recorder cannot subscribe to the GripperCommand goal directly.
        # We mirror the commanded position onto a plain topic here, at the source, so
        # the recorder can cache it and write it into the gripper slot of `action`.
        # This captures the true squeeze INTENT (0.0 = close/hold, 0.12 = open) rather
        # than the measured jaw gap from /joint_states (which stalls at the cube width
        # ~0.078 during a grasp and would teach the policy NOT to squeeze).
        self.left_gripper_cmd_echo_pub = self.create_publisher(
            Float64, '/left_gripper_cmd_echo', 10)
        self.right_gripper_cmd_echo_pub = self.create_publisher(
            Float64, '/right_gripper_cmd_echo', 10)
        
        # ✅ WAIT FOR GRIPPER SERVERS
        self.get_logger().info("Waiting for gripper action servers...")
        left_ready = self.left_gripper_client.wait_for_server(timeout_sec=10.0)
        right_ready = self.right_gripper_client.wait_for_server(timeout_sec=10.0)
        
        if left_ready:
            self.get_logger().info("✅ LEFT gripper action server READY")
        else:
            self.get_logger().error("❌ LEFT gripper action server NOT AVAILABLE")
        
        if right_ready:
            self.get_logger().info("✅ RIGHT gripper action server READY")
        else:
            self.get_logger().error("❌ RIGHT gripper action server NOT AVAILABLE")


        # Track gripper state to avoid duplicate commands
        self._last_left_gripper_pos = None
        self._last_right_gripper_pos = None

        # GRIPPER DEBOUNCE (2026-06-15): a floating/bouncing Quest trigger fires
        # many rapid open/close edges — seen as bursts of 4+ action goals within
        # <1 s AND the IDLE controller's gripper firing on its own. That flood of
        # GripperCommand goals overloads the controller_manager and stutters the
        # 150 Hz arm loop (the "freeze/lag right after closing the gripper").
        # We rate-limit goals to one per _GRIP_MIN_INTERVAL per side and always
        # converge to the LATEST desired position via flush_grippers().
        self._grip_desired = {"left": GRIPPER_CLOSE_POSITION,
                              "right": GRIPPER_CLOSE_POSITION}
        self._grip_last_send = {"left": 0.0, "right": 0.0}
        self._GRIP_MIN_INTERVAL = 0.2  # s -> at most 5 gripper goals/s/side

        self.joint_names = []
        self.get_logger().info("[ROS2] JointTrajectory publisher initialized on /joint_trajectory")
        self.get_logger().info("[ROS2] Gripper action clients initialized")

        self.head_image_pub = self.create_publisher(
            Image, '/camera/head/image_raw', 10)
        self.left_wrist_image_pub = self.create_publisher(
            Image, '/camera/left_wrist/image_raw', 10)
        self.right_wrist_image_pub = self.create_publisher(
            Image, '/camera/right_wrist/image_raw', 10)
        self._image_target_size = (640, 480)  # (W, H) — lerobot recorder expects 640x480
        self.get_logger().info(
            "[ROS2] Image publishers ready: /camera/{head,left_wrist,right_wrist}/image_raw @ 640x480 rgb8"
        )

        self._reset_callback = None
        self.create_subscription(Bool, '/teleop_xr/reset', self._cb_reset, 1)

        # DEBUG TELEMETRY (2026-06-15): expose internal teleop state on plain
        # topics so lerobot_recorder_debug.py can time-align it with the arm and
        # answer, at the gripper-close freeze: did the XR controller keep moving?
        # was IK still engaged? did the command (/joint_trajectory) keep moving
        # while the arm (/joint_states) froze, or did everything stop?
        #   /teleop_xr/ik_active     Bool   — IKController.active (engaged or not)
        #   /teleop_xr/xr_{l,r}_pos  PointStamped — raw XR controller position
        self.ik_active_pub = self.create_publisher(Bool, '/teleop_xr/ik_active', 10)
        self.xr_pos_pub = {
            "left":  self.create_publisher(PointStamped, '/teleop_xr/xr_left_pos', 10),
            "right": self.create_publisher(PointStamped, '/teleop_xr/xr_right_pos', 10),
        }
        #   /teleop_xr/ik_target_{l,r} — the teleop TARGET EE position the solver
        #   is asked to reach. If this MOVES at the freeze but joints don't, the
        #   solver is pinned; if it's FROZEN, it's input/snapshot.
        self.ik_target_pub = {
            "left":  self.create_publisher(PointStamped, '/teleop_xr/ik_target_left', 10),
            "right": self.create_publisher(PointStamped, '/teleop_xr/ik_target_right', 10),
        }

    def publish_ik_target(self, target_xyz: dict) -> None:
        """Publish the last teleop target EE translations (debug only)."""
        try:
            stamp = self.get_clock().now().to_msg()
            for side, xyz in target_xyz.items():
                if side not in self.ik_target_pub or xyz is None:
                    continue
                msg = PointStamped()
                msg.header.stamp = stamp
                msg.point.x = float(xyz[0])
                msg.point.y = float(xyz[1])
                msg.point.z = float(xyz[2])
                self.ik_target_pub[side].publish(msg)
        except Exception:
            pass

    def publish_ik_debug(self, active: bool, xr_state) -> None:
        """Publish IK-engaged flag + raw XR controller positions (debug only)."""
        try:
            self.ik_active_pub.publish(Bool(data=bool(active)))
            stamp = self.get_clock().now().to_msg()
            for dev in xr_state.devices:
                role = dev.role.value if dev.role else None
                hand = dev.handedness.value if dev.handedness else None
                if role != "controller" or hand not in ("left", "right"):
                    continue
                pose = dev.gripPose or dev.pose
                if not pose:
                    continue
                msg = PointStamped()
                msg.header.stamp = stamp
                msg.point.x = float(pose.position.get("x", 0.0))
                msg.point.y = float(pose.position.get("y", 0.0))
                msg.point.z = float(pose.position.get("z", 0.0))
                self.xr_pos_pub[hand].publish(msg)
        except Exception:
            pass

    def _cb_reset(self, _msg: "Bool") -> None:
        if self._reset_callback:
            self._reset_callback()
    
    def publish_trajectory(self, joint_names: list[str], q_solved: np.ndarray):
        """Publish solved joint angles as JointTrajectory"""
        msg = JointTrajectory()

        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = "world"
        # msg.joint_names = joint_names

        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = "world"
        msg.joint_names = joint_names

        
        point = JointTrajectoryPoint()
        point.positions = [float(angle) for angle in q_solved]
        point.time_from_start.sec = 0
        # 10 ms — just longer than the ~7ms publish period.
        # Keeps JTC lag minimal so on release there is barely any queued motion.
        point.time_from_start.nanosec = 10_000_000
        msg.points = [point]
        self.joint_traj_pub.publish(msg)

    def publish_trajectory_timed(self, joint_names: list[str], q_target: np.ndarray,
                                 duration_sec: float = 2.5):
        """One-shot timed move to q_target. For reset/home — NOT for streaming IK.

        Unlike publish_trajectory (10 ms point, meant for per-frame IK streaming),
        this gives JTC a real duration so it splines smoothly from the current
        pose to the target instead of snapping. Use for reset to avoid the
        high-jerk lurch that torques the arm and its mount.
        """
        msg = JointTrajectory()
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = "world"
        msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(angle) for angle in q_target]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        msg.points = [point]
        self.joint_traj_pub.publish(msg)

    def send_gripper_command(self, side: str, position: float):
        """
        Record the DESIRED gripper position and send it, rate-limited.

        Args:
            side: "left" or "right"
            position: 0.0 (closed) to 0.12 (open) in meters

        Rate-limiting (see __init__ GRIPPER DEBOUNCE): a bouncing trigger used to
        flood the controller; now at most one goal per _GRIP_MIN_INTERVAL/side is
        sent. Deferred commands are flushed by flush_grippers() so the gripper
        still converges to the final commanded state.
        """
        self._grip_desired[side] = position
        self._maybe_send_gripper(side)

    def _maybe_send_gripper(self, side: str) -> None:
        position = self._grip_desired[side]
        last = (self._last_left_gripper_pos if side == "left"
                else self._last_right_gripper_pos)
        # Already at the desired position — nothing to send.
        if last is not None and abs(position - last) < 0.001:
            return
        now = time.monotonic()
        if now - self._grip_last_send[side] < self._GRIP_MIN_INTERVAL:
            return  # within the rate-limit window; flush_grippers() sends it later
        self._do_send_gripper(side, position, now)

    def flush_grippers(self) -> None:
        """Send any deferred gripper command once its rate-limit window passes.

        Call this regularly from the main loop. It makes a fast or bouncy toggle
        converge to the final commanded state (within ~_GRIP_MIN_INTERVAL)
        without ever flooding the controller_manager.
        """
        self._maybe_send_gripper("left")
        self._maybe_send_gripper("right")

    def _do_send_gripper(self, side: str, position: float, now: float) -> None:
        if side == "left":
            self._last_left_gripper_pos = position
            client = self.left_gripper_client
        else:
            self._last_right_gripper_pos = position
            client = self.right_gripper_client
        self._grip_last_send[side] = now

        try:
            goal = GripperCommand.Goal()
            goal.command.position = float(position)
            goal.command.max_effort = GRIPPER_MAX_EFFORT

            # Send goal asynchronously
            client.send_goal_async(goal)
            self.get_logger().info(
                f"GRIPPER {side.upper()}: position={position:.3f}m"
            )

            # GRIPPER INTENT ECHO: mirror the commanded position onto a plain topic
            # so the lerobot recorder can log the true squeeze intent. Only fires
            # when a command actually goes out, in the same 0.0..0.12 space that
            # deploy commands.
            echo_pub = (self.left_gripper_cmd_echo_pub if side == "left"
                        else self.right_gripper_cmd_echo_pub)
            echo_pub.publish(Float64(data=float(position)))
        except Exception as e:
            self.get_logger().error(f"Gripper {side} error: {e}")

    def _publish_bgr(self, publisher, bgr_frame: np.ndarray, frame_id: str) -> None:
        """Downsample BGR frame to 640x480, convert to rgb8, publish as Image."""
        if bgr_frame is None or bgr_frame.size == 0:
            return
        h, w = bgr_frame.shape[:2]
        tw, th = self._image_target_size
        if (w, h) != (tw, th):
            bgr_frame = cv2.resize(bgr_frame, (tw, th), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.height = th
        msg.width = tw
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = tw * 3
        msg.data = rgb.tobytes()
        publisher.publish(msg)

    def publish_head_image(self, bgr_frame: np.ndarray) -> None:
        self._publish_bgr(self.head_image_pub, bgr_frame, 'head_camera')

    def publish_left_wrist_image(self, bgr_frame: np.ndarray) -> None:
        self._publish_bgr(self.left_wrist_image_pub, bgr_frame, 'left_wrist_camera')

    def publish_right_wrist_image(self, bgr_frame: np.ndarray) -> None:
        self._publish_bgr(self.right_wrist_image_pub, bgr_frame, 'right_wrist_camera')


# ==================== REST OF DEMO CODE ====================

@dataclass
class DemoCLI(CommonCLI):
    """CLI options for the unified TeleopXR demo."""

    mode: Literal["teleop", "ik"] = "teleop"
    head_device: Union[int, str, None] = None
    wrist_left_device: Union[int, str, None] = None
    wrist_right_device: Union[int, str, None] = None
    camera: Dict[str, Union[int, str]] = field(default_factory=dict)
    no_tui: bool = False
    robot_class: Optional[str] = None
    robot_args: str = "{}"
    list_robots: bool = False
    enable_events: bool = True


class TUIHandler(logging.Handler):
    def __init__(self, log_queue: Deque[str]):
        super().__init__()
        self.log_queue = log_queue
        self.formatter = logging.Formatter(
            "%(asctime)s - %(message)s", datefmt="%H:%M:%S"
        )

    def emit(self, record):
        try:
            msg = self.format(record)
            self.log_queue.append(msg)
        except Exception:
            self.handleError(record)


def generate_state_table(xr_state: Optional[XRState] = None) -> Table:
    table = Table(
        title="[bold cyan]XR Device State[/bold cyan]",
        box=box.ROUNDED,
        expand=True,
        title_justify="left",
    )
    table.add_column("Role", style="cyan", no_wrap=True, width=10)
    table.add_column("Hand", style="magenta", width=6)
    table.add_column("Position (x, y, z)", style="green", width=20)
    table.add_column("Orientation (x, y, z, w)", style="yellow", width=26)
    table.add_column("Inputs", style="blue")

    if not xr_state or not xr_state.devices:
        table.add_row("-", "-", "-", "-", "[dim]Waiting for data...[/dim]")
        return table

    devices = list(xr_state.devices)

    def sort_key(d):
        role = d.role.value if d.role else ""
        hand = d.handedness.value if d.handedness else ""
        priority = {"head": 0, "controller": 1, "hand": 2}
        hand_prio = {"left": 0, "right": 1, "none": 2}
        return (priority.get(role, 99), hand_prio.get(hand, 99))

    devices.sort(key=sort_key)

    for dev in devices:
        role = dev.role.value if dev.role else "unknown"
        hand = dev.handedness.value if dev.handedness else "none"

        pose = dev.pose or dev.gripPose
        if pose:
            pos = pose.position
            ort = pose.orientation
            pos_str = (
                f"{pos.get('x', 0):.2f}, {pos.get('y', 0):.2f}, {pos.get('z', 0):.2f}"
            )
            ort_str = f"{ort.get('x', 0):.2f}, {ort.get('y', 0):.2f}, {ort.get('z', 0):.2f}, {ort.get('w', 1):.2f}"
        else:
            pos_str = "-"
            ort_str = "-"

        inputs_parts = []
        if dev.gamepad:
            buttons = dev.gamepad.buttons
            axes = dev.gamepad.axes

            pressed = [i for i, b in enumerate(buttons) if b.pressed]
            if pressed:
                inputs_parts.append(f"Btn:{pressed}")

            active_axes = [f"{i}:{v:.1f}" for i, v in enumerate(axes) if abs(v) > 0.1]
            if active_axes:
                inputs_parts.append(f"Ax:{','.join(active_axes)}")

        if dev.joints:
            inputs_parts.append(f"{len(dev.joints)} joints")

        inputs_str = " | ".join(inputs_parts) if inputs_parts else "-"

        table.add_row(role, hand, pos_str, ort_str, inputs_str)

    return table


def generate_ik_status_table(
    active: bool,
    solve_time: float,
    parse_time: float,
    reload_status: str,
    reload_detail: str,
    xr_state: XRState | None,
    controller: "IKController",
    robot: "BaseRobot",
    current_q: np.ndarray,
    gripper_left_state: str = "CLOSED",
    gripper_right_state: str = "CLOSED",
) -> Panel:
    table = Table(box=box.ROUNDED, expand=True)
    table.add_column("Metric", style="cyan")
    table.add_column("Value", style="green")

    status_style = "bold green" if active else "dim yellow"
    status_text = "ACTIVE" if active else "IDLE (Hold Grip)"
    table.add_row("IK Status", f"[{status_style}]{status_text}[/{status_style}]")
    table.add_row("Solve Time", f"{solve_time * 1000:.2f} ms")
    table.add_row("Parse Time", f"{parse_time * 1000:.2f} ms")

    reload_style_map = {
        "ready": "dim",
        "reloading": "bold yellow",
        "done": "bold green",
        "failed": "bold red",
    }
    reload_style = reload_style_map.get(reload_status.lower(), "dim")
    table.add_row("Reload", f"[{reload_style}]{reload_detail}[/{reload_style}]")
    
    # ROS2 STATUS
    table.add_row("ROS2 Publish", "[bold green]Active OK[/bold green]")
    
    # Gripper status
    left_color = "bold green" if gripper_left_state == "OPEN" else "bold red"
    right_color = "bold green" if gripper_right_state == "OPEN" else "bold red"
    table.add_row("Left Gripper", f"[{left_color}]{gripper_left_state}[/{left_color}]")
    table.add_row("Right Gripper", f"[{right_color}]{gripper_right_state}[/{right_color}]")

    if active and xr_state:
        curr_poses = controller._get_device_poses(xr_state)
        snap_poses = controller.snapshot_xr

        import jax.numpy as jnp

        current_fk = robot.forward_kinematics(jnp.array(current_q))

        if "right" in curr_poses:
            t_ctrl_r = curr_poses["right"].translation()
            table.add_row(
                "Right Controller Pos",
                f"x={t_ctrl_r[0]:.3f} y={t_ctrl_r[1]:.3f} z={t_ctrl_r[2]:.3f}",
            )

        if "right" in current_fk:
            t_robot_r = current_fk["right"].translation()
            table.add_row(
                "Right Robot Hand Pos",
                f"x={t_robot_r[0]:.3f} y={t_robot_r[1]:.3f} z={t_robot_r[2]:.3f}",
            )

        table.add_section()

        for hand in ["left", "right"]:
            if hand in curr_poses and hand in snap_poses:
                t_curr = curr_poses[hand].translation()
                t_init = snap_poses[hand].translation()
                delta = t_curr - t_init
                table.add_row(
                    f"{hand.title()} Delta (XR)",
                    f"x={delta[0]:.3f} y={delta[1]:.3f} z={delta[2]:.3f}",
                )

    return Panel(table, title="[bold]IK Status[/bold]", border_style="blue")


def generate_ik_controls_panel() -> Panel:
    text = Text()
    text.append("- Hold ", style="dim")
    text.append("BOTH GRIPS", style="bold yellow")
    text.append(" to engage IK control\n", style="dim")
    text.append("- ", style="dim")
    text.append("TRIGGER", style="bold green")
    text.append(" -> Gripper OPEN/CLOSE\n", style="dim")
    text.append("- Double-click ", style="dim")
    text.append("DEADMAN (Grip)", style="bold magenta")
    text.append(" to reset joints\n", style="dim")
    text.append("- Press ", style="dim")
    text.append("R", style="bold cyan")
    text.append(" to reload robot class", style="dim")

    return Panel(
        text,
        title="[bold blue]IK Key Bindings[/bold blue]",
        title_align="left",
        box=box.ROUNDED,
        padding=(0, 1),
    )


def generate_log_panel(log_queue: Deque[str]) -> Panel:
    return Panel(
        Group(*[str(m) for m in list(log_queue)[-15:]]),
        title="[bold]Logs[/bold]",
        border_style="white",
        box=box.ROUNDED,
    )


class IKWorker(threading.Thread):
    """Dedicated worker thread for IK calculations + ROS2 publishing"""

    def __init__(
        self,
        controller: "IKController",
        robot: "BaseRobot",
        teleop: Teleop,
        state_container: dict[str, Any],
        logger: logging.Logger,
        ros2_node: Optional[IKJointTrajectoryPublisher] = None,
    ):
        super().__init__(daemon=True)
        self.controller = controller
        self.robot = robot
        self.teleop = teleop
        self.state_container = state_container
        self.logger = logger
        self.ros2_node = ros2_node
        self.latest_xr_state: Optional[XRState] = None
        self.new_state_event = threading.Event()
        self.running = True
        self.teleop_loop = None
        self._worker_lock = threading.Lock()

    def update_state(self, state: XRState):
        self.latest_xr_state = state
        self.new_state_event.set()

    def set_teleop_loop(self, loop: asyncio.AbstractEventLoop):
        if self.teleop_loop is None:
            self.teleop_loop = loop
            if "q" in self.state_container:
                joint_dict = {
                    name: float(val)
                    for name, val in zip(
                        self.robot.actuated_joint_names, self.state_container["q"]
                    )
                }
                asyncio.run_coroutine_threadsafe(
                    self.teleop.publish_joint_state(joint_dict),
                    self.teleop_loop,
                )
                if self.ros2_node:
                    self.ros2_node.publish_trajectory(
                        self.robot.actuated_joint_names,
                        self.state_container["q"]
                    )

    def run(self):
        while self.running:
            if not self.new_state_event.wait(timeout=0.1):
                continue

            self.new_state_event.clear()

            state = self.latest_xr_state
            if state is None:
                continue

            try:
                with self._worker_lock:
                    q_current = self.state_container["q"]
                    was_active = self.controller.active

                    t0 = time.perf_counter()
                    new_config = np.array(self.controller.step(state, q_current))
                    dt = time.perf_counter() - t0

                    self.state_container["solve_time"] = dt
                    self.state_container["active"] = self.controller.active
                    is_active = self.controller.active

                    # DEBUG: publish the teleop target EE position(s) the solver
                    # was asked to reach this step (freeze diagnosis).
                    if self.ros2_node and getattr(
                            self.controller, "last_target_xyz", None):
                        self.ros2_node.publish_ik_target(
                            self.controller.last_target_xyz)

                    if not was_active and is_active:
                        self.logger.info("in_control start - Taking Snapshots")
                        self.logger.info(
                            f"Init XR: {list(self.controller.snapshot_xr.keys())}"
                        )

                    if not np.array_equal(new_config, q_current):
                        self.state_container["q"] = new_config
                        joint_dict = {
                            name: float(val)
                            for name, val in zip(
                                self.robot.actuated_joint_names, new_config
                            )
                        }

                        if self.teleop_loop and self.teleop_loop.is_running():
                            asyncio.run_coroutine_threadsafe(
                                self.teleop.publish_joint_state(joint_dict),
                                self.teleop_loop,
                            )
                        
                        if self.ros2_node:
                            self.ros2_node.publish_trajectory(
                                self.robot.actuated_joint_names,
                                new_config
                            )

            except Exception as e:
                self.logger.error(f"Error in IK Worker: {e}")

    def reload_robot_in_place(
        self, replacement: "BaseRobot"
    ) -> tuple[bool, str, np.ndarray]:
        with self._worker_lock:
            current_q = np.array(self.state_container.get("q", np.array([])))
            default_q = np.array(replacement.get_default_config())
            if current_q.size > 0 and current_q.shape != default_q.shape:
                return (
                    False,
                    "Joint dimension changed; cannot patch robot in place without recreating solver",
                    current_q,
                )

            live_robot = self.robot
            live_robot.__class__ = replacement.__class__
            live_robot_state = cast(dict[str, Any], cast(object, live_robot.__dict__))
            replacement_state = cast(dict[str, Any], cast(object, replacement.__dict__))
            live_robot_state.clear()
            live_robot_state.update(replacement_state)

            self.robot = live_robot
            self.controller.robot = live_robot
            self.controller.reset()
            self.state_container["active"] = False

            q_next = default_q if current_q.size == 0 else current_q
            self.state_container["q"] = q_next
            return True, "Robot class reloaded in-place (solver preserved)", q_next


class TerminalKeyReader:
    def __init__(self, enabled: bool):
        self.enabled = enabled and sys.stdin.isatty() and sys.platform != "win32"
        self._fd: Optional[int] = None
        self._old_settings = None

    def __enter__(self):
        if not self.enabled:
            return self
        fd = sys.stdin.fileno()
        self._fd = fd
        assert _termios is not None
        assert _tty is not None
        self._old_settings = _termios.tcgetattr(fd)
        _tty.setcbreak(fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        fd = self._fd
        old_settings = self._old_settings
        if not self.enabled or fd is None or old_settings is None:
            return
        assert _termios is not None
        _termios.tcsetattr(fd, _termios.TCSADRAIN, old_settings)

    def poll_key(self) -> Optional[str]:
        fd = self._fd
        if not self.enabled or fd is None:
            return None
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return None
        data = os.read(fd, 1)
        if not data:
            return None
        return data.decode("utf-8", errors="ignore")


def main():
    cli = tyro.cli(DemoCLI)

    if cli.mode == "ik":
        ensure_ik_dependencies()

    if cli.list_robots:
        list_robots_or_exit()

    log_queue: Deque[str] = deque(maxlen=50)
    event_log: deque[ButtonEvent] = deque(maxlen=MAX_EVENT_LOG_SIZE)

    handlers = []
    if not cli.no_tui:
        handlers.append(TUIHandler(log_queue))
    else:
        handlers.append(logging.StreamHandler())

    logging.basicConfig(level=logging.INFO, handlers=handlers, force=True)
    logging.getLogger("jaxls").setLevel(logging.WARNING)

    loguru_logger.remove()
    if not cli.no_tui:

        def tui_sink(message):
            log_queue.append(message)

        loguru_logger.add(
            tui_sink, level="INFO", format="<green>{time:HH:mm:ss}</green> - {message}"
        )
    else:
        loguru_logger.add(sys.stderr, level="INFO")

    if not cli.no_tui:
        logging.getLogger("uvicorn.access").setLevel(logging.WARNING)
        logging.getLogger("uvicorn.error").setLevel(logging.WARNING)

    logger = logging.getLogger("demo")
    logger.info(
        f"Starting TeleopXR Demo in {cli.mode.upper()} mode on {cli.host}:{cli.port}"
    )

    # INITIALIZE ROS2
    ros2_node = None
    if cli.mode == "ik" and HAS_ROS2:
        try:
            rclpy.init()
            ros2_node = IKJointTrajectoryPublisher()
            logger.info("[ROS2] Initialized JointTrajectory publisher and gripper clients")
            # Wire camera frames -> ROS2 image publishers.
            # Callbacks run on the camera reader thread; rclpy publishers are
            # thread-safe and don't need spin() to publish.
            from teleop_xr.video_stream import register_frame_callback
            register_frame_callback("head",        ros2_node.publish_head_image)
            register_frame_callback("wrist_left",  ros2_node.publish_left_wrist_image)
            register_frame_callback("wrist_right", ros2_node.publish_right_wrist_image)
            logger.info("[ROS2] Camera callbacks registered (head / wrist_left / wrist_right)")
        except Exception as e:
            logger.warning(f"[ROS2] Failed to initialize: {e}")

    robot = None
    solver = None
    controller = None
    ik_worker = None
    robot_args: dict[str, Any] = {}
    state_container: dict[str, Any] = {
        "active": False,
        "solve_time": 0.0,
        "parse_time": 0.0,
        "reload_status": "ready",
        "reload_detail": "Ready (press R)",
        "xr_state": None,
        "gripper_left": "CLOSED",
        "gripper_right": "CLOSED",
    }

    camera_views = build_camera_views_config(
        head=cli.head_device,
        wrist_left=cli.wrist_left_device,
        wrist_right=cli.wrist_right_device,
        extra_streams=cli.camera,
    )

    robot_vis = None
    if cli.mode == "ik":
        try:
            from teleop_xr.ik.loader import load_robot_class
            from teleop_xr.ik.solver import PyrokiSolver
            from teleop_xr.ik.controller import IKController
        except ImportError as e:
            logger.error(
                f"IK dependencies not installed: {e}. Install with: pip install 'teleop-xr[ik]'"
            )
            sys.exit(1)

        robot_cls = load_robot_class(cli.robot_class)
        robot_args = json.loads(cli.robot_args)
        logger.info(f"Initializing {robot_cls.__name__} with args: {robot_args}")
        robot = robot_cls(**robot_args)
        solver = PyrokiSolver(robot)
        # 2-tap filter (~1 frame of latency). Weaker filter than the prior 3-tap
        # to reduce input lag — set to None entirely if you want raw IK output.
        ik_filter_weights = np.array([0.3, 0.7])
        controller = IKController(robot, solver, filter_weights=ik_filter_weights)
        state_container["q"] = np.array(robot.get_default_config())
        robot_vis = robot.get_vis_config()

    settings = TeleopSettings(
        host=cli.host,
        port=cli.port,
        robot_vis=robot_vis,
        input_mode=cli.input_mode,
        camera_views=camera_views,
        speed=robot.default_speed_ratio if robot else 1.0,
    )

    teleop = Teleop(settings=settings)
    teleop.set_pose(np.eye(4))

    def do_reset_pose() -> None:
        """Send both arms to their default config. Safe to call from any thread."""
        if not (cli.mode == "ik" and robot is not None and ik_worker is not None and controller is not None):
            logger.warning("Reset ignored: not in IK mode or components not ready")
            return
        default_q = np.array(robot.get_default_config())
        state_container["q"] = default_q
        controller.reset()
        if ik_worker.teleop_loop:
            joint_dict = {n: float(v) for n, v in zip(robot.actuated_joint_names, default_q)}
            asyncio.run_coroutine_threadsafe(
                teleop.publish_joint_state(joint_dict), ik_worker.teleop_loop)
            if ros2_node:
                # Timed move (2.5 s) instead of the 10 ms streaming point, so the
                # reset splines smoothly to home rather than snapping hard.
                # was: ros2_node.publish_trajectory(robot.actuated_joint_names, default_q)
                ros2_node.publish_trajectory_timed(robot.actuated_joint_names, default_q, duration_sec=2.5)
        logger.info("Robot reset to start pose")

    if ros2_node:
        ros2_node._reset_callback = do_reset_pose

    processor: Optional[EventProcessor] = None
    if cli.enable_events:
        processor = EventProcessor(cli.event_settings())

        def log_event(event: ButtonEvent):
            event_log.append(event)
            logger.info(
                f"Event: {event.type.value} on {event.button.value} ({event.controller.value})"
            )

        processor.on_button_down(callback=log_event)
        processor.on_button_up(callback=log_event)
        processor.on_double_press(callback=log_event)
        processor.on_long_press(callback=log_event)

        # ============================================================
        # GRIPPER TRIGGER MAPPING
        # ============================================================
        def on_trigger_down(event: ButtonEvent):
            """Trigger pressed -> Open gripper"""
            if event.button != XRButton.TRIGGER:
                return
            if cli.mode != "ik" or ros2_node is None:
                return

            # Real hardware command
            side = event.controller.value  # "left" or "right"
            ros2_node.send_gripper_command(side, GRIPPER_OPEN_POSITION)
            state_container[f"gripper_{side}"] = "OPEN"

            # Also update the ghost visualization.
            # RACE FIX (2026-06-15): mutate ONLY the finger slots IN PLACE. The old
            # code copied q, edited the finger, then wrote the WHOLE array back —
            # which clobbered the IK worker's concurrent arm-joint update (it does
            # state_container["q"] = new_config every cycle), snapping the arm back
            # to its trigger-time pose when the gripper was toggled mid-motion.
            # In-place finger writes can never revert the arm joints.
            if robot is not None and ik_worker is not None and ik_worker.teleop_loop:
                joint_names = robot.actuated_joint_names
                # URDF limit is 0.044, so clamp the viz value
                viz_value = min(GRIPPER_OPEN_POSITION, 0.044)
                q = state_container["q"]
                for i, name in enumerate(joint_names):
                    if f"{side}_finger_joint" in name:   # catches joint1 and joint2
                        q[i] = viz_value
                joint_dict = {n: float(v) for n, v in zip(joint_names, q)}
                asyncio.run_coroutine_threadsafe(
                    teleop.publish_joint_state(joint_dict),
                    ik_worker.teleop_loop,
                )

            logger.info(f"{side.upper()} gripper OPEN (trigger pressed)")
        
        def on_trigger_up(event: ButtonEvent):
            """Trigger released -> Close gripper"""
            if event.button != XRButton.TRIGGER:
                return
            if cli.mode != "ik" or ros2_node is None:
                return

            side = event.controller.value  # "left" or "right"
            ros2_node.send_gripper_command(side, GRIPPER_CLOSE_POSITION)
            state_container[f"gripper_{side}"] = "CLOSED"

            # Reset finger joint in viz state so IK trajectory reflects closed gripper.
            # RACE FIX (2026-06-15): mutate ONLY the finger slots in place (see
            # on_trigger_down). Writing the whole q array back here was clobbering
            # the IK worker's arm update and jerking the arm on the open->close
            # toggle — the exact "behaves oddly after open then close" symptom.
            if robot is not None and ik_worker is not None and ik_worker.teleop_loop:
                q = state_container["q"]
                for i, name in enumerate(robot.actuated_joint_names):
                    if f"{side}_finger_joint" in name:
                        q[i] = GRIPPER_CLOSE_POSITION

            logger.info(f"{side.upper()} gripper CLOSED (trigger released)")
        
        processor.on_button_down(button=XRButton.TRIGGER, callback=on_trigger_down)
        processor.on_button_up(button=XRButton.TRIGGER, callback=on_trigger_up)

        # Reset pose on squeeze double-click
        def on_reset_pose(event: ButtonEvent):
            if event.button == XRButton.SQUEEZE:
                do_reset_pose()

        # Disabled: double-press SQUEEZE reset snapped joints mid-task. Re-enable
        # by uncommenting the next line if you want joint reset back.
        # processor.on_double_press(button=XRButton.SQUEEZE, callback=on_reset_pose)

    if cli.mode == "ik" and controller and robot:
        ik_worker = IKWorker(
            controller, 
            robot, 
            teleop, 
            state_container, 
            logger,
            ros2_node=ros2_node
        )
        ik_worker.start()

    if controller is not None:
        teleop.bind_control_mode_provider(lambda: controller.get_mode().value)

    def on_xr_update(_pose: np.ndarray, message: dict[str, Any]):
        try:
            if ik_worker:
                try:
                    loop = asyncio.get_running_loop()
                    ik_worker.set_teleop_loop(loop)
                except RuntimeError:
                    pass

            t_parse_start = time.perf_counter()

            if processor:
                processor.process(_pose, message)

            xr_data = message.get("data", message)
            state = XRState.model_validate(xr_data)

            state_container["parse_time"] = time.perf_counter() - t_parse_start
            state_container["xr_state"] = state

            if ik_worker:
                ik_worker.update_state(state)

            # DEBUG TELEMETRY: mirror IK-engaged + XR controller positions onto
            # topics so the debug recorder can see input vs command vs arm at the
            # freeze. Runs at XR rate; rclpy publishers are thread-safe.
            if ros2_node is not None:
                ros2_node.publish_ik_debug(
                    state_container.get("active", False), state)
        except Exception:
            pass

    teleop.subscribe(on_xr_update)

    if cli.no_tui:
        loguru_logger.info("TUI Disabled. Running in headless mode.")
        try:
            teleop.run()
        except KeyboardInterrupt:
            pass
        finally:
            if ik_worker:
                ik_worker.running = False
                ik_worker.join()
            if ros2_node and rclpy.ok():
                rclpy.shutdown()
        return

    # --- TUI Loop ---
    console = Console()
    layout = Layout()

    if cli.mode == "ik":
        layout.split_row(Layout(name="left", ratio=1), Layout(name="right", ratio=1))
        layout["left"].split_column(
            Layout(name="state", ratio=2),
            Layout(name="logs", ratio=3),
        )
        layout["right"].split_column(
            Layout(name="status", ratio=2),
            Layout(name="controls", size=7),
        )
    else:
        layout.split_row(Layout(name="left", ratio=3), Layout(name="right", ratio=2))
        layout["right"].split_column(
            Layout(name="events", ratio=4), Layout(name="help", size=3)
        )

    try:
        t = threading.Thread(target=teleop.run, daemon=True)
        t.start()

        time.sleep(0.5)

        with TerminalKeyReader(enabled=True) as key_reader:
            with Live(layout, refresh_per_second=10, console=console):
                while t.is_alive():
                    if cli.mode == "ik":
                        layout["state"].update(
                            generate_state_table(state_container["xr_state"])
                        )
                    else:
                        layout["left"].update(
                            generate_state_table(state_container["xr_state"])
                        )

                    if cli.mode == "ik" and controller and robot:
                        layout["status"].update(
                            generate_ik_status_table(
                                state_container["active"],
                                state_container["solve_time"],
                                state_container["parse_time"],
                                state_container.get("reload_status", "ready"),
                                state_container.get("reload_detail", "Ready (press R)"),
                                state_container["xr_state"],
                                controller,
                                robot,
                                state_container.get("q", np.array([])),
                                state_container.get("gripper_left", "CLOSED"),
                                state_container.get("gripper_right", "CLOSED"),
                            )
                        )
                        layout["controls"].update(generate_ik_controls_panel())
                        layout["logs"].update(generate_log_panel(log_queue))

                    # Non-blocking: processes /teleop_xr/reset and any other
                    # incoming callbacks (fires at 10 Hz, negligible DDS load).
                    if ros2_node:
                        rclpy.spin_once(ros2_node, timeout_sec=0)
                        # Flush any gripper command deferred by the rate-limiter so
                        # a fast/bouncy toggle still reaches its final state.
                        ros2_node.flush_grippers()

                    key = key_reader.poll_key()
                    if key and key.lower() == 'r':
                        do_reset_pose()

                    time.sleep(0.1)

    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        if ik_worker:
            ik_worker.running = False
            ik_worker.join()
        if ros2_node and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
