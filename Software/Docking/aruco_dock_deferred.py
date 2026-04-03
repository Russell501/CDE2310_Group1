#!/usr/bin/env python3
"""
aruco_dock_deferred.py — Run on the LOCAL LAPTOP (not the RPi).

Deferred two-marker ArUco docking in two phases:

PHASE 1 — PASSIVE SCOUTING  (runs alongside frontier exploration)
  Continuously subscribes to /aruco/markers.  On the FIRST valid detection of
  each marker ID the node converts the camera-frame observation to a map-frame
  Nav2 goal and persists it to DOCK_POSES_FILE (YAML).  Both marker IDs are
  saved independently.

PHASE 2 — SEQUENTIAL DOCKING  (triggered after exploration completes)
  Runs two docking sequences back-to-back:

  Step 1 — STATIC DOCK (STATIC_MARKER_ID = 0)
    Navigate to saved pre-approach pose → cmd_vel fine-approach → docked.
    After success the robot reverses UNDOCK_DISTANCE to clear the dock.

  Step 2 — MOVING DOCK (MOVING_MARKER_ID = 1)
    Navigate to saved pre-approach pose → cmd_vel fine-approach → docked.
    After success waits for a fresh ArUco re-detection as a launch signal,
    then fires the launch sequence.

  Two ways to trigger Phase 2:
    (a) Publish  True  on  /start_docking  (std_msgs/Bool)
    (b) Pass  --dock-now  on the command line (uses whatever is in YAML)

Prerequisites:
  1. On RPi : aruco_live.py publishing visualization_msgs/MarkerArray on /aruco/markers
  2. On laptop: Full Nav2 stack (map_server, bt_navigator, planner, controller)
  3. On laptop: Localization via Cartographer (map→odom TF exists)
  4. On laptop:
       source /opt/ros/humble/setup.bash
       export ROS_DOMAIN_ID=30
  5. On laptop (if not installed):
       sudo apt install ros-humble-tf-transformations
     Or the inline fallback below handles it automatically.
"""

import argparse
import math
import os
import threading
import time

import yaml

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

try:
    from tf_transformations import euler_from_quaternion, quaternion_from_euler
except ImportError:
    # Pure-Python fallback — no extra packages required
    def euler_from_quaternion(q):
        x, y, z, w = q
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll  = math.atan2(t0, t1)
        t2    = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(t2)
        t3    = 2.0 * (w * z + x * y)
        t4    = 1.0 - 2.0 * (y * y + z * z)
        yaw   = math.atan2(t3, t4)
        return roll, pitch, yaw

    def quaternion_from_euler(roll, pitch, yaw):
        cy, sy = math.cos(yaw   * 0.5), math.sin(yaw   * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll  * 0.5), math.sin(roll  * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,  # x
            cr * sp * cy + sr * cp * sy,  # y
            cr * cp * sy - sr * sp * cy,  # z
            cr * cp * cy + sr * sp * sy,  # w
        )


# ---------------------------------------------------------------------------
# Tuning constants
# ---------------------------------------------------------------------------
TARGET_DISTANCE        = 0.10   # m — final docking distance (cmd_vel approach)
NAV2_APPROACH_DISTANCE = 0.35   # m — Nav2 goal distance; outside costmap inflation
LATERAL_TOLERANCE      = 0.01   # m — |cam_x| must be within this to succeed
DISTANCE_TOLERANCE     = 0.01   # m — |cam_z - TARGET_DISTANCE| to succeed
MAX_RETRIES            = 4      # retries per docking attempt

CAMERA_X_OFFSET        = 0.04   # m — forward distance base_link → camera lens

# Marker ID assignments
STATIC_MARKER_ID       = 0      # first dock  — static, no launch signal
MOVING_MARKER_ID       = 1      # second dock — moving, waits for launch signal

# After static docking, reverse this far before navigating to the moving dock
UNDOCK_DISTANCE        = 0.30   # m
UNDOCK_SPEED           = 0.06   # m/s

# How long to wait for the re-detection launch signal at the moving dock
LAUNCH_SIGNAL_TIMEOUT  = 30.0   # s

# Where to persist map-frame docking goals across node restarts
DOCK_POSES_FILE        = '/tmp/aruco_dock_poses.yaml'


# ---------------------------------------------------------------------------
# YAML helpers
# ---------------------------------------------------------------------------

def _pose_to_dict(ps: PoseStamped) -> dict:
    return {
        'x':  ps.pose.position.x,
        'y':  ps.pose.position.y,
        'qx': ps.pose.orientation.x,
        'qy': ps.pose.orientation.y,
        'qz': ps.pose.orientation.z,
        'qw': ps.pose.orientation.w,
    }


def _pose_from_dict(d: dict) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id    = 'map'
    ps.pose.position.x    = float(d['x'])
    ps.pose.position.y    = float(d['y'])
    ps.pose.position.z    = 0.0
    ps.pose.orientation.x = float(d['qx'])
    ps.pose.orientation.y = float(d['qy'])
    ps.pose.orientation.z = float(d['qz'])
    ps.pose.orientation.w = float(d['qw'])
    return ps


# ---------------------------------------------------------------------------
# MarkerScouter — passive Phase-1 node
# ---------------------------------------------------------------------------

class MarkerScouter(Node):
    """
    Runs throughout frontier exploration (Phase 1).

    • Subscribes to /aruco/markers.
        - On the FIRST valid detection of each marker ID, computes the
          map-frame Nav2 approach goal and saves it to memory + YAML.
        - Keeps latest_cam_x / latest_cam_z / latest_stamp current for
          the Phase-2 cmd_vel approach and launch-signal detection.
    • Subscribes to /start_docking (std_msgs/Bool) — sets dock_event
      when True arrives, waking the main thread for Phase 2.
    """

    def __init__(self, tf_buffer: tf2_ros.Buffer) -> None:
        super().__init__('aruco_marker_scouter')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(MarkerArray, '/aruco/markers', self._marker_cb, qos)
        self.create_subscription(Bool, '/start_docking', self._dock_cb, 10)

        self._tf_buffer        = tf_buffer
        self._saved_ids: set   = set()
        self.saved_poses: dict = {}             # {marker_id (int): PoseStamped}
        self.dock_event        = threading.Event()

        # Live readings — refreshed by every valid marker callback
        self.latest_cam_x: float = 0.0
        self.latest_cam_z: float = 0.0
        self.latest_stamp: float = 0.0   # time.monotonic() of last valid marker msg

        # Baseline for launch-signal detection (set via arm_launch_signal())
        self._launch_arm_time: float = -1.0

        self._load_poses()

    # ── Persistence ──────────────────────────────────────────────────────────

    def _load_poses(self) -> None:
        if not os.path.exists(DOCK_POSES_FILE):
            return
        try:
            with open(DOCK_POSES_FILE, 'r') as fh:
                data = yaml.safe_load(fh) or {}
            for raw_id, d in data.items():
                mid = int(raw_id)
                self.saved_poses[mid] = _pose_from_dict(d)
                self._saved_ids.add(mid)
            self.get_logger().info(
                f'Loaded {len(self.saved_poses)} dock pose(s) from {DOCK_POSES_FILE} '
                f'— IDs: {sorted(self._saved_ids)}')
        except Exception as exc:
            self.get_logger().warn(f'Could not load {DOCK_POSES_FILE}: {exc}')

    def _save_poses(self) -> None:
        data = {mid: _pose_to_dict(ps) for mid, ps in self.saved_poses.items()}
        # Atomic write: write to .tmp then rename to avoid partial files
        tmp_path = DOCK_POSES_FILE + '.tmp'
        try:
            with open(tmp_path, 'w') as fh:
                yaml.dump(data, fh)
            os.replace(tmp_path, DOCK_POSES_FILE)
        except Exception as exc:
            self.get_logger().warn(f'Could not write {DOCK_POSES_FILE}: {exc}')

    # ── Launch-signal helpers ─────────────────────────────────────────────────

    def arm_launch_signal(self) -> None:
        """
        Record a timestamp baseline.  Any marker reading received AFTER this
        call counts as the launch signal for the moving dock sequence.
        Call this immediately after the moving dock cmd_vel approach succeeds.
        """
        self._launch_arm_time = time.monotonic()

    def is_launch_signal_ready(self) -> bool:
        """True once a fresh marker reading has arrived since arm_launch_signal()."""
        return (self._launch_arm_time >= 0.0 and
                self.latest_stamp > self._launch_arm_time)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _marker_cb(self, msg: MarkerArray) -> None:
        for marker in msg.markers:
            p   = marker.pose.position
            q   = marker.pose.orientation
            mid = marker.id

            if p.z <= 0.0:
                continue  # reject corrupt detections

            # Always refresh live readings (cmd_vel approach + launch signal)
            self.latest_cam_x = p.x
            self.latest_cam_z = p.z
            self.latest_stamp = time.monotonic()

            # Only compute and save a map-frame goal on the FIRST detection of each ID
            if mid in self._saved_ids:
                continue

            goal = self._compute_map_goal(p.x, p.z, (q.x, q.y, q.z, q.w))
            if goal is None:
                self.get_logger().warn(
                    f'Marker {mid} seen but TF unavailable — will retry on next detection.')
                continue

            self.saved_poses[mid] = goal
            self._saved_ids.add(mid)
            self._save_poses()
            self.get_logger().info(
                f'[Scout] Marker {mid} locked — dock goal: '
                f'({goal.pose.position.x:.3f}, {goal.pose.position.y:.3f})')

    def _dock_cb(self, msg: Bool) -> None:
        if msg.data and not self.dock_event.is_set():
            self.get_logger().info(
                'Received /start_docking — transitioning to Phase 2.')
            self.dock_event.set()

    # ── Map-frame goal computation ─────────────────────────────────────────────

    def _compute_map_goal(
        self,
        cam_x: float,
        cam_z: float,
        cam_q: tuple,
    ) -> 'PoseStamped | None':
        """
        Convert a camera-frame marker observation to a map-frame PoseStamped
        docking goal using the marker's outward normal for a head-on approach.
        """
        try:
            tf_s = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warn(f'TF lookup failed during scouting: {exc}')
            return None

        t = tf_s.transform.translation
        r = tf_s.transform.rotation
        _, _, yaw = euler_from_quaternion([r.x, r.y, r.z, r.w])

        fwd = CAMERA_X_OFFSET + cam_z
        lat = -cam_x

        qx, qy, qz, qw = cam_q
        nx = 2.0 * (qx * qz + qw * qy)
        nz = 1.0 - 2.0 * (qx * qx + qy * qy)
        if nz > 0:
            nx, nz = -nx, -nz

        norm_fwd, norm_lat = nz, -nx
        norm_len = math.hypot(norm_fwd, norm_lat)
        if norm_len > 1e-6:
            norm_fwd /= norm_len
            norm_lat /= norm_len
        else:
            norm_fwd, norm_lat = -1.0, 0.0

        rx, ry = t.x, t.y
        mx = rx + math.cos(yaw) * fwd - math.sin(yaw) * lat
        my = ry + math.sin(yaw) * fwd + math.cos(yaw) * lat

        nmx = math.cos(yaw) * norm_fwd - math.sin(yaw) * norm_lat
        nmy = math.sin(yaw) * norm_fwd + math.cos(yaw) * norm_lat

        gx    = mx + NAV2_APPROACH_DISTANCE * nmx
        gy    = my + NAV2_APPROACH_DISTANCE * nmy
        g_yaw = math.atan2(-nmy, -nmx)

        self.get_logger().info(
            f'  marker map: ({mx:.3f}, {my:.3f})  '
            f'robot map: ({rx:.3f}, {ry:.3f})  '
            f'goal: ({gx:.3f}, {gy:.3f})  yaw: {math.degrees(g_yaw):.1f}°')

        goal = PoseStamped()
        goal.header.frame_id    = 'map'
        goal.header.stamp       = self.get_clock().now().to_msg()
        goal.pose.position.x    = gx
        goal.pose.position.y    = gy
        goal.pose.position.z    = 0.0
        qx2, qy2, qz2, qw2     = quaternion_from_euler(0.0, 0.0, g_yaw)
        goal.pose.orientation.x = qx2
        goal.pose.orientation.y = qy2
        goal.pose.orientation.z = qz2
        goal.pose.orientation.w = qw2
        return goal


# ---------------------------------------------------------------------------
# Phase-2 helpers
# ---------------------------------------------------------------------------

def navigate_to_goal(
    navigator: BasicNavigator,
    goal_pose: PoseStamped,
    scouter_node: MarkerScouter,
) -> TaskResult:
    """Send goal to Nav2, block until complete. Spins scouter to keep TF alive."""
    if not navigator.goToPose(goal_pose):
        navigator.get_logger().error('Nav2 rejected the goal.')
        return TaskResult.FAILED

    log        = navigator.get_logger()
    last_log_t = time.monotonic()

    while not navigator.isTaskComplete():
        rclpy.spin_once(scouter_node, timeout_sec=0.0)
        feedback = navigator.getFeedback()
        if feedback and time.monotonic() - last_log_t >= 1.0:
            log.info(
                f'Navigating — distance remaining: {feedback.distance_remaining:.2f} m')
            last_log_t = time.monotonic()

    result = navigator.getResult()
    log.info(f'Nav2 result: {result}')
    return result


def cmd_vel_approach(
    scouter_node: MarkerScouter,
    cmd_vel_pub,
    log,
) -> bool:
    """
    Three-phase cmd_vel fine-approach using live /aruco/markers readings.

    Phase 1 — Align  : rotate in place until |cam_x| < ALIGN_TOL.
    Phase 2 — Drive  : proportional fwd + angular until cam_z < BLIND_THRESHOLD.
    Phase 3 — Reckon : dead-reckon remaining gap when marker is too close.

    Returns True on success, False on abort.
    """
    K_LINEAR         = 0.3
    K_ANGULAR        = 1.5
    MAX_LINEAR       = 0.08
    MAX_ANGULAR      = 0.50
    ALIGN_TOL        = 0.03
    BLIND_THRESHOLD  = 0.20
    APPROACH_TIMEOUT = 25.0
    LOST_MARKER_S    = 3.0

    twist    = Twist()
    deadline = time.monotonic() + APPROACH_TIMEOUT

    def stop():
        twist.linear.x  = 0.0
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)

    def fresh():
        return (scouter_node.latest_stamp > 0.0 and
                time.monotonic() - scouter_node.latest_stamp < LOST_MARKER_S)

    # ── Phase 1: Align ───────────────────────────────────────────────────────
    log.info('Phase 1 — aligning …')
    while time.monotonic() < deadline:
        rclpy.spin_once(scouter_node, timeout_sec=0.05)
        if not fresh():
            if scouter_node.latest_stamp == 0.0:
                continue
            log.error('Marker lost during alignment — aborting.')
            stop()
            return False
        cam_x   = scouter_node.latest_cam_x
        ang_cmd = float(max(-MAX_ANGULAR, min(MAX_ANGULAR, -K_ANGULAR * cam_x)))
        log.info(f'[ALIGN] cam_x={cam_x:+.4f} m  angular.z={ang_cmd:+.3f} rad/s')
        if abs(cam_x) < ALIGN_TOL:
            log.info(f'Aligned: cam_x={cam_x:.4f} m')
            break
        twist.linear.x  = 0.0
        twist.angular.z = ang_cmd
        cmd_vel_pub.publish(twist)
    else:
        stop()
        log.error('Alignment timed out.')
        return False

    # ── Phase 2: Drive ───────────────────────────────────────────────────────
    log.info('Phase 2 — driving toward marker …')
    last_cam_z = float('inf')

    while time.monotonic() < deadline:
        rclpy.spin_once(scouter_node, timeout_sec=0.05)
        if not fresh():
            if last_cam_z < BLIND_THRESHOLD:
                log.info(
                    f'Blind zone (last cam_z={last_cam_z:.4f} m) — dead-reckoning.')
                break
            log.error(
                f'Marker lost unexpectedly (last cam_z={last_cam_z:.4f} m) — aborting.')
            stop()
            return False
        cam_x      = scouter_node.latest_cam_x
        cam_z      = scouter_node.latest_cam_z
        last_cam_z = cam_z
        dist_err   = cam_z - TARGET_DISTANCE
        if abs(dist_err) < DISTANCE_TOLERANCE and abs(cam_x) < LATERAL_TOLERANCE:
            stop()
            log.info(f'Approach complete: cam_x={cam_x:.4f} m  cam_z={cam_z:.4f} m')
            return True
        twist.linear.x  = float(max(-MAX_LINEAR, min(MAX_LINEAR, K_LINEAR * dist_err)))
        twist.angular.z = float(max(-MAX_ANGULAR, min(MAX_ANGULAR, -K_ANGULAR * cam_x)))
        cmd_vel_pub.publish(twist)
    else:
        stop()
        log.error('Drive phase timed out.')
        return False

    # ── Phase 3: Dead-reckoning ───────────────────────────────────────────────
    remaining = last_cam_z - TARGET_DISTANCE
    if remaining <= 0.0:
        stop()
        log.info('Already at TARGET_DISTANCE — done.')
        return True
    drive_speed = MAX_LINEAR * 0.5
    t_drive     = remaining / drive_speed
    log.info(
        f'Phase 3 — dead-reckoning {remaining:.3f} m at {drive_speed:.2f} m/s '
        f'({t_drive:.2f} s) …')
    t_end = min(time.monotonic() + t_drive, deadline)
    while time.monotonic() < t_end:
        twist.linear.x  = drive_speed
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)
        rclpy.spin_once(scouter_node, timeout_sec=0.02)
    stop()
    log.info('Dead-reckoning complete — docked.')
    return True


def undock(
    scouter_node: MarkerScouter,
    cmd_vel_pub,
    log,
    distance: float = UNDOCK_DISTANCE,
) -> None:
    """Reverse straight back by `distance` metres to clear the dock."""
    t_drive = distance / UNDOCK_SPEED
    log.info(f'Undocking — reversing {distance:.2f} m ({t_drive:.1f} s) …')
    twist = Twist()
    twist.linear.x  = -UNDOCK_SPEED
    twist.angular.z =  0.0
    t_end = time.monotonic() + t_drive
    while time.monotonic() < t_end:
        cmd_vel_pub.publish(twist)
        rclpy.spin_once(scouter_node, timeout_sec=0.05)
    twist.linear.x = 0.0
    cmd_vel_pub.publish(twist)
    log.info('Undock complete.')


def wait_for_launch_signal(
    scouter_node: MarkerScouter,
    log,
    timeout_s: float = LAUNCH_SIGNAL_TIMEOUT,
) -> bool:
    """
    Block until a fresh ArUco detection arrives after arm_launch_signal() was called.
    Returns True if the signal is received within timeout_s, False otherwise.
    """
    log.info('Waiting for launch signal (ArUco re-detection) …')
    deadline  = time.monotonic() + timeout_s
    last_warn = time.monotonic()

    while time.monotonic() < deadline:
        rclpy.spin_once(scouter_node, timeout_sec=0.05)
        if scouter_node.is_launch_signal_ready():
            log.info(
                f'Launch signal received: '
                f'cam_x={scouter_node.latest_cam_x:.4f} m  '
                f'cam_z={scouter_node.latest_cam_z:.4f} m')
            return True
        if time.monotonic() - last_warn > 5.0:
            log.warn('Still waiting for launch signal …')
            last_warn = time.monotonic()

    log.warn(f'No launch signal within {timeout_s:.0f} s — skipping launch.')
    return False


def run_docking_sequence(
    label: str,
    goal_pose: PoseStamped,
    navigator: BasicNavigator,
    scouter_node: MarkerScouter,
    cmd_vel_pub,
    log,
) -> bool:
    """
    Navigate to goal_pose and run the cmd_vel fine-approach.
    Retries up to MAX_RETRIES times.
    Returns True on success, False if all attempts fail.
    """
    log.info(f'=== {label} ===')
    for attempt in range(MAX_RETRIES + 1):
        log.info(f'[{label}] Attempt {attempt + 1}/{MAX_RETRIES + 1}')

        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        result = navigate_to_goal(navigator, goal_pose, scouter_node)

        if result == TaskResult.CANCELED:
            log.warn(f'[{label}] Nav2 goal canceled — aborting.')
            return False
        if result != TaskResult.SUCCEEDED:
            log.error(f'[{label}] Nav2 failed to reach goal — aborting.')
            return False

        log.info(f'[{label}] At pre-approach position — starting cmd_vel approach.')

        if cmd_vel_approach(scouter_node, cmd_vel_pub, log):
            log.info(f'[{label}] Docking SUCCESSFUL.')
            return True

        if attempt < MAX_RETRIES:
            log.warn(f'[{label}] cmd_vel approach failed — retrying …')
        else:
            log.error(f'[{label}] FAILED after {MAX_RETRIES + 1} attempt(s).')

    return False


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description='Deferred two-marker ArUco docking node.')
    parser.add_argument(
        '--dock-now', action='store_true',
        help='Skip /start_docking wait and dock immediately using saved YAML poses.')
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)

    navigator    = BasicNavigator()
    tf_buffer    = tf2_ros.Buffer()
    scouter_node = MarkerScouter(tf_buffer)
    _tf_listener = tf2_ros.TransformListener(tf_buffer, scouter_node)  # noqa: F841
    cmd_vel_pub  = scouter_node.create_publisher(Twist, '/cmd_vel', 10)

    log = navigator.get_logger()

    # ── Wait for TF chain ─────────────────────────────────────────────────────
    log.info('Checking map→base_link TF (SLAM localization) …')
    tf_ok = False
    for i in range(60):
        try:
            tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            tf_ok = True
            break
        except Exception:
            rclpy.spin_once(scouter_node, timeout_sec=0.1)
            if i % 10 == 0:
                log.info('Waiting for map→base_link transform …')

    if not tf_ok:
        log.error('map→base_link TF unavailable after 30 s — is SLAM running?')
        scouter_node.destroy_node()
        navigator.destroy_node()
        rclpy.shutdown()
        return

    log.info('TF OK — Phase 1 (passive scouting) active.')

    # ── Phase 1: passive scouting in a background executor ───────────────────
    bg_executor = MultiThreadedExecutor()
    bg_executor.add_node(scouter_node)
    bg_thread = threading.Thread(target=bg_executor.spin, daemon=True)
    bg_thread.start()

    try:
        # ── Wait for Phase-2 trigger ──────────────────────────────────────────
        if parsed.dock_now:
            log.info('--dock-now: skipping /start_docking wait.')
        else:
            log.info(
                'Scouting … publish  True  on  /start_docking  '
                'when frontier exploration finishes.')
            scouter_node.dock_event.wait()

        # ── Stop background executor; switch to manual spinning ───────────────
        bg_executor.remove_node(scouter_node)
        bg_executor.shutdown(timeout_sec=2.0)
        bg_thread.join(timeout=3.0)
        log.info('Background executor stopped — Phase 2 beginning.')

        # ── Verify both poses were scouted ────────────────────────────────────
        missing = [mid for mid in (STATIC_MARKER_ID, MOVING_MARKER_ID)
                   if mid not in scouter_node.saved_poses]
        if missing:
            log.error(
                f'Missing saved poses for marker ID(s) {missing}. '
                f'Seen IDs: {sorted(scouter_node.saved_poses.keys())}. '
                'Were both markers visible during exploration? Aborting.')
            return

        # ─────────────────────────────────────────────────────────────────────
        # STEP 1 — STATIC DOCK (marker 0)
        # ─────────────────────────────────────────────────────────────────────
        static_ok = run_docking_sequence(
            label       = f'STATIC DOCK (marker {STATIC_MARKER_ID})',
            goal_pose   = scouter_node.saved_poses[STATIC_MARKER_ID],
            navigator   = navigator,
            scouter_node= scouter_node,
            cmd_vel_pub = cmd_vel_pub,
            log         = log,
        )

        if not static_ok:
            log.error('Static docking failed — skipping moving dock.')
            return

        # ── Undock from static dock ───────────────────────────────────────────
        undock(scouter_node, cmd_vel_pub, log)

        # ─────────────────────────────────────────────────────────────────────
        # STEP 2 — MOVING DOCK (marker 1)
        # ─────────────────────────────────────────────────────────────────────
        moving_ok = run_docking_sequence(
            label       = f'MOVING DOCK (marker {MOVING_MARKER_ID})',
            goal_pose   = scouter_node.saved_poses[MOVING_MARKER_ID],
            navigator   = navigator,
            scouter_node= scouter_node,
            cmd_vel_pub = cmd_vel_pub,
            log         = log,
        )

        if not moving_ok:
            log.error('Moving docking failed — launch sequence aborted.')
            return

        # ── Launch signal ─────────────────────────────────────────────────────
        # Arm immediately after docking so any reading from this point counts.
        scouter_node.arm_launch_signal()

        if wait_for_launch_signal(scouter_node, log):
            log.info('Holding 1 s before launch …')
            time.sleep(1.0)
            log.info('>>> LAUNCH SEQUENCE INITIATED (placeholder) <<<')
        else:
            log.warn('Launch signal not received — launch sequence skipped.')

    except KeyboardInterrupt:
        log.info('Interrupted — canceling any active Nav2 task.')
        navigator.cancelTask()

    finally:
        scouter_node.destroy_node()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
