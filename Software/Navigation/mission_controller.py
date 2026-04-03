#!/usr/bin/env python3
"""
mission_controller.py — Consolidated mission controller for CDE2310 Group 1.

Run on the LOCAL LAPTOP (not the RPi).

Combines:
  - Passive ArUco marker logging during exploration
  - tracking_back coverage sweep (if markers not all found)
  - Nav2 + cmd_vel docking for Station A (stationary)
  - Nav2 + cmd_vel docking for Station B (moving target)
  - Ball dispensing placeholder

Prerequisites:
  1. On RPi: aruco_live.py publishing MarkerArray on /aruco/markers
  2. Cartographer running (rslam) — provides map + TF
  3. Nav2 running — provides path planning
  4. m-explore running — provides frontier exploration
  5. source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=30

Usage:
  python3 mission_controller.py
"""

import math
import time
from collections import deque

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.action import ActionClient

import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

try:
    from tf_transformations import euler_from_quaternion, quaternion_from_euler
except ImportError:
    def euler_from_quaternion(q):
        x, y, z, w = q
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(t2)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw

    def quaternion_from_euler(roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )


# =============================================================================
# CONFIGURATION — edit these to match your setup
# =============================================================================

# PLACEHOLDER: set these to your actual ArUco marker IDs
STATION_A_MARKER_ID = 0      # ArUco ID for Station A (stationary)
STATION_B_MARKER_ID = 1      # ArUco ID for Station B (moving)

REQUIRED_MARKERS = {STATION_A_MARKER_ID, STATION_B_MARKER_ID}

# Docking tuning
TARGET_DISTANCE        = 0.10   # m — final distance from marker
NAV2_APPROACH_DISTANCE = 0.35   # m — Nav2 stops here, cmd_vel takes over
LATERAL_TOLERANCE      = 0.01   # m
DISTANCE_TOLERANCE     = 0.01   # m
MARKER_WAIT_TIMEOUT    = 30.0   # s
MAX_DOCK_RETRIES       = 4
CAMERA_X_OFFSET        = 0.04   # m — base_link to camera lens

# Tracking back (coverage sweep)
VISIT_RADIUS   = 0.3   # m — mark cells within this as visited
GOAL_SPACING   = 0.5   # m — grid spacing for coverage goals
COVERAGE_TICK  = 1.0    # s — how often to check coverage

# m-explore monitoring
# PLACEHOLDER: set this to the exact substring m-explore prints when done
EXPLORE_DONE_TOPIC = '/explore/frontiers'  # or monitor stdout


# =============================================================================
# MARKER LISTENER — shared across all phases
# =============================================================================

class MarkerListener(Node):
    """
    Subscribes to /aruco/markers.
    - Always updates latest_* fields (used by cmd_vel approach).
    - When armed, captures a snapshot (used by Nav2 goal computation).
    - Passively logs all detected marker IDs + map-frame positions.
    """

    def __init__(self) -> None:
        super().__init__('mission_controller')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(MarkerArray, '/aruco/markers', self._marker_cb, qos)

        # Snapshot (armed capture)
        self._armed: bool = False
        self.snapshot_ready: bool = False
        self.cam_x: float = 0.0
        self.cam_z: float = 0.0
        self.cam_q: tuple = (0.0, 0.0, 0.0, 1.0)

        # Live readings (cmd_vel approach)
        self.latest_cam_x: float = 0.0
        self.latest_cam_z: float = 0.0
        self.latest_stamp: float = 0.0

        # Marker log: {marker_id: (map_x, map_y, map_yaw)}
        self.detected_markers: dict = {}

        # TF for marker logging
        self.tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Map data (for tracking_back coverage)
        self.map_data = None
        self.visited = None
        self.map_resolution = None
        self.map_origin = None
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)

        # cmd_vel publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def arm(self) -> None:
        self.snapshot_ready = False
        self._armed = True

    def _marker_cb(self, msg: MarkerArray) -> None:
        if not msg.markers:
            return

        for marker in msg.markers:
            p = marker.pose.position
            q = marker.pose.orientation
            marker_id = marker.id

            if p.z <= 0.0:
                continue

            # Always update latest (for cmd_vel approach)
            self.latest_cam_x = p.x
            self.latest_cam_z = p.z
            self.latest_stamp = time.monotonic()

            # Armed snapshot (takes first valid marker)
            if self._armed:
                self.cam_x = p.x
                self.cam_z = p.z
                self.cam_q = (q.x, q.y, q.z, q.w)
                self._armed = False
                self.snapshot_ready = True
                self.get_logger().info(
                    f'Snapshot: id={marker_id} cam_x={p.x:.4f} cam_z={p.z:.4f}')

            # Passive logging — convert to map frame and store
            if marker_id not in self.detected_markers:
                map_pose = self._camera_to_map(p.x, p.z, q)
                if map_pose is not None:
                    self.detected_markers[marker_id] = map_pose
                    self.get_logger().info(
                        f'NEW MARKER LOGGED: id={marker_id} '
                        f'map=({map_pose[0]:.2f}, {map_pose[1]:.2f}, '
                        f'yaw={math.degrees(map_pose[2]):.1f}°)')
                    self.get_logger().info(
                        f'Markers found so far: {list(self.detected_markers.keys())}')

    def _camera_to_map(self, cam_x, cam_z, q_msg):
        """Convert camera-frame marker position to map-frame (x, y, yaw)."""
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except Exception:
            return None

        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Camera → base_link → map
        fwd = CAMERA_X_OFFSET + cam_z
        lat = -cam_x
        map_x = t.x + fwd * math.cos(yaw) - lat * math.sin(yaw)
        map_y = t.y + fwd * math.sin(yaw) + lat * math.cos(yaw)

        # Marker normal direction (for approach heading)
        qx, qy, qz, qw = q_msg.x, q_msg.y, q_msg.z, q_msg.w
        nx = 2.0 * (qx * qz + qw * qy)
        nz = 1.0 - 2.0 * (qx * qx + qy * qy)
        if nz > 0:
            nx, nz = -nx, -nz
        marker_angle_cam = math.atan2(-(-nx), -nz)
        marker_angle_map = yaw + marker_angle_cam

        return (map_x, map_y, marker_angle_map)

    def _map_cb(self, msg):
        w, h = msg.info.width, msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        self.map_data = np.array(msg.data).reshape((h, w))

        if self.visited is None:
            self.visited = np.zeros_like(self.map_data, dtype=bool)
            self.visited[self.map_data != 0] = True
            self.get_logger().info(
                f'Map received: {w}x{h}, '
                f'{np.sum(~self.visited)} free cells to cover')

    def has_all_markers(self) -> bool:
        return REQUIRED_MARKERS.issubset(self.detected_markers.keys())

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())


# =============================================================================
# DOCKING — Nav2 approach + cmd_vel final (Station A)
# =============================================================================

def wait_for_snapshot(node: MarkerListener, timeout_s: float = MARKER_WAIT_TIMEOUT) -> bool:
    node.arm()
    log = node.get_logger()
    deadline = time.monotonic() + timeout_s
    last_warn = time.monotonic()

    while not node.snapshot_ready:
        if time.monotonic() > deadline:
            log.error('No marker within timeout.')
            return False
        if time.monotonic() - last_warn > 5.0:
            log.warn('Waiting for /aruco/markers ...')
            last_warn = time.monotonic()
        rclpy.spin_once(node, timeout_sec=0.05)
    return True


def compute_nav2_goal(
    cam_x: float, cam_z: float, cam_q: tuple,
    node: MarkerListener, navigator: BasicNavigator,
) -> 'PoseStamped | None':
    """Compute a Nav2 goal NAV2_APPROACH_DISTANCE in front of the marker."""
    try:
        tf_stamped = node.tf_buffer.lookup_transform(
            'map', 'base_link', rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=2),
        )
    except Exception as exc:
        node.get_logger().error(f'TF lookup failed: {exc}')
        return None

    t = tf_stamped.transform.translation
    q = tf_stamped.transform.rotation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    # Camera → base_link
    fwd = CAMERA_X_OFFSET + cam_z
    lat = -cam_x

    # Marker normal vector
    qx, qy, qz, qw = cam_q
    nx = 2.0 * (qx * qz + qw * qy)
    nz = 1.0 - 2.0 * (qx * qx + qy * qy)
    if nz > 0:
        nx, nz = -nx, -nz
    norm_len = math.hypot(nx, nz)
    if norm_len > 1e-6:
        nx /= norm_len
        nz /= norm_len
    else:
        nx, nz = -1.0, 0.0

    # Marker position in map frame
    marker_map_x = t.x + fwd * math.cos(yaw) - lat * math.sin(yaw)
    marker_map_y = t.y + fwd * math.sin(yaw) + lat * math.cos(yaw)

    # Normal in map frame
    norm_cam_x, norm_cam_y = nz, -nx
    norm_map_x = norm_cam_x * math.cos(yaw) - norm_cam_y * math.sin(yaw)
    norm_map_y = norm_cam_x * math.sin(yaw) + norm_cam_y * math.cos(yaw)

    # Goal = NAV2_APPROACH_DISTANCE along the normal, facing the marker
    goal_x = marker_map_x + NAV2_APPROACH_DISTANCE * norm_map_x
    goal_y = marker_map_y + NAV2_APPROACH_DISTANCE * norm_map_y
    approach_yaw = math.atan2(-norm_map_y, -norm_map_x)

    quat = quaternion_from_euler(0, 0, approach_yaw)

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = goal_x
    goal.pose.position.y = goal_y
    goal.pose.orientation.x = quat[0]
    goal.pose.orientation.y = quat[1]
    goal.pose.orientation.z = quat[2]
    goal.pose.orientation.w = quat[3]

    node.get_logger().info(
        f'Nav2 goal: ({goal_x:.2f}, {goal_y:.2f}) '
        f'yaw={math.degrees(approach_yaw):.1f}°')
    return goal


def navigate_to_goal(navigator: BasicNavigator, goal: PoseStamped, node: MarkerListener) -> TaskResult:
    """Send a Nav2 goal and block until completion, spinning the marker node."""
    navigator.goToPose(goal)
    while not navigator.isTaskComplete():
        rclpy.spin_once(node, timeout_sec=0.1)
    return navigator.getResult()


def cmd_vel_approach(node: MarkerListener, log) -> bool:
    """
    Three-phase cmd_vel approach to TARGET_DISTANCE.
    Phase 1: Align (rotate until marker centred)
    Phase 2: Drive (proportional forward + angular)
    Phase 3: Dead-reckon (marker lost in blind zone)
    """
    K_LINEAR        = 0.3
    K_ANGULAR       = 1.5
    MAX_LINEAR      = 0.08
    MAX_ANGULAR     = 0.50
    ALIGN_TOL       = 0.03
    BLIND_THRESHOLD = 0.20
    APPROACH_TIMEOUT = 25.0
    LOST_MARKER_S   = 3.0

    twist = Twist()
    deadline = time.monotonic() + APPROACH_TIMEOUT

    def stop():
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.cmd_vel_pub.publish(twist)

    def fresh():
        return (node.latest_stamp > 0.0 and
                time.monotonic() - node.latest_stamp < LOST_MARKER_S)

    # Phase 1: Align
    log.info('cmd_vel Phase 1 — aligning...')
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if not fresh():
            if node.latest_stamp == 0.0:
                continue
            log.error('Marker lost during alignment.')
            stop()
            return False

        cam_x = node.latest_cam_x
        ang_cmd = float(max(-MAX_ANGULAR, min(MAX_ANGULAR, -K_ANGULAR * cam_x)))
        if abs(cam_x) < ALIGN_TOL:
            log.info(f'Aligned: cam_x={cam_x:.4f} m')
            break
        twist.linear.x = 0.0
        twist.angular.z = ang_cmd
        node.cmd_vel_pub.publish(twist)
    else:
        stop()
        log.error('Alignment timed out.')
        return False

    # Phase 2: Drive
    log.info('cmd_vel Phase 2 — driving...')
    last_cam_z = float('inf')

    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if not fresh():
            if last_cam_z < BLIND_THRESHOLD:
                log.info(f'Blind zone (cam_z={last_cam_z:.4f}) — dead-reckoning.')
                break
            log.error(f'Marker lost (cam_z={last_cam_z:.4f}).')
            stop()
            return False

        cam_x = node.latest_cam_x
        cam_z = node.latest_cam_z
        last_cam_z = cam_z
        dist_err = cam_z - TARGET_DISTANCE

        if abs(dist_err) < DISTANCE_TOLERANCE and abs(cam_x) < LATERAL_TOLERANCE:
            stop()
            log.info(f'Approach complete: cam_x={cam_x:.4f} cam_z={cam_z:.4f}')
            return True

        twist.linear.x = float(max(-MAX_LINEAR, min(MAX_LINEAR, K_LINEAR * dist_err)))
        twist.angular.z = float(max(-MAX_ANGULAR, min(MAX_ANGULAR, -K_ANGULAR * cam_x)))
        node.cmd_vel_pub.publish(twist)
    else:
        stop()
        log.error('Drive timed out.')
        return False

    # Phase 3: Dead-reckon
    remaining = last_cam_z - TARGET_DISTANCE
    if remaining <= 0.0:
        stop()
        return True

    drive_speed = MAX_LINEAR * 0.5
    t_drive = remaining / drive_speed
    log.info(f'cmd_vel Phase 3 — dead-reckoning {remaining:.3f} m ({t_drive:.2f} s)')
    t_end = min(time.monotonic() + t_drive, deadline)
    while time.monotonic() < t_end:
        twist.linear.x = drive_speed
        twist.angular.z = 0.0
        node.cmd_vel_pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.02)
    stop()
    log.info('Dead-reckoning complete.')
    return True


def dock_station(
    node: MarkerListener,
    navigator: BasicNavigator,
    station_name: str,
) -> bool:
    """Full docking sequence: Nav2 approach → cmd_vel final → placeholder dispense."""
    log = node.get_logger()

    for attempt in range(MAX_DOCK_RETRIES + 1):
        log.info(f'=== {station_name} dock attempt {attempt + 1}/{MAX_DOCK_RETRIES + 1} ===')

        # Wait for marker snapshot
        if not wait_for_snapshot(node):
            log.error(f'{station_name}: could not get marker reading.')
            break

        # Spin TF buffer briefly
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)

        # Compute Nav2 goal
        goal = compute_nav2_goal(node.cam_x, node.cam_z, node.cam_q, node, navigator)
        if goal is None:
            log.error(f'{station_name}: could not compute goal.')
            break

        # Navigate
        result = navigate_to_goal(navigator, goal, node)
        if result == TaskResult.CANCELED:
            log.warn(f'{station_name}: Nav2 canceled.')
            break
        elif result != TaskResult.SUCCEEDED:
            log.error(f'{station_name}: Nav2 failed.')
            if attempt < MAX_DOCK_RETRIES:
                log.warn('Retrying...')
                continue
            break

        log.info(f'{station_name}: Nav2 reached goal. Starting cmd_vel approach.')

        # cmd_vel final approach
        if cmd_vel_approach(node, log):
            log.info(f'{station_name}: DOCKED SUCCESSFULLY.')
            return True

        if attempt < MAX_DOCK_RETRIES:
            log.warn(f'{station_name}: cmd_vel failed — retrying...')
        else:
            log.error(f'{station_name}: FAILED after {MAX_DOCK_RETRIES + 1} attempts.')

    return False


# =============================================================================
# BALL DISPENSING — placeholder
# =============================================================================

def dispense_balls(node: MarkerListener, station_name: str):
    """
    PLACEHOLDER: Replace this with your actual ball dispensing sequence.

    For Station A: 3 balls with team-specific timing, all must stay 10s.
    For Station B: 3 balls into moving receptacle.
    """
    log = node.get_logger()
    log.info(f'>>> {station_name}: DISPENSE SEQUENCE START (placeholder) <<<')

    # PLACEHOLDER: your dispensing logic here
    # Example timing:
    for i in range(3):
        log.info(f'{station_name}: Dispensing ball {i + 1}/3...')
        time.sleep(2.0)  # PLACEHOLDER: replace with actual timing
        log.info(f'{station_name}: Ball {i + 1} dispensed.')

    log.info(f'>>> {station_name}: DISPENSE SEQUENCE COMPLETE <<<')


# =============================================================================
# TRACKING BACK — coverage sweep for missing markers
# =============================================================================

def run_tracking_back(node: MarkerListener, navigator: BasicNavigator) -> bool:
    """
    BFS coverage sweep of free cells on the completed map.
    Navigates to unvisited areas hoping to find missing ArUco markers.
    Returns True once all required markers are found, or False if map fully covered.
    """
    log = node.get_logger()
    log.info('Starting tracking_back coverage sweep for missing markers...')
    log.info(f'Missing markers: {REQUIRED_MARKERS - set(node.detected_markers.keys())}')

    if node.map_data is None:
        log.error('No map data available — cannot run coverage sweep.')
        return False

    while rclpy.ok():
        # Check if we found all markers during the sweep
        if node.has_all_markers():
            log.info('All markers found during coverage sweep!')
            return True

        # Get robot position
        robot_pos = _get_robot_position(node)
        if robot_pos is None:
            rclpy.spin_once(node, timeout_sec=0.5)
            continue

        # Mark current area as visited
        _mark_visited(node, *robot_pos)

        # Coverage stats
        total_free = np.sum(node.map_data == 0)
        visited_free = np.sum(node.visited & (node.map_data == 0))
        pct = visited_free / total_free * 100 if total_free > 0 else 100

        # Find next unvisited cell
        target = _find_nearest_unvisited(node, *robot_pos)
        if target is None:
            log.info(f'Coverage complete ({pct:.1f}%) — no more unvisited cells.')
            return False

        log.info(f'Coverage: {pct:.1f}% — navigating to ({target[0]:.2f}, {target[1]:.2f})')

        # Navigate to target
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x = target[0]
        goal.pose.position.y = target[1]
        goal.pose.orientation.w = 1.0

        result = navigate_to_goal(navigator, goal, node)

        # After arriving (or failing), check markers again
        if node.has_all_markers():
            log.info('All markers found during coverage sweep!')
            return True

        if result != TaskResult.SUCCEEDED:
            log.warn('Navigation to coverage target failed — trying next.')

    return False


def _get_robot_position(node: MarkerListener):
    try:
        t = node.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        return t.transform.translation.x, t.transform.translation.y
    except Exception:
        return None


def _world_to_grid(node, x, y):
    col = int((x - node.map_origin.position.x) / node.map_resolution)
    row = int((y - node.map_origin.position.y) / node.map_resolution)
    return row, col


def _grid_to_world(node, row, col):
    x = node.map_origin.position.x + (col + 0.5) * node.map_resolution
    y = node.map_origin.position.y + (row + 0.5) * node.map_resolution
    return x, y


def _mark_visited(node, robot_x, robot_y):
    r, c = _world_to_grid(node, robot_x, robot_y)
    radius_cells = int(VISIT_RADIUS / node.map_resolution)
    h, w = node.visited.shape
    for dr in range(-radius_cells, radius_cells + 1):
        for dc in range(-radius_cells, radius_cells + 1):
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w:
                if dr * dr + dc * dc <= radius_cells * radius_cells:
                    node.visited[nr, nc] = True


def _find_nearest_unvisited(node, robot_x, robot_y):
    r, c = _world_to_grid(node, robot_x, robot_y)
    h, w = node.visited.shape
    step = max(1, int(GOAL_SPACING / node.map_resolution))

    seen = set()
    queue = deque([(r, c)])
    seen.add((r, c))

    while queue:
        cr, cc = queue.popleft()
        if (not node.visited[cr, cc] and
                cr % step == 0 and cc % step == 0):
            return _grid_to_world(node, cr, cc)

        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = cr + dr, cc + dc
            if (0 <= nr < h and 0 <= nc < w and
                    (nr, nc) not in seen and
                    node.map_data[nr, nc] == 0):
                seen.add((nr, nc))
                queue.append((nr, nc))
    return None


# =============================================================================
# NAVIGATE TO LOGGED MARKER POSITION
# =============================================================================

def navigate_to_marker(
    node: MarkerListener,
    navigator: BasicNavigator,
    marker_id: int,
) -> bool:
    """Navigate to the map-frame position logged for a specific marker."""
    log = node.get_logger()

    if marker_id not in node.detected_markers:
        log.error(f'Marker {marker_id} was never logged — cannot navigate to it.')
        return False

    map_x, map_y, map_yaw = node.detected_markers[marker_id]

    # Goal is NAV2_APPROACH_DISTANCE back from the marker along its normal
    approach_x = map_x + NAV2_APPROACH_DISTANCE * math.cos(map_yaw + math.pi)
    approach_y = map_y + NAV2_APPROACH_DISTANCE * math.sin(map_yaw + math.pi)

    quat = quaternion_from_euler(0, 0, map_yaw)

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = approach_x
    goal.pose.position.y = approach_y
    goal.pose.orientation.x = quat[0]
    goal.pose.orientation.y = quat[1]
    goal.pose.orientation.z = quat[2]
    goal.pose.orientation.w = quat[3]

    log.info(f'Navigating to marker {marker_id} approach point: '
             f'({approach_x:.2f}, {approach_y:.2f})')

    result = navigate_to_goal(navigator, goal, node)
    if result != TaskResult.SUCCEEDED:
        log.error(f'Failed to navigate to marker {marker_id}.')
        return False

    log.info(f'Arrived at marker {marker_id} approach point.')
    return True


# =============================================================================
# MISSION STATE MACHINE
# =============================================================================

def main(args=None) -> None:
    rclpy.init(args=args)

    node = MarkerListener()
    navigator = BasicNavigator()
    log = node.get_logger()

    # ── Wait for TF chain ────────────────────────────────────────────────────
    log.info('Waiting for map→base_link TF...')
    tf_ok = False
    for i in range(60):
        try:
            node.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            tf_ok = True
            break
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.1)
            if i % 10 == 0:
                log.info('Still waiting for TF...')

    if not tf_ok:
        log.error('TF not available after 30s. Is Cartographer running?')
        node.destroy_node()
        navigator.destroy_node()
        rclpy.shutdown()
        return

    log.info('TF chain OK.')

    try:
        # ==================================================================
        # PHASE 1: EXPLORE — wait for m-explore to finish
        # ==================================================================
        log.info('='*60)
        log.info('PHASE 1: EXPLORATION')
        log.info('m-explore is mapping. Passively logging ArUco markers...')
        log.info(f'Looking for marker IDs: {REQUIRED_MARKERS}')
        log.info('='*60)

        # Spin and let the marker callback do its thing.
        # The user will see markers being logged as they're found.
        # Wait for the user to signal that m-explore is done.
        log.info('')
        log.info('>>> Waiting for m-explore to finish. <<<')
        log.info('>>> When m-explore reports no frontiers, press ENTER here. <<<')
        log.info('')

        # Spin in background while waiting for user input
        import threading
        explore_done = threading.Event()

        def wait_for_input():
            input()  # blocks until ENTER
            explore_done.set()

        input_thread = threading.Thread(target=wait_for_input, daemon=True)
        input_thread.start()

        while not explore_done.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)

        log.info('m-explore done. Markers found so far: '
                 f'{list(node.detected_markers.keys())}')

        # ==================================================================
        # PHASE 2: TRACKING BACK (if needed)
        # ==================================================================
        if not node.has_all_markers():
            log.info('='*60)
            log.info('PHASE 2: TRACKING BACK')
            log.info(f'Missing markers: '
                     f'{REQUIRED_MARKERS - set(node.detected_markers.keys())}')
            log.info('Running coverage sweep to find them...')
            log.info('='*60)

            found_all = run_tracking_back(node, navigator)

            if not found_all:
                log.error('Coverage sweep complete but not all markers found!')
                log.error(f'Found: {list(node.detected_markers.keys())}')
                log.error(f'Missing: '
                          f'{REQUIRED_MARKERS - set(node.detected_markers.keys())}')
                # Continue anyway with whatever we have
        else:
            log.info('Both markers found during exploration — skipping tracking_back.')

        # ==================================================================
        # PHASE 3: DOCK STATION A (stationary)
        # ==================================================================
        if STATION_A_MARKER_ID in node.detected_markers:
            log.info('='*60)
            log.info('PHASE 3: STATION A (stationary)')
            log.info('='*60)

            # Navigate to Station A's logged position
            if navigate_to_marker(node, navigator, STATION_A_MARKER_ID):
                # Now dock using live marker detection
                if dock_station(node, navigator, 'Station A'):
                    dispense_balls(node, 'Station A')
                else:
                    log.error('Station A docking failed.')
            else:
                log.error('Could not navigate to Station A.')
        else:
            log.error(f'Station A marker (id={STATION_A_MARKER_ID}) never found — skipping.')

        # ==================================================================
        # PHASE 4: DOCK STATION B (moving)
        # ==================================================================
        if STATION_B_MARKER_ID in node.detected_markers:
            log.info('='*60)
            log.info('PHASE 4: STATION B (moving target)')
            log.info('='*60)

            # Navigate to Station B's logged position
            if navigate_to_marker(node, navigator, STATION_B_MARKER_ID):
                # Dock — same Nav2+cmd_vel but the marker is moving
                # The cmd_vel approach handles this since it tracks live readings
                if dock_station(node, navigator, 'Station B'):
                    dispense_balls(node, 'Station B')
                else:
                    log.error('Station B docking failed.')
            else:
                log.error('Could not navigate to Station B.')
        else:
            log.error(f'Station B marker (id={STATION_B_MARKER_ID}) never found — skipping.')

        # ==================================================================
        # DONE
        # ==================================================================
        log.info('='*60)
        log.info('MISSION COMPLETE')
        log.info('='*60)

    except KeyboardInterrupt:
        log.info('Mission interrupted — stopping robot.')
        node.stop_robot()
        navigator.cancelTask()

    finally:
        node.destroy_node()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
