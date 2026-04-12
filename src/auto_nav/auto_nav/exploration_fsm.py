#!/usr/bin/env python3
"""
ultimate_mission_controller.py — The Master FSM for CDE2310 Group 1.

Features:
1. Triggering: Autonomous via explore_lite's ExploreStatus.
2. State Management: YAML persistence for ArUco map poses.
3. Fault Tolerance: BFS Coverage Sweep for missing markers.
4. Hardware Logic: Undocks (reverses) after Station A, Launch Signal at Station B.
5. Threading: Main thread for ROS callbacks, background thread for mission logic.
"""

import os
import math
import time
import yaml
import threading
from collections import deque


import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

import tf2_ros
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from std_msgs.msg import String, Int32
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid

from explore_lite_msgs.msg import ExploreStatus
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_srvs.srv import Trigger

# --- Pure-Python Quaternions ---
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
# CONFIGURATION
# =============================================================================
STATION_A_MARKER_ID = 0
STATION_B_MARKER_ID = 1
REQUIRED_MARKERS = {STATION_A_MARKER_ID, STATION_B_MARKER_ID}

TARGET_DISTANCE        = 0.10   # m - Final dock distance
NAV2_APPROACH_DISTANCE = 0.50   # m - Precise runway distance from live marker
YAML_APPROACH_DISTANCE = 1.0    # m - Coarse approach distance using YAML pose
CAMERA_X_OFFSET        = 0.04   # m 
MAX_DOCK_RETRIES       = 3

UNDOCK_DISTANCE        = 0.30   # m - Distance to reverse after Station A
UNDOCK_SPEED           = 0.06   # m/s

VISIT_RADIUS  = 0.3 # m - Coverage sweep mark radius
GOAL_SPACING  = 0.5 # m - Coverage sweep grid spacing

EXPLORATION_TIMEOUT    = 600.0  # s - Max time to wait for EXPLORATION_COMPLETE before forcing proceed
SWEEP_PARTIAL_TIMEOUT  = 120.0  # s - BFS sweep timeout after at least one marker is found

# Frontier cleanup — short-hop BFS fallback. Runs AFTER explore_lite completes with
# missing markers. Instead of navigating all the way to a frontier in one shot, we
# BFS to the nearest frontier, trace a path through known free space, and hop along
# it in small increments (hop_distance). After each hop the map has updated, so we
# re-plan. Ported from frontier_short_hop.py.
FRONTIER_BLACKLIST_RADIUS     = 0.30  # m - failed-goal exclusion radius (> Nav2 xy_goal_tolerance=0.25)
FRONTIER_BLACKLIST_TTL        = 90.0  # s - time-decaying blacklist TTL
FRONTIER_MIN_CELLS            = 4     # min connected frontier size (at 0.05 m/cell ≈ 20 cm edge)
FRONTIER_MAX_ITERATIONS       = 30    # safety cap on cleanup nav attempts per run
FRONTIER_HOP_DISTANCE         = 0.6   # m - max travel per hop goal
FRONTIER_FINAL_STANDOFF       = 0.35  # m - stop this far from frontier on last hop
FRONTIER_CELL_CLEARANCE       = 0.22  # m - clearance for any cell we'd send the robot to
FRONTIER_FAILURES_BEFORE_SPIN = 3     # consecutive failures triggering a Spin recovery

# Station A — timed firing sequence (delay before each ball)
STATION_A_FIRE_DELAYS = [6.0, 9.0, 1.0]

# Station B — trigger-based firing (moving receptacle)
STATION_B_TRIGGER_MARKER_ID = 2     # ID of the marker that signals hole alignment
STATION_B_BALLS             = 3     # number of balls to fire at Station B
TRIGGER_COOLDOWN            = 5.0   # s - wait after firing for target to pass

# Station B cmd_vel approach tuning
ALIGN_TOL        = 0.03   # m  - |cam_x| threshold to finish alignment phase
BLIND_THRESHOLD  = 0.20   # m  - cam_z below which detection becomes unreliable
K_LINEAR         = 0.3    # m/s per m of distance error
K_ANGULAR        = 1.5    # rad/s per m of lateral error
MAX_LINEAR       = 0.08   # m/s
MAX_ANGULAR      = 0.50   # rad/s
APPROACH_TIMEOUT = 25.0   # s
LOST_MARKER_S    = 3.0    # s - tolerated gap before declaring marker lost

DOCK_POSES_FILE = '/tmp/aruco_dock_poses.yaml'
VISUAL_SERVO_STUBBED = False  # set True for bench testing without robot

class UltimateMissionController(Node):
    def __init__(self):
        super().__init__('ultimate_mission_controller')

        # Declare parameter to allow clearing the pose cache on startup.
        # Default True: the map frame is anchored to each SLAM run's start pose,
        # so poses from a previous run are in a different coordinate system and
        # MUST NOT be reused. Set clear_cache:=false only if you know the map
        # frame has been preserved (e.g. reusing a saved map).
        self.declare_parameter('clear_cache', True)
        if self.get_parameter('clear_cache').value:
            self.get_logger().info('Clearing old dock poses cache...')
            if os.path.exists(DOCK_POSES_FILE):
                os.remove(DOCK_POSES_FILE)

        # State Variables
        self.detected_markers = {}  # {id: (map_x, map_y, normal_yaw)}
        self.exploration_done = threading.Event()

        # Frontier cleanup state — time-decaying blacklist: [(x, y, timestamp_sec), ...]
        self.frontier_blacklist = []

        # Costmap data for clearance checking during frontier cleanup
        self.costmap_data = None
        self.costmap_resolution = None
        self.costmap_origin = None

        # Live Camera Data
        self.latest_cam_x = 0.0
        self.latest_cam_z = 0.0
        self.latest_stamp = 0.0
        self.latest_stamp_by_id = {}   # {marker_id: timestamp}
        self.latest_cam_by_id = {}  # {marker_id: (cam_x, cam_z, stamp)}
        self.latest_marker_q_by_id = {}  # {marker_id: (qx, qy, qz, qw)}
        self._cam_lock = threading.Lock()
        self._marker_lock = threading.Lock()

        # Station B trigger — set by marker_cb when STATION_B_TRIGGER_MARKER_ID is seen
        self.trigger_event = threading.Event()

        # Map Data
        self.map_data = None
        self.map_resolution = 0.05
        self.map_origin = None

        # Load any previously saved YAML states
        self.load_poses()

        # --- ROS 2 Interfaces ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cmd_vel_pub          = self.create_publisher(Twist, '/cmd_vel', 10)
        self.phase_pub            = self.create_publisher(String, '/mission_phase', 10)
        self.trigger_count_pub    = self.create_publisher(Int32, '/station_b_trigger_count', 10)
        self.balls_fired_pub      = self.create_publisher(Int32, '/station_b_balls_fired', 10)
        self._stb_trigger_count   = 0
        self._stb_balls_fired     = 0
        self.flywheel_start_client = self.create_client(Trigger, '/start_flywheel')
        self.fire_ball_client      = self.create_client(Trigger, '/fire_ball')
        self.flywheel_stop_client  = self.create_client(Trigger, '/stop_flywheel')
        
        self.create_subscription(MarkerArray, '/aruco/markers', self.marker_cb, 
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.create_subscription(ExploreStatus, 'explore/status', self.explore_status_cb,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_cb, 10)

        # Start the sequential mission thread
        self.mission_thread = threading.Thread(target=self.run_mission, daemon=True)
        self.mission_thread.start()

    def _pub_int(self, publisher, value: int):
        msg = Int32()
        msg.data = value
        publisher.publish(msg)

    def _call_service(self, client, label):
        """Generic blocking service call. Returns True on success."""
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'{label} service not available — skipping.')
            return False
        future = client.call_async(Trigger.Request())
        while not future.done():
            time.sleep(0.05)
        result = future.result()
        if result and result.success:
            self.get_logger().info(f'{label}: {result.message}')
            return True
        self.get_logger().error(f'{label} service call failed.')
        return False

    def _fire_sequence(self, delays, station_name):
        """
        Spin up flywheel, fire one ball per delay entry (delay = wait BEFORE that ball),
        then stop flywheel. Flywheel runs continuously across all shots.
        """
        log = self.get_logger()
        log.info(f'{station_name}: starting flywheel...')
        self._call_service(self.flywheel_start_client, 'start_flywheel')

        for i, delay in enumerate(delays):
            if delay > 0.0:
                log.info(f'{station_name}: waiting {delay}s before ball {i + 1}...')
                time.sleep(delay)
            log.info(f'{station_name}: firing ball {i + 1}/{len(delays)}')
            self._call_service(self.fire_ball_client, 'fire_ball')

        log.info(f'{station_name}: all {len(delays)} balls fired — stopping flywheel.')
        self._call_service(self.flywheel_stop_client, 'stop_flywheel')

    def _publish_phase(self, phase: str):
        msg = String()
        msg.data = phase
        self.phase_pub.publish(msg)

    # =========================================================================
    # PERSISTENCE (YAML)
    # =========================================================================
    def load_poses(self):
        if not os.path.exists(DOCK_POSES_FILE):
            return
        try:
            with open(DOCK_POSES_FILE, 'r') as fh:
                data = yaml.safe_load(fh) or {}
            for raw_id, pose in data.items():
                self.detected_markers[int(raw_id)] = (pose['x'], pose['y'], pose['yaw'])
            self.get_logger().info(f'Loaded {len(self.detected_markers)} dock poses from YAML: {list(self.detected_markers.keys())}')
        except Exception as exc:
            self.get_logger().warn(f'Could not load {DOCK_POSES_FILE}: {exc}')

    def save_poses(self):
        with self._marker_lock:
            data = {mid: {'x': p[0], 'y': p[1], 'yaw': p[2]} for mid, p in self.detected_markers.items()}
        tmp_path = DOCK_POSES_FILE + '.tmp'
        try:
            with open(tmp_path, 'w') as fh:
                yaml.dump(data, fh)
            os.replace(tmp_path, DOCK_POSES_FILE)
        except Exception as exc:
            self.get_logger().warn(f'Could not write {DOCK_POSES_FILE}: {exc}')

    # =========================================================================
    # CALLBACKS
    # =========================================================================
    def explore_status_cb(self, msg: ExploreStatus):
        if msg.status == ExploreStatus.EXPLORATION_COMPLETE:
            if not self.exploration_done.is_set():
                self.get_logger().info('>> EXPLORATION COMPLETE SIGNAL RECEIVED <<')
                self.exploration_done.set()

    def map_cb(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((h, w))

    def costmap_cb(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        self.costmap_resolution = msg.info.resolution
        self.costmap_origin = msg.info.origin
        self.costmap_data = np.array(msg.data, dtype=np.int8).reshape((h, w))

    def marker_cb(self, msg: MarkerArray):
        if not msg.markers: return

        for marker in msg.markers:
            p = marker.pose.position
            if p.z <= 0.0: continue

            now = time.monotonic()
            q = marker.pose.orientation
            with self._cam_lock:
                self.latest_cam_x = p.x          # kept for backward compat with visual_servo
                self.latest_cam_z = p.z
                self.latest_stamp = now
                self.latest_stamp_by_id[marker.id] = now
                self.latest_cam_by_id[marker.id] = (p.x, p.z, now)
                self.latest_marker_q_by_id[marker.id] = (q.x, q.y, q.z, q.w)

            # Station B trigger marker — signal the fire event
            if marker.id == STATION_B_TRIGGER_MARKER_ID:
                self.trigger_event.set()

            marker_id = marker.id

            # Reject obviously poor sightings (too far, too close, too lateral).
            # These are the same quality gates we used for first sightings — we
            # don't want to overwrite a good stored pose with a garbage update.
            if abs(p.x) > 0.15 or p.z > 2.0 or p.z < 0.15:
                continue
            map_pose = self.camera_to_map(p.x, p.z, marker.pose.orientation)
            if not map_pose:
                continue

            with self._marker_lock:
                was_known = marker_id in self.detected_markers
                self.detected_markers[marker_id] = map_pose
            self.save_poses()
            if was_known:
                self.get_logger().debug(
                    f'MARKER REFRESHED: ID={marker_id} at Map({map_pose[0]:.2f}, {map_pose[1]:.2f})')
            else:
                self.get_logger().info(
                    f'NEW MARKER LOGGED: ID={marker_id} at Map({map_pose[0]:.2f}, {map_pose[1]:.2f})')

    def camera_to_map(self, cam_x, cam_z, q_msg):
        """Converts observation to Map coordinates AND the outward Normal Yaw"""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None

        _, _, robot_yaw = euler_from_quaternion([tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w])
        
        fwd, lat = CAMERA_X_OFFSET + cam_z, -cam_x
        map_x = tf.transform.translation.x + fwd * math.cos(robot_yaw) - lat * math.sin(robot_yaw)
        map_y = tf.transform.translation.y + fwd * math.sin(robot_yaw) + lat * math.cos(robot_yaw)

        qx, qy, qz, qw = q_msg.x, q_msg.y, q_msg.z, q_msg.w
        nx, nz = 2.0*(qx*qz + qw*qy), 1.0 - 2.0*(qx*qx + qy*qy)
        if nz > 0: nx, nz = -nx, -nz
        
        norm_map_x = nz * math.cos(robot_yaw) - (-nx) * math.sin(robot_yaw)
        norm_map_y = nz * math.sin(robot_yaw) + (-nx) * math.cos(robot_yaw)
        normal_yaw = math.atan2(norm_map_y, norm_map_x)

        return (map_x, map_y, normal_yaw)

    # =========================================================================
    # MISSION SEQUENCE
    # =========================================================================
    def run_mission(self):
        navigator = BasicNavigator()
        log = self.get_logger()

        log.info('Waiting for TF Tree...')
        while rclpy.ok():
            try:
                self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                break
            except Exception:
                time.sleep(0.5)

        # --- PHASE 1: EXPLORE ---
        log.info('='*50 + '\nPHASE 1: EXPLORATION\n' + '='*50)
        self._publish_phase('EXPLORE')

        with self._marker_lock:
            have_all = REQUIRED_MARKERS.issubset(self.detected_markers.keys())

        if have_all:
            log.info('YAML states loaded! Bypassing exploration wait.')
        else:
            signalled = self.exploration_done.wait(timeout=EXPLORATION_TIMEOUT)
            if not signalled:
                log.warn('Exploration timeout — EXPLORATION_COMPLETE never received. Forcing proceed.')
            self.exploration_done.clear()

            with self._marker_lock:
                have_all = REQUIRED_MARKERS.issubset(self.detected_markers.keys())
                found = list(self.detected_markers.keys())
            log.info(f'Exploration ended. Markers found so far: {found}')

            if not have_all:
                log.warn('Not all markers found — running in-process frontier cleanup.')
                self._publish_phase('FRONTIER_CLEANUP')
                self.run_frontier_cleanup(navigator)

        # --- PHASE 2: SWEEP (If needed) ---
        with self._marker_lock:
            have_all = REQUIRED_MARKERS.issubset(self.detected_markers.keys())
        if not have_all:
            log.info('='*50 + '\nPHASE 2: COVERAGE SWEEP\n' + '='*50)
            self._publish_phase('SWEEP')
            self.run_coverage_sweep(navigator)

        # --- PHASE 3: DOCK STATION A ---
        with self._marker_lock:
            have_a = STATION_A_MARKER_ID in self.detected_markers
            have_b = STATION_B_MARKER_ID in self.detected_markers

        if not have_a and not have_b:
            log.error('='*50)
            log.error('MISSION ABORT: Neither Station A nor Station B marker was ever found.')
            log.error('Exploration and coverage sweep both failed to locate required markers.')
            log.error('='*50)
            self._publish_phase('ABORTED')
            navigator.destroy_node()
            return

        if have_a:
            self._publish_phase('DOCK_A')
            if self.execute_docking(navigator, STATION_A_MARKER_ID, "Station A"):
                log.info('>>> STATION A: FIRING SEQUENCE <<<')
                self._fire_sequence(STATION_A_FIRE_DELAYS, "Station A")
                self.execute_undock()
        else:
            log.error('Skipping Station A — marker never detected.')

        # --- PHASE 4: DOCK STATION B ---
        if have_b:
            self._publish_phase('DOCK_B')
            if self.execute_docking_b(navigator):
                log.info('>>> STATION B: FIRING SEQUENCE <<<')
                self._fire_station_b_sequence()
        else:
            log.error('Skipping Station B — marker never detected.')

        self._publish_phase('COMPLETE')
        log.info('='*50 + '\nALL MISSIONS COMPLETE!\n' + '='*50)
        navigator.destroy_node()

    # =========================================================================
    # HARDWARE & DOCKING LOGIC
    # =========================================================================
    def _wait_for_live_marker(self, marker_id, timeout=10.0):
        """Wait for a fresh live detection of marker_id and return (map_x, map_y, normal_yaw) or None."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._cam_lock:
                entry = self.latest_cam_by_id.get(marker_id)
                q_tuple = self.latest_marker_q_by_id.get(marker_id)
            if entry is not None and q_tuple is not None:
                cam_x, cam_z, stamp = entry
                if time.monotonic() - stamp < 2.0:
                    q_msg = Quaternion(x=q_tuple[0], y=q_tuple[1], z=q_tuple[2], w=q_tuple[3])
                    return self.camera_to_map(cam_x, cam_z, q_msg)
            time.sleep(0.1)
        return None

    def execute_docking(self, navigator, marker_id, name):
        log = self.get_logger()

        with self._marker_lock:
            if marker_id not in self.detected_markers:
                log.error(f'{name} Marker (ID:{marker_id}) missing. Skipping.')
                return False
            map_x, map_y, normal_yaw = self.detected_markers[marker_id]

        for attempt in range(1, MAX_DOCK_RETRIES + 1):
            log.info(f'{name} dock attempt {attempt}/{MAX_DOCK_RETRIES}...')

            # Step 1: Navigate toward stored marker, stopping YAML_APPROACH_DISTANCE short
            try:
                tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                robot_x = tf.transform.translation.x
                robot_y = tf.transform.translation.y
            except Exception:
                log.warn(f'{name}: TF lookup failed — falling back to normal-based approach.')
                robot_x = map_x + YAML_APPROACH_DISTANCE * math.cos(normal_yaw)
                robot_y = map_y + YAML_APPROACH_DISTANCE * math.sin(normal_yaw)

            dx = map_x - robot_x
            dy = map_y - robot_y
            dist = math.hypot(dx, dy)

            if dist > YAML_APPROACH_DISTANCE:
                approach_x = map_x - YAML_APPROACH_DISTANCE * (dx / dist)
                approach_y = map_y - YAML_APPROACH_DISTANCE * (dy / dist)
            else:
                approach_x = robot_x
                approach_y = robot_y
            approach_yaw = math.atan2(dy, dx)

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = navigator.get_clock().now().to_msg()
            goal.pose.position.x, goal.pose.position.y = approach_x, approach_y
            q = quaternion_from_euler(0, 0, approach_yaw)
            goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = q

            log.info(f'Navigating to {name} approach area (YAML pose)...')
            navigator.goToPose(goal)
            while not navigator.isTaskComplete():
                time.sleep(0.1)

            if navigator.getResult() != TaskResult.SUCCEEDED:
                log.warn(f'Failed to reach {name} approach area on attempt {attempt}.')
                continue

            # Step 2: Navigate to runway
            with self._marker_lock:
                if marker_id not in self.detected_markers:
                    log.warn(f'{name}: marker lost — skipping attempt {attempt}.')
                    continue
                map_x, map_y, normal_yaw = self.detected_markers[marker_id]

            runway_x   = map_x + NAV2_APPROACH_DISTANCE * math.cos(normal_yaw)
            runway_y   = map_y + NAV2_APPROACH_DISTANCE * math.sin(normal_yaw)
            runway_yaw = math.atan2(-math.sin(normal_yaw), -math.cos(normal_yaw))

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = navigator.get_clock().now().to_msg()
            goal.pose.position.x, goal.pose.position.y = runway_x, runway_y
            qx, qy, qz, qw = quaternion_from_euler(0, 0, runway_yaw)
            goal.pose.orientation.x = qx
            goal.pose.orientation.y = qy
            goal.pose.orientation.z = qz
            goal.pose.orientation.w = qw

            log.info(f'Navigating to {name} runway...')
            navigator.goToPose(goal)
            while not navigator.isTaskComplete():
                time.sleep(0.1)

            if navigator.getResult() != TaskResult.SUCCEEDED:
                log.warn(f'Failed to reach {name} runway on attempt {attempt}.')
                continue

            # Step 3: Visual servo to close the final gap
            log.info(f'{name}: runway reached — engaging visual servo...')
            if self.visual_servo():
                return True
            log.warn(f'{name} visual servo failed on attempt {attempt}.')

        log.error(f'{name} docking failed after {MAX_DOCK_RETRIES} attempts.')
        return False

    def visual_servo(self):
        if VISUAL_SERVO_STUBBED:
            self.get_logger().warn('[TEST MODE] Visual servo stubbed — returning success.')
            return True
        twist = Twist()
        deadline = time.monotonic() + 25.0
        last_cam_z = None

        while time.monotonic() < deadline:
            with self._cam_lock:
                stamp = self.latest_stamp
                cam_x = self.latest_cam_x
                cam_z = self.latest_cam_z

            if time.monotonic() - stamp > 2.0:
                break   # lost marker

            last_cam_z  = cam_z
            dist_err    = cam_z - TARGET_DISTANCE

            if abs(dist_err) < 0.01 and abs(cam_x) < 0.03:
                self.cmd_vel_pub.publish(Twist())
                return True

            twist.linear.x  = float(max(-0.08, min(0.08, 0.3 * dist_err)))
            twist.angular.z = float(max(-0.5,  min(0.5, -1.5 * cam_x)))
            if abs(cam_x) > 0.05:
                twist.linear.x = 0.0

            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        # Dead-reckon if marker was seen at least once
        if last_cam_z is not None:
            remaining = last_cam_z - TARGET_DISTANCE
            if remaining > 0:
                twist.linear.x, twist.angular.z = 0.04, 0.0
                t_end = time.monotonic() + (remaining / 0.04)
                while time.monotonic() < min(t_end, deadline):
                    self.cmd_vel_pub.publish(twist)
                    time.sleep(0.05)

        self.cmd_vel_pub.publish(Twist())
        return False

    def execute_undock(self):
        self.get_logger().info(f'Undocking: Reversing {UNDOCK_DISTANCE}m...')
        twist = Twist()
        twist.linear.x = -UNDOCK_SPEED
        t_end = time.monotonic() + (UNDOCK_DISTANCE / UNDOCK_SPEED)
        while time.monotonic() < t_end:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self.cmd_vel_pub.publish(Twist())

    # =========================================================================
    # STATION B — MOVING RECEPTACLE DOCKING
    # =========================================================================
    def _cmd_vel_approach_moving(self):
        """
        3-phase cmd_vel approach adapted from aruco_dock_moving.py.

        Phase 1 — Align : rotate in place until |cam_x| < ALIGN_TOL.
        Phase 2 — Drive : proportional control; exit to Phase 3 when cam_z
                          drops below BLIND_THRESHOLD (camera blind zone).
        Phase 3 — Reckon: dead-reckon remaining gap at half speed.

        Main thread (MultiThreadedExecutor) keeps marker_cb firing — no
        spin_once needed here.
        """
        log      = self.get_logger()
        twist    = Twist()
        deadline = time.monotonic() + APPROACH_TIMEOUT

        def stop():
            self.cmd_vel_pub.publish(Twist())

        def fresh():
            with self._cam_lock:
                entry = self.latest_cam_by_id.get(STATION_B_MARKER_ID)
            if entry is None:
                return False
            _, _, stamp = entry
            return (time.monotonic() - stamp) < LOST_MARKER_S

        # ── Phase 1: Align ────────────────────────────────────────────────────
        log.info('Station B approach — Phase 1: aligning...')
        while time.monotonic() < deadline:
            if not fresh():
                with self._cam_lock:
                    first = STATION_B_MARKER_ID not in self.latest_cam_by_id
                if first:
                    time.sleep(0.05)
                    continue
                stop()
                log.error('Marker lost during alignment — aborting.')
                return False

            with self._cam_lock:
                cam_x, _, _ = self.latest_cam_by_id[STATION_B_MARKER_ID]

            ang_cmd = float(max(-MAX_ANGULAR, min(MAX_ANGULAR, -K_ANGULAR * cam_x)))
            if abs(cam_x) < ALIGN_TOL:
                log.info(f'Aligned: cam_x={cam_x:.4f} m')
                break

            twist.linear.x  = 0.0
            twist.angular.z = ang_cmd
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        else:
            stop()
            log.error('Alignment timed out.')
            return False

        # ── Phase 2: Drive ────────────────────────────────────────────────────
        log.info('Station B approach — Phase 2: driving...')
        last_cam_z = float('inf')

        while time.monotonic() < deadline:
            if not fresh():
                if last_cam_z < BLIND_THRESHOLD:
                    log.info(f'Marker lost in blind zone (last cam_z={last_cam_z:.3f} m) — dead-reckoning.')
                    break
                stop()
                log.error(f'Marker lost unexpectedly (last cam_z={last_cam_z:.3f} m) — aborting.')
                return False

            with self._cam_lock:
                cam_x, cam_z, _ = self.latest_cam_by_id[STATION_B_MARKER_ID]
            last_cam_z = cam_z
            dist_err   = cam_z - TARGET_DISTANCE

            if abs(dist_err) < 0.01 and abs(cam_x) < ALIGN_TOL:
                stop()
                log.info(f'Approach complete: cam_x={cam_x:.4f} m  cam_z={cam_z:.4f} m')
                return True

            twist.linear.x  = float(max(-MAX_LINEAR, min(MAX_LINEAR,  K_LINEAR  * dist_err)))
            twist.angular.z = float(max(-MAX_ANGULAR, min(MAX_ANGULAR, -K_ANGULAR * cam_x)))
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        else:
            stop()
            log.error('Drive phase timed out.')
            return False

        # ── Phase 3: Dead-reckoning ───────────────────────────────────────────
        remaining = last_cam_z - TARGET_DISTANCE
        if remaining <= 0.0:
            stop()
            log.info('Already at TARGET_DISTANCE — done.')
            return True

        drive_speed = MAX_LINEAR * 0.5
        t_end = min(time.monotonic() + (remaining / drive_speed), deadline)
        log.info(f'Station B approach — Phase 3: dead-reckoning {remaining:.3f} m...')
        while time.monotonic() < t_end:
            twist.linear.x  = drive_speed
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        stop()
        log.info('Dead-reckoning complete — docked.')
        return True

    def execute_docking_b(self, navigator):
        """Station B docking — uses alignment-first cmd_vel approach for moving target."""
        log = self.get_logger()

        with self._marker_lock:
            if STATION_B_MARKER_ID not in self.detected_markers:
                log.error('Station B marker missing. Skipping.')
                return False
            map_x, map_y, normal_yaw = self.detected_markers[STATION_B_MARKER_ID]

        runway_x   = map_x + NAV2_APPROACH_DISTANCE * math.cos(normal_yaw)
        runway_y   = map_y + NAV2_APPROACH_DISTANCE * math.sin(normal_yaw)
        runway_yaw = math.atan2(-math.sin(normal_yaw), -math.cos(normal_yaw))

        for attempt in range(1, MAX_DOCK_RETRIES + 1):
            log.info(f'Station B dock attempt {attempt}/{MAX_DOCK_RETRIES}...')

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = navigator.get_clock().now().to_msg()
            goal.pose.position.x, goal.pose.position.y = runway_x, runway_y
            q = quaternion_from_euler(0, 0, runway_yaw)
            goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = q

            log.info('Navigating to Station B runway...')
            navigator.goToPose(goal)
            while not navigator.isTaskComplete():
                time.sleep(0.1)

            if navigator.getResult() != TaskResult.SUCCEEDED:
                log.warn(f'Failed to reach Station B runway on attempt {attempt}.')
                continue

            log.info('Runway reached. Engaging moving-target approach...')
            if self._cmd_vel_approach_moving():
                return True
            log.warn(f'Station B approach failed on attempt {attempt}.')

        log.error(f'Station B docking failed after {MAX_DOCK_RETRIES} attempts.')
        return False

    def _fire_station_b_sequence(self):
        """
        Fire STATION_B_BALLS balls at Station B.
        Each ball is fired when the trigger marker (ID=STATION_B_TRIGGER_MARKER_ID)
        is detected, indicating the hole has moved into position.
        """
        log = self.get_logger()
        log.info('Station B: starting flywheel...')
        self._call_service(self.flywheel_start_client, 'start_flywheel')

        self._stb_trigger_count = 0
        self._stb_balls_fired   = 0
        self._pub_int(self.trigger_count_pub, 0)
        self._pub_int(self.balls_fired_pub, 0)

        for ball in range(1, STATION_B_BALLS + 1):
            # Drain any stale trigger state and wait for a fresh sighting
            self.trigger_event.clear()
            log.info(f'Station B: waiting for trigger marker (ball {ball}/{STATION_B_BALLS})...')
            detected = self.trigger_event.wait(timeout=30.0)
            if not detected:
                log.error(f'Station B: trigger timeout — no signal for ball {ball}. Aborting launch sequence.')
                break

            self._stb_trigger_count += 1
            self._pub_int(self.trigger_count_pub, self._stb_trigger_count)
            log.info(f'Station B: hole detected — firing ball {ball}')
            self._call_service(self.fire_ball_client, 'fire_ball')
            self._stb_balls_fired = ball
            self._pub_int(self.balls_fired_pub, self._stb_balls_fired)

            if ball < STATION_B_BALLS:
                # Wait for the trigger marker to leave the camera frame for at least
                # TRIGGER_COOLDOWN seconds before accepting the next trigger. This
                # prevents the same pass of the receptacle from firing multiple balls.
                log.info(f'Station B: cooldown — waiting for trigger marker to clear...')
                cooldown_deadline = time.monotonic() + TRIGGER_COOLDOWN
                while time.monotonic() < cooldown_deadline:
                    with self._cam_lock:
                        last_trigger_seen = self.latest_stamp_by_id.get(STATION_B_TRIGGER_MARKER_ID, 0.0)
                    if time.monotonic() - last_trigger_seen > TRIGGER_COOLDOWN:
                        break
                    time.sleep(0.05)

        log.info('Station B: all balls fired — stopping flywheel.')
        self._call_service(self.flywheel_stop_client, 'stop_flywheel')

    # =========================================================================
    # FRONTIER CLEANUP — short-hop BFS fallback
    # =========================================================================
    # Ported from standalone frontier_short_hop.py. Runs AFTER explore_lite
    # signals EXPLORATION_COMPLETE but some required markers are still missing.
    #
    # Per iteration:
    #   1. BFS to the nearest frontier cell, recording parents.
    #   2. Trace the BFS parent chain back to get a path through known free space.
    #   3. Walk along that path until hop_distance or final_standoff_dist.
    #   4. Send that intermediate point as a NavigateToPose goal.
    #   5. On arrival the map has updated — re-run BFS. Repeat.
    def run_frontier_cleanup(self, navigator):
        log = self.get_logger()

        while self.map_data is None or self.map_origin is None:
            log.info('Frontier cleanup waiting for /map...')
            time.sleep(0.5)

        consecutive_failures = 0
        final_retry_done = False

        for iteration in range(FRONTIER_MAX_ITERATIONS):
            with self._marker_lock:
                if REQUIRED_MARKERS.issubset(self.detected_markers.keys()):
                    log.info('All required markers found — ending frontier cleanup.')
                    return

            try:
                tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                robot_x = tf.transform.translation.x
                robot_y = tf.transform.translation.y
            except Exception as e:
                log.warn(f'TF lookup failed during frontier cleanup: {e}')
                time.sleep(0.5)
                continue

            # Recovery spin if we've failed too many times in a row
            if consecutive_failures >= FRONTIER_FAILURES_BEFORE_SPIN:
                log.warn(
                    f'{consecutive_failures} consecutive failures — issuing recovery spin.'
                )
                consecutive_failures = 0
                navigator.spin(spin_dist=2 * math.pi, time_allowance=20)
                while not navigator.isTaskComplete():
                    time.sleep(0.1)
                continue

            h, w = self.map_data.shape
            sr, sc = self._world_to_grid(robot_x, robot_y)
            if not (0 <= sr < h and 0 <= sc < w):
                log.warn('Robot outside map bounds')
                time.sleep(0.5)
                continue

            frontier_rc, parents = self._bfs_to_nearest_frontier(sr, sc)
            if frontier_rc is None:
                unk = int(np.sum(self.map_data == -1))
                if not final_retry_done and len(self.frontier_blacklist) > 0:
                    log.info(
                        f'No reachable frontiers — clearing blacklist '
                        f'({len(self.frontier_blacklist)} entries) for final retry pass. '
                        f'Unknown cells: {unk}')
                    self.frontier_blacklist.clear()
                    final_retry_done = True
                    continue
                log.info(
                    f'No reachable frontiers remain. Unknown cells: {unk}')
                return

            # Found a frontier — reset final-retry flag
            final_retry_done = False

            fx, fy = self._grid_to_world(*frontier_rc)
            hop = self._pick_hop_target(frontier_rc, parents, (robot_x, robot_y))
            if hop is None:
                now = time.monotonic()
                log.warn(
                    f'Frontier ({fx:.2f},{fy:.2f}): no clear hop cell — blacklisting')
                self.frontier_blacklist.append((fx, fy, now))
                consecutive_failures += 1
                continue

            hx, hy, yaw, travelled = hop
            log.info(
                f'Frontier cleanup [{iteration + 1}/{FRONTIER_MAX_ITERATIONS}]: '
                f'frontier ({fx:.2f},{fy:.2f}); hop to ({hx:.2f},{hy:.2f}) '
                f'~{travelled:.2f}m')

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = navigator.get_clock().now().to_msg()
            goal.pose.position.x = hx
            goal.pose.position.y = hy
            q = quaternion_from_euler(0, 0, yaw)
            goal.pose.orientation.x, goal.pose.orientation.y = q[0], q[1]
            goal.pose.orientation.z, goal.pose.orientation.w = q[2], q[3]

            # Blame the frontier (not the hop) on failure
            current_blacklist_xy = (fx, fy)

            navigator.goToPose(goal)
            while not navigator.isTaskComplete():
                with self._marker_lock:
                    have_all = REQUIRED_MARKERS.issubset(self.detected_markers.keys())
                if have_all:
                    navigator.cancelTask()
                    log.info('Missing markers found mid-transit — cancelling cleanup goal.')
                    return
                time.sleep(0.1)

            if navigator.getResult() != TaskResult.SUCCEEDED:
                log.warn(
                    f'NavigateToPose failed for frontier ({fx:.2f},{fy:.2f}) — blacklisting.')
                now = time.monotonic()
                self.frontier_blacklist.append((*current_blacklist_xy, now))
                consecutive_failures += 1
            else:
                log.info('Hop reached — re-planning on fresh map.')
                consecutive_failures = 0

        log.warn(
            f'Frontier cleanup hit iteration cap ({FRONTIER_MAX_ITERATIONS}) '
            f'without completing the required marker set.'
        )

    # ---- frontier helpers (map -> grid, BFS, short-hop) -------------------

    def _world_to_grid(self, x, y):
        col = int((x - self.map_origin.position.x) / self.map_resolution)
        row = int((y - self.map_origin.position.y) / self.map_resolution)
        return row, col

    def _grid_to_world(self, row, col):
        x = self.map_origin.position.x + (col + 0.5) * self.map_resolution
        y = self.map_origin.position.y + (row + 0.5) * self.map_resolution
        return x, y

    def _is_frontier_cell(self, r, c):
        """A free cell (0) with at least one unknown (-1) 4-neighbour."""
        h, w = self.map_data.shape
        if self.map_data[r, c] != 0:
            return False
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w:
                if self.map_data[nr, nc] == -1:
                    return True
        return False

    def _has_clearance(self, r, c, radius_m):
        """
        Check that a cell has no obstacles within radius_m. Prefer the
        live global costmap (inflation + dynamic obstacles + static layer).
        Fall back to the raw SLAM map if costmap not yet received.
        """
        wx, wy = self._grid_to_world(r, c)
        if self.costmap_data is not None:
            cw = self.costmap_data.shape[1]
            ch = self.costmap_data.shape[0]
            cc_col = int((wx - self.costmap_origin.position.x)
                         / self.costmap_resolution)
            cc_row = int((wy - self.costmap_origin.position.y)
                         / self.costmap_resolution)
            cells = int(math.ceil(radius_m / self.costmap_resolution))
            r0 = max(0, cc_row - cells)
            r1 = min(ch, cc_row + cells + 1)
            c0 = max(0, cc_col - cells)
            c1 = min(cw, cc_col + cells + 1)
            patch = self.costmap_data[r0:r1, c0:c1]
            return not np.any((patch >= 50) & (patch != 255))
        # Fallback: raw SLAM map
        cells = int(math.ceil(radius_m / self.map_resolution))
        h, w = self.map_data.shape
        r0, r1 = max(0, r - cells), min(h, r + cells + 1)
        c0, c1 = max(0, c - cells), min(w, c + cells + 1)
        return not np.any(self.map_data[r0:r1, c0:c1] > 0)

    def _prune_frontier_blacklist(self):
        now = time.monotonic()
        self.frontier_blacklist = [
            (x, y, t) for (x, y, t) in self.frontier_blacklist
            if now - t < FRONTIER_BLACKLIST_TTL]

    def _is_frontier_blacklisted(self, x, y):
        self._prune_frontier_blacklist()
        return any(math.hypot(x - bx, y - by) < FRONTIER_BLACKLIST_RADIUS
                   for bx, by, _ in self.frontier_blacklist)

    def _frontier_cluster(self, r, c, cap=200):
        """Flood-fill connected frontier cells, return list of (r,c)."""
        seen = {(r, c)}
        q = deque([(r, c)])
        cluster = []
        while q and len(cluster) < cap:
            cr, cc = q.popleft()
            cluster.append((cr, cc))
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                nr, nc = cr + dr, cc + dc
                if (nr, nc) not in seen and self._is_frontier_cell(nr, nc):
                    seen.add((nr, nc))
                    q.append((nr, nc))
        return cluster

    def _snap_centroid_to_path(self, cluster, parents, sr, sc):
        """
        Compute the cluster centroid and find the cluster cell closest to
        it that is also reachable (in the BFS parents map).
        """
        cr_avg = sum(r for r, _ in cluster) / len(cluster)
        cc_avg = sum(c for _, c in cluster) / len(cluster)
        reachable = [(r, c) for (r, c) in cluster if (r, c) in parents]
        if not reachable:
            candidates = []
            for (r, c) in cluster:
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nr, nc = r + dr, c + dc
                    if (nr, nc) in parents:
                        candidates.append((nr, nc))
            if not candidates:
                return None
            reachable = candidates
        return min(reachable,
                   key=lambda rc: (rc[0] - cr_avg) ** 2 + (rc[1] - cc_avg) ** 2)

    def _bfs_to_nearest_frontier(self, sr, sc):
        """
        BFS over free cells, recording parents. When we hit a frontier
        cell, flood-fill its connected cluster and use the centroid
        (snapped to nearest reachable free cell) as the goal candidate.
        Returns (frontier_rc, parents) or (None, parents).
        """
        h, w = self.map_data.shape
        seen = np.zeros_like(self.map_data, dtype=bool)
        parents = {}
        q = deque([(sr, sc)])
        seen[sr, sc] = True
        parents[(sr, sc)] = None

        while q:
            cr, cc = q.popleft()
            if self._is_frontier_cell(cr, cc):
                cluster = self._frontier_cluster(cr, cc)
                if len(cluster) >= FRONTIER_MIN_CELLS:
                    centroid_rc = self._snap_centroid_to_path(
                        cluster, parents, sr, sc)
                    if centroid_rc is not None:
                        wx, wy = self._grid_to_world(*centroid_rc)
                        if not self._is_frontier_blacklisted(wx, wy):
                            return centroid_rc, parents
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = cr + dr, cc + dc
                if (0 <= nr < h and 0 <= nc < w and not seen[nr, nc]
                        and self.map_data[nr, nc] == 0):
                    seen[nr, nc] = True
                    parents[(nr, nc)] = (cr, cc)
                    q.append((nr, nc))
        return None, parents

    def _pick_hop_target(self, frontier_rc, parents, robot_xy):
        """
        Walk the BFS path from robot -> frontier and pick the cell that is
        either (a) hop_distance metres from the robot along the path, or
        (b) final_standoff_dist metres before the frontier — whichever
        comes first. The cell must have full clearance.
        Returns (x, y, yaw, accumulated_dist) or None.
        """
        path = []
        cur = frontier_rc
        while cur is not None:
            path.append(cur)
            cur = parents.get(cur)
        path.reverse()  # robot -> frontier

        if len(path) < 2:
            return None

        rx, ry = robot_xy
        fx, fy = self._grid_to_world(*frontier_rc)

        accumulated = 0.0
        prev_xy = (rx, ry)
        chosen = None

        for cell in path[1:]:
            cx, cy = self._grid_to_world(*cell)
            accumulated += math.hypot(cx - prev_xy[0], cy - prev_xy[1])
            prev_xy = (cx, cy)

            dist_to_frontier = math.hypot(fx - cx, fy - cy)

            if accumulated >= FRONTIER_HOP_DISTANCE:
                if self._has_clearance(*cell, FRONTIER_CELL_CLEARANCE):
                    chosen = cell
                    break
                continue

            if dist_to_frontier <= FRONTIER_FINAL_STANDOFF:
                if self._has_clearance(*cell, FRONTIER_CELL_CLEARANCE):
                    chosen = cell
                    break

        if chosen is None:
            for cell in reversed(path[1:]):
                if self._has_clearance(*cell, FRONTIER_CELL_CLEARANCE):
                    chosen = cell
                    break

        if chosen is None:
            return None

        cx, cy = self._grid_to_world(*chosen)
        yaw = math.atan2(fy - cy, fx - cx)
        return cx, cy, yaw, accumulated

    # =========================================================================
    # BFS COVERAGE SWEEP
    # =========================================================================
    def run_coverage_sweep(self, navigator):
        log = self.get_logger()
        while self.map_data is None or self.map_origin is None:
            time.sleep(0.5)

        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x, robot_y = tf.transform.translation.x, tf.transform.translation.y
        except Exception as e:
            log.error(f'Cannot get robot pose for sweep: {e}')
            return

        waypoints = self._bfs_all_waypoints(robot_x, robot_y)
        log.info(f'BFS sweep planned: {len(waypoints)} waypoints.')

        partial_deadline = None  # set when at least one marker is found

        for i, (row, col) in enumerate(waypoints):
            with self._marker_lock:
                have_all = REQUIRED_MARKERS.issubset(self.detected_markers.keys())
                have_any = len(self.detected_markers) > 0
            if have_all:
                log.info('All markers found! Ending sweep early.')
                return
            if have_any and partial_deadline is None:
                partial_deadline = time.monotonic() + SWEEP_PARTIAL_TIMEOUT
                log.info(f'At least one marker found — sweep will timeout in {SWEEP_PARTIAL_TIMEOUT}s if the rest are not found.')
            if partial_deadline is not None and time.monotonic() > partial_deadline:
                log.warn(f'Sweep timeout — could not find all markers within {SWEEP_PARTIAL_TIMEOUT}s of first detection. Proceeding with what we have.')
                return

            gx = self.map_origin.position.x + (col + 0.5) * self.map_resolution
            gy = self.map_origin.position.y + (row + 0.5) * self.map_resolution

            try:
                tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                rx, ry = tf.transform.translation.x, tf.transform.translation.y
            except Exception:
                rx, ry = gx, gy

            goal_yaw = math.atan2(gy - ry, gx - rx)

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = navigator.get_clock().now().to_msg()
            goal.pose.position.x, goal.pose.position.y = gx, gy
            q = quaternion_from_euler(0, 0, goal_yaw)
            goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = q

            navigator.goToPose(goal)
            while not navigator.isTaskComplete():
                with self._marker_lock:
                    have_all = REQUIRED_MARKERS.issubset(self.detected_markers.keys())
                if have_all:
                    navigator.cancelTask()
                    log.info('All markers found mid-transit! Ending sweep early.')
                    return
                if partial_deadline is not None and time.monotonic() > partial_deadline:
                    navigator.cancelTask()
                    log.warn(f'Sweep timeout mid-transit — proceeding with found markers.')
                    return
                time.sleep(0.1)

        log.info('BFS sweep complete — all waypoints visited.')

    def _bfs_all_waypoints(self, robot_x, robot_y):
        map_h, map_w = self.map_data.shape
        stride = max(1, int(GOAL_SPACING / self.map_resolution))
        orig_x = self.map_origin.position.x
        orig_y = self.map_origin.position.y

        rc = max(0, min(map_w - 1, (int((robot_x - orig_x) / self.map_resolution) // stride) * stride))
        rr = max(0, min(map_h - 1, (int((robot_y - orig_y) / self.map_resolution) // stride) * stride))

        queue = deque([(rr, rc)])
        seen = {(rr, rc)}
        waypoints = []

        while queue:
            r, c = queue.popleft()
            if self.map_data[r, c] == 0:
                waypoints.append((r, c))

            for dr, dc in [(-stride, 0), (stride, 0), (0, -stride), (0, stride)]:
                nr, nc = r + dr, c + dc
                if (nr, nc) not in seen and 0 <= nr < map_h and 0 <= nc < map_w and 0 <= self.map_data[nr, nc] <= 50:
                    seen.add((nr, nc))
                    queue.append((nr, nc))

        return waypoints

def main(args=None):
    rclpy.init(args=args)
    node = UltimateMissionController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.cmd_vel_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()