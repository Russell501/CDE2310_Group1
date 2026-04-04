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
import subprocess
from collections import deque

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
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
NAV2_APPROACH_DISTANCE = 0.50   # m - Runway distance (Nav2 stops here)
CAMERA_X_OFFSET        = 0.04   # m 
MAX_DOCK_RETRIES       = 3

UNDOCK_DISTANCE        = 0.30   # m - Distance to reverse after Station A
UNDOCK_SPEED           = 0.06   # m/s
LAUNCH_SIGNAL_TIMEOUT  = 30.0   # s - Wait time for Station B launch signal

VISIT_RADIUS  = 0.3 # m - Coverage sweep mark radius
GOAL_SPACING  = 0.5 # m - Coverage sweep grid spacing

MAX_EXPLORE_RESTARTS   = 2
EXPLORATION_TIMEOUT    = 600.0  # s - Max time to wait for EXPLORATION_COMPLETE before forcing proceed

# Ball firing sequences — (pre-fire delay in seconds, ) for each ball
# Station A: ball 1 immediately, ball 2 after 9 s, ball 3 after 1 s
STATION_A_FIRE_DELAYS = [0.0, 9.0, 1.0]
# Station B: adjust delays as required
STATION_B_FIRE_DELAYS = [0.0, 1.0, 1.0]

DOCK_POSES_FILE = '/tmp/aruco_dock_poses.yaml'

class UltimateMissionController(Node):
    def __init__(self):
        super().__init__('ultimate_mission_controller')

        # State Variables
        self.detected_markers = {}  # {id: (map_x, map_y, normal_yaw)}
        self.exploration_done = threading.Event()
        
        # Live Camera Data
        self.latest_cam_x = 0.0
        self.latest_cam_z = 0.0
        self.latest_stamp = 0.0
        self.latest_stamp_by_id = {}   # {marker_id: timestamp}
        self.launch_arm_time = -1.0    # For Station B
        self._cam_lock = threading.Lock()
        self._marker_lock = threading.Lock()

        # Map Data
        self.map_data = None
        self.map_resolution = 0.05
        self.map_origin = None

        # Load any previously saved YAML states
        self.load_poses()

        # --- ROS 2 Interfaces ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cmd_vel_pub     = self.create_publisher(Twist, '/cmd_vel', 10)
        self.phase_pub       = self.create_publisher(String, '/mission_phase', 10)
        self.flywheel_start_client = self.create_client(Trigger, '/start_flywheel')
        self.fire_ball_client      = self.create_client(Trigger, '/fire_ball')
        self.flywheel_stop_client  = self.create_client(Trigger, '/stop_flywheel')
        
        self.create_subscription(MarkerArray, '/aruco/markers', self.marker_cb, 
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.create_subscription(ExploreStatus, 'explore/status', self.explore_status_cb,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)

        # Start the sequential mission thread
        self.mission_thread = threading.Thread(target=self.run_mission, daemon=True)
        self.mission_thread.start()

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
    # EXPLORE_LITE RESTART
    # =========================================================================
    def restart_explore_lite(self):
        self.get_logger().info('Killing explore_lite process...')
        subprocess.run(['pkill', '-f', 'explore_lite'], check=False)
        time.sleep(2.0)  # Let it fully die before relaunching
        self.get_logger().info('Relaunching explore_lite...')
        subprocess.Popen([
            'ros2', 'launch', 'explore_lite', 'explore.launch.py',
            # Uncomment if using a custom params file:
            # 'params_file:=/absolute/path/to/explore_params.yaml',
        ])

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
        self.map_data = np.array(msg.data).reshape((h, w))

    def marker_cb(self, msg: MarkerArray):
        if not msg.markers: return

        for marker in msg.markers:
            p = marker.pose.position
            if p.z <= 0.0: continue

            now = time.monotonic()
            with self._cam_lock:
                self.latest_cam_x = p.x
                self.latest_cam_z = p.z
                self.latest_stamp = now
                self.latest_stamp_by_id[marker.id] = now

            marker_id = marker.id
            with self._marker_lock:
                already_known = marker_id in self.detected_markers

            if not already_known:
                map_pose = self.camera_to_map(p.x, p.z, marker.pose.orientation)
                if map_pose:
                    with self._marker_lock:
                        self.detected_markers[marker_id] = map_pose
                    self.save_poses()
                    self.get_logger().info(f'NEW MARKER LOGGED: ID={marker_id} at Map({map_pose[0]:.2f}, {map_pose[1]:.2f})')

    def camera_to_map(self, cam_x, cam_z, q_msg):
        """Converts observation to Map coordinates AND the outward Normal Yaw"""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
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
            for attempt in range(MAX_EXPLORE_RESTARTS + 1):
                signalled = self.exploration_done.wait(timeout=EXPLORATION_TIMEOUT)
                if not signalled:
                    log.warn('Exploration timeout — EXPLORATION_COMPLETE never received. Forcing proceed.')
                self.exploration_done.clear()

                with self._marker_lock:
                    have_all = REQUIRED_MARKERS.issubset(self.detected_markers.keys())
                    found = list(self.detected_markers.keys())

                log.info(f'Exploration ended. Markers found so far: {found}')

                if have_all or attempt == MAX_EXPLORE_RESTARTS:
                    break

                log.warn(f'Not all markers found. Restarting explore_lite ({attempt + 1}/{MAX_EXPLORE_RESTARTS})...')
                self._publish_phase('REMAP')
                self.restart_explore_lite()

        # --- PHASE 2: SWEEP (If needed) ---
        with self._marker_lock:
            have_all = REQUIRED_MARKERS.issubset(self.detected_markers.keys())
        if not have_all:
            log.info('='*50 + '\nPHASE 2: COVERAGE SWEEP\n' + '='*50)
            self._publish_phase('SWEEP')
            self.run_coverage_sweep(navigator)

        # --- PHASE 3: DOCK STATION A ---
        self._publish_phase('DOCK_A')
        if self.execute_docking(navigator, STATION_A_MARKER_ID, "Station A"):
            log.info('>>> STATION A: FIRING SEQUENCE <<<')
            self._fire_sequence(STATION_A_FIRE_DELAYS, "Station A")
            self.execute_undock()

        # --- PHASE 4: DOCK STATION B ---
        self._publish_phase('DOCK_B')
        if self.execute_docking(navigator, STATION_B_MARKER_ID, "Station B"):
            self.launch_arm_time = time.monotonic()
            if self.wait_for_launch_signal():
                log.info('>>> STATION B: FIRING SEQUENCE <<<')
                self._fire_sequence(STATION_B_FIRE_DELAYS, "Station B")
            else:
                log.warn('Station B: Launch signal missed. Aborting fire sequence.')

        self._publish_phase('COMPLETE')
        log.info('='*50 + '\nALL MISSIONS COMPLETE!\n' + '='*50)
        navigator.destroy_node()

    # =========================================================================
    # HARDWARE & DOCKING LOGIC
    # =========================================================================
    def execute_docking(self, navigator, marker_id, name):
        log = self.get_logger()

        with self._marker_lock:
            if marker_id not in self.detected_markers:
                log.error(f'{name} Marker (ID:{marker_id}) missing. Skipping.')
                return False
            map_x, map_y, normal_yaw = self.detected_markers[marker_id]

        runway_x = map_x + NAV2_APPROACH_DISTANCE * math.cos(normal_yaw)
        runway_y = map_y + NAV2_APPROACH_DISTANCE * math.sin(normal_yaw)
        runway_yaw = math.atan2(-math.sin(normal_yaw), -math.cos(normal_yaw))

        for attempt in range(1, MAX_DOCK_RETRIES + 1):
            log.info(f'{name} dock attempt {attempt}/{MAX_DOCK_RETRIES}...')

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = navigator.get_clock().now().to_msg()
            goal.pose.position.x, goal.pose.position.y = runway_x, runway_y
            q = quaternion_from_euler(0, 0, runway_yaw)
            goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = q

            log.info(f'Navigating to {name} Runway...')
            navigator.goToPose(goal)
            while not navigator.isTaskComplete():
                time.sleep(0.1)

            if navigator.getResult() != TaskResult.SUCCEEDED:
                log.warn(f'Failed to reach {name} runway on attempt {attempt}.')
                continue

            log.info('Runway reached. Engaging Visual Servoing...')
            if self.visual_servo():
                return True
            log.warn(f'{name} visual servo failed on attempt {attempt}.')

        log.error(f'{name} docking failed after {MAX_DOCK_RETRIES} attempts.')
        return False

    def visual_servo(self):
        self.get_logger().info('[TEST MODE] Visual servo stubbed — returning success.')
        return True  # TODO: remove for real deployment

        twist = Twist()
        deadline = time.monotonic() + 25.0
        last_cam_z = None   # None means marker not yet seen

        while time.monotonic() < deadline:
            with self._cam_lock:
                stamp = self.latest_stamp
                cam_x = self.latest_cam_x
                cam_z = self.latest_cam_z

            if time.monotonic() - stamp > 2.0:
                break  # Lost marker, fall through to dead-reckon

            last_cam_z = cam_z
            dist_err = cam_z - TARGET_DISTANCE

            if abs(dist_err) < 0.01 and abs(cam_x) < 0.03:
                self.cmd_vel_pub.publish(Twist())
                return True  # Explicit success

            twist.linear.x = float(max(-0.08, min(0.08, 0.3 * dist_err)))
            twist.angular.z = float(max(-0.5, min(0.5, -1.5 * cam_x)))
            if abs(cam_x) > 0.05:
                twist.linear.x = 0.0

            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        # Dead-reckoning: only attempt if marker was seen at least once
        if last_cam_z is None:
            self.cmd_vel_pub.publish(Twist())
            return False  # Never saw the marker

        remaining = last_cam_z - TARGET_DISTANCE
        if remaining > 0:
            twist.linear.x, twist.angular.z = 0.04, 0.0
            t_end = time.monotonic() + (remaining / 0.04)
            while time.monotonic() < min(t_end, deadline):
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.05)

        self.cmd_vel_pub.publish(Twist())
        return False  # Dead-reckoning: best-effort, success unconfirmed

    def execute_undock(self):
        self.get_logger().info(f'Undocking: Reversing {UNDOCK_DISTANCE}m...')
        twist = Twist()
        twist.linear.x = -UNDOCK_SPEED
        t_end = time.monotonic() + (UNDOCK_DISTANCE / UNDOCK_SPEED)
        while time.monotonic() < t_end:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self.cmd_vel_pub.publish(Twist())

    def wait_for_launch_signal(self):
        self.get_logger().info('Waiting for ArUco Launch Signal (Station B marker)...')
        deadline = time.monotonic() + LAUNCH_SIGNAL_TIMEOUT
        while time.monotonic() < deadline:
            with self._cam_lock:
                stamp = self.latest_stamp_by_id.get(STATION_B_MARKER_ID, -1.0)
            if stamp > self.launch_arm_time:
                return True
            time.sleep(0.1)
        return False

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

        for i, (row, col) in enumerate(waypoints):
            with self._marker_lock:
                have_all = REQUIRED_MARKERS.issubset(self.detected_markers.keys())
            if have_all:
                log.info('Missing markers found! Ending sweep early.')
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
                    return
                time.sleep(0.1)

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
                if (nr, nc) not in seen and 0 <= nr < map_h and 0 <= nc < map_w and self.map_data[nr, nc] <= 50:
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