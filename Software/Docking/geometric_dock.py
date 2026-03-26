#!/usr/bin/env python3
"""
aruco_dock.py — Run on the LOCAL LAPTOP (not the RPi).

Hybrid ArUco docking (No Nav2 Required!):
  Phase 1: Open-loop geometric "Dog-Leg" maneuver to a 0.5m runway.
  Phase 2: Closed-loop 3-phase visual approach (Align, Drive, Reckon).

Prerequisites:
  1. On RPi: aruco_live.py publishing visualization_msgs/MarkerArray on /aruco/markers
  2. On laptop: source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=30
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray

# ---------------------------------------------------------------------------
# Tuning constants
# ---------------------------------------------------------------------------
TARGET_DISTANCE        = 0.10   # m — final docking distance
RUNWAY_DISTANCE        = 0.50   # m — Dog-leg goal distance
LATERAL_TOLERANCE      = 0.01   # m — |cam_x| must be within this to succeed
DISTANCE_TOLERANCE     = 0.01   # m — |cam_z - TARGET_DISTANCE| to succeed
MARKER_WAIT_TIMEOUT    = 30.0   # s — give up if no marker seen within this time
MAX_RETRIES            = 4      # number of retries after the first attempt

CAMERA_X_OFFSET        = 0.04   # m

# --- DOG-LEG HARDWARE TUNING MULTIPLIERS ---
GEOM_LINEAR_SPEED  = 0.10  # m/s
GEOM_ANGULAR_SPEED = 0.40  # rad/s
LINEAR_CORRECTION  = 1.15  # Multiplier if it drives too short
ANGULAR_CORRECTION = 0.85  # Multiplier if it turns too much


def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

# ---------------------------------------------------------------------------
# MarkerListener — minimal Node for /aruco/markers subscription
# ---------------------------------------------------------------------------

class MarkerListener(Node):
    def __init__(self) -> None:
        super().__init__('aruco_marker_listener')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(MarkerArray, '/aruco/markers', self._cb, qos)

        self._armed: bool         = False
        self.snapshot_ready: bool = False
        self.cam_x: float         = 0.0
        self.cam_z: float         = 0.0
        self.cam_q: tuple         = (0.0, 0.0, 0.0, 1.0)
        
        # Always-current readings used by the cmd_vel approach loop
        self.latest_cam_x: float  = 0.0
        self.latest_cam_z: float  = 0.0
        self.latest_stamp: float  = 0.0

    def arm(self) -> None:
        """Arm for the next valid marker message, discarding any prior reading."""
        self.snapshot_ready = False
        self._armed = True

    def _cb(self, msg: MarkerArray) -> None:
        if not msg.markers:
            return
        p = msg.markers[0].pose.position
        q = msg.markers[0].pose.orientation
        if p.z <= 0.0:          
            return
            
        # Always keep latest reading current (used by cmd_vel approach)
        self.latest_cam_x = p.x
        self.latest_cam_z = p.z
        self.latest_stamp = time.monotonic()
        if not self._armed:
            return
            
        self.cam_x = p.x
        self.cam_z = p.z
        self.cam_q = (q.x, q.y, q.z, q.w)
        self._armed = False
        self.snapshot_ready = True
        self.get_logger().info(
            f'Marker snapshot: cam_x={self.cam_x:.4f} m  cam_z={self.cam_z:.4f} m')

# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------

def wait_for_snapshot(
    marker_node: MarkerListener,
    timeout_s: float = MARKER_WAIT_TIMEOUT,
) -> bool:
    marker_node.arm()
    log      = marker_node.get_logger()
    deadline = time.monotonic() + timeout_s
    last_warn = time.monotonic()

    while not marker_node.snapshot_ready:
        if time.monotonic() > deadline:
            log.error('No marker received within timeout — is aruco_live.py running?')
            return False
        if time.monotonic() - last_warn > 5.0:
            log.warn('Waiting for /aruco/markers …')
            last_warn = time.monotonic()
        rclpy.spin_once(marker_node, timeout_sec=0.05)

    return True

def execute_blind_move(cmd_vel_pub, log, linear: float, angular: float, duration: float, name: str):
    """Publishes cmd_vel blindly for a specific amount of time for the Dog-Leg."""
    log.info(f'{name} ({duration:.2f}s)...')
    twist = Twist()
    twist.linear.x = float(linear)
    twist.angular.z = float(angular)
    end_time = time.monotonic() + duration
    while time.monotonic() < end_time:
        cmd_vel_pub.publish(twist)
        time.sleep(0.05)
    cmd_vel_pub.publish(Twist())
    time.sleep(0.5)

def cmd_vel_approach(
    marker_node: MarkerListener,
    cmd_vel_pub,
    log,
) -> bool:
    """
    Three-phase cmd_vel approach to TARGET_DISTANCE in front of the marker.
    Phase 1 — Align  : rotate in place until marker is centred.
    Phase 2 — Drive  : proportional forward + angular correction.
    Phase 3 — Reckon : marker invisible at close range; dead-reckon the remaining gap.
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
        return (marker_node.latest_stamp > 0.0 and
                time.monotonic() - marker_node.latest_stamp < LOST_MARKER_S)

    # ── Phase 1: Align ───────────────────────────────────────────────────────
    log.info('Phase 1 — aligning with marker …')
    while time.monotonic() < deadline:
        rclpy.spin_once(marker_node, timeout_sec=0.05)
        if not fresh():
            if marker_node.latest_stamp == 0.0:
                continue
            log.error('Marker lost during alignment — aborting.')
            stop()
            return False

        cam_x   = marker_node.latest_cam_x
        ang_cmd = float(max(-MAX_ANGULAR, min(MAX_ANGULAR, -K_ANGULAR * cam_x)))
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
        rclpy.spin_once(marker_node, timeout_sec=0.05)

        if not fresh():
            if last_cam_z < BLIND_THRESHOLD:
                log.info(f'Marker lost in blind zone (last cam_z={last_cam_z:.4f} m) — switching to dead-reckoning.')
                break   # → Phase 3
            log.error(f'Marker lost unexpectedly (last cam_z={last_cam_z:.4f} m) — aborting.')
            stop()
            return False

        cam_x      = marker_node.latest_cam_x
        cam_z      = marker_node.latest_cam_z
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

    # ── Phase 3: Dead-reckoning (blind zone) ─────────────────────────────────
    remaining = last_cam_z - TARGET_DISTANCE
    if remaining <= 0.0:
        stop()
        log.info('Already at or past TARGET_DISTANCE — done.')
        return True

    drive_speed = MAX_LINEAR * 0.5   
    t_drive     = remaining / drive_speed
    log.info(f'Phase 3 — dead-reckoning {remaining:.3f} m at {drive_speed:.2f} m/s ({t_drive:.2f} s) …')
    t_end = min(time.monotonic() + t_drive, deadline)
    while time.monotonic() < t_end:
        twist.linear.x  = drive_speed
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)
        rclpy.spin_once(marker_node, timeout_sec=0.02)
    stop()
    log.info('Dead-reckoning complete — docked.')
    return True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)

    marker_node = MarkerListener()
    cmd_vel_pub = marker_node.create_publisher(Twist, '/cmd_vel', 10)
    log = marker_node.get_logger()

    log.info('Starting Hybrid Docking: Open-Loop Dog-Leg -> Closed-Loop cmd_vel')

    try:
        for attempt in range(MAX_RETRIES + 1):
            log.info(f'=== Docking attempt {attempt + 1}/{MAX_RETRIES + 1} ===')

            # ── STEP 1: WAIT FOR SNAPSHOT ───────────────────────────────────
            log.info('Waiting for marker snapshot …')
            if not wait_for_snapshot(marker_node):
                log.error('Aborting: could not get a marker reading.')
                break

            cam_x = marker_node.cam_x
            cam_z = marker_node.cam_z
            cam_q = marker_node.cam_q

            # ── STEP 2: OPEN-LOOP DOG-LEG (RUNWAY APPROACH) ─────────────────
            # Calculate the geometry directly from the snapshot
            qx, qy, qz, qw = cam_q
            m_x = CAMERA_X_OFFSET + cam_z
            m_y = -cam_x

            nx = 2.0 * (qx * qz + qw * qy)
            nz = 1.0 - 2.0 * (qx * qx + qy * qy)
            if nz > 0:
                nx, nz = -nx, -nz
            norm_x, norm_y = nz, -nx

            length = math.hypot(norm_x, norm_y)
            if length > 1e-6:
                norm_x /= length
                norm_y /= length
            else:
                norm_x, norm_y = -1.0, 0.0

            t_x = m_x + RUNWAY_DISTANCE * norm_x
            t_y = m_y + RUNWAY_DISTANCE * norm_y

            theta_1 = math.atan2(t_y, t_x)
            distance = math.hypot(t_x, t_y)
            absolute_final_angle = math.atan2(-norm_y, -norm_x)
            theta_2 = normalize_angle(absolute_final_angle - theta_1)

            log.info(f'Dog-Leg: Drive {distance:.3f}m, Turn1 {math.degrees(theta_1):.1f}°, Turn2 {math.degrees(theta_2):.1f}°')

            # Execute blind maneuvers using the multipliers
            t1_time = (abs(theta_1) / GEOM_ANGULAR_SPEED) * ANGULAR_CORRECTION
            t1_dir = GEOM_ANGULAR_SPEED if theta_1 > 0 else -GEOM_ANGULAR_SPEED
            execute_blind_move(cmd_vel_pub, log, 0.0, t1_dir, t1_time, "Turn 1 (Approach)")

            drive_time = (distance / GEOM_LINEAR_SPEED) * LINEAR_CORRECTION
            execute_blind_move(cmd_vel_pub, log, GEOM_LINEAR_SPEED, 0.0, drive_time, "Drive (Hypotenuse)")

            t2_time = (abs(theta_2) / GEOM_ANGULAR_SPEED) * ANGULAR_CORRECTION
            t2_dir = GEOM_ANGULAR_SPEED if theta_2 > 0 else -GEOM_ANGULAR_SPEED
            execute_blind_move(cmd_vel_pub, log, 0.0, t2_dir, t2_time, "Turn 2 (Align to Runway)")

            # Give the camera a moment to catch up and grab a fresh frame
            time.sleep(1.0)
            for _ in range(10):
                rclpy.spin_once(marker_node, timeout_sec=0.05)

            # ── STEP 3: CLOSED-LOOP CMD_VEL APPROACH ────────────────────────
            if cmd_vel_approach(marker_node, cmd_vel_pub, log):
                log.info('Docking SUCCESSFUL — robot is within tolerance.')
                break

            # ── STEP 4: RETRY ───────────────────────────────────────────────
            if attempt < MAX_RETRIES:
                log.warn('cmd_vel approach failed — retrying the entire sequence…')
            else:
                log.error(f'Docking FAILED after {MAX_RETRIES + 1} attempt(s).')

    except KeyboardInterrupt:
        log.info('Interrupted — stopping robot.')
        cmd_vel_pub.publish(Twist())

    finally:
        marker_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
