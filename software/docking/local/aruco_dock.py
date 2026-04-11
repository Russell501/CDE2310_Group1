#!/usr/bin/env python3
"""
aruco_dock.py — Run on the LOCAL LAPTOP (not the RPi).

Nav2-based ArUco docking. Captures a single snapshot of the marker pose,
converts it to a map-frame Nav2 goal, lets Nav2 navigate with full obstacle
avoidance, then verifies the final position and retries once if needed.

Prerequisites:
  1. On RPi: aruco_live.py publishing visualization_msgs/MarkerArray on /aruco/markers
  2. On laptop: Full Nav2 stack active (map_server, bt_navigator, planner, controller)
  3. On laptop: Localization active via Cartographer (map→odom TF exists)
  4. On laptop:
       source /opt/ros/humble/setup.bash
       export ROS_DOMAIN_ID=30
  5. On laptop (if not already installed):
       sudo apt install ros-humble-tf-transformations
     Or the inline fallback below handles it automatically.

Camera mount: front of robot, middle plate, facing forward.
  CAMERA_X_OFFSET is the forward distance from base_link origin to the camera.
  Measure this on your robot and update the constant below.
"""

import math
import time

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
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
        roll = math.atan2(t0, t1)
        t2 = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(t2)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw

    def quaternion_from_euler(roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5),   math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5),  math.sin(roll * 0.5)
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
MARKER_WAIT_TIMEOUT    = 30.0   # s — give up if no marker seen within this time
MAX_RETRIES            = 4      # number of retries after the first attempt

# Forward distance from base_link origin to the camera lens.
# The camera is mounted at the FRONT of the robot on the middle plate.
# Measure this on your robot (approx 80 mm for TurtleBot3 middle plate front).
CAMERA_X_OFFSET      = 0.04   # m


# ---------------------------------------------------------------------------
# MarkerListener — minimal Node for /aruco/markers subscription
# ---------------------------------------------------------------------------

class MarkerListener(Node):
    """
    Subscribes to /aruco/markers and captures a single pose snapshot on demand.

    Call arm() to prepare for a new reading, then drive this node with
    rclpy.spin_once() until snapshot_ready is True.
    """

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
        # Always-current readings used by the cmd_vel approach loop
        self.latest_cam_x: float  = 0.0
        self.latest_cam_z: float  = 0.0
        self.latest_stamp: float  = 0.0   # monotonic time of last received marker

    def arm(self) -> None:
        """Arm for the next valid marker message, discarding any prior reading."""
        self.snapshot_ready = False
        self._armed = True

    def _cb(self, msg: MarkerArray) -> None:
        if not msg.markers:
            return
        p = msg.markers[0].pose.position
        q = msg.markers[0].pose.orientation
        if p.z <= 0.0:          # reject obviously corrupt detections
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
    """
    Arm MarkerListener and spin until a valid snapshot arrives or timeout.

    Returns True on success, False on timeout.
    Only spins marker_node so BasicNavigator's executor is not disturbed.
    """
    marker_node.arm()
    log      = marker_node.get_logger()
    deadline = time.monotonic() + timeout_s
    last_warn = time.monotonic()

    while not marker_node.snapshot_ready:
        if time.monotonic() > deadline:
            log.error(
                f'No marker received within {timeout_s:.0f} s — '
                'is aruco_live.py running and the marker in view?')
            return False
        if time.monotonic() - last_warn > 5.0:
            log.warn('Waiting for /aruco/markers …')
            last_warn = time.monotonic()
        rclpy.spin_once(marker_node, timeout_sec=0.05)

    return True


def camera_to_map_goal(
    cam_x: float,
    cam_z: float,
    cam_q: tuple,
    navigator: BasicNavigator,
    tf_buffer: tf2_ros.Buffer,
) -> 'PoseStamped | None':
    """
    Convert a camera-frame marker observation to a map-frame PoseStamped docking goal.
    Calculates the Normal vector to ensure a perpendicular head-on approach.
    """
    try:
        tf_stamped = tf_buffer.lookup_transform(
            'map',
            'base_link',
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=2),
        )
    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as exc:
        navigator.get_logger().error(f'TF lookup map→base_link failed: {exc}')
        return None

    t   = tf_stamped.transform.translation
    q   = tf_stamped.transform.rotation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    robot_x = t.x
    robot_y = t.y

    # 1. Camera → base_link
    fwd = CAMERA_X_OFFSET + cam_z
    lat = -cam_x

    # 2. Extract Marker Normal Vector from Quaternion
    qx, qy, qz, qw = cam_q
    # Rotate [0, 0, 1] by quaternion to get the marker's Z-axis vector
    nx = 2.0 * (qx * qz + qw * qy)
    nz = 1.0 - 2.0 * (qx * qx + qy * qy)

    # 3. Safeguard: Ensure the vector points OUT towards the camera (-Z in cam frame)
    if nz > 0:
        nx = -nx
        nz = -nz

    # 4. Marker Normal Vector in base_link frame
    norm_fwd = nz
    norm_lat = -nx

    # Normalize the 2D floor vector
    norm_len = math.hypot(norm_fwd, norm_lat)
    if norm_len > 1e-6:
        norm_fwd /= norm_len
        norm_lat /= norm_len
    else:
        norm_fwd, norm_lat = -1.0, 0.0 # Fallback

    # 5. Convert Marker Position to Map Frame
    marker_x = robot_x + math.cos(yaw) * fwd - math.sin(yaw) * lat
    marker_y = robot_y + math.sin(yaw) * fwd + math.cos(yaw) * lat

    # 6. Convert Normal Vector to Map Frame
    norm_map_x = math.cos(yaw) * norm_fwd - math.sin(yaw) * norm_lat
    norm_map_y = math.sin(yaw) * norm_fwd + math.cos(yaw) * norm_lat

    # 7. Calculate Final Goal Pose
    # Place goal NAV2_APPROACH_DISTANCE straight out from the marker's face
    goal_x = marker_x + NAV2_APPROACH_DISTANCE * norm_map_x
    goal_y = marker_y + NAV2_APPROACH_DISTANCE * norm_map_y

    # Robot must face the opposite direction of the outward normal
    goal_yaw = math.atan2(-norm_map_y, -norm_map_x)

    log = navigator.get_logger()
    log.info(
        f'Marker map pos: ({marker_x:.3f}, {marker_y:.3f})  '
        f'robot map pos: ({robot_x:.3f}, {robot_y:.3f})')
    log.info(
        f'Nav2 goal: ({goal_x:.3f}, {goal_y:.3f})  '
        f'yaw: {math.degrees(goal_yaw):.1f}°')

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp    = navigator.get_clock().now().to_msg()
    goal.pose.position.x = goal_x
    goal.pose.position.y = goal_y
    goal.pose.position.z = 0.0
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, goal_yaw)
    goal.pose.orientation.x = qx
    goal.pose.orientation.y = qy
    goal.pose.orientation.z = qz
    goal.pose.orientation.w = qw
    return goal


def navigate_to_goal(
    navigator: BasicNavigator,
    goal_pose: PoseStamped,
    marker_node: MarkerListener,
) -> TaskResult:
    """
    Send a goal to Nav2 and block until complete, logging progress at ~1 Hz.
    Spins marker_node each iteration to keep the TF buffer alive.

    Returns the TaskResult enum value.
    """
    if not navigator.goToPose(goal_pose):
        navigator.get_logger().error('Nav2 rejected the goal.')
        return TaskResult.FAILED

    log        = navigator.get_logger()
    last_log_t = time.monotonic()

    while not navigator.isTaskComplete():
        rclpy.spin_once(marker_node, timeout_sec=0.0)   # keep TF buffer fed
        feedback = navigator.getFeedback()
        if feedback and time.monotonic() - last_log_t >= 1.0:
            log.info(f'Navigating — distance remaining: {feedback.distance_remaining:.2f} m')
            last_log_t = time.monotonic()

    result = navigator.getResult()
    log.info(f'Nav2 finished — result: {result}')
    return result


def cmd_vel_approach(
    marker_node: MarkerListener,
    cmd_vel_pub,
    log,
) -> bool:
    """
    Three-phase cmd_vel approach to TARGET_DISTANCE in front of the marker.

    Phase 1 — Align  : rotate in place until marker is centred (|cam_x| < ALIGN_TOL).
                       Ensures the robot faces the marker head-on before driving.
    Phase 2 — Drive  : proportional forward + angular correction until cam_z drops
                       below BLIND_THRESHOLD (close-range detection limit).
    Phase 3 — Reckon : marker invisible at close range; dead-reckon the remaining gap.

    Returns True when the robot reaches TARGET_DISTANCE, False on abort.
    """
    K_LINEAR         = 0.3    # m/s per m of distance error
    K_ANGULAR        = 1.5    # rad/s per m of lateral error
    MAX_LINEAR       = 0.08   # m/s — lower cap for close-range safety
    MAX_ANGULAR      = 0.50   # rad/s
    ALIGN_TOL        = 0.03   # m — |cam_x| threshold to exit alignment phase
    BLIND_THRESHOLD  = 0.20   # m — cam_z below which detection becomes unreliable
    APPROACH_TIMEOUT = 25.0   # s
    LOST_MARKER_S    = 3.0    # s — tolerated gap before declaring marker lost

    twist    = Twist()
    deadline = time.monotonic() + APPROACH_TIMEOUT

    def stop():
        twist.linear.x  = 0.0
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)

    def fresh():
        """True if the latest marker reading arrived within LOST_MARKER_S."""
        return (marker_node.latest_stamp > 0.0 and
                time.monotonic() - marker_node.latest_stamp < LOST_MARKER_S)

    # ── Phase 1: Align ───────────────────────────────────────────────────────
    log.info('Phase 1 — aligning with marker …')
    while time.monotonic() < deadline:
        rclpy.spin_once(marker_node, timeout_sec=0.05)
        if not fresh():
            if marker_node.latest_stamp == 0.0:
                continue   # still waiting for first reading
            log.error('Marker lost during alignment — aborting.')
            stop()
            return False

        cam_x   = marker_node.latest_cam_x
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
    last_cam_z = float('inf')   # only trusted once Phase 2 receives a reading

    while time.monotonic() < deadline:
        rclpy.spin_once(marker_node, timeout_sec=0.05)

        if not fresh():
            # Marker lost — decide whether to dead-reckon or abort
            if last_cam_z < BLIND_THRESHOLD:
                log.info(
                    f'Marker lost in blind zone (last cam_z={last_cam_z:.4f} m) — '
                    'switching to dead-reckoning.')
                break   # → Phase 3
            log.error(
                f'Marker lost unexpectedly (last cam_z={last_cam_z:.4f} m) — aborting.')
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

    drive_speed = MAX_LINEAR * 0.5   # half speed for the blind section
    t_drive     = remaining / drive_speed
    log.info(
        f'Phase 3 — dead-reckoning {remaining:.3f} m at {drive_speed:.2f} m/s '
        f'({t_drive:.2f} s) …')
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

    # BasicNavigator manages its own internal executor.
    # MarkerListener is driven manually with rclpy.spin_once().
    # Do NOT put both in a shared MultiThreadedExecutor — it will deadlock.
    navigator   = BasicNavigator()
    marker_node = MarkerListener()
    cmd_vel_pub = marker_node.create_publisher(Twist, '/cmd_vel', 10)

    # TransformListener runs a background thread that populates tf_buffer
    # from /tf and /tf_static. Attach it to marker_node for a shared clock.
    tf_buffer   = tf2_ros.Buffer()
    _tf_listener = tf2_ros.TransformListener(tf_buffer, marker_node)  # noqa: F841

    log = navigator.get_logger()

    # Skip waitUntilNav2Active() entirely — it requires lifecycle manager services
    # that may not exist in your Nav2 launch configuration.
    # Instead, just verify the map→base_link TF chain is available.
    log.info('Checking that map→base_link TF is available (SLAM localization) …')
    tf_ok = False
    for i in range(60):  # wait up to 30 s
        try:
            tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            tf_ok = True
            break
        except Exception:
            rclpy.spin_once(marker_node, timeout_sec=0.1)
            if i % 10 == 0:
                log.info('Waiting for map→base_link transform …')

    if not tf_ok:
        log.error('map→base_link TF not available after 30 s. '
                  'Is your SLAM node running?')
        marker_node.destroy_node()
        navigator.destroy_node()
        rclpy.shutdown()
        return

    log.info('TF chain OK — ready to dock.')

    try:
        for attempt in range(MAX_RETRIES + 1):
            log.info(f'=== Docking attempt {attempt + 1}/{MAX_RETRIES + 1} ===')

            # ── STEP 1: WAIT ────────────────────────────────────────────────
            log.info('Waiting for marker snapshot …')
            if not wait_for_snapshot(marker_node):
                log.error('Aborting: could not get a marker reading.')
                break

            cam_x = marker_node.cam_x
            cam_z = marker_node.cam_z
            cam_q = marker_node.cam_q

            # ── STEP 2: COMPUTE ─────────────────────────────────────────────
            # Give the TF buffer a moment to have current transforms
            for _ in range(10):
                rclpy.spin_once(marker_node, timeout_sec=0.05)

            goal_pose = camera_to_map_goal(cam_x, cam_z, cam_q, navigator, tf_buffer)
            if goal_pose is None:
                log.error('Aborting: could not compute a valid goal pose.')
                break

            # ── STEP 3: NAVIGATE ────────────────────────────────────────────
            result = navigate_to_goal(navigator, goal_pose, marker_node)

            if result == TaskResult.CANCELED:
                log.warn('Nav2 goal was canceled.')
                break
            elif result != TaskResult.SUCCEEDED:
                log.error('Nav2 failed to reach the goal. Aborting.')
                break

            log.info('Nav2 reached goal successfully.')

            # ── STEP 4: CMD_VEL APPROACH ────────────────────────────────────
            if cmd_vel_approach(marker_node, cmd_vel_pub, log):
                log.info('Docking SUCCESSFUL — robot is within tolerance.')
                break

            # ── STEP 5: RETRY ───────────────────────────────────────────────
            if attempt < MAX_RETRIES:
                log.warn('cmd_vel approach failed — retrying …')
            else:
                log.error(f'Docking FAILED after {MAX_RETRIES + 1} attempt(s).')

    except KeyboardInterrupt:
        log.info('Interrupted — canceling any active Nav2 task.')
        navigator.cancelTask()

    finally:
        marker_node.destroy_node()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
