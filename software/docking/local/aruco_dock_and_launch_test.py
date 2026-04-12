#!/usr/bin/env python3
"""
aruco_dock.py — Standalone docking + launch test.

Prerequisites: Cartographer and Nav2 must already be running.

Behaviour:
  1. Wait until any ArUco marker is detected.
  2. Convert its camera pose to a map-frame runway point.
  3. Navigate to the runway via Nav2.
  4. Visual-servo to the marker.
  5. Fire 3 balls (0 s → ball 1, wait 9 s → ball 2, wait 1 s → ball 3).

Run with:
    python3 aruco_dock.py
"""

import math
import time
import threading

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import MarkerArray
from std_srvs.srv import Trigger
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# =============================================================================
# CONFIGURATION
# =============================================================================
NAV2_APPROACH_DISTANCE = 0.50   # m — how far in front of the marker to stop
TARGET_DISTANCE        = 0.10   # m — final docking distance
CAMERA_X_OFFSET        = 0.04   # m — camera offset from base_link

FIRE_DELAYS = [6.0, 9.0, 1.0]  # pre-fire wait (s) before each of the 3 balls


# =============================================================================
# HELPERS
# =============================================================================
def euler_from_quaternion(q):
    x, y, z, w = q
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)   # yaw only


def quaternion_from_euler(yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)   # (x, y, z, w)


# =============================================================================
# NODE
# =============================================================================
class ArucoDockerNode(Node):
    def __init__(self):
        super().__init__('aruco_docker')

        self.target_marker = None       # (map_x, map_y, normal_yaw)
        self._marker_event = threading.Event()
        self._cam_lock = threading.Lock()
        self.latest_cam_x  = 0.0
        self.latest_cam_z  = 0.0
        self.latest_stamp  = 0.0

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_vel_pub           = self.create_publisher(Twist, '/cmd_vel', 10)
        self.flywheel_start_client = self.create_client(Trigger, '/start_flywheel')
        self.fire_ball_client      = self.create_client(Trigger, '/fire_ball')
        self.flywheel_stop_client  = self.create_client(Trigger, '/stop_flywheel')

        self.create_subscription(
            MarkerArray, '/aruco/markers', self._marker_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        self._mission_thread = threading.Thread(target=self._run, daemon=True)
        self._mission_thread.start()

    # -------------------------------------------------------------------------
    def _marker_cb(self, msg: MarkerArray):
        for marker in msg.markers:
            p = marker.pose.position
            if p.z <= 0.0:
                continue

            now = time.monotonic()
            with self._cam_lock:
                self.latest_cam_x = p.x
                self.latest_cam_z = p.z
                self.latest_stamp = now

            # Only log the first detection — use it as the docking target
            if self.target_marker is None:
                pose = self._camera_to_map(p.x, p.z, marker.pose.orientation)
                if pose:
                    self.target_marker = pose
                    self.get_logger().info(
                        f'Marker ID {marker.id} detected at map '
                        f'({pose[0]:.2f}, {pose[1]:.2f}) — docking!')
                    self._marker_event.set()

    def _camera_to_map(self, cam_x, cam_z, q_msg):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
        except Exception:
            return None

        robot_yaw = euler_from_quaternion([
            tf.transform.rotation.x, tf.transform.rotation.y,
            tf.transform.rotation.z, tf.transform.rotation.w,
        ])

        fwd, lat = CAMERA_X_OFFSET + cam_z, -cam_x
        map_x = tf.transform.translation.x + fwd * math.cos(robot_yaw) - lat * math.sin(robot_yaw)
        map_y = tf.transform.translation.y + fwd * math.sin(robot_yaw) + lat * math.cos(robot_yaw)

        qx, qy, qz, qw = q_msg.x, q_msg.y, q_msg.z, q_msg.w
        nx, nz = 2.0 * (qx * qz + qw * qy), 1.0 - 2.0 * (qx * qx + qy * qy)
        if nz > 0:
            nx, nz = -nx, -nz
        norm_map_x = nz * math.cos(robot_yaw) - (-nx) * math.sin(robot_yaw)
        norm_map_y = nz * math.sin(robot_yaw) + (-nx) * math.cos(robot_yaw)
        normal_yaw = math.atan2(norm_map_y, norm_map_x)

        return (map_x, map_y, normal_yaw)

    # -------------------------------------------------------------------------
    def _run(self):
        log = self.get_logger()
        navigator = BasicNavigator()

        # Wait for TF
        log.info('Waiting for TF tree...')
        while rclpy.ok():
            try:
                self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                break
            except Exception:
                time.sleep(0.5)

        # Wait for first marker detection
        log.info('Waiting for ArUco marker...')
        self._marker_event.wait()

        map_x, map_y, normal_yaw = self.target_marker

        # Navigate to runway
        runway_x   = map_x + NAV2_APPROACH_DISTANCE * math.cos(normal_yaw)
        runway_y   = map_y + NAV2_APPROACH_DISTANCE * math.sin(normal_yaw)
        runway_yaw = math.atan2(-math.sin(normal_yaw), -math.cos(normal_yaw))

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp    = navigator.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = runway_x, runway_y
        qx, qy, qz, qw = quaternion_from_euler(runway_yaw)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        log.info('Navigating to runway...')
        navigator.goToPose(goal)
        while not navigator.isTaskComplete():
            time.sleep(0.1)

        if navigator.getResult() != TaskResult.SUCCEEDED:
            log.error('Failed to reach runway. Aborting.')
            navigator.destroy_node()
            return

        # Visual servo
        log.info('Runway reached — engaging visual servo...')
        if not self._visual_servo():
            log.warn('Visual servo ended without explicit success — proceeding anyway.')

        # Fire sequence
        self._fire_sequence()

        log.info('Test complete.')
        navigator.destroy_node()

    # -------------------------------------------------------------------------
    def _visual_servo(self):
        twist    = Twist()
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

    # -------------------------------------------------------------------------
    def _call_service(self, client, label):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'{label} service unavailable — skipping.')
            return False
        future = client.call_async(Trigger.Request())
        while not future.done():
            time.sleep(0.05)
        result = future.result()
        return result is not None and result.success

    def _fire_sequence(self):
        log = self.get_logger()
        log.info('Starting flywheel...')
        self._call_service(self.flywheel_start_client, 'start_flywheel')

        for i, delay in enumerate(FIRE_DELAYS):
            if delay > 0.0:
                log.info(f'Waiting {delay}s before ball {i + 1}...')
                time.sleep(delay)
            log.info(f'Firing ball {i + 1}/{len(FIRE_DELAYS)}')
            self._call_service(self.fire_ball_client, 'fire_ball')

        log.info(f'All {len(FIRE_DELAYS)} balls fired — stopping flywheel.')
        self._call_service(self.flywheel_stop_client, 'stop_flywheel')


# =============================================================================
# ENTRY POINT
# =============================================================================
def main():
    rclpy.init()
    node = ArucoDockerNode()
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
