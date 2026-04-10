#!/usr/bin/env python3
"""
station_b_launcher.py — Standalone Station B launch sequence.

Assumes the robot is already docked. Waits for ArUco marker ID=2 as the
hole-alignment trigger, then fires one ball per detection, up to TOTAL_BALLS.

Interfaces with ball_launcher.py on the RPi via ROS 2 services:
  /start_flywheel  — spin up flywheel motors
  /fire_ball       — fire one ball (servo actuation handled on RPi)
  /stop_flywheel   — stop flywheel motors

Run with:
    python3 station_b_launcher.py
"""

import time

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import threading

from visualization_msgs.msg import MarkerArray
from std_srvs.srv import Trigger

# =============================================================================
# CONFIGURATION
# =============================================================================
TOTAL_BALLS             = 3      # number of balls to fire
TRIGGER_MARKER_ID       = 2      # ArUco marker ID that signals hole alignment
TRIGGER_COOLDOWN        = 5.0    # s — wait after each fire before listening again
TRIGGER_TIMEOUT         = 30.0   # s — give up waiting for a trigger signal


# =============================================================================
# NODE
# =============================================================================
class StationBLauncherNode(Node):
    def __init__(self):
        super().__init__('station_b_launcher')

        self._trigger_event = threading.Event()
        self._latest_cam_x  = 0.0
        self._latest_cam_z  = 0.0

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(MarkerArray, '/aruco/markers', self._marker_cb, qos)

        self.flywheel_start_client = self.create_client(Trigger, '/start_flywheel')
        self.fire_ball_client      = self.create_client(Trigger, '/fire_ball')
        self.flywheel_stop_client  = self.create_client(Trigger, '/stop_flywheel')

        self._mission_thread = threading.Thread(target=self._run, daemon=True)
        self._mission_thread.start()

    def _marker_cb(self, msg: MarkerArray):
        for marker in msg.markers:
            if marker.id != TRIGGER_MARKER_ID:
                continue
            p = marker.pose.position
            if p.z <= 0.0:
                continue
            self._latest_cam_x = p.x
            self._latest_cam_z = p.z
            self._trigger_event.set()

    def _call_service(self, client, label) -> bool:
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'{label} service unavailable — skipping.')
            return False
        future = client.call_async(Trigger.Request())
        while not future.done():
            time.sleep(0.05)
        result = future.result()
        if result and result.success:
            self.get_logger().info(f'{label}: {result.message}')
            return True
        self.get_logger().error(f'{label} failed.')
        return False

    def _run(self):
        log = self.get_logger()

        log.info('Station B launcher ready — starting flywheel...')
        self._call_service(self.flywheel_start_client, 'start_flywheel')

        balls_fired = 0
        while balls_fired < TOTAL_BALLS:
            log.info(f'Waiting for trigger marker ID={TRIGGER_MARKER_ID} '
                     f'(ball {balls_fired + 1}/{TOTAL_BALLS})...')

            self._trigger_event.clear()
            detected = self._trigger_event.wait(timeout=TRIGGER_TIMEOUT)

            if not detected:
                log.error(f'Trigger timeout — no signal for ball {balls_fired + 1}. '
                          'Aborting launch sequence.')
                break

            log.info(f'Hole aligned: cam_x={self._latest_cam_x:.4f} m  '
                     f'cam_z={self._latest_cam_z:.4f} m — firing ball {balls_fired + 1}.')
            self._call_service(self.fire_ball_client, 'fire_ball')
            balls_fired += 1

            if balls_fired < TOTAL_BALLS:
                log.info(f'Waiting {TRIGGER_COOLDOWN}s before next detection...')
                time.sleep(TRIGGER_COOLDOWN)

        if balls_fired == TOTAL_BALLS:
            log.info(f'All {TOTAL_BALLS} balls fired successfully.')
        else:
            log.warn(f'Launch sequence ended early — {balls_fired}/{TOTAL_BALLS} balls fired.')

        log.info('Stopping flywheel...')
        self._call_service(self.flywheel_stop_client, 'stop_flywheel')


# =============================================================================
# ENTRY POINT
# =============================================================================
def main():
    rclpy.init()
    node = StationBLauncherNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
