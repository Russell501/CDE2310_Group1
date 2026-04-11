#!/usr/bin/env python3
"""
ball_launcher.py — ROS 2 node that controls the flywheel and ball-firing servo on the RPi.

Services:
  /start_flywheel  — spin up the flywheel motors at full speed
  /fire_ball       — trigger the servo to release one ball (PLACEHOLDER — implement servo later)
  /stop_flywheel   — stop the flywheel motors

The FSM controls timing between balls. Motors run continuously across all shots.

Run on the RPi with:
    python3 ball_launcher.py
"""

import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

try:
    from gpiozero import Motor
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False
    print('WARNING: gpiozero not available — motor control will be simulated.')

# =============================================================================
# CONFIGURATION
# =============================================================================
# Motor A (Left)
MOTOR_A_FWD, MOTOR_A_BWD, MOTOR_A_EN = 23, 24, 13

# Motor B (Right)
MOTOR_B_FWD, MOTOR_B_BWD, MOTOR_B_EN = 22, 27, 12


# =============================================================================
# NODE
# =============================================================================
class BallLauncherNode(Node):
    def __init__(self):
        super().__init__('ball_launcher')

        if _GPIO_AVAILABLE:
            self.motor_a = Motor(forward=MOTOR_A_FWD, backward=MOTOR_A_BWD, enable=MOTOR_A_EN)
            self.motor_b = Motor(forward=MOTOR_B_FWD, backward=MOTOR_B_BWD, enable=MOTOR_B_EN)
            self.get_logger().info('GPIO motors initialised.')
        else:
            self.motor_a = self.motor_b = None

        self.create_service(Trigger, '/start_flywheel', self._start_flywheel_cb)
        self.create_service(Trigger, '/fire_ball',      self._fire_ball_cb)
        self.create_service(Trigger, '/stop_flywheel',  self._stop_flywheel_cb)

        self.get_logger().info('Ball launcher ready.')

    # -------------------------------------------------------------------------
    def _start_flywheel_cb(self, request, response):
        self.get_logger().info('Flywheel START — motors running at full speed.')
        if self.motor_a and self.motor_b:
            self.motor_a.forward(1.0)
            self.motor_b.forward(1.0)
        else:
            self.get_logger().warn('GPIO unavailable — simulating flywheel start.')
        response.success = True
        response.message = 'Flywheel running'
        return response

    def _fire_ball_cb(self, request, response):
        # TODO: implement servo trigger here
        self.get_logger().info('FIRE — servo trigger (placeholder).')
        response.success = True
        response.message = 'Ball fired (servo placeholder)'
        return response

    def _stop_flywheel_cb(self, request, response):
        self.get_logger().info('Flywheel STOP.')
        if self.motor_a and self.motor_b:
            self.motor_a.stop()
            self.motor_b.stop()
        else:
            self.get_logger().warn('GPIO unavailable — simulating flywheel stop.')
        response.success = True
        response.message = 'Flywheel stopped'
        return response

    def stop_motors(self):
        if self.motor_a:
            self.motor_a.stop()
        if self.motor_b:
            self.motor_b.stop()


def main():
    rclpy.init()
    node = BallLauncherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nStopping motors...')
        node.stop_motors()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
