#!/usr/bin/env python3
"""
servo.py — simple script to move a servo on GPIO pin 18 to a user-specified angle.
"""

import sys
import time

try:
    from gpiozero import AngularServo
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False
    print('WARNING: gpiozero not available')

SERVO_PIN = 18
MIN_ANGLE = 0
MAX_ANGLE = 180


def parse_angle(value):
    try:
        angle = float(value)
    except ValueError:
        raise ValueError(f'Invalid angle: {value!r}')
    return angle


def main():
    if len(sys.argv) > 1:
        angle_input = sys.argv[1]
    else:
        angle_input = input('Enter servo angle: ').strip()

    try:
        angle = parse_angle(angle_input)
    except ValueError as exc:
        print(exc)
        sys.exit(1)

    print(f'Setting servo on GPIO {SERVO_PIN} to angle {angle}')

    if _GPIO_AVAILABLE:
        servo = AngularServo(SERVO_PIN, min_angle=MIN_ANGLE, max_angle=MAX_ANGLE)
        servo.angle = angle
        time.sleep(1)
        print('Servo move complete.')
    else:
        print('GPIO unavailable — simulated servo move.')


if __name__ == '__main__':
    main()
