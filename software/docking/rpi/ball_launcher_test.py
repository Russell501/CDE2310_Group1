#!/usr/bin/env python3
"""
ball_launcher_test.py — simple test to spin the ball launcher motors at full speed.

Run on the RPi with:
    python3 ball_launcher_test.py
"""

try:
    from gpiozero import Motor
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False
    print('WARNING: gpiozero not available — motor control will be simulated.')

# Motor A (Left)
MOTOR_A_FWD, MOTOR_A_BWD, MOTOR_A_EN = 23, 24, 13

# Motor B (Right)
MOTOR_B_FWD, MOTOR_B_BWD, MOTOR_B_EN = 22, 27, 12


def main():
    if _GPIO_AVAILABLE:
        motor_a = Motor(forward=MOTOR_A_FWD, backward=MOTOR_A_BWD, enable=MOTOR_A_EN)
        motor_b = Motor(forward=MOTOR_B_FWD, backward=MOTOR_B_BWD, enable=MOTOR_B_EN)
        print('Starting motors at full speed...')
        motor_a.forward(1.0)
        motor_b.forward(1.0)
        try:
            print('Motors are running. Press Ctrl+C to stop.')
            while True:
                pass
        except KeyboardInterrupt:
            print('\nStopping motors...')
            motor_a.stop()
            motor_b.stop()
    else:
        print('GPIO unavailable')


if __name__ == '__main__':
    main()
