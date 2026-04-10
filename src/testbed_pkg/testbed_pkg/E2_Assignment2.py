import rclpy
import time
import RPi.GPIO as GPIO
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Set pin numbering convention
# BCM is usually preferred for GPIO numbering (refers to the Broadcom chip channel)
GPIO.setmode(GPIO.BCM)

# Choose an appropriate pwm channel to be used to control the servo
# GPIO 18 is a standard hardware PWM pin on the Raspberry Pi
servo_pin = 18

# Set the pin as an output
GPIO.setup(servo_pin, GPIO.OUT)

# Initialise the servo to be controlled by pwm with 50 Hz frequency
p = GPIO.PWM(servo_pin, 50)

# Set servo to 90 degrees as it's starting position
p.start(7.5)

class LidarTestBed(Node):
    def __init__(self):
        super().__init__('lidar_test_bed')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Cartographer usually listens to /scan, so we subscribe to it too
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile)

    def listener_callback(self, msg):
        # The 'ranges' array contains distances.
        # Index 0 is often directly in front, but check your LIDAR orientation.
        # For TurtleBot3, 0 degrees is the first element.
        front_distance = msg.ranges[0]


        # Check if measured distance is approximately 1m
        if 0.95 <= front_distance <= 1.05:
            self.get_logger().info(f'Distance is {front_distance:.2f}m. Triggering Servo and Solenoid!')
            self.trigger_actuators()

    def trigger_actuators(self):
        # This is where you'd send commands to your servo/solenoid
        # e.g., publishing to a /servo_angle or /solenoid_trigger topic
        pass

def main(args=None):
    rclpy.init(args=args)
    node = LidarTestBed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
