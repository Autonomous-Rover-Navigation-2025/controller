#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import Jetson.GPIO as GPIO
import time

ENCODER_PINS = {
    "front_left": 19,
    "front_right": 22,
    "rear_left": 29,
    "rear_right": 13
}

TIMER_PERIOD = 0.5  # seconds


class WheelEncoderNode(Node):

    def __init__(self):
        super().__init__('wheel_encoder_node')
        self.publisher_ = self.create_publisher(Float32MultiArray,
                                                '/encoder_ticks', 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        GPIO.setmode(GPIO.BOARD)
        for pin in ENCODER_PINS.values():
            GPIO.setup(pin, GPIO.IN)

    def measure_pwm(self, pin):
        count = 0
        start_time = time.time()
        last_state = GPIO.input(pin)
        while (time.time() - start_time) < TIMER_PERIOD:
            current_state = GPIO.input(pin)
            if last_state == 0 and current_state == 1:
                count += 1
            last_state = current_state
        return count

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [
            float(self.measure_pwm(ENCODER_PINS[wheel])) for wheel in
            ["front_left", "front_right", "rear_left", "rear_right"]
        ]
        self.publisher_.publish(msg)
        self.get_logger().debug(f"Published ticks: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = WheelEncoderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        GPIO.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
