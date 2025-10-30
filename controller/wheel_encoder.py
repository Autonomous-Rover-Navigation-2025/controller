#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import Jetson.GPIO as GPIO
import time

# ---------------------------------------------------------
# CONFIG
# ---------------------------------------------------------
GPIO.setmode(GPIO.BOARD)  # Using physical header pin numbers

# Map each wheel to its A/B pins (your selection)
ENCODER_PINS = {
    "front_left":  {"A": 19, "B": 36},
    "front_right": {"A": 22, "B": 35},
    "rear_left":   {"A": 29, "B": 38},
    "rear_right":  {"A": 16, "B": 18},
}
WHEEL_ORDER = ["front_left", "front_right", "rear_left", "rear_right"]

TIMER_PERIOD = 0.02  # 50 Hz publish

# Quadrature transition table: rows=prev_state(0..3), cols=curr_state(0..3) -> -1/0/+1
STEP = [
#    0,  1,  2,  3   (curr)
    [ 0, +1, -1,  0],  # prev=0
    [-1,  0,  0, +1],  # prev=1
    [+1,  0,  0, -1],  # prev=2
    [ 0, -1, +1,  0],  # prev=3
]


# ---------------------------------------------------------
# NODE
# ---------------------------------------------------------
class WheelEncoderNode(Node):
    def __init__(self):
        super().__init__('wheel_encoder_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/encoder_ticks', 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        # Configure pins as inputs (Jetson ignores internal pull-ups; use external pull-ups if needed)
        for pins in ENCODER_PINS.values():
            GPIO.setup(pins["A"], GPIO.IN)
            GPIO.setup(pins["B"], GPIO.IN)

        # Per-wheel state
        self.prev_state = {}
        for wheel, pins in ENCODER_PINS.items():
            a = GPIO.input(pins["A"])
            b = GPIO.input(pins["B"])
            self.prev_state[wheel] = (a << 1) | b  # 0..3

        self.get_logger().info("Quadrature encoders initialized (polling, signed ticks).")

    def measure_quadrature(self, wheel: str) -> int:
        """Poll A/B during TIMER_PERIOD, return signed tick delta."""
        pins = ENCODER_PINS[wheel]
        count = 0
        prev = self.prev_state[wheel]
        start = time.time()
        while (time.time() - start) <= TIMER_PERIOD:
            a = GPIO.input(pins["A"])
            b = GPIO.input(pins["B"])
            curr = (a << 1) | b
            if curr != prev:
                # If both bits flipped, it's an invalid jump (missed state) → ignore
                if (curr ^ prev) == 0b11:
                    delta = 0
                else:
                    delta = STEP[prev][curr]
            else:
                delta = 0

            if delta != 0:
                count += delta
            # Always update prev so we can recover after invalid jumps
            prev = curr

        self.prev_state[wheel] = prev
        return count

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [float(self.measure_quadrature(w)) for w in WHEEL_ORDER]
        self.publisher_.publish(msg)


# ---------------------------------------------------------
# MAIN
# ---------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = WheelEncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            GPIO.cleanup()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
