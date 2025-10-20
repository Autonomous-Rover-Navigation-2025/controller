#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import Jetson.GPIO as GPIO

# -------------------------
# CONFIGURATION
# -------------------------
GPIO.setmode(GPIO.BOARD)

ENCODER_PINS = {
    "front_left":  {"A": 19, "B": 36},
    "front_right": {"A": 22, "B": 35},
    "rear_left":   {"A": 29, "B": 38},
    "rear_right":  {"A": 13, "B": 37},
}

for wheel in ENCODER_PINS.values():
    GPIO.setup(wheel["A"], GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(wheel["B"], GPIO.IN, pull_up_down=GPIO.PUD_UP)

# -------------------------
# Encoder Class
# -------------------------
class Encoder:
    def __init__(self, pin_a, pin_b):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.ticks = 0
        self.direction = 0
        self.last_state = (GPIO.input(pin_a), GPIO.input(pin_b))
        GPIO.add_event_detect(pin_a, GPIO.BOTH, callback=self.update)
        GPIO.add_event_detect(pin_b, GPIO.BOTH, callback=self.update)

    def update(self, channel):
        a = GPIO.input(self.pin_a)
        b = GPIO.input(self.pin_b)
        if self.last_state == (0, 0):
            if (a, b) == (1, 0):
                self.ticks += 1; self.direction = 1
            elif (a, b) == (0, 1):
                self.ticks -= 1; self.direction = -1
        elif self.last_state == (1, 0):
            if (a, b) == (1, 1):
                self.ticks += 1; self.direction = 1
            elif (a, b) == (0, 0):
                self.ticks -= 1; self.direction = -1
        elif self.last_state == (1, 1):
            if (a, b) == (0, 1):
                self.ticks += 1; self.direction = 1
            elif (a, b) == (1, 0):
                self.ticks -= 1; self.direction = -1
        elif self.last_state == (0, 1):
            if (a, b) == (0, 0):
                self.ticks += 1; self.direction = 1
            elif (a, b) == (1, 1):
                self.ticks -= 1; self.direction = -1
        self.last_state = (a, b)

# -------------------------
# ROS Node
# -------------------------
class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')
        self.encoders = {name: Encoder(pins["A"], pins["B"]) for name, pins in ENCODER_PINS.items()}
        self.publisher = self.create_publisher(Int32MultiArray, 'encoder_ticks', 10)
        self.timer = self.create_timer(0.1, self.publish_status)  # 10 Hz

        self.get_logger().info("Quadrature encoders initialized (interrupt-driven).")

    def publish_status(self):
        msg = Int32MultiArray()
        msg.data = []

        ticks_list = []
        direction_list = []

        for wheel, encoder in self.encoders.items():
            msg.data.append(encoder.ticks)
            ticks_list.append(f"{wheel}: {encoder.ticks}")
            dir_str = "Forward" if encoder.direction == 1 else "Backward" if encoder.direction == -1 else "Stopped"
            direction_list.append(f"{wheel}: {dir_str}")

        # Log ticks and directions separately
        self.get_logger().debug("Ticks -> " + " | ".join(ticks_list))
        self.get_logger().debug("Directions -> " + " | ".join(direction_list))

        self.publisher.publish(msg)



# -------------------------
# MAIN
# -------------------------
def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down encoder node.")
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# import Jetson.GPIO as GPIO
# import threading

# # ---------------------------------------------------------
# # CONFIG
# # ---------------------------------------------------------
# GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering on Jetson

# # Define the GPIO pins connected to each wheel's encoder A and B channels
# ENCODER_PINS = {
#     "front_left":  {"A": 19, "B": 36},
#     "front_right": {"A": 22, "B": 35},
#     "rear_left":   {"A": 29, "B": 38},
#     "rear_right":  {"A": 13, "B": 37},
# }

# # Order in which we will publish wheel ticks
# WHEEL_ORDER = ["front_left", "front_right", "rear_left", "rear_right"]

# TIMER_PERIOD = 0.02  # Publish encoder data at 50 Hz

# # Quadrature lookup table to decode tick direction
# # Rows: previous state, Columns: current state, Values: -1, 0, +1
# STEP = [
#     [0, 1, -1, 0],
#     [-1, 0, 0, 1],
#     [1, 0, 0, -1],
#     [0, -1, 1, 0],
# ]

# # ---------------------------------------------------------
# # NODE
# # ---------------------------------------------------------
# class WheelEncoderNode(Node):
#     def __init__(self):
#         super().__init__('wheel_encoder_node')
#         # Publisher to send wheel ticks to ROS 2 topic
#         self.publisher_ = self.create_publisher(Float32MultiArray, '/encoder_ticks', 10)
#         self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

#         # Store previous state and cumulative tick counts per wheel
#         self.prev_state = {}
#         self.tick_counts = {}
#         self.lock = threading.Lock()  # Protect shared data in multithreaded callbacks

#         # Initialize GPIO pins and attach interrupts for each wheel
#         for wheel, pins in ENCODER_PINS.items():
#             GPIO.setup(pins["A"], GPIO.IN)
#             GPIO.setup(pins["B"], GPIO.IN)

#             # Read initial encoder state
#             a = GPIO.input(pins["A"])
#             b = GPIO.input(pins["B"])
#             self.prev_state[wheel] = (a << 1) | b
#             self.tick_counts[wheel] = 0

#             # Attach interrupts on both A and B edges
#             GPIO.add_event_detect(pins["A"], GPIO.BOTH, callback=self.create_callback(wheel))
#             GPIO.add_event_detect(pins["B"], GPIO.BOTH, callback=self.create_callback(wheel))

#         self.get_logger().info("Quadrature encoders initialized (interrupt-driven).")

#     # -------------------------------
#     # Tick update function
#     # -------------------------------
#     def update_wheel_ticks(self, wheel):
#         """
#         Read encoder pins for the given wheel,
#         compute the change in ticks using the quadrature table,
#         and update the cumulative tick count.
#         """
#         pins = ENCODER_PINS[wheel]
#         a = GPIO.input(pins["A"])
#         b = GPIO.input(pins["B"])
#         curr = (a << 1) | b  # Current A/B state as 2-bit number

#         prev = self.prev_state[wheel]  # Previous A/B state
#         delta = STEP[prev][curr]       # Determine tick change (-1, 0, +1)

#         with self.lock:  # Protect shared tick_counts
#             self.tick_counts[wheel] += delta

#         self.prev_state[wheel] = curr  # Update previous state for next transition

#     def create_callback(self, wheel):
#         """
#         Return a callback function for a specific wheel.
#         This is called on any A or B pin change.
#         """
#         def callback(channel):
#             self.update_wheel_ticks(wheel)
#         return callback

#     def timer_callback(self):
#         """
#         Periodically publish signed tick counts for all wheels.
#         Tick counts are reset after publishing to measure per-period delta.
#         """
#         msg = Float32MultiArray()
#         with self.lock:
#             # Collect tick counts in order
#             msg.data = [float(self.tick_counts[w]) for w in WHEEL_ORDER]
#             # Reset counts for next period
#             for w in WHEEL_ORDER:
#                 self.tick_counts[w] = 0
#         self.publisher_.publish(msg)
#         self.get_logger().debug(f"Published signed ticks: {msg.data}")

# # ---------------------------------------------------------
# # MAIN
# # ---------------------------------------------------------
# def main(args=None):
#     rclpy.init(args=args)
#     node = WheelEncoderNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         try:
#             GPIO.cleanup()  # Reset all GPIO pins
#         except Exception:
#             pass
#         try:
#             rclpy.shutdown()
#         except Exception:
#             pass


# if __name__ == '__main__':
#     main()


# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# import Jetson.GPIO as GPIO
# import time

# # ---------------------------------------------------------
# # CONFIG
# # ---------------------------------------------------------
# GPIO.setmode(GPIO.BOARD)  # Using physical header pin numbers

# # Map each wheel to its A/B pins (your selection)
# ENCODER_PINS = {
#     "front_left":  {"A": 19, "B": 36},
#     "front_right": {"A": 22, "B": 35},
#     "rear_left":   {"A": 29, "B": 38},
#     "rear_right":  {"A": 13, "B": 37},
# }
# WHEEL_ORDER = ["front_left", "front_right", "rear_left", "rear_right"]

# TIMER_PERIOD = 0.02  # 50 Hz publish

# # Quadrature transition table: rows=prev_state(0..3), cols=curr_state(0..3) -> -1/0/+1
# STEP = [
# #    0,  1,  2,  3   (curr)
#     [ 0, +1, -1,  0],  # prev=0
#     [-1,  0,  0, +1],  # prev=1
#     [+1,  0,  0, -1],  # prev=2
#     [ 0, -1, +1,  0],  # prev=3
# ]


# # ---------------------------------------------------------
# # NODE
# # ---------------------------------------------------------
# class WheelEncoderNode(Node):
#     def __init__(self):
#         super().__init__('wheel_encoder_node')
#         self.publisher_ = self.create_publisher(Float32MultiArray, '/encoder_ticks', 10)
#         self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

#         # Configure pins as inputs (Jetson ignores internal pull-ups; use external pull-ups if needed)
#         for pins in ENCODER_PINS.values():
#             GPIO.setup(pins["A"], GPIO.IN)
#             GPIO.setup(pins["B"], GPIO.IN)

#         # Per-wheel state
#         self.prev_state = {}
#         for wheel, pins in ENCODER_PINS.items():
#             a = GPIO.input(pins["A"])
#             b = GPIO.input(pins["B"])
#             self.prev_state[wheel] = (a << 1) | b  # 0..3

#         self.get_logger().info("Quadrature encoders initialized (polling, signed ticks).")

#     def measure_quadrature(self, wheel: str) -> int:
#         """Poll A/B during TIMER_PERIOD, return signed tick delta."""
#         pins = ENCODER_PINS[wheel]
#         count = 0
#         start = time.time()
#         prev = self.prev_state[wheel]

#         while (time.time() - start) < TIMER_PERIOD:
#             a = GPIO.input(pins["A"])
#             b = GPIO.input(pins["B"])
#             curr = (a << 1) | b
#             delta = STEP[prev][curr]
#             if delta != 0:
#                 count += delta
#             # --- LOG A and B pin values ---
#             # self.get_logger().info(f"{wheel}: A={a}, B={b}, delta={delta}")
#             # Always update prev so we can recover after invalid jumps
#             prev = curr

#         self.prev_state[wheel] = prev
#         return count

#     def timer_callback(self):
#         msg = Float32MultiArray()
#         msg.data = [float(self.measure_quadrature(w)) for w in WHEEL_ORDER]
#         self.publisher_.publish(msg)
#         self.get_logger().debug(f"Published signed ticks: {msg.data}")


# # ---------------------------------------------------------
# # MAIN
# # ---------------------------------------------------------
# def main(args=None):
#     rclpy.init(args=args)
#     node = WheelEncoderNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         try:
#             GPIO.cleanup()
#         except Exception:
#             pass
#         try:
#             rclpy.shutdown()
#         except Exception:
#             pass


# if __name__ == '__main__':
#     main()
