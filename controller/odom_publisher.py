import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray
import Jetson.GPIO as GPIO
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
import time
import math

# Define GPIO pins for each wheel's encoder
ENCODER_PINS = {
    "front_left": 19,
    "front_right": 22,
    "rear_left": 29,
    "rear_right": 13
}

# Global state variables
encoder_ticks = {
    "front_left": 0,
    "front_right": 0,
    "rear_left": 0,
    "rear_right": 0
}

distance = {
    "front_left": 0.0,
    "front_right": 0.0,
    "rear_left": 0.0,
    "rear_right": 0.0
}

speed = {
    "front_left": 0.0,
    "front_right": 0.0,
    "rear_left": 0.0,
    "rear_right": 0.0
}

# Constants
TIMER_PERIOD = 0.5  # seconds
TICKS_PER_ROTATION = 620
WHEEL_DIAMETER = 0.1524  #meters
CIRCUMFERENCE = (3.1416 * WHEEL_DIAMETER)

#L (float): Half of the wheelbase [Center to Center] (distance from front to rear wheel / 2) in meters.
HALF_OF_WHEEL_BASE = ((0.31 - 0.05 - 0.05) / 2)
#W (float): Half of the track width [Center to Center] (distance between left and right wheels / 2) in meters.
HALF_OF_TRACK_WIDTH = ((0.05552 + 0.02 + 0.24 + 0.02 + 0.05552) / 2)


class WheelOdometer(Node):

    def __init__(self):
        super().__init__('wheel_odometry')
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Use BOARD pin numbering
        GPIO.setmode(GPIO.BOARD)
        for wheel, pin in ENCODER_PINS.items():
            GPIO.setup(pin, GPIO.IN)

    def measure_pwm(self, pin, timeout=TIMER_PERIOD):
        """Measure rising edges on a GPIO pin within a time window."""
        count = 0
        start_time = time.time()
        last_state = GPIO.input(pin)

        while (time.time() - start_time) < timeout:
            current_state = GPIO.input(pin)
            if last_state == 0 and current_state == 1:
                count += 1
            last_state = current_state

        return count

    def compute_robot_velocity(self, speed, L, W):
        """
        Compute the robot's linear (vx, vy) and angular (omega_z) velocity 
        for a 4-wheeled Mecanum drive (Using individuals speed (m/s) directly)

        Args:
            speed (dict): Dictionary with keys "front_left", "front_right", 
                        "rear_left", "rear_right" (values in m/s).
            L (float): Half of the wheelbase [Center to Center] (distance from front to rear wheel / 2) in meters.
            W (float): Half of the track width [Center to Center] (distance between left and right wheels / 2) in meters.

        Returns:
            tuple: (vx, vy, omega_z) velocities in m/s and rad/s.
        """
        v1 = float(speed["front_left"])
        v2 = float(speed["front_right"])
        v3 = float(speed["rear_left"])
        v4 = float(speed["rear_right"])

        a = L + W  # Distance factor for rotation

        vx = (v1 + v2 + v3 + v4) / 4.0
        vy = (-v1 + v2 + v3 - v4) / 4.0
        omega_z = (-v1 + v2 - v3 + v4) / (4.0 * a)

        return vx, vy, omega_z

    def timer_callback(self):
        """Periodically measure and publish encoder data."""
        for wheel, pin in ENCODER_PINS.items():
            encoder_ticks[wheel] = self.measure_pwm(pin)
            distance[wheel] = (encoder_ticks[wheel] /
                               TICKS_PER_ROTATION) * CIRCUMFERENCE
            speed[wheel] = distance[wheel] / TIMER_PERIOD

        tick_msg = Float32MultiArray()
        tick_msg.data = [
            float(encoder_ticks["front_left"]),
            float(encoder_ticks["front_right"]),
            float(encoder_ticks["rear_left"]),
            float(encoder_ticks["rear_right"])
        ]
        # self.get_logger().info(f"tick_msg.data : {tick_msg.data}")
        self.get_logger().info(
            f"tick_msg.data -->  'front_left' (pin:{ENCODER_PINS['front_left']}): {tick_msg.data[0]}, 'front_right' (pin:{ENCODER_PINS['front_right']}):{tick_msg.data[1]}, 'rear_left' (pin:{ENCODER_PINS['rear_left']}):{tick_msg.data[2]}, 'rear_right' (pin:{ENCODER_PINS['rear_right']}):{tick_msg.data[3]}"
        )

        speed_msg = Float32MultiArray()
        speed_msg.data = [
            float(speed["front_left"]),
            float(speed["front_right"]),
            float(speed["rear_left"]),
            float(speed["rear_right"])
        ]
        # self.get_logger().info(f"speed_msg.data : {speed_msg.data}")
        self.get_logger().info(
            f"speed_msg.data --> 'front_left' (pin:{ENCODER_PINS['front_left']}): {speed_msg.data[0]:.3f}, 'front_right' (pin:{ENCODER_PINS['front_right']}):{speed_msg.data[1]:.3f}, 'rear_left' (pin:{ENCODER_PINS['rear_left']}):{speed_msg.data[2]:.3f}, rear_right' (pin:{ENCODER_PINS['rear_right']}):{speed_msg.data[3]:.3f}"
        )
        vx, vy, omega_z = self.compute_robot_velocity(speed,
                                                      HALF_OF_WHEEL_BASE,
                                                      HALF_OF_TRACK_WIDTH)
        self.get_logger().info(
            f"vx = {vx:.2f} m/s, vy = {vy:.2f} m/s, omega_z = {omega_z:.2f} rad/s"
        )

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9

        # Integrate velocity to update pose
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_theta = omega_z * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        self.get_logger().info(
            f"self.x = {self.x:.2f} m, self.y = {self.y:.2f} m, self.theta = {self.theta:.2f} rad"
        )

        # Prepare odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega_z
        # self.get_logger().info(f"Publish odom = {odom}")
        self.odom_pub.publish(odom)

        # Publish TF
        # t = TransformStamped()
        # t.header.stamp = current_time.to_msg()
        # t.header.frame_id = "odom"
        # t.child_frame_id = "base_link"
        # t.transform.translation.x = self.x
        # t.transform.translation.y = self.y
        # t.transform.translation.z = 0.0
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]
        # # self.get_logger().info(f"Publish TF = {t}")
        # self.tf_broadcaster.sendTransform(t)
        # self.last_time = current_time


# ==== Entry Point ====


def main(args=None):
    rclpy.init(args=args)
    wheelodom_node = WheelOdometer()

    try:
        rclpy.spin(wheelodom_node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        wheelodom_node.destroy_node()
        GPIO.cleanup()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
