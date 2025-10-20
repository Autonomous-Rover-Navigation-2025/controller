#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import math
from tf_transformations import euler_from_quaternion, quaternion_from_euler

# === Constants ===
TICKS_PER_ROTATION = 620
WHEEL_DIAMETER = 0.1524  # meters
CIRCUMFERENCE = math.pi * WHEEL_DIAMETER

# (kept for reference but NOT used for speed; actual dt is used)
TIMER_PERIOD = 0.5

HALF_WHEEL_BASE = (0.31 - 0.05 - 0.05) / 2
HALF_TRACK_WIDTH = (0.05552 + 0.02 + 0.24 + 0.02 + 0.05552) / 2

# === Node & Topic Names ===
PUBLISHER_NODE_NAME = "wheel_odom_publisher"
PUBLISH_TOPIC_NAME = "/wheel_odom"
SUBSCRIPTION_TOPIC_ENCODER = "/encoder_ticks"


class WheelOdomPublisher(Node):

    def __init__(self):
        super().__init__(PUBLISHER_NODE_NAME)

        # Publishers and broadcasters
        self.odom_pub = self.create_publisher(Odometry, PUBLISH_TOPIC_NAME, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriptions
        self.create_subscription(Float32MultiArray, SUBSCRIPTION_TOPIC_ENCODER,
                                 self.encoder_callback, 10)

        # Initialize state variables
        self.last_time = None  # <-- initialize lazily
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.imu_yaw = 0.0
        self.imu_angular_z = 0.0

    # === Encoder Callback ===
    def encoder_callback(self, msg):
        if len(msg.data) != 4:
            self.get_logger().warn(
                "Expected 4 encoder values, got different count.")
            return

        current_time = self.get_clock().now()

        # First message just seeds the clock
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            # non-increasing timestamp; skip
            return
        self.last_time = current_time

        # Optional: warn if dt is unexpectedly large (can hurt integration quality)
        if dt > 0.2:
            self.get_logger().warn(f"Large dt detected: {dt:.3f}s")

        ticks = list(msg.data)

        # Flip RIGHT side (FR = index 1, RR = index 3)
        ticks[1] = -ticks[1]
        ticks[3] = -ticks[3]

        # compute velocities using actual dt
        vx, vy = self.compute_velocity(ticks, dt)

        # # --- INFO LOG: velocities and basics ---
        # self.get_logger().info(
        #     f"velocities -> vx: {vx:.3f} m/s, vy: {vy:.3f} m/s, wz: {self.imu_angular_z:.3f} rad/s | "
        #     f"dt: {dt:.3f}s | ticks: {[int(t) for t in msg.data]}")

        # integrate position
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt

        self.x += delta_x
        self.y += delta_y

        # keep your original behavior: use IMU yaw directly (no fallback added)
        self.theta = self.imu_yaw

        self.get_logger().debug(
            f"dt={dt:.3f}s | x={self.x:.3f}, y={self.y:.3f}, θ={self.theta:.3f} | "
            f"vx={vx:.2f} m/s, vy={vy:.2f} m/s, ωz={self.imu_angular_z:.2f} rad/s"
        )

        self.publish_odometry(current_time, vx, vy, self.imu_angular_z)

    # === Velocity Computation (uses dt) ===
    def compute_velocity(self, ticks, dt):
        # ticks are assumed to be counts accumulated since the previous message
        distances = [(tick / TICKS_PER_ROTATION) * CIRCUMFERENCE
                     for tick in ticks]
        speeds = [d / dt for d in distances]  # m/s using actual dt
        v1, v2, v3, v4 = speeds

        # mecanum forward kinematics (adjust signs if your wiring is inverted)
        vx = (v1 + v2 + v3 + v4) / 4.0
        vy = (v1 - v2 - v3 + v4) / 4.0
        return vx, vy

    # === Odometry Publisher ===
    def publish_odometry(self, stamp, vx, vy, omega_z):
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega_z

        self.odom_pub.publish(odom)
        self.get_logger().debug(
            f"Odometry published: x={self.x:.3f}, y={self.y:.3f}, θ={self.theta:.3f}, "
            f"vx={vx:.3f}, vy={vy:.3f}, ωz={omega_z:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
