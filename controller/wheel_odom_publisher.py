#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Quaternion
import tf2_ros
import math
from tf_transformations import quaternion_from_euler

# === Constants ===
TICKS_PER_ROTATION = 620
WHEEL_DIAMETER = 0.1524  # meters
CIRCUMFERENCE = math.pi * WHEEL_DIAMETER

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
        self.create_subscription(
            Float32MultiArray,
            SUBSCRIPTION_TOPIC_ENCODER,
            self.encoder_callback,
            10,
        )

        # Initialize state variables
        self.last_time = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.get_logger().info("Wheel Odometry Publisher Initialized ✅")

    # === Encoder Callback ===
    def encoder_callback(self, msg):
        if len(msg.data) != 4:
            self.get_logger().warn(
                f"Expected 4 encoder values, got {len(msg.data)}."
            )
            return

        current_time = self.get_clock().now()

        # First message initializes timing
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_time = current_time

        if dt > 0.2:
            self.get_logger().warn(f"Large dt detected: {dt:.3f}s")

        ticks = list(msg.data)

        # Compute linear & angular velocity from encoder ticks
        vx, vy, omega_z = self.compute_velocity(ticks, dt)

        # Integrate pose using velocities
        self.theta += omega_z * dt
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt

        self.x += delta_x
        self.y += delta_y

        # Publish odometry
        self.publish_odometry(current_time, vx, vy, omega_z)

        self.get_logger().debug(
            f"dt={dt:.3f}s | x={self.x:.3f}, y={self.y:.3f}, θ={self.theta:.3f} | "
            f"vx={vx:.3f} m/s, vy={vy:.3f} m/s, ωz={omega_z:.3f} rad/s"
        )

    # === Compute Linear & Angular Velocities ===
    def compute_velocity(self, ticks, dt):
        # Convert ticks to linear wheel distances
        distances = [(tick / TICKS_PER_ROTATION) * CIRCUMFERENCE for tick in ticks]
        speeds = [d / dt for d in distances]  # wheel speeds (m/s)

        v1, v2, v3, v4 = speeds  # front_left, front_right, rear_left, rear_right
        L = HALF_WHEEL_BASE + HALF_TRACK_WIDTH  # effective robot half dimension

        # Mecanum forward kinematics
        vx = (v1 + v2 + v3 + v4) / 4.0
        vy = (-v1 + v2 + v3 - v4) / 4.0
        omega_z = (-v1 + v2 - v3 + v4) / (4.0 * L)

        return vx, vy, omega_z

    # === Publish Odometry ===
    def publish_odometry(self, stamp, vx, vy, omega_z):
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # Velocities
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega_z

        # Publish odometry message
        self.odom_pub.publish(odom)

        # Optionally broadcast TF transform (odom → base_link)
        transform = tf2_ros.TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

        self.get_logger().debug(
            f"Odometry published: x={self.x:.3f}, y={self.y:.3f}, θ={self.theta:.3f}, "
            f"vx={vx:.3f}, vy={vy:.3f}, ωz={omega_z:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
