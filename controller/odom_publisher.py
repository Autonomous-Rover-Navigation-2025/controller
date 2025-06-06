#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
from tf_transformations import euler_from_quaternion, quaternion_from_euler

# === Constants ===
TICKS_PER_ROTATION = 620
WHEEL_DIAMETER = 0.1524  # meters
CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
TIMER_PERIOD = 0.5  # seconds
HALF_WHEEL_BASE = (0.31 - 0.05 - 0.05) / 2
HALF_TRACK_WIDTH = (0.05552 + 0.02 + 0.24 + 0.02 + 0.05552) / 2


class OdomPublisher(Node):

    def __init__(self):
        super().__init__('odom_publisher')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Float32MultiArray, '/encoder_ticks',
                                 self.encoder_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        self.last_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.imu_yaw = 0.0
        self.imu_angular_z = 0.0

    def imu_callback(self, msg):
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.imu_yaw = yaw
        self.imu_angular_z = msg.angular_velocity.z

    def encoder_callback(self, msg):
        if len(msg.data) != 4:
            self.get_logger().warn("Expected 4 encoder values")
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        vx, vy = self.compute_velocity(msg.data)

        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt

        self.x += delta_x
        self.y += delta_y
        self.theta = self.imu_yaw
        self.get_logger().info(
            f"x = {self.x}, y = {self.y}, theta = {self.theta}\n vx = {vx:.2f} m/s, vy = {vy:.2f} m/s, omega_z = {self.imu_angular_z:.2f} rad/s"
        )
        self.publish_odometry(current_time, vx, vy, self.imu_angular_z)

    def compute_velocity(self, ticks):
        distances = [
            tick / TICKS_PER_ROTATION * CIRCUMFERENCE for tick in ticks
        ]
        speeds = [d / TIMER_PERIOD for d in distances]
        v1, v2, v3, v4 = speeds

        vx = (v1 + v2 + v3 + v4) / 4.0
        vy = (v1 - v2 - v3 + v4) / 4.0
        return vx, vy

    def publish_odometry(self, stamp, vx, vy, omega_z):
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega_z

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
