#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import csv
from datetime import datetime

class RealSenseIMULogger(Node):
    def __init__(self):
        super().__init__('realsense_imu_logger')

        # ---- CSV Setup ----
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"imu_data_{timestamp}.csv"
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'ros_time_sec', 'ros_time_nanosec',
            'dt_sec', 'frequency_hz',
            'accel_x', 'accel_y', 'accel_z',
            'gyro_x', 'gyro_y', 'gyro_z',
            'vel_x', 'vel_y', 'vel_z',
            'pos_x', 'pos_y', 'pos_z'
        ])
        self.get_logger().info(f"Logging IMU data to {self.csv_filename}")

        # ---- Subscription ----
        self.subscription = self.create_subscription(
            Imu,
            '/camera/imu',  # or '/camera/imu/data_raw'
            self.imu_callback,
            10
        )

        # ---- Initialize velocity and position ----
        self.prev_time = None
        self.vel = [0.0, 0.0, 0.0]  # vx, vy, vz
        self.pos = [0.0, 0.0, 0.0]  # x, y, z

    def imu_callback(self, msg: Imu):
        # ---- Compute delta time ----
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            dt = 0.0
            freq = 0.0
        else:
            dt = current_time - self.prev_time
            freq = 1.0 / dt if dt > 0 else 0.0
        self.prev_time = current_time

        # ---- Get acceleration ----
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # ---- Integrate to get velocity ----
        self.vel[0] += ax * dt
        self.vel[1] += ay * dt
        self.vel[2] += az * dt

        # ---- Integrate velocity to get position ----
        self.pos[0] += self.vel[0] * dt
        self.pos[1] += self.vel[1] * dt
        self.pos[2] += self.vel[2] * dt

        # ---- Write to CSV ----
        self.csv_writer.writerow([
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            dt, freq,
            ax, ay, az,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            self.vel[0], self.vel[1], self.vel[2],
            self.pos[0], self.pos[1], self.pos[2]
        ])

        # ---- Log info ----
        self.get_logger().info(
            f"Time: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} | "
            f"dt: {dt:.4f}s ({freq:.2f} Hz) | "
            f"Accel: ({ax:.3f}, {ay:.3f}, {az:.3f}) | "
            f"Vel: ({self.vel[0]:.3f}, {self.vel[1]:.3f}, {self.vel[2]:.3f}) | "
            f"Pos: ({self.pos[0]:.3f}, {self.pos[1]:.3f}, {self.pos[2]:.3f})"
        )

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info("CSV file closed and node stopped.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseIMULogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt — stopping logger.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
