# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# from std_msgs.msg import Header
# from smbus2 import SMBus
# import math


# # MPU6050 registers and addresses
# BUS = 1
# MPU6050_ADDR = 0x68
# PWR_MGMT_1 = 0x6B
# ACCEL_XOUT_H = 0x3B
# GYRO_XOUT_H = 0x43
# PUBLISH_TOPIC_NAME = "/imu"


# class MPU6050Node(Node):
#     def __init__(self):
#         super().__init__('imu_sensor_mpu6050_publisher')
#         self.publisher_ = self.create_publisher(Imu, PUBLISH_TOPIC_NAME, 10)
#         self.bus = SMBus(BUS)

#         # Initialize MPU
#         self.init_mpu()
#         self.get_logger().info("✅ IMU Sensor MPU6050 initialized successfully.")

#         # Publish at 10 Hz
#         timer_period = 0.1
#         self.timer = self.create_timer(timer_period, self.publish_imu_data)
#         self.get_logger().info(
#             f"Publishing IMU data every {timer_period}s on topic: {PUBLISH_TOPIC_NAME}"
#         )

#     # -------------------------------
#     # 🔧 SENSOR INITIALIZATION & I/O
#     # -------------------------------

#     def init_mpu(self):
#         """Wake up the MPU6050."""
#         self.bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
#         who = self.bus.read_byte_data(MPU6050_ADDR, 0x75)
#         self.get_logger().info(f"WHO_AM_I register: 0x{who:02x}")

#     def read_word(self, reg):
#         """Read a word and convert to signed integer."""
#         high = self.bus.read_byte_data(MPU6050_ADDR, reg)
#         low = self.bus.read_byte_data(MPU6050_ADDR, reg + 1)
#         value = (high << 8) + low
#         if value >= 0x8000:
#             value = -((65535 - value) + 1)
#         return value

#     def get_accel_data(self):
#         """Get accelerometer data (g)."""
#         ax = self.read_word(ACCEL_XOUT_H) / 16384.0
#         ay = self.read_word(ACCEL_XOUT_H + 2) / 16384.0
#         az = self.read_word(ACCEL_XOUT_H + 4) / 16384.0
#         return ax, ay, az

#     def get_gyro_data(self):
#         """Get gyroscope data (°/s)."""
#         gx = self.read_word(GYRO_XOUT_H) / 131.0
#         gy = self.read_word(GYRO_XOUT_H + 2) / 131.0
#         gz = self.read_word(GYRO_XOUT_H + 4) / 131.0
#         return gx, gy, gz

#     # -------------------------------
#     # 🧭 ORIENTATION CALCULATIONS
#     # -------------------------------

#     def calculate_orientation(self, ax, ay, az):
#         """
#         Estimate roll and pitch from accelerometer data.
#         yaw is set to 0 (no magnetometer).
#         Returns (roll, pitch, yaw) in radians.
#         """
#         roll = math.atan2(ay, az)
#         pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
#         yaw = 0.0
#         return roll, pitch, yaw

#     def euler_to_quaternion(self, roll, pitch, yaw):
#         """Convert Euler angles (radians) to quaternion (x, y, z, w)."""
#         cy = math.cos(yaw * 0.5)
#         sy = math.sin(yaw * 0.5)
#         cp = math.cos(pitch * 0.5)
#         sp = math.sin(pitch * 0.5)
#         cr = math.cos(roll * 0.5)
#         sr = math.sin(roll * 0.5)

#         qw = cr * cp * cy + sr * sp * sy
#         qx = sr * cp * cy - cr * sp * sy
#         qy = cr * sp * cy + sr * cp * sy
#         qz = cr * cp * sy - sr * sp * cy

#         return qx, qy, qz, qw

#     # -------------------------------
#     # 📤 IMU PUBLISHER
#     # -------------------------------

#     def publish_imu_data(self):
#         """Read MPU data and publish as sensor_msgs/Imu."""
#         msg = Imu()

#         # Header
#         msg.header = Header()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = 'imu_link'

#         # Accelerometer (convert to m/s²)
#         ax, ay, az = self.get_accel_data()
#         msg.linear_acceleration.x = ax * 9.80665
#         msg.linear_acceleration.y = ay * 9.80665
#         msg.linear_acceleration.z = az * 9.80665

#         # Gyroscope (convert to rad/s)
#         gx, gy, gz = self.get_gyro_data()
#         msg.angular_velocity.x = math.radians(gx)
#         msg.angular_velocity.y = math.radians(gy)
#         msg.angular_velocity.z = math.radians(gz)

#         # Orientation
#         roll, pitch, yaw = self.calculate_orientation(ax, ay, az)
#         qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)
#         msg.orientation.x = qx
#         msg.orientation.y = qy
#         msg.orientation.z = qz
#         msg.orientation.w = qw

#         # Publish
#         self.publisher_.publish(msg)

#         # Log concise message
#         self.get_logger().info(
#             f"Accel[g]: ({ax:.2f}, {ay:.2f}, {az:.2f}) | "
#             f"Gyro[°/s]: ({gx:.2f}, {gy:.2f}, {gz:.2f}) | "
#             f"Orien[°]: (R={math.degrees(roll):.1f}, P={math.degrees(pitch):.1f}, Y={math.degrees(yaw):.1f})"
#         )

#     # -------------------------------
#     # 🧹 CLEAN SHUTDOWN
#     # -------------------------------

#     def destroy_node(self):
#         """Clean shutdown."""
#         self.bus.close()
#         super().destroy_node()


# # -------------------------------
# # 🚀 MAIN ENTRY POINT
# # -------------------------------

# def main(args=None):
#     rclpy.init(args=args)
#     node = MPU6050Node()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
