#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from smbus2 import SMBus
import tf2_ros
import time
import math
import numpy as np
from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as R
from controller.mpu6050_driver import MPU6050Driver

# ================================================================
# 🧭 MPU6050 Register Map
# ================================================================
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
WHO_AM_I = 0x75


# ================================================================
# ⚙️ MAIN NODE
# ================================================================
class MPU6050Node(Node):

    def __init__(self):
        super().__init__('imu_publisher')

        # --- Step 1: Initialize Parameters ---
        self._declare_and_get_parameters()

        # --- Step 2: Initialize Hardware ---
        # Initialize sensor driver with ROS node reference
        self.sensor = MPU6050Driver(self, self.bus_number)
        self.sensor.initialize(accel_range = 3, gyro_range = 3, auto_calibrate = True)

        # --- Step 3: Initialize ROS Components ---
        self._init_publishers_and_timers()
        self._init_tf_broadcaster()
        self.filter = Madgwick()
        # Store New and previous quaternion values
        self.prev_global_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.new_global_quaternion = np.array([1.0, 0.0, 0.0, 0.0])  
        # Store last timestamp and velocity
        self.prev_time = time.time()
        self.linear_velocity = [0.0, 0.0, 0.0]
        
        # Bias estimation and motion detection (will be set from parameters)
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        self.bias_samples = []
        self.is_calibrated = False
        self.stationary_count = 0

        self.get_logger().info("✅ MPU6050 Node initialized successfully.")

    # ================================================================
    # 🧩 PARAMETER SETUP
    # ================================================================
    def _declare_and_get_parameters(self):
        """Declare all configurable parameters."""
        self.declare_parameter('bus_number', 1)
        self.declare_parameter('imu_publish_topic', '/imu')
        self.declare_parameter('imu_publish_period', 0.5)
        self.declare_parameter('imu_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('tf_frame_id', 'base_link')
        self.declare_parameter('tf_child_frame_id', 'imu')
        
        # Calibration parameters
        self.declare_parameter('bias_calibration_samples', 100)
        self.declare_parameter('motion_threshold', 0.1)
        self.declare_parameter('stationary_threshold', 10)

        # Retrieve parameter values
        self.bus_number = int(self.get_parameter('bus_number').value)
        self.imu_topic = self.get_parameter('imu_publish_topic').value
        self.publish_period = float(self.get_parameter('imu_publish_period').value)
        self.frame_id = self.get_parameter('imu_frame_id').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.tf_frame = self.get_parameter('tf_frame_id').value
        self.tf_child = self.get_parameter('tf_child_frame_id').value
        
        # Calibration parameters
        self.bias_calibration_samples = int(self.get_parameter('bias_calibration_samples').value)
        self.motion_threshold = float(self.get_parameter('motion_threshold').value)
        self.stationary_threshold = int(self.get_parameter('stationary_threshold').value)

    # ================================================================
    # 📡 ROS PUBLISHER AND TIMER
    # ================================================================
    def _init_publishers_and_timers(self):
        """Set up publisher and periodic timer."""
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
        self.timer = self.create_timer(self.publish_period,
                                       self._publish_imu_data)
        self.get_logger().info(
            f"Publishing IMU data every {self.publish_period}s on {self.imu_topic}"
        )

    # ================================================================
    # 🧭 TF BROADCASTER SETUP
    # ================================================================
    def _init_tf_broadcaster(self):
        """Initialize TF broadcaster (optional)."""
        self.tf_broadcaster = None
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
            self.get_logger().info("TF Broadcaster enabled for IMU data.")

    def _estimate_accel_bias(self, ax, ay, az):
        """Estimate accelerometer bias during stationary periods."""
        if not self.is_calibrated:
            self.bias_samples.append([ax, ay, az])
            
            if len(self.bias_samples) >= self.bias_calibration_samples:
                # Calculate mean bias
                bias_array = np.array(self.bias_samples)
                self.accel_bias = np.mean(bias_array, axis=0)
                self.is_calibrated = True
                self.get_logger().info(f"Accelerometer bias estimated: {self.accel_bias}")
                self.get_logger().info("Bias calibration complete. Starting motion detection.")
    
    def _compensate_gravity(self, ax, ay, az, quaternion):
        """Remove gravity component from accelerometer readings using orientation."""
        # Convert quaternion to rotation matrix
        q = quaternion
        # Normalize quaternion
        q_norm = q / np.linalg.norm(q)
        
        # Gravity vector in world frame (pointing down)
        gravity_world = np.array([0.0, 0.0, -9.80665])  # m/s²
        
        # Rotate gravity to body frame
        # Quaternion rotation: v' = q * v * q^-1
        qw, qx, qy, qz = q_norm
        
        # Rotation matrix from quaternion
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
        ])
        
        # Gravity in body frame (rotate world->body)
        gravity_body = R @ gravity_world
        
        # Remove gravity and bias
        accel_compensated = np.array([ax, ay, az]) - gravity_body - self.accel_bias
        
        return accel_compensated
    
    def _detect_motion(self, accel_compensated):
        """Detect if the rover is in motion based on compensated acceleration."""
        accel_magnitude = np.linalg.norm(accel_compensated)
        
        if accel_magnitude < self.motion_threshold:
            self.stationary_count += 1
            return False
        else:
            self.stationary_count = 0
            return True
    
    def _compute_linear_velocity(self, ax_ms2, ay_ms2, az_ms2):
        """Integrate linear acceleration to get linear velocity (m/s) with proper compensation."""
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # Integrate: v = v + a * dt
        self.linear_velocity[0] += ax_ms2 * dt
        self.linear_velocity[1] += ay_ms2 * dt
        self.linear_velocity[2] += az_ms2 * dt

        return self.linear_velocity, dt
    
    # ================================================================
    # 🧭 1️⃣ Get Orientation (from gyro + accel)
    # ================================================================
    def get_orientation(self, gx, gy, gz, ax, ay, az):
        # Convert gyro from deg/s to rad/s for filter
        gyro = np.radians(np.array([gx, gy, gz]))
        accel = np.array([ax, ay, az])

        # Update Madgwick filter and cache quaternion for next step
        new_quaternion = self.filter.updateIMU(self.prev_global_quaternion, gyro, accel)

        # Check for invalid or zero quaternions
        if new_quaternion is None or np.any(np.isnan(new_quaternion)):
            self.get_logger().warn("⚠️ Madgwick returned invalid quaternion, keeping previous orientation.")
            new_quaternion = self.prev_global_quaternion

        norm_q = np.linalg.norm(new_quaternion)
        if norm_q < 1e-6:
            self.get_logger().warn("⚠️ Zero-norm quaternion detected, using previous quaternion.")
            new_quaternion = self.prev_global_quaternion
            norm_q = np.linalg.norm(new_quaternion)

        # Normalize quaternion
        new_quaternion /= norm_q
        self.prev_global_quaternion = new_quaternion

        # Convert to scipy format: [x, y, z, w]
        quat = [new_quaternion[1], new_quaternion[2], new_quaternion[3], new_quaternion[0]]

        # Convert quaternion → Euler angles (roll, pitch, yaw)
        try:
            r = R.from_quat(quat)
            roll, pitch, yaw = r.as_euler('xyz', degrees=True)
        except ValueError:
            self.get_logger().warn("⚠️ Failed to convert quaternion to Euler. Resetting to (0,0,0).")
            roll, pitch, yaw = 0.0, 0.0, 0.0

        return roll, pitch, yaw, new_quaternion


    # ================================================================
    # 🧱 TF BROADCASTER
    # ================================================================
    def _broadcast_tf(self, quaternion):
        """Broadcast IMU orientation as a TF transform."""
        if not self.tf_broadcaster:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.tf_frame
        t.child_frame_id = self.tf_child
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.w = quaternion[0]
        t.transform.rotation.x = quaternion[1]
        t.transform.rotation.y = quaternion[2]
        t.transform.rotation.z = quaternion[3]


        self.tf_broadcaster.sendTransform(t)

    # ================================================================
    # 🧾 PUBLISH IMU DATA
    # ================================================================
    def _publish_imu_data(self):
        """Collect, compute, and publish IMU data."""
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # --- Read raw sensor data ---
        ax, ay, az = self.sensor.read_accel(g=False, samples=20)
        gx, gy, gz = self.sensor.read_gyro(samples=20)
    
        # --- Convert units to angular_velocity ---
        msg.angular_velocity.x = math.radians(gx)
        msg.angular_velocity.y = math.radians(gy)
        msg.angular_velocity.z = math.radians(gz)

        # --- Compute orientation first (needed for gravity compensation) ---
        roll, pitch, yaw, self.new_global_quaternion = self.get_orientation(gx, gy, gz, ax, ay, az)

        # --- Compute linear velocity with proper compensation ---
        linear_velocity, dt = self._compute_linear_velocity( ax, ay, az)
        
        # --- Set compensated acceleration in message ---
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.orientation.x = self.new_global_quaternion[1]  # x component
        msg.orientation.y = self.new_global_quaternion[2]  # y component  
        msg.orientation.z = self.new_global_quaternion[3]  # z component
        msg.orientation.w = self.new_global_quaternion[0]  # w component

        # --- Publish IMU message ---
        self.imu_pub.publish(msg)
        self._broadcast_tf(self.new_global_quaternion)

        # # --- Log summary ---
        self.get_logger().info(f"Linear Accel [m/s²]: ({msg.linear_acceleration.x:.2f}, {msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f})")
        self.get_logger().info(f"Linear Vel     [m/s]: ({linear_velocity[0]:.2f}, {linear_velocity[1]:.2f}, {linear_velocity[2]:.2f}) | Δt={dt:.2f}s")
        self.get_logger().info(f"Gyro [Radians]: ({msg.angular_velocity.x:.2f}, {msg.angular_velocity.y:.2f}, {msg.angular_velocity.z:.2f})") 
        self.get_logger().info(f"RPY [°]: ({math.degrees(roll):.2f}, {math.degrees(pitch):.2f}, {math.degrees(yaw):.2f})") # 
        self.get_logger().info(f"Quat [wxyz]: ({self.new_global_quaternion[0]:.2f}, {self.new_global_quaternion[1]:.2f}, {self.new_global_quaternion[2]:.2f}, {self.new_global_quaternion[3]:.2f})")

    # ================================================================
    # 🧹 CLEANUP
    # ================================================================
    def destroy_node(self):
        """Clean shutdown."""
        try:
            self.sensor.close()
        except Exception:
            pass
        super().destroy_node()

# ================================================================
# 🚀 MAIN ENTRY POINT
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("IMU Node interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# from std_msgs.msg import Header
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster
# import tf_transformations

# class IMUPublisher(Node):

#     def __init__(self):
#         super().__init__('imu_publisher')
#         self.sub = self.create_subscription(Imu, '/camera/imu',
#                                             self.imu_callback, 10)
#         self.pub = self.create_publisher(Imu, '/imu', 10)

#         # TF broadcaster
#         self.br = TransformBroadcaster(self)

#     def imu_callback(self, msg):
#         # Create a new message to avoid modifying the original
#         new_msg = Imu()
#         new_msg.header = Header()
#         new_msg.header.stamp = self.get_clock().now().to_msg()
#         new_msg.header.frame_id = 'base_link'

#         # Linear acceleration
#         new_msg.linear_acceleration.x = msg.linear_acceleration.z
#         new_msg.linear_acceleration.y = -msg.linear_acceleration.x
#         new_msg.linear_acceleration.z = msg.linear_acceleration.y

#         # Angular velocity
#         new_msg.angular_velocity.x = msg.angular_velocity.z
#         new_msg.angular_velocity.y = -msg.angular_velocity.x
#         new_msg.angular_velocity.z = msg.angular_velocity.y

#         # Orientation: apply rotation Rz(-90 deg) * Rx(-90 deg)
#         q_orig = [
#             msg.orientation.x, msg.orientation.y, msg.orientation.z,
#             msg.orientation.w
#         ]
#         q_rot = tf_transformations.quaternion_from_euler(-1.5708, 0.0, -1.5708)
#         q_new = tf_transformations.quaternion_multiply(q_rot, q_orig)

#         new_msg.orientation.x = q_new[0]
#         new_msg.orientation.y = q_new[1]
#         new_msg.orientation.z = q_new[2]
#         new_msg.orientation.w = q_new[3]
#         new_msg.orientation_covariance = msg.orientation_covariance

#         # Preserve covariances
#         new_msg.angular_velocity_covariance = msg.angular_velocity_covariance
#         new_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

#         # --- Broadcast transform base_link → imu ---

#         t = TransformStamped()
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = 'base_link'
#         t.child_frame_id = 'imu'  # or 'camera_imu_frame' if you prefer

#         t.transform.translation.x = 0.0
#         t.transform.translation.y = 0.0
#         t.transform.translation.z = 0.0

#         t.transform.rotation.x = q_rot[0]
#         t.transform.rotation.y = q_rot[1]
#         t.transform.rotation.z = q_rot[2]
#         t.transform.rotation.w = q_rot[3]

#         self.br.sendTransform(t)

#         self.pub.publish(new_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = IMUPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
