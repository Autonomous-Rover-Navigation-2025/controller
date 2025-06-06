# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import Jetson.GPIO as GPIO
# import time
# import json

# # Define GPIO pins for each wheel's encoder
# ENCODER_PINS = {
#     "front_left": 19,
#     "front_right": 22,
#     "rear_left": 29,
#     "rear_right": 13
# }

# # Global state variables
# encoder_ticks = {
#     "front_left": 0,
#     "front_right": 0,
#     "rear_left": 0,
#     "rear_right": 0
# }

# last_tick_time = {
#     "front_left": None,
#     "front_right": None,
#     "rear_left": None,
#     "rear_right": None
# }

# freq = {
#     "front_left": None,
#     "front_right": None,
#     "rear_left": None,
#     "rear_right": None
# }

# rpm = {
#     "front_left": 0.0,
#     "front_right": 0.0,
#     "rear_left": 0.0,
#     "rear_right": 0.0
# }

# distance = {
#     "front_left": 0.0,
#     "front_right": 0.0,
#     "rear_left": 0.0,
#     "rear_right": 0.0
# }

# speed = {
#     "front_left": 0.0,
#     "front_right": 0.0,
#     "rear_left": 0.0,
#     "rear_right": 0.0
# }

# # Constants
# TIMER_PERIOD = 0.5  # seconds
# TICKS_PER_ROTATION = 620
# TPR = 360
# WHEEL_DIAMETER = 0.1524 #meters
# BASE_DISTANCE_BETWEEN_WHEELS = 0.26 #meters
# CIRCUMFERENCE = (3.1416 * WHEEL_DIAMETER )
# #L (float): Half of the wheelbase [Center to Center] (distance from front to rear wheel / 2) in meters.
# HALF_OF_WHEEL_BASE  = ( (0.31 - 0.05 -0.05) /2 )
# #W (float): Half of the track width [Center to Center] (distance between left and right wheels / 2) in meters.
# HALF_OF_TRACK_WIDTH  = ((0.05552 + 0.02 +0.24 + 0.02 + 0.05552) /2 )
# class WheelOdometer(Node):
#     def __init__(self):
#         super().__init__('wheel_odometry')
#         self.publisher_ = self.create_publisher(String, '/wheel_encoder_data', 10)
#         self.publish_timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

#         GPIO.setmode(GPIO.BOARD)
#         for wheel, pin in ENCODER_PINS.items():
#             GPIO.setup(pin, GPIO.IN)
#             GPIO.add_event_detect(pin, GPIO.RISING, callback=self.make_encoder_callback(wheel))

#     # Encoder tick callback generator
#     def make_encoder_callback(self, wheel):
#         def callback(channel):
#             self.update_parms(wheel)
#         return callback

#     def timer_callback(self):
#         data_dict = {}

#         for wheel in encoder_ticks:
#             data_dict[wheel] = {
#                 'ticks': self.get_encoder_ticks(wheel),
#                 'freq': self.get_frequency(wheel),
#                 'rpm': self.get_rpm(wheel),
#                 'distance_m': self.get_distance(wheel),
#                 'speed_mps': self.get_speed(wheel)
#             }
#             encoder_ticks[wheel] = 0  # Reset after publishing
#         self.get_logger().info(f"Ticks --> 'front_left' (pin:{ENCODER_PINS['front_left']}):{data_dict['front_left']['ticks']},    'front_right' (pin:{ENCODER_PINS['front_right']}):{data_dict['front_right']['ticks']},     'rear_left' (pin:{ENCODER_PINS['rear_left']}):{data_dict['rear_left']['ticks']},       'rear_right' (pin:{ENCODER_PINS['rear_right']}):{data_dict['rear_right']['ticks']}")
#         # self.get_logger().info(f"freq --> 'front_left'  (pin:{ENCODER_PINS['front_left']}):{data_dict['front_left']['freq']},     'front_right' (pin:{ENCODER_PINS['front_right']}):{data_dict['front_right']['freq']},      'rear_left' (pin:{ENCODER_PINS['rear_left']}):{data_dict['rear_left']['freq']},        'rear_right' (pin:{ENCODER_PINS['rear_right']}):{data_dict['rear_right']['freq']}")
#         # self.get_logger().info(f"rpm --> 'front_left'   (pin:{ENCODER_PINS['front_left']}):{data_dict['front_left']['rpm']},      'front_right' (pin:{ENCODER_PINS['front_right']}):{data_dict['front_right']['rpm']},       'rear_left' (pin:{ENCODER_PINS['rear_left']}):{data_dict['rear_left']['rpm']},         'rear_right' (pin:{ENCODER_PINS['rear_right']}):{data_dict['rear_right']['rpm']}")
#         self.get_logger().info(f"Speed(m/s) --> 'front_left' (pin:{ENCODER_PINS['front_left']}):{data_dict['front_left']['speed_mps']:.3f},'front_right' (pin:{ENCODER_PINS['front_right']}):{data_dict['front_right']['speed_mps']:.3f}, 'rear_left' (pin:{ENCODER_PINS['rear_left']}):{data_dict['rear_left']['speed_mps']:.3f},   'rear_right' (pin:{ENCODER_PINS['rear_right']}):{data_dict['rear_right']['speed_mps']:.3f}")

       
#         vx, vy, omega_z = self.compute_robot_velocity(speed, HALF_OF_WHEEL_BASE, HALF_OF_TRACK_WIDTH)
#         self.get_logger().info(f"vx = {vx:.2f} m/s, vy = {vy:.2f} m/s, omega_z = {omega_z:.2f} rad/s") 



#     # ==== Helper Functions ====

#     def update_parms(self, wheel):
#         now = time.time()
#         last = last_tick_time[wheel]
#         if last is not None:
#             delta = now - last
#             freq[wheel] =  1.0 / delta if delta > 0 else 0.0
#             rpm[wheel] = ( freq[wheel] * 60.0) / TICKS_PER_ROTATION
#             speed[wheel] = ( (rpm[wheel] * CIRCUMFERENCE ) / 60 )
#             distance[wheel] =  (speed[wheel] * TIMER_PERIOD )
#         last_tick_time[wheel] = now
#         encoder_ticks[wheel] += 1

#     def get_frequency(self, wheel):
#         return freq[wheel]

#     def get_rpm(self, wheel):
#         return rpm[wheel]

#     def get_encoder_ticks(self, wheel):
#         return encoder_ticks[wheel]

#     def get_rpm(self, wheel):
#         return rpm[wheel]

#     def get_distance(self, wheel):
#         return distance[wheel]

#     def get_speed(self, wheel):
#         return speed[wheel]
    
#     def compute_robot_velocity(self, speed, L, W):
#         """
#         Compute the robot's linear (vx, vy) and angular (omega_z) velocity 
#         for a 4-wheeled Mecanum drive (Using individuals speed (m/s) directly)

#         Args:
#             speed (dict): Dictionary with keys "front_left", "front_right", 
#                         "rear_left", "rear_right" (values in m/s).
#             L (float): Half of the wheelbase [Center to Center] (distance from front to rear wheel / 2) in meters.
#             W (float): Half of the track width [Center to Center] (distance between left and right wheels / 2) in meters.

#         Returns:
#             tuple: (vx, vy, omega_z) velocities in m/s and rad/s.
#         """
#         v1 = float(speed["front_left"])
#         v2 = float(speed["front_right"])
#         v3 = float(speed["rear_left"])
#         v4 = float(speed["rear_right"])

#         a = L + W  # Distance factor for rotation

#         vx = (v1 + v2 + v3 + v4) / 4.0
#         vy = (-v1 + v2 + v3 - v4) / 4.0
#         omega_z = (-v1 + v2 - v3 + v4) / (4.0 * a)

#         return vx, vy, omega_z

# # ==== Entry Point ====

# def main(args=None):
#     rclpy.init(args=args)
#     wheelodom_node = WheelOdometer()

#     try:
#         rclpy.spin(wheelodom_node)
#     except KeyboardInterrupt:
#         print("KeyboardInterrupt received. Shutting down...")
#     except Exception as e:
#         print(f"An error occurred: {e}")
#     finally:
#         wheelodom_node.destroy_node()
#         GPIO.cleanup()
#         if rclpy.ok():
#             rclpy.shutdown()

# if __name__ == '__main__':
#     main()




import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import Jetson.GPIO as GPIO
import time


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
TPR = 360
WHEEL_DIAMETER = 0.1524 #meters
BASE_DISTANCE_BETWEEN_WHEELS = 0.26 #meters
CIRCUMFERENCE = (3.1416 * WHEEL_DIAMETER )

#L (float): Half of the wheelbase [Center to Center] (distance from front to rear wheel / 2) in meters.
HALF_OF_WHEEL_BASE  = ( (0.31 - 0.05 -0.05) /2 )
#W (float): Half of the track width [Center to Center] (distance between left and right wheels / 2) in meters.
HALF_OF_TRACK_WIDTH  = ((0.05552 + 0.02 +0.24 + 0.02 + 0.05552) /2 )
class WheelOdometer(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        self.publisher_ = self.create_publisher(String, '/wheel_ticks', 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

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
            distance[wheel] = (encoder_ticks[wheel] / TICKS_PER_ROTATION ) * CIRCUMFERENCE
            speed[wheel] = distance[wheel] / TIMER_PERIOD


        tick_msg = Float32MultiArray()
        tick_msg.data = [
            float(encoder_ticks["front_left"]),
            float(encoder_ticks["front_right"]),
            float(encoder_ticks["rear_left"]),
            float(encoder_ticks["rear_right"])
        ]
        # self.get_logger().info(f"tick_msg.data : {tick_msg.data}")  
        self.get_logger().info(f"tick_msg.data -->  'front_left' (pin:{ENCODER_PINS['front_left']}): {tick_msg.data[0]}, 'front_right' (pin:{ENCODER_PINS['front_right']}):{tick_msg.data[1]}, 'rear_left' (pin:{ENCODER_PINS['rear_left']}):{tick_msg.data[2]}, rear_right' (pin:{ENCODER_PINS['rear_right']}):{tick_msg.data[3]}") 

        # distance_msg = Float32MultiArray()
        # distance_msg.data = [
        #     float(distance["front_left"]),
        #     float(distance["front_right"]),
        #     float(distance["rear_left"]),
        #     float(distance["rear_right"])
        # ]
        # self.get_logger().info(f"distance_msg.data : {distance_msg.data}") 
        # self.get_logger().info(f"distance_msg.data --> 'front_left' (pin:{ENCODER_PINS['front_left']}): {distance_msg.data[0]:.3f}, 'front_right' (pin:{ENCODER_PINS['front_right']}):{distance_msg.data[1]:.3f}, 'rear_left' (pin:{ENCODER_PINS['rear_left']}):{distance_msg.data[2]:.3f}, rear_right' (pin:{ENCODER_PINS['rear_right']}):{distance_msg.data[3]:.3f}") 


        speed_msg = Float32MultiArray()
        speed_msg.data = [
            float(speed["front_left"]),
            float(speed["front_right"]),
            float(speed["rear_left"]),
            float(speed["rear_right"])
        ]
        # self.get_logger().info(f"speed_msg.data : {speed_msg.data}") 
        self.get_logger().info(f"speed_msg.data --> 'front_left' (pin:{ENCODER_PINS['front_left']}): {speed_msg.data[0]:.3f}, 'front_right' (pin:{ENCODER_PINS['front_right']}):{speed_msg.data[1]:.3f}, 'rear_left' (pin:{ENCODER_PINS['rear_left']}):{speed_msg.data[2]:.3f}, rear_right' (pin:{ENCODER_PINS['rear_right']}):{speed_msg.data[3]:.3f}") 
        vx, vy, omega_z = self.compute_robot_velocity(speed, HALF_OF_WHEEL_BASE, HALF_OF_TRACK_WIDTH)
        self.get_logger().info(f"vx = {vx:.2f} m/s, vy = {vy:.2f} m/s, omega_z = {omega_z:.2f} rad/s") 


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




# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import Jetson.GPIO as GPIO
# import time
# import json
# import math
# from geometry_msgs.msg import TransformStamped
# import tf_transformations
# import tf2_ros
# from nav_msgs.msg import Odometry

# # Define GPIO pins for each wheel's encoder
# ENCODER_PINS = {
#     "front_left": 19,
#     "front_right": 22,
#     "rear_left": 29,
#     "rear_right": 13
# }

# # Global state variables
# encoder_ticks = {
#     "front_left": 0,
#     "front_right": 0,
#     "rear_left": 0,
#     "rear_right": 0
# }

# distance = {
#     "front_left": 0.0,
#     "front_right": 0.0,
#     "rear_left": 0.0,
#     "rear_right": 0.0
# }

# speed = {
#     "front_left": 0.0,
#     "front_right": 0.0,
#     "rear_left": 0.0,
#     "rear_right": 0.0
# }


# # Constants
# TIMER_PERIOD = 1  # seconds
# TICKS_PER_ROTATION = 620
# WHEEL_DIAMETER = 0.1524 #meters
# BASE_DISTANCE_BETWEEN_WHEELS = 0.26 #meters
# CIRCUMFERENCE = (math.pi * WHEEL_DIAMETER )
# #L (float): Half of the wheelbase [Center to Center] (distance from front to rear wheel / 2) in meters.
# HALF_OF_WHEEL_BASE  = ( (0.31 - 0.05 -0.05) /2 )
# #W (float): Half of the track width [Center to Center] (distance between left and right wheels / 2) in meters.
# HALF_OF_TRACK_WIDTH  = ((0.05552 + 0.02 +0.24 + 0.02 + 0.05552) /2 )
# class WheelOdometer(Node):
#     def __init__(self):
#         super().__init__('wheel_odometry')


#         self.tick_to_distance = CIRCUMFERENCE / TICKS_PER_ROTATION
#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0.0  # radians
#         self.last_calu_time = None
        
#         self.publisher_ = self.create_publisher(String, '/wheel_encoder_data', 10)
#         self.publish_timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

#         GPIO.setmode(GPIO.BOARD)
#         for wheel, pin in ENCODER_PINS.items():
#             GPIO.setup(pin, GPIO.IN)
#             GPIO.add_event_detect(pin, GPIO.RISING, callback=self.make_encoder_callback(wheel))

#     # Encoder tick callback generator
#     def make_encoder_callback(self, wheel):
#         def callback(channel):
#             self.update_ticks(wheel)
#         return callback
    
#     def update_ticks(self, wheel):
#         encoder_ticks[wheel] += 1

#     def timer_callback(self):
#         now = time.time()
#         dt = now - self.last_calu_time
#         self.last_calu_time = now
#         self.get_logger().info(f"encoder_ticks -->  'front_left' (pin:{ENCODER_PINS['front_left']}): {encoder_ticks['front_left']}, 'front_right' (pin:{ENCODER_PINS['front_right']}):{encoder_ticks['front_right']}, 'rear_left' (pin:{ENCODER_PINS['rear_left']}):{encoder_ticks['rear_left']}, 'rear_right' (pin:{ENCODER_PINS['rear_right']}):{encoder_ticks['rear_right']}") 
#         self.tick_to_distance = CIRCUMFERENCE / TICKS_PER_ROTATION
#        for wheel, ticks in tick_counts.items():
#             rps = ticks / ticks_per_revolution
#             linear_speed = rps * 2 * math.pi * wheel_radius
#             wheel_speeds[wheel] = linear_speed

#         encoder_ticks["front_left"] = 0
#         encoder_ticks["front_right"] = 0
#         encoder_ticks["rear_left"] = 0
#         encoder_ticks["rear_right"] = 0

#         vx, vy, omega_z = self.compute_robot_velocity(speed,
#                                                       HALF_OF_WHEEL_BASE,
#                                                       HALF_OF_TRACK_WIDTH)
#         self.get_logger().info(f"vx = {vx:.2f} m/s, vy = {vy:.2f} m/s, omega_z = {omega_z:.2f} rad/s")

#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds * 1e-9

#         # Integrate velocity to update pose
#         delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
#         delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
#         delta_theta = omega_z * dt

#         self.x += delta_x
#         self.y += delta_y
#         self.theta += delta_theta

#         self.get_logger().info(f"self.x = {self.x:.2f} m, self.y = {self.y:.2f} m, self.theta = {self.theta:.2f} rad")

#         # Prepare odometry message
#         odom = Odometry()
#         odom.header.stamp = current_time.to_msg()
#         odom.header.frame_id = "odom"
#         odom.child_frame_id = "base_link"

#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y
#         odom.pose.pose.position.z = 0.0
#         q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
#         odom.pose.pose.orientation.x = q[0]
#         odom.pose.pose.orientation.y = q[1]
#         odom.pose.pose.orientation.z = q[2]
#         odom.pose.pose.orientation.w = q[3]

#         odom.twist.twist.linear.x = vx
#         odom.twist.twist.linear.y = vy
#         odom.twist.twist.angular.z = omega_z
#         self.get_logger().info(f"Publish odom = {odom}")
#         self.odom_pub.publish(odom)

#         # Publish TF
#         t = TransformStamped()
#         t.header.stamp = current_time.to_msg()
#         t.header.frame_id = "odom"
#         t.child_frame_id = "base_link"
#         t.transform.translation.x = self.x
#         t.transform.translation.y = self.y
#         t.transform.translation.z = 0.0
#         t.transform.rotation.x = q[0]
#         t.transform.rotation.y = q[1]
#         t.transform.rotation.z = q[2]
#         t.transform.rotation.w = q[3]
#         self.get_logger().info(f"Publish TF = {t}")
#         self.tf_broadcaster.sendTransform(t)
#         self.last_time = current_time
       

#     def update_from_ticks(self, delta_left_ticks, delta_right_ticks, dt):
#         # Convert ticks to distance
#         d_left = delta_left_ticks * self.tick_to_distance
#         d_right = delta_right_ticks * self.tick_to_distance

#         # Compute velocities (m/s)
#         v_left = d_left / dt
#         v_right = d_right / dt

#         # Linear and angular velocity
#         v = (v_left + v_right) / 2
#         omega = (v_right - v_left) / self.L

#         # Update pose using simple kinematics
#         self.x += v * math.cos(self.theta) * dt
#         self.y += v * math.sin(self.theta) * dt
#         self.theta += omega * dt
#         self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

#         return v, omega

#     def get_pose(self):
#         return self.x, self.y, self.theta



# # ==== Entry Point ====

# def main(args=None):
#     rclpy.init(args=args)
#     wheelodom_node = WheelOdometer()

#     try:
#         rclpy.spin(wheelodom_node)
#     except KeyboardInterrupt:
#         print("KeyboardInterrupt received. Shutting down...")
#     except Exception as e:
#         print(f"An error occurred: {e}")
#     finally:
#         wheelodom_node.destroy_node()
#         GPIO.cleanup()
#         if rclpy.ok():
#             rclpy.shutdown()

# if __name__ == '__main__':
#     main()