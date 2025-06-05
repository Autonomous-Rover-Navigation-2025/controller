#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations


class IMUPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.sub = self.create_subscription(Imu, '/camera/imu',
                                            self.imu_callback, 10)
        self.pub = self.create_publisher(Imu, '/imu', 10)

        # TF broadcaster
        self.br = TransformBroadcaster(self)

    def imu_callback(self, msg):
        # Create a new message to avoid modifying the original
        new_msg = Imu()
        new_msg.header = Header()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = 'base_link'

        # Linear acceleration
        new_msg.linear_acceleration.x = msg.linear_acceleration.z
        new_msg.linear_acceleration.y = -msg.linear_acceleration.x
        new_msg.linear_acceleration.z = msg.linear_acceleration.y

        # Angular velocity
        new_msg.angular_velocity.x = msg.angular_velocity.z
        new_msg.angular_velocity.y = -msg.angular_velocity.x
        new_msg.angular_velocity.z = msg.angular_velocity.y

        # Orientation: apply rotation Rz(-90 deg) * Rx(-90 deg)
        q_orig = [
            msg.orientation.x, msg.orientation.y, msg.orientation.z,
            msg.orientation.w
        ]
        q_rot = tf_transformations.quaternion_from_euler(-1.5708, 0.0, -1.5708)
        q_new = tf_transformations.quaternion_multiply(q_rot, q_orig)

        new_msg.orientation.x = q_new[0]
        new_msg.orientation.y = q_new[1]
        new_msg.orientation.z = q_new[2]
        new_msg.orientation.w = q_new[3]
        new_msg.orientation_covariance = msg.orientation_covariance

        # Preserve covariances
        new_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        new_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

        # --- Broadcast transform base_link → imu ---

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu'  # or 'camera_imu_frame' if you prefer

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = q_rot[0]
        t.transform.rotation.y = q_rot[1]
        t.transform.rotation.z = q_rot[2]
        t.transform.rotation.w = q_rot[3]

        self.br.sendTransform(t)

        self.pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
