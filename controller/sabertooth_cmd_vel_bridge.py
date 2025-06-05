#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class SabertoothBridge(Node):

    def __init__(self):
        super().__init__('sabertooth_cmd_vel_bridge')

        # Declare ROS parameters
        self.declare_parameter('wheel_base',
                               0.26)  # Distance between wheels (meters)
        self.declare_parameter('max_speed',
                               1.21)  # Max linear speed per wheel (m/s)
        self.declare_parameter('port',
                               '/dev/ttyTHS0')  # Serial device for Sabertooth
        self.declare_parameter('baud', 9600)  # Baud rate

        # Load parameters
        self.L = self.get_parameter('wheel_base').value
        self.v_max = self.get_parameter('max_speed').value
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        # Serial connection
        self.ser = serial.Serial(port, baud, timeout=0.1)

        # Sabertooth motor addresses
        self.addr_left = 129
        self.addr_right = 128

        # Subscribe to velocity commands
        self.subscription = self.create_subscription(Twist, '/cmd_vel_nav',
                                                     self.cmd_vel_callback, 10)

        self.get_logger().info(
            'Sabertooth bridge initialized and listening on /cmd_vel_nav')

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        # Compute individual wheel velocities
        v_l = v + w * self.L / 2
        v_r = v - w * self.L / 2

        # Convert to motor command values (range: -127 to 127)
        val_l = self.clamp_and_int(v_l / self.v_max * 127, -127, 127)
        val_r = self.clamp_and_int(v_r / self.v_max * 127, -127, 127)

        # Send to left motor
        self.send_motor_command(self.addr_left, val_l)

        # Send to right motor
        self.send_motor_command(self.addr_right, val_r)

    def clamp_and_int(self, x, min_val, max_val):
        return int(max(min_val, min(max_val, round(x))))

    def calculate_checksum(self, address, command, value):
        return (address + command + value) & 0x7F

    def send_packet(self, address, command, value):
        checksum = self.calculate_checksum(address, command, value)
        packet = bytes([address, command, value, checksum])
        self.ser.write(packet)
        self.get_logger().info(
            f"Sent: addr={address}, cmd={command}, val={value}, chk={checksum}"
        )

    def send_motor_command(self, address, speed):
        """Send forward/reverse commands based on speed sign."""
        if speed >= 0:
            self.send_packet(address, 0, speed)  # Forward
            self.send_packet(address, 4, speed)  # Optional: ramp
        else:
            self.send_packet(address, 1, abs(speed))  # Reverse
            self.send_packet(address, 5, abs(speed))  # Optional: ramp


def main(args=None):
    rclpy.init(args=args)
    node = SabertoothBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
