'''
⚙️ ROS2 Serial Communication Node for Rover Motor Control (serial_com)
📥 Subscriber Details
        Subscribed Topic: /cmd_vel

        Message Type: geometry_msgs/msg/Twist

        Purpose: Receives velocity commands from higher-level control nodes (e.g., keyboard teleop).

🧠 Core Functionality
        Decodes incoming linear (x) and angular (z) velocity values.

        Calculates motor speeds for left and right sides based on command type:

        Forward / Backward motion (based on linear.x)

        Left / Right turns (based on angular.z)

        Stops all motors when no command is active.

🔁 Serial Communication with Motor Driver (Sabertooth)
        Uses UART over /dev/ttyTHS0 to communicate with a Sabertooth Motor Controller.

        Constructs and sends 4-byte serial packets:
        [address, command, value, checksum]

        Calculates a checksum for packet integrity.

        Sends different commands for forward, reverse, and braking control depending on motor direction and address.

🔌 Sabertooth Controller Setup
        Address 128 → Right Motor

        Address 129 → Left Motor

        Commands:

                0 / 1: Forward / Reverse

                4 / 5: Forward / Reverse with ramping or braking support
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
import serial


sabertooth_serial = serial.Serial('/dev/ttyTHS0', 9600, timeout=1)

class Rover_Movement(Node):
    pose_x=0.0
    pose_y=0.0
    theta=0.0

    pose_msg = Pose()

    def __init__(self):
        super().__init__('serial_com')
        self.main_left = 0
        self.main_right = 0
        self.left_side = 0.0
        self.right_side = 0.0

        self.pose_x=0.0
        self.pose_y=0.0
        self.theta=0.0

        self.checksum = 0
        self.packet = 0

        self.address = 0

        self.sub = self.create_subscription(Twist, '/cmd_vel_nav', self.cmd_callback, 10)
        
        
    def calculate_checksum(self, address, command, value):
        return (address + command + int(value)) & 0b01111111
        #self.checksum = (address + command + value) & 0b01111111

    def send_sabertooth_command(self, serial_port, address, command, value):
        self.checksum = self.calculate_checksum(address, command, value)
        self.packet = bytes([address, command, int(value), self.checksum])
        serial_port.write(self.packet)
        print(f"Sent command to Sabertooth: Address: {address}, Command: {command}, Value: {int(value)}, Checksum: {self.checksum}")
        


    def cmd_callback(self, msg):
        print("cmd received!!!")
        print("sabertooth_serial started : /dev/ttyTHS0") #@Sahil
        if (msg.linear.x != 0.0 and msg.angular.z == 0.0):
            #For Forward motion
            if msg.linear.x > 0.0 :
                self.left_side = int(msg.linear.x)
                self.right_side = int(msg.linear.x)
                self.main_left = int(self.main_left + self.left_side)
                self.main_right = int(self.main_right + self.right_side)
                self.main_left = max(min(self.main_left, 127), -127)
                self.main_right = max(min(self.main_right, 127), -127)
                print(f"main_left: {self.main_left}, main_right: {self.main_right}, left_side: {self.left_side}, right_side: {self.right_side} .")
                for address in [128,129]:
                    if (address == 129):
                        if (self.main_left >= 0):
                            for command in [0,4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.main_left)
                        else:
                            for command in [1,5]:
                                true_left = abs(self.main_left)
                                self.send_sabertooth_command(sabertooth_serial, address, command, true_left)
                    else:
                        if (self.main_right >= 0):
                            for command in [0,4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.main_right)
                        else :
                            for command in [1,5]:
                                true_right = abs(self.main_right)
                                self.send_sabertooth_command(sabertooth_serial, address, command, true_right)
            else:
                #For Backward motion
                self.left_side = int(msg.linear.x)
                self.right_side = int(msg.linear.x)
                self.main_left = int(self.main_left + self.left_side)
                self.main_right = int(self.main_right + self.right_side)
                self.main_left = max(min(self.main_left, 127), -127)
                self.main_right = max(min(self.main_right, 127), -127)
                print(f"main_left: {self.main_left}, main_right: {self.main_right}, left_side: {self.left_side}, right_side: {self.right_side} .")
                for address in [128,129]:
                    if (address == 129):
                        if (self.main_left >= 0):
                            for command in [0,4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.main_left)
                        else:
                            for command in [1,5]:
                                true_left = abs(self.main_left)
                                self.send_sabertooth_command(sabertooth_serial, address, command, true_left)
                    else:
                        if (self.main_right >= 0):
                            for command in [0,4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.main_right)
                        else :
                            for command in [1,5]:
                                true_right = abs(self.main_right)
                                self.send_sabertooth_command(sabertooth_serial, address, command, true_right)

        
        elif (msg.linear.x == 0.0 and msg.angular.z != 0.0):
            print("turning")
            #For right motion
            if(msg.angular.z > 0.0):
                self.left_side = -int(abs(msg.angular.z))
                self.right_side = int(abs(msg.angular.z))
                self.main_left = int(self.main_left + self.left_side)
                self.main_right =  int(self.main_right + self.right_side)
                self.main_left = max(min(self.main_left, 127), -127)
                self.main_right = max(min(self.main_right, 127), -127)
                print(f"main_left: {self.main_left}, main_right: {self.main_right}, left_side: {self.left_side}, right_side: {self.right_side} .")
                for address in [128,129]:
                    if (address == 129):
                        if (self.main_left >= 0):
                            for command in [0,4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.main_left)
                        else:
                            for command in [1,5]:
                                true_left = abs(self.main_left)
                                self.send_sabertooth_command(sabertooth_serial, address, command, true_left)
                    else:
                        if (self.main_right >= 0):
                            for command in [0,4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.main_right)
                        else :
                            for command in [1,5]:
                                true_right = abs(self.main_right)
                                self.send_sabertooth_command(sabertooth_serial, address, command, true_right)
                
            else :
                #for Left motion
                self.left_side = int(abs(msg.angular.z))
                self.right_side = -int(abs(msg.angular.z))
                self.main_left =  int(self.main_left + self.left_side)
                self.main_right = int(self.main_right + self.right_side)
                self.main_left = max(min(self.main_left, 127), -127)
                self.main_right = max(min(self.main_right, 127), -127)
                print(f"main_left: {self.main_left}, main_right: {self.main_right}, left_side: {self.left_side}, right_side: {self.right_side} .")
                for address in [128,129]:
                    if (address == 129):
                        if (self.main_left >= 0):
                            for command in [0,4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.main_left)
                        else:
                            for command in [1,5]:
                                true_left = abs(self.main_left)
                                self.send_sabertooth_command(sabertooth_serial, address, command, true_left)
                    else:
                        if (self.main_right >= 0):
                            for command in [0,4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.main_right)
                        else :
                            for command in [1,5]:
                                true_right = abs(self.main_right)
                                self.send_sabertooth_command(sabertooth_serial, address, command, true_right)
               

        else :
            for address in [128,129]:
                for command in [0,1,4,5]:
                    self.send_sabertooth_command(sabertooth_serial, address, command, 0)
            
            for address in [128,129]:
                for command in [0,1,4,5]:
                    self.send_sabertooth_command(sabertooth_serial, address, command, 0)

            print("Stopping!!!!!!!")

def main(args=None):
    rclpy.init(args=args)
    serial_com = Rover_Movement()
    rclpy.spin(serial_com)
    serial_com.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
