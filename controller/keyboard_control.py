'''
🎮 ROS2 Keyboard Control Node (keyboard_control)
📥 User Interaction
        This ROS2 node captures real-time keyboard input from the terminal using WASD keys to manually control a robot or rover. The key commands are:

            W - Move Forward
            S - Move Backward
            A - Turn Left
            D - Turn Right
            Q - Stop and Exit

📤 What This Node Publishes
        Topic: /cmd_vel

        Message Type: geometry_msgs/msg/Twist

        Purpose: Sends velocity commands (linear and angular) to another ROS2 node (e.g., Serial_Com) that likely handles motor control via serial communication.

✅ Key Features
        Converts user keystrokes into Twist velocity commands.

        Publishes the commands over ROS2 to /cmd_vel for controlling robot motion.

        Logs activity and provides user-friendly terminal feedback.

        Ensures the robot stops when exiting the program.

'''

# Import necessary ROS and system modules
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
import sys
import tty
import termios
import time
"""  This defines a custom class for BasicKey ControlNode """


class Basic_Key_ControlNode(Node):
    """
    Initializes the ControlNode for sending velocity commands over ROS 2.

    - Sets the name of the node to 'control_node'.
    - Creates a Twist message instance for publishing movement commands.
    - Sets up a publisher to the '/command' topic (used by Serial_Com Node).
    - Displays control instructions (WASD keys) in the console log.
    """

    def __init__(self):
        # Initialize the ROS node which controls the Motor through simple Key Cmd.
        super().__init__('keyboard_control')

        #is creating an instance of the Twist message type from the ROS 2 geometry_msgs package.
        self.twist_msg_ = Twist()

        # Publisher for sending velocity commands to the Serial_Com Node
        self.Serial_Com_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        self.get_logger().info(
            "Use keys: [W] Forward, [S] Backward, [A] Left, [D] Right, [Q] Quit"
        )

    """ 
        What it does:
          -  Takes in two floats: linear velocity (x) and angular velocity (z)
          -  Sets these values on the Twist message
          -  Publishes the message to the  topic
          -  Logs the topic name and what was published
    """

    def publish_twist(self, linear_x: float, angular_z: float):
        msg = Twist()
        self.twist_msg_.linear.x = linear_x
        self.twist_msg_.angular.z = angular_z
        self.Serial_Com_pub.publish(self.twist_msg_)
        Serial_Com_pub_topic_name = self.Serial_Com_pub.topic
        self.get_logger().info(
            f"Published: self.twist_msg_ : {self.twist_msg_} on Topic={Serial_Com_pub_topic_name}"
        )


def catch_keypress():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = Basic_Key_ControlNode()
    linear_velocity = 0.0
    angular_velocity = 0.0
    try:
        while True:
            key = catch_keypress().lower()
            if key == 'w':
                linear_velocity += 0.1
                angular_velocity = 0.0
                node.publish_twist(linear_velocity, angular_velocity)
                node.get_logger().info(
                    f"You pressed {key.upper()} — moving forward with linear_velocity: {linear_velocity}, angular_velocity: {angular_velocity}"
                )
            elif key == 's':
                linear_velocity -= 0.1
                angular_velocity = 0.0
                node.publish_twist(linear_velocity, angular_velocity)
                node.get_logger().info(
                    f"You pressed {key} Key, Move Rover Backward with linear_velocity: {linear_velocity} and angular_velocity: {angular_velocity}"
                )
            elif key == 'a':
                angular_velocity += 0.1
                linear_velocity = 0.0
                node.publish_twist(linear_velocity, angular_velocity)
                node.get_logger().info(
                    f"You pressed {key} Key, which Turns Rover Left with linear_velocity: {linear_velocity} and angular_velocity: {angular_velocity}"
                )
            elif key == 'd':
                angular_velocity -= 0.1
                linear_velocity = 0.0
                node.publish_twist(linear_velocity, angular_velocity)
                node.get_logger().info(
                    f"You pressed {key} Key, which Turns Rover Right with linear_velocity: {linear_velocity} and angular_velocity: {angular_velocity}"
                )
            elif key == 'q':
                linear_velocity = 0.0
                angular_velocity = 0.0
                node.publish_twist(linear_velocity, angular_velocity)
                node.get_logger().info(
                    f"You pressed {key} Key, Exiting. Stopping robot with linear_velocity: {linear_velocity} and angular_velocity: {angular_velocity}"
                )
                break
            else:
                node.get_logger().warn(
                    f"Unsupported key: '{key}'. Use W/S/A/D/Q.")
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        node.publish_twist(0.0, 0.0)  # Ensure robot stops
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
