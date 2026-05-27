# Save this file as: ~/web_robot_ws/src/robot_control/robot_control/robot_cmd_publisher.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

# This class will publish Twist messages to control the robot
class RobotCmdPublisher(Node):

    def __init__(self):
        super().__init__('robot_cmd_publisher')
        # Create a publisher for the /cmd_vel topic
        # The quality of service (QoS) setting 'qos_profile_sensor_data'
        # is often good for real-time control commands.
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_teleop', 10) # Using 'diff_drive_controller/cmd_vel_unstamped' as defined in your YAML
        self.timer_period = 0.1 # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.get_logger().info('Robot Command Publisher Node has started.')
        self.get_logger().info('Use W/A/S/D for movement. Press Q to quit.')

        # Store the original terminal settings to restore them later
        self.settings = termios.tcgetattr(sys.stdin)

    def timer_callback(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.publisher_.publish(twist)

    # Function to get keyboard input without waiting for Enter
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        # The `select.select()` function is used here to check if input is available
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1) # Timeout after 0.1s
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def move_robot_with_keyboard(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == 'w':
                    self.linear_speed = 0.5 # Move forward at 0.5 m/s
                    self.angular_speed = 0.0
                    self.get_logger().info('Moving forward!') # Added log
                elif key == 's':
                    self.linear_speed = -0.5 # Move backward at 0.5 m/s
                    self.angular_speed = 0.0
                    self.get_logger().info('Moving backward!') # Added log
                elif key == 'a':
                    self.linear_speed = 0.0
                    self.angular_speed = 0.4 # Turn left at 0.5 rad/s
                    self.get_logger().info('Turning left!') # Added log
                elif key == 'd':
                    self.linear_speed = 0.0
                    self.angular_speed = -0.4 # Turn right at 0.5 rad/s
                    self.get_logger().info('Turning right!') # Added log
                elif key == ' ': # Spacebar to stop
                    self.linear_speed = 0.0
                    self.angular_speed = 0.0
                    self.get_logger().info('Stopping!') # Added log
                elif key == 'q':
                    self.get_logger().info('Quitting robot controller.')
                    break
                else:
                    # If no key pressed or an unrecognized key, maintain current speed (or stop)
                    pass # Or set to 0.0 to stop if no key pressed: self.linear_speed = 0.0; self.angular_speed = 0.0

                rclpy.spin_once(self) # Process ROS callbacks
                
        except Exception as e:
            self.get_logger().error(f'Error in keyboard control: {e}')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings) # Restore terminal settings
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    robot_cmd_publisher = RobotCmdPublisher()
    robot_cmd_publisher.move_robot_with_keyboard()

if __name__ == '__main__':
    main()
