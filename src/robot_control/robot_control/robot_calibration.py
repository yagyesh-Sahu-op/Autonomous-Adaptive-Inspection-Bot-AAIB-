# Save this file as: ~/web_robot_ws/src/robot_control/robot_control/robot_cmd_publisher.py

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select
import math 
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

# This class will publish Twist messages to control the robot
class RobotCalibration(Node):

    def __init__(self):
        super().__init__('robot_calibration')
        # Create a publisher for the /cmd_vel topic
        # The quality of service (QoS) setting 'qos_profile_sensor_data'
        # is often good for real-time control commands.
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_teleop', 10) # Using 'diff_drive_controller/cmd_vel_unstamped' as defined in your YAML
        self.timer_period = 0.1 # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.linear_speed = 0.0
        self.angular_speed = 0.0

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered', # Make sure this matches your robot's odometry topic
            self.odom_callback,
            10
        )

        self.current_pose = None
        self.previous_yaw = None
        self.accumulated_rotation = 0.0
        self.tracking_rotation = False


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
    
    def odom_callback(self, msg):
        """Store current odometry and track accumulated rotation"""
        self.current_pose = msg.pose.pose
        
        if self.tracking_rotation:
            current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
            
            if self.previous_yaw is not None:
                delta_yaw = self.normalize_angle(current_yaw - self.previous_yaw)
                self.accumulated_rotation += delta_yaw
            
            self.previous_yaw = current_yaw
        
    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def stop_robot(self):
        """Send stop command to robot"""
        self.current_command = Twist()
        self.linear_speed = -0.5
        self.angular_speed = -0.5 
        self.publishing_enabled = True
        time.sleep(0.2)  # Ensure stop command is sent
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.publishing_enabled = False

    def rotation_test_callback(self, request, response):
        """Service callback for rotation calibration test"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('Starting Rotation Calibration Test')
        self.get_logger().info('=' * 60)
        
        # Wait for initial odometry
        timeout = 5.0
        start_wait = time.time()
        while self.current_pose is None and (time.time() - start_wait) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_pose is None:
            self.get_logger().error("No odom received!")
            return

        # Reset tracking
        self.accumulated_rotation = 0.0
        initial_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        self.get_logger().info(f"Initial orientation (yaw): {math.degrees(initial_yaw):.2f}°")
        self.previous_yaw = initial_yaw
        self.tracking_rotation = True  # ← Enable tracking in odom_callback

        self.get_logger().info(f"Initial yaw: {math.degrees(initial_yaw):.2f}°")

        # Correct duration
        self.rotation_angular_vel = 1.0
        self.rotation_duration = (2 * math.pi) / self.rotation_angular_vel  # 6.28s

        # Rotate
        test_start = self.get_clock().now()
        while (self.get_clock().now() - test_start).nanoseconds < self.rotation_duration * 1e9:
            self.linear_speed = 0.0
            self.angular_speed = self.rotation_angular_vel
            rclpy.spin_once(self, timeout_sec=0.001)

        # STOP correctly
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.tracking_rotation = False
        time.sleep(0.5)
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)

        # Get results
        
        # actual_rotation_rad = self.accumulated_rotation
        self.get_logger().info(f'final Z: {self.current_pose.orientation.z:.2f}')
        actual_rotation_rad = self.normalize_angle(self.quaternion_to_yaw(self.current_pose.orientation) - initial_yaw)
        self.get_logger().info(f'final yaw: {math.degrees(self.quaternion_to_yaw(self.current_pose.orientation)):.2f}°')
        actual_rotation_deg = abs(math.degrees(actual_rotation_rad))
        self.get_logger().info(f'present yaw: {math.degrees(self.quaternion_to_yaw(self.current_pose.orientation)):.2f}°')
        commanded_rotation_deg = 360.0
        error_deg = actual_rotation_deg - commanded_rotation_deg
        error_percent = (error_deg / commanded_rotation_deg) * 100
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('ROTATION TEST RESULTS')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Commanded rotation: {commanded_rotation_deg:.2f}°')
        self.get_logger().info(f'Actual rotation: {actual_rotation_deg:.2f}°')
        self.get_logger().info(f'Error: {error_deg:+.2f}° ({error_percent:+.2f}%)')
        
        # if abs(actual_rotation_deg) < 10:
        #     self.get_logger().error('✗ Robot did not move!')
        #     self.get_logger().error('Troubleshooting:')
        #     self.get_logger().error(f'  1. Test manually: ros2 topic pub --rate 10 {self.cmd_vel_topic} geometry_msgs/msg/Twist "{{angular: {{z: 0.5}}}}"')
        #     self.get_logger().error('  2. Check controllers: ros2 control list_controllers')
        #     self.get_logger().error('  3. Try teleop: ros2 run teleop_twist_keyboard teleop_twist_keyboard')
        #     response.success = False
        #     response.message = "Robot did not move - check controller status"
        #     return response
        
        # # Calculate corrected wheel_separation
        # corrected_separation = self.current_wheel_separation * (commanded_rotation_deg / actual_rotation_deg)
        
        # self.get_logger().info('=' * 60)
        # self.get_logger().info('CALIBRATION RECOMMENDATION')
        # self.get_logger().info('=' * 60)
        # self.get_logger().info(f'Current wheel_separation: {self.current_wheel_separation:.4f} m')
        # self.get_logger().info(f'Suggested wheel_separation: {corrected_separation:.4f} m')
        
        # change = corrected_separation - self.current_wheel_separation
        # change_percent = (change / self.current_wheel_separation) * 100
        
        # self.get_logger().info(f'Adjustment: {change:+.4f} m ({change_percent:+.2f}%)')
        
        # if abs(error_percent) < 2.0:
        #     self.get_logger().info('✓ Calibration is EXCELLENT (error < 2%)')
        # elif abs(error_percent) < 5.0:
        #     self.get_logger().info('✓ Calibration is GOOD (error < 5%)')
        # else:
        #     self.get_logger().warn('⚠ Calibration needs improvement (error > 5%)')
        
        # self.get_logger().info('=' * 60)
        
        # response.success = True
        # response.message = f"Rotation: {actual_rotation_deg:.2f}° (error: {error_percent:+.2f}%), Suggested: {corrected_separation:.4f} m"
        # return response
    


    def linear_test_callback(self, request, response):
        """Service callback for linear motion calibration test"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('Starting Linear Motion Calibration Test')
        self.get_logger().info('=' * 60)
        
        # Wait for odometry
        timeout = 5.0
        start_wait = time.time()
        while self.current_pose is None and (time.time() - start_wait) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.current_pose is None:
            response.success = False
            response.message = "No odometry data received!"
            return response
        
        # Store initial position
        initial_x = self.current_pose.position.x
        initial_y = self.current_pose.position.y
        self.linear_vel = 0.5 # m/s
        self.linear_duration = 4.0 # seconds (should move ~2m at 0.5 m/s)
        self.get_logger().info(f'Initial position: ({initial_x:.4f}, {initial_y:.4f})')
        self.get_logger().info(f'Commanding linear at {self.linear_vel} m/s for {self.linear_duration:.2f}s')
        self.get_logger().info(f'Expected distance: {self.linear_vel * self.linear_duration:.2f} m')
        
        # Set linear command and enable publishing
        # self.current_command = Twist()
        # self.current_command.linear.x = self.linear_vel
        # self.publishing_enabled = True
        
        # Wait for duration
        # test_start_time = time.time()
        # while (time.time() - test_start_time) < self.linear_duration:
        #     self.linear_speed = self.linear_vel
        #     self.angular_speed = 0.0
        #     rclpy.spin_once(self, timeout_sec=0.02)
        
        # # Stop
        # self.get_logger().info('Stopping robot...')
        # self.linear_speed = 0.0
        # self.angular_speed = 0.0  
        # time.sleep(0.8)  
        # # self.stop_robot()
        # time.sleep(1.5)
        # for _ in range(20):
        #     rclpy.spin_once(self, timeout_sec=0.5)
        test_start_time = self.get_clock().now()

        while (self.get_clock().now() - test_start_time).nanoseconds < self.linear_duration * 1e9:
            self.linear_speed = self.linear_vel
            self.angular_speed = 0.0
            rclpy.spin_once(self, timeout_sec=0.01)

        # Stop robot cleanly
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        # allow odom update
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
        # Calculate distance
        final_x = self.current_pose.position.x
        final_y = self.current_pose.position.y
        
        actual_distance = math.sqrt((final_x - initial_x)**2 + (final_y - initial_y)**2)
        commanded_distance = self.linear_vel * self.linear_duration
        error = actual_distance - commanded_distance
        error_percent = (error / commanded_distance) * 100
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('LINEAR MOTION TEST RESULTS')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Final position: ({final_x:.4f}, {final_y:.4f})')
        self.get_logger().info(f'Commanded: {commanded_distance:.4f} m')
        self.get_logger().info(f'Actual: {actual_distance:.4f} m')
        self.get_logger().info(f'Error: {error:+.4f} m ({error_percent:+.2f}%)')
        
        # if abs(actual_distance) < 0.01:
        #     self.get_logger().error('✗ Robot did not move!')
        #     response.success = False
        #     response.message = "Robot did not move"
        #     return response
        
        # corrected_radius = self.current_wheel_radius * (commanded_distance / actual_distance)
        
        # self.get_logger().info('=' * 60)
        # self.get_logger().info('CALIBRATION RECOMMENDATION')
        # self.get_logger().info('=' * 60)
        # self.get_logger().info(f'Current wheel_radius: {self.current_wheel_radius:.4f} m')
        # self.get_logger().info(f'Suggested wheel_radius: {corrected_radius:.4f} m')
        
        # change = corrected_radius - self.current_wheel_radius
        # change_percent = (change / self.current_wheel_radius) * 100
        # self.get_logger().info(f'Adjustment: {change:+.4f} m ({change_percent:+.2f}%)')
        
        # if abs(error_percent) < 2.0:
        #     self.get_logger().info('✓ EXCELLENT')
        # elif abs(error_percent) < 5.0:
        #     self.get_logger().info('✓ GOOD')
        # else:
        #     self.get_logger().warn('⚠ Needs improvement')
        
        # self.get_logger().info('=' * 60)
        
        # response.success = True
        # response.message = f"Distance: {actual_distance:.4f} m, Suggested: {corrected_radius:.4f} m"
        # return response











    # def move_robot_with_keyboard(self):
    #     try:
    #         while rclpy.ok():
    #             key = self.get_key()
    #             if key == 'w':
    #                 self.linear_speed = 0.5 # Move forward at 0.5 m/s
    #                 self.angular_speed = 0.0
    #                 self.get_logger().info('Moving forward!') # Added log
    #             elif key == 's':
    #                 self.linear_speed = -0.5 # Move backward at 0.5 m/s
    #                 self.angular_speed = 0.0
    #                 self.get_logger().info('Moving backward!') # Added log
    #             elif key == 'a':
    #                 self.linear_speed = 0.0
    #                 self.angular_speed = 1.0 # Turn left at 0.5 rad/s
    #                 self.get_logger().info('Turning left!') # Added log
    #             elif key == 'd':
    #                 self.linear_speed = 0.0
    #                 self.angular_speed = -1.0 # Turn right at 0.5 rad/s
    #                 self.get_logger().info('Turning right!') # Added log
    #             elif key == ' ': # Spacebar to stop
    #                 self.linear_speed = 0.0
    #                 self.angular_speed = 0.0
    #                 self.get_logger().info('Stopping!') # Added log
    #             elif key == 'q':
    #                 self.get_logger().info('Quitting robot controller.')
    #                 break
    #             else:
    #                 # If no key pressed or an unrecognized key, maintain current speed (or stop)
    #                 pass # Or set to 0.0 to stop if no key pressed: self.linear_speed = 0.0; self.angular_speed = 0.0

    #             rclpy.spin_once(self) # Process ROS callbacks
                
    #     except Exception as e:
    #         self.get_logger().error(f'Error in keyboard control: {e}')
    #     finally:
    #         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings) # Restore terminal settings
    #         self.destroy_node()
    #         rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    robot_calibration = RobotCalibration()
    robot_calibration.rotation_test_callback(None, None) # Directly call the test callback for demonstration

if __name__ == '__main__':
    main()
    rclpy.shutdown()
