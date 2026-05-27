#!/usr/bin/env python3

"""
Odometry Calibration Node for Differential Drive Robots
This node helps calibrate wheel_radius and wheel_separation parameters
by comparing commanded movements with actual odometry feedback.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
import math
import time
from enum import Enum


class CalibrationState(Enum):
    IDLE = 0
    ROTATION_TEST = 1
    STRAIGHT_LINE_TEST = 2
    COLLECTING_DATA = 3


class OdometryCalibrationNode(Node):
    def __init__(self):
        super().__init__('odometry_calibration_node')
        
        # Declare parameters
        self.declare_parameter('cmd_vel_topic', '/diff_drive_controller/cmd_vel_unstamped')
        self.declare_parameter('odom_topic', '/diff_drive_controller/odom')
        self.declare_parameter('current_wheel_radius', 0.0316)
        self.declare_parameter('current_wheel_separation', 0.202)
        self.declare_parameter('rotation_angular_velocity', 1.0)  # rad/s
        self.declare_parameter('linear_velocity', 0.2)  # m/s
        self.declare_parameter('rotation_duration', 6.28)  # seconds for 360°
        self.declare_parameter('linear_duration', 5.0)  # seconds for 1m
        
        # Get parameters
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.current_wheel_radius = self.get_parameter('current_wheel_radius').value
        self.current_wheel_separation = self.get_parameter('current_wheel_separation').value
        self.rotation_angular_vel = self.get_parameter('rotation_angular_velocity').value
        self.linear_vel = self.get_parameter('linear_velocity').value
        self.rotation_duration = self.get_parameter('rotation_duration').value
        self.linear_duration = self.get_parameter('linear_duration').value
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        
        # Services
        self.rotation_test_srv = self.create_service(
            Trigger,
            'calibrate_rotation',
            self.rotation_test_callback
        )
        self.linear_test_srv = self.create_service(
            Trigger,
            'calibrate_linear',
            self.linear_test_callback
        )
        
        # State variables
        self.state = CalibrationState.IDLE
        self.initial_pose = None
        self.current_pose = None
        self.test_start_time = None
        self.test_duration = 0.0
        
        # Results
        self.rotation_results = {}
        self.linear_results = {}
        
        self.get_logger().info('Odometry Calibration Node Started')
        self.get_logger().info(f'Current parameters:')
        self.get_logger().info(f'  wheel_radius: {self.current_wheel_radius:.4f} m')
        self.get_logger().info(f'  wheel_separation: {self.current_wheel_separation:.4f} m')
        self.get_logger().info('Services available:')
        self.get_logger().info('  - /calibrate_rotation')
        self.get_logger().info('  - /calibrate_linear')

    def odom_callback(self, msg):
        """Store current odometry data"""
        self.current_pose = msg.pose.pose
        
    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle"""
        # Extract yaw from quaternion
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
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)
    
    def rotation_test_callback(self, request, response):
        """Service callback for rotation calibration test"""
        if self.state != CalibrationState.IDLE:
            response.success = False
            response.message = "Calibration already in progress!"
            return response
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Starting Rotation Calibration Test')
        self.get_logger().info('=' * 60)
        
        # Wait for initial odometry
        timeout = 5.0
        start_wait = time.time()
        while self.current_pose is None and (time.time() - start_wait) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.current_pose is None:
            response.success = False
            response.message = "No odometry data received!"
            return response
        
        # Store initial position
        self.initial_pose = self.current_pose
        initial_yaw = self.quaternion_to_yaw(self.initial_pose.orientation)
        
        self.get_logger().info(f'Initial yaw: {math.degrees(initial_yaw):.2f}°')
        self.get_logger().info(f'Commanding rotation at {self.rotation_angular_vel} rad/s for {self.rotation_duration:.2f}s')
        self.get_logger().info(f'Expected rotation: 360°')
        
        # Send rotation command
        rotation_msg = Twist()
        rotation_msg.angular.z = self.rotation_angular_vel
        
        self.state = CalibrationState.ROTATION_TEST
        self.test_start_time = time.time()
        
        # Rotate for specified duration
        while (time.time() - self.test_start_time) < self.rotation_duration:
            self.cmd_vel_pub.publish(rotation_msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # Stop robot
        self.stop_robot()
        time.sleep(6.28)  # Let robot settle
        rclpy.spin_once(self, timeout_sec=0.1)
        
        # Calculate actual rotation
        final_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        actual_rotation_rad = self.normalize_angle(final_yaw - initial_yaw)
        actual_rotation_deg = math.degrees(actual_rotation_rad)
        
        # Handle wrap-around for full rotations
        if abs(actual_rotation_rad) < math.pi:
            # Likely completed more than 180° but wrapped around
            if actual_rotation_rad < 0:
                actual_rotation_deg = 360 + actual_rotation_deg
        
        commanded_rotation_deg = 360.0
        error_deg = actual_rotation_deg - commanded_rotation_deg
        error_percent = (error_deg / commanded_rotation_deg) * 100
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('ROTATION TEST RESULTS')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Commanded rotation: {commanded_rotation_deg:.2f}°')
        self.get_logger().info(f'Actual rotation: {actual_rotation_deg:.2f}°')
        self.get_logger().info(f'Error: {error_deg:.2f}° ({error_percent:.2f}%)')
        
        # Calculate corrected wheel_separation
        if abs(actual_rotation_deg) > 10:  # Avoid division by very small numbers
            corrected_separation = self.current_wheel_separation * (commanded_rotation_deg / actual_rotation_deg)
            self.get_logger().info('=' * 60)
            self.get_logger().info('CALIBRATION RECOMMENDATION')
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'Current wheel_separation: {self.current_wheel_separation:.4f} m')
            self.get_logger().info(f'Suggested wheel_separation: {corrected_separation:.4f} m')
            self.get_logger().info(f'Adjustment: {(corrected_separation - self.current_wheel_separation):.4f} m')
            self.get_logger().info('=' * 60)
            
            self.rotation_results = {
                'commanded_deg': commanded_rotation_deg,
                'actual_deg': actual_rotation_deg,
                'error_deg': error_deg,
                'error_percent': error_percent,
                'current_separation': self.current_wheel_separation,
                'suggested_separation': corrected_separation
            }
            
            response.success = True
            response.message = f"Rotation test complete. Actual: {actual_rotation_deg:.2f}°, Suggested separation: {corrected_separation:.4f} m"
        else:
            self.get_logger().warn('Rotation too small to calculate correction!')
            response.success = False
            response.message = "Rotation too small to calibrate"
        
        self.state = CalibrationState.IDLE
        return response
    
    def linear_test_callback(self, request, response):
        """Service callback for linear motion calibration test"""
        if self.state != CalibrationState.IDLE:
            response.success = False
            response.message = "Calibration already in progress!"
            return response
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Starting Linear Motion Calibration Test')
        self.get_logger().info('=' * 60)
        
        # Wait for initial odometry
        timeout = 5.0
        start_wait = time.time()
        while self.current_pose is None and (time.time() - start_wait) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.current_pose is None:
            response.success = False
            response.message = "No odometry data received!"
            return response
        
        # Store initial position
        self.initial_pose = self.current_pose
        initial_x = self.initial_pose.position.x
        initial_y = self.initial_pose.position.y
        
        self.get_logger().info(f'Initial position: ({initial_x:.4f}, {initial_y:.4f})')
        self.get_logger().info(f'Commanding linear motion at {self.linear_vel} m/s for {self.linear_duration:.2f}s')
        self.get_logger().info(f'Expected distance: {self.linear_vel * self.linear_duration:.2f} m')
        
        # Send linear command
        linear_msg = Twist()
        linear_msg.linear.x = self.linear_vel
        
        self.state = CalibrationState.STRAIGHT_LINE_TEST
        self.test_start_time = time.time()
        
        # Drive for specified duration
        while (time.time() - self.test_start_time) < self.linear_duration:
            self.cmd_vel_pub.publish(linear_msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # Stop robot
        self.stop_robot()
        time.sleep(5.0)  # Let robot settle
        rclpy.spin_once(self, timeout_sec=0.1)
        
        # Calculate actual distance
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
        self.get_logger().info(f'Commanded distance: {commanded_distance:.4f} m')
        self.get_logger().info(f'Actual distance: {actual_distance:.4f} m')
        self.get_logger().info(f'Error: {error:.4f} m ({error_percent:.2f}%)')
        
        # Calculate corrected wheel_radius
        if abs(actual_distance) > 0.01:  # Avoid division by very small numbers
            corrected_radius = self.current_wheel_radius * (commanded_distance / actual_distance)
            self.get_logger().info('=' * 60)
            self.get_logger().info('CALIBRATION RECOMMENDATION')
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'Current wheel_radius: {self.current_wheel_radius:.4f} m')
            self.get_logger().info(f'Suggested wheel_radius: {corrected_radius:.4f} m')
            self.get_logger().info(f'Adjustment: {(corrected_radius - self.current_wheel_radius):.4f} m')
            self.get_logger().info('=' * 60)
            
            self.linear_results = {
                'commanded_m': commanded_distance,
                'actual_m': actual_distance,
                'error_m': error,
                'error_percent': error_percent,
                'current_radius': self.current_wheel_radius,
                'suggested_radius': corrected_radius
            }
            
            response.success = True
            response.message = f"Linear test complete. Actual: {actual_distance:.4f} m, Suggested radius: {corrected_radius:.4f} m"
        else:
            self.get_logger().warn('Distance too small to calculate correction!')
            response.success = False
            response.message = "Distance too small to calibrate"
        
        self.state = CalibrationState.IDLE
        return response


def main(args=None):
    rclpy.init(args=args)
    
    calibration_node = OdometryCalibrationNode()
    
    try:
        rclpy.spin(calibration_node)
    except KeyboardInterrupt:
        pass
    finally:
        calibration_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
