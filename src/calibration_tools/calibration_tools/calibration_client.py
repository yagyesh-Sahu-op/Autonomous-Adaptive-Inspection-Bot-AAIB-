#!/usr/bin/env python3

"""
Client script to trigger odometry calibration tests
Usage:
    python3 calibration_client.py rotation  # Test rotation
    python3 calibration_client.py linear    # Test linear motion
    python3 calibration_client.py both      # Test both
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys


class CalibrationClient(Node):
    def __init__(self):
        super().__init__('calibration_client')
        
        # Create service clients
        self.rotation_client = self.create_client(Trigger, 'calibrate_rotation')
        self.linear_client = self.create_client(Trigger, 'calibrate_linear')
        
    def call_rotation_test(self):
        """Call rotation calibration service"""
        self.get_logger().info('Waiting for rotation calibration service...')
        
        if not self.rotation_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Rotation calibration service not available!')
            return False
        
        request = Trigger.Request()
        future = self.rotation_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✓ {response.message}')
            else:
                self.get_logger().error(f'✗ {response.message}')
            return response.success
        else:
            self.get_logger().error('Service call failed!')
            return False
    
    def call_linear_test(self):
        """Call linear motion calibration service"""
        self.get_logger().info('Waiting for linear calibration service...')
        
        if not self.linear_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Linear calibration service not available!')
            return False
        
        request = Trigger.Request()
        future = self.linear_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✓ {response.message}')
            else:
                self.get_logger().error(f'✗ {response.message}')
            return response.success
        else:
            self.get_logger().error('Service call failed!')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: python3 calibration_client.py [rotation|linear|both]")
        return
    
    test_type = sys.argv[1].lower()
    
    client = CalibrationClient()
    
    try:
        if test_type == 'rotation':
            print("\n" + "="*60)
            print("ROTATION CALIBRATION TEST")
            print("="*60)
            print("This will rotate the robot 360° and measure accuracy.")
            print("Make sure the robot has clear space to rotate.")
            input("Press Enter to start...")
            client.call_rotation_test()
            
        elif test_type == 'linear':
            print("\n" + "="*60)
            print("LINEAR MOTION CALIBRATION TEST")
            print("="*60)
            print("This will drive the robot 1m straight and measure accuracy.")
            print("Make sure the robot has clear space ahead.")
            input("Press Enter to start...")
            client.call_linear_test()
            
        elif test_type == 'both':
            print("\n" + "="*60)
            print("FULL CALIBRATION TEST")
            print("="*60)
            print("This will run both rotation and linear tests.")
            print()
            
            # Rotation test
            print("Step 1: Rotation Test")
            print("Make sure the robot has clear space to rotate.")
            input("Press Enter to start rotation test...")
            success1 = client.call_rotation_test()
            
            print("\n")
            input("Press Enter to continue to linear test...")
            
            # Linear test
            print("\nStep 2: Linear Motion Test")
            print("Make sure the robot has clear space ahead.")
            input("Press Enter to start linear test...")
            success2 = client.call_linear_test()
            
            print("\n" + "="*60)
            print("CALIBRATION COMPLETE")
            print("="*60)
            if success1 and success2:
                print("✓ Both tests completed successfully!")
                print("Check the calibration node output for recommendations.")
            else:
                print("✗ Some tests failed. Check the logs.")
            print("="*60)
            
        else:
            print(f"Unknown test type: {test_type}")
            print("Valid options: rotation, linear, both")
    
    except KeyboardInterrupt:
        print("\nCalibration cancelled.")
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
