import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class RotateRobot(Node):
    def __init__(self):
        super().__init__('rotate_robot_node')
        
        # 1. Use your specific controlling topic
        self.publisher_ = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        
        # 2. Subscribe to filtered odometry for better accuracy
        self.subscription = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        self.target_angle = math.radians(90.0) # Convert degrees to radians
        self.current_yaw = 0.0
        self.start_yaw = None
        self.reached = False

    def get_yaw_from_quaternion(self, q):
        # Helper to convert quaternion to Euler Yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        self.current_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        
        if self.start_yaw is None:
            self.start_yaw = self.current_yaw
            return

        # Calculate how much we have rotated
        rotated_so_far = abs(self.current_yaw - self.start_yaw)
        
        # Handle wrap-around (jumping from PI to -PI)
        if rotated_so_far > math.pi:
            rotated_so_far = abs(rotated_so_far - (2 * math.pi))

        error = self.target_angle - rotated_so_far
        msg_vel = Twist()

        if error > 0.01 and not self.reached:
            # Proportional speed: slows down as it gets closer
            msg_vel.angular.z = max(0.7, error * 0.5) 
            self.get_logger().info(f'Rotating... Error: {math.degrees(error):.2f}°')
        else:
            msg_vel.angular.z = 0.0
            if not self.reached:
                self.get_logger().info('Target Reached!')
                self.reached = True
        
        self.publisher_.publish(msg_vel)

def main(args=None):
    rclpy.init(args=args)
    node = RotateRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()