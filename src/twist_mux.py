import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelMux(Node):

    def __init__(self):
        super().__init__('twist_mux')

        self.pub = self.create_publisher(Twist, 'diff_drive_controller/cmd_vel_unstamped', 10)

        self.sub_teleop = self.create_subscription(
            Twist,
            '/cmd_vel_teleop',
            self.teleop_cb,
            10
        )

        self.sub_auto = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.auto_cb,
            10
        )

        self.last_source = None

    def teleop_cb(self, msg):
        self.last_source = 'teleop'
        self.pub.publish(msg)

    def auto_cb(self, msg):
        self.last_source = 'auto'
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CmdVelMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
