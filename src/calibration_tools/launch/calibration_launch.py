from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch the odometry calibration node"""
    
    # Declare launch arguments
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='cmd_vel_teleop',
        description='Topic to publish velocity commands'
    )
    
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/diff_drive_controller/odom',
        description='Topic to subscribe to odometry'
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        'current_wheel_radius',
        default_value='0.0316',
        description='Current wheel radius in meters'
    )
    
    wheel_separation_arg = DeclareLaunchArgument(
        'current_wheel_separation',
        default_value='0.202',
        description='Current wheel separation in meters'
    )
    
    # Calibration node
    calibration_node = Node(
        package='calibration_tools',  # Change to your package name
        executable='odometry_calibration_node',
        name='odometry_calibration_node',
        output='screen',
        parameters=[{
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'current_wheel_radius': LaunchConfiguration('current_wheel_radius'),
            'current_wheel_separation': LaunchConfiguration('current_wheel_separation'),
            'rotation_angular_velocity': 1.0,  # rad/s
            'linear_velocity': 0.2,  # m/s
            'rotation_duration': 6.28,  # seconds (2π for 360°)
            'linear_duration': 5.0,  # seconds (1m at 0.2 m/s)
        }]
    )
    
    return LaunchDescription([
        cmd_vel_topic_arg,
        odom_topic_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        calibration_node,
    ])
