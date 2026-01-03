from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,

                # Frames
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',

                # Laser
                'scan_topic': '/scan_laser',
                'minimum_range': 0.12,

                # SLAM mode
                'mode': 'mapping',

                # Performance / stability
                'transform_publish_period': 0.05,
                'tf_buffer_duration': 30.0,

                # Map settings
                'resolution': 0.05,
                'max_laser_range': 10.0,

                # Publishing
                'publish_tf': True
            }]
        )
    ])
