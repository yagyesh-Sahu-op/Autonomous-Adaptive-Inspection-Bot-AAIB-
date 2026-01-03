import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('robot_description')
    gazebo_ros_share_dir = get_package_share_directory('gazebo_ros')

    # Path to your URDF file
    urdf_file_name = 'web_robot.urdf' # Make sure this matches your file name
    default_model_path = os.path.join(pkg_share_dir, 'urdf', urdf_file_name)

    # Declare the 'model' argument
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Path to robot URDF/Xacro file'
    )

    # Start Gazebo with the empty world and ROS 2 communication plugins
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share_dir, 'launch', 'gazebo.launch.py')
        ),
        # Optional: You can specify a different world here if you create one
        # launch_arguments={'world': os.path.join(pkg_share_dir, 'worlds', 'your_world.world')}.items()
        # launch_arguments={'gui': 'false'}.items() # For running Gazebo headless (no GUI)
    )

    # Robot State Publisher Node (publishes robot's TF transforms)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}],
        output='screen'
    )

    # THIS IS THE NODE THAT USES THE BUILT-IN spawn_entity.py SCRIPT
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', # This topic contains your URDF
                   '-entity', 'web_robot',         # Name your robot in Gazebo
                   '-x', '0.0', '-y', '0.0', '-z', '0.0'], # Initial spawn position
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity, # Include the spawn_entity node in your launch description
    ])
