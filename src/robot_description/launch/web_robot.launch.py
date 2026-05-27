import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    pkg_share_dir = get_package_share_directory('robot_description')
    
    # Declare the 'model' argument, which will point to your URDF file
    # For now, we'll point to a placeholder. You'll replace this with your actual URDF later.
    default_model_path = os.path.join(pkg_share_dir, 'urdf', 'web_robot.urdf.xacro') # Example URDF from urdf_tutorial if available
    
    # Ensure your URDF is located in robot_description/urdf/your_robot.urdf
    # If you don't have an URDF yet, copy '01-myfirst.urdf' from urdf_tutorial to robot_description/urdf/
    # For a placeholder:
    # default_model_path = os.path.join(pkg_share_dir, 'urdf', 'your_robot.urdf') # <-- Replace with your robot's URDF

    # If you want to use the urdf_tutorial's 01-myfirst.urdf directly, you would do this:
    # urdf_tutorial_share_dir = get_package_share_directory('urdf_tutorial')
    # default_model_path = os.path.join(urdf_tutorial_share_dir, 'urdf', '01-myfirst.urdf')

    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path,
        description='Path to robot URDF/Xacro file'
    )
    # Robot State Publisher Node (publishes robot's TF transforms)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    # Joint State Publisher GUI Node (for manual joint control in RViz2)
    # This is helpful for debugging your URDF
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz2 Node (to visualize the robot)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # Set a default RViz configuration file if you have one
        # arguments=['-d', os.path.join(pkg_share_dir, 'rviz', 'your_config.rviz')]
    )

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     name='model', 
        #     default_value=default_model_path,
        #     description='Path to robot URDF/Xacro file'
        # ),
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])