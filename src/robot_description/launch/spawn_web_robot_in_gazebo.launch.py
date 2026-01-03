import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('robot_description')
    gazebo_ros_share_dir = get_package_share_directory('gazebo_ros')
    world_path = os.path.expanduser(
        # '~/web_robot_ws/worlds/oil_factory.world'
        '~/web_robot_ws/worlds/gazebo_models_worlds_collection/worlds/powerplant.world'
    )
    # Path to URDF
    urdf_file_name = 'web_robot.urdf.xacro'
    default_model_path = os.path.join(pkg_share_dir, 'urdf', urdf_file_name)

    # Path to controller config
    controllers_file = os.path.join(pkg_share_dir, 'config', 'web_robot_controllers.yaml')

    # Declare the 'model' argument
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Path to robot URDF/Xacro file'
    )

   # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    ) 

    # Robot description (xacro → URDF XML string)
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # Start Gazebo (empty world by default)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'true'
        }.items(),
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Spawn the robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', 'web_robot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # Controller Manager (ros2_control_node)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description}, 
                    controllers_file],
        output="screen"
    )
 
    # Load joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # Load diff_drive_controller
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller",
                    "--controller-manager", "/controller_manager"],
        output="screen"
    )
    # controller_manager_timeout = 50
    # ros2_control_params = controllers_file
    # diff_drive_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=["diff_drive_controller",
    #                "--param-file", ros2_control_params],
    #     output="screen"
    # )

    # Add delays to avoid race conditions
    delay_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delay_diff_drive_controller = TimerAction(
        period=8.0,
        actions=[diff_drive_controller_spawner]
    )
    # Delay diff_drive until joint_state_broadcaster is up
    # delay_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_start=[diff_drive_controller_spawner],
    #     )
    # )
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share_dir, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', LaunchConfiguration('rvizconfig')],
        # arguments=['-d', os.path.join(pkg_share_dir, 'rviz', 'config.rviz')]
    )
    twist_mux = Node(
        package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[os.path.join(pkg_share_dir,'config/twist_mux.yaml')],
            output='screen'
    )
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',

        # Topic remappings
        remappings=[
            ('cloud_in', '/scan'),
            ('scan', '/scan_laser'),
        ],

        # Parameters
        parameters=[{
            'target_frame': 'laser_link',
            'min_height': -0.1,
            'max_height': 0.1,
            'angle_min': -3.14,
            'angle_max': 3.14,
            'angle_increment': 0.01,
            'range_min': 0.1,
            'range_max': 10.0,
        }]
    )
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_description'),
                'launch',
                'slam.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    return LaunchDescription([
        declare_use_sim_time_cmd,
        model_arg,
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity,
        # ros2_control_node,
        delay_joint_state_broadcaster,
        # diff_drive_controller_spawner,
        delay_diff_drive_controller,
        # delay_diff_drive_spawner,
        robot_localization_node,
        pointcloud_to_laserscan_node,
        twist_mux,
        slam_launch
        # rviz_node
       
    ])
