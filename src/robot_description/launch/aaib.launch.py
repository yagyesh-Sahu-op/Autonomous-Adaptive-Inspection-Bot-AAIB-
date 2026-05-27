import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                             TimerAction, GroupAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share_dir = get_package_share_directory('robot_description')
    gazebo_ros_share_dir = get_package_share_directory('gazebo_ros')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    world_path = os.path.expanduser(
        '~/web_robot_ws/worlds/campus_navigation.world'
    )

    urdf_file_name = 'web_robot.urdf.xacro'
    default_model_path = os.path.join(pkg_share_dir, 'urdf', urdf_file_name)
    
    nav2_params_file = os.path.join(pkg_share_dir, 'config', 'nav2_params.yaml')
    map_file = '/home/yagyesh/web_robot_ws/src/robot_description/map/map1.yaml'

    # ── Declare arguments ──────────────────────────────────────────────────────
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Path to robot URDF/Xacro file'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # ── Gazebo ─────────────────────────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'true'
        }.items(),
    )

    # ── Robot State Publisher ──────────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    # ── Spawn robot in Gazebo ──────────────────────────────────────────────────
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', 'web_robot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # ── Controllers ────────────────────────────────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    delay_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delay_diff_drive_controller = TimerAction(
        period=8.0,
        actions=[diff_drive_controller_spawner]
    )

    # ── EKF (robot_localization) ───────────────────────────────────────────────
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share_dir, 'config/ekf.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )

    # ── PointCloud → LaserScan ─────────────────────────────────────────────────
    # FIX: remaps scan_laser (lidar ray output) → cloud_in input
    # FIX: output goes to /scan_raw, then relayed as RELIABLE to /scan
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/scan_laser'),   # your lidar publishes here
            ('scan', '/scan'),         # publish to intermediate topic
        ],
        parameters=[{
            'target_frame': 'laser_link',
            'use_sim_time': True,
            'min_height': -0.1,
            'max_height': 0.1,
            'angle_min': -3.14159,
            'angle_max':  3.14159,
            'angle_increment': 0.01,
            'range_min': 0.1,
            'range_max': 10.0,
            'use_inf': True,
            'use_sim_time': use_sim_time,
        }]
    )

    # FIX: relay /scan_raw (BEST_EFFORT) → /scan (RELIABLE) so AMCL can subscribe
    # topic_tools relay copies messages and lets QoS be negotiated properly
    scan_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='scan_qos_relay',
        output='screen',
        parameters=[{
            'input_topic': '/scan_raw',
            'output_topic': '/scan',
            'use_sim_time': use_sim_time,
        }]
    )

    # ── Twist Mux ──────────────────────────────────────────────────────────────
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[os.path.join(pkg_share_dir, 'config/twist_mux.yaml')],
        output='screen',
        arguments=['--ros-args'],
        remappings=[('/cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')]
    )

    # ── Camera feed ────────────────────────────────────────────────────────────
    camera_feed = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{
            'port': 8080,
            'address': '0.0.0.0',
            'default_stream_type': 'mjpeg'
            
        }]
    )

    # ── Nav2 Localization (map_server + AMCL) ──────────────────────────────────
    # FIX: pass use_sim_time explicitly so map→odom TF uses sim clock
    localization_launch = TimerAction(
        period=15.0,   # wait for Gazebo + robot to fully spawn first
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
                ),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': 'true',          # FIX: must match Gazebo clock
                    'params_file': nav2_params_file,
                }.items()
            )
        ]
    )

    # ── Nav2 Navigation stack ──────────────────────────────────────────────────
    # FIX: delay navigation until localization has time to publish map→odom TF
    navigation_launch = TimerAction(
        period=22.0,   # give localization 5s head start after its 10s delay
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_params_file,
                }.items()
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        model_arg,

        # Stage 1: Simulation environment
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity,

        # Stage 2: Controllers (delayed to wait for Gazebo)
        delay_joint_state_broadcaster,
        delay_diff_drive_controller,

        # Stage 3: Sensors & odometry
        robot_localization_node,
        pointcloud_to_laserscan_node,
        scan_relay_node,          # FIX: QoS bridge for /scan

        # Stage 4: Navigation utilities
        twist_mux,
        camera_feed,

        # Stage 5: Nav2 (delayed to ensure robot + sensors are ready)
        localization_launch,
        navigation_launch,
    ])