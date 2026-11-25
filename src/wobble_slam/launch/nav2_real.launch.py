"""Launch Nav2 localization and navigation on the physical Wobble robot."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the real-robot Nav2 launch description."""
    slam_dir = get_package_share_directory('wobble_slam')
    description_dir = get_package_share_directory('wobble_description')
    control_dir = get_package_share_directory('wobble_control')
    ekf_dir = get_package_share_directory('wobble_ekf')
    imu_dir = get_package_share_directory('wobble_imu')
    odom_dir = get_package_share_directory('wobble_odometry')
    sensor_dir = get_package_share_directory('wobble_sensor')

    map_file = LaunchConfiguration('map')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    use_rviz = LaunchConfiguration('use_rviz')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    robot_description = ParameterValue(
        Command([
            'xacro ',
            os.path.join(description_dir, 'urdf', 'wobble.urdf.xacro'),
        ]),
        value_type=str,
    )
    nav2_params = os.path.join(slam_dir, 'config', 'nav2_real_params.yaml')

    hardware_nodes = [
        Node(
            package='wobble_control',
            executable='motor_node',
            name='motor_node',
            parameters=[os.path.join(control_dir, 'config', 'motor.yaml')],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }],
            output='screen',
        ),
        Node(
            package='wobble_sensor',
            executable='encoder_node',
            name='encoder_node',
            parameters=[os.path.join(sensor_dir, 'config', 'encoder.yaml')],
            output='screen',
        ),
        Node(
            package='wobble_odometry',
            executable='odom_node',
            name='odom_node',
            parameters=[os.path.join(odom_dir, 'config', 'odometry.yaml')],
            output='screen',
        ),
        Node(
            package='wobble_imu',
            executable='imu_node',
            name='imu_node',
            parameters=[os.path.join(imu_dir, 'config', 'imu.yaml')],
            output='screen',
        ),
        Node(
            package='wobble_ekf',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[os.path.join(ekf_dir, 'config', 'ekf.yaml')],
            output='screen',
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[{
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'auto_standby': False,
            }],
            output='screen',
        ),
    ]

    localization_nodes = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[nav2_params, {'yaml_filename': map_file}],
            output='screen',
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[
                nav2_params,
                {
                    'set_initial_pose': True,
                    'initial_pose.x': ParameterValue(
                        initial_pose_x,
                        value_type=float,
                    ),
                    'initial_pose.y': ParameterValue(
                        initial_pose_y,
                        value_type=float,
                    ),
                    'initial_pose.yaw': ParameterValue(
                        initial_pose_yaw,
                        value_type=float,
                    ),
                },
            ],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
            output='screen',
        ),
    ]

    navigation_nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[nav2_params],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
            output='screen',
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            parameters=[nav2_params],
            output='screen',
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav2_params],
            output='screen',
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[nav2_params],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
            output='screen',
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[nav2_params],
            output='screen',
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            parameters=[nav2_params],
            output='screen',
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            parameters=[nav2_params],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel_smoothed'),
            ],
            output='screen',
        ),
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            parameters=[nav2_params],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'velocity_smoother',
                    'collision_monitor',
                    'bt_navigator',
                    'waypoint_follower',
                ],
            }],
            output='screen',
        ),
    ]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(slam_dir, 'config', 'nav2.rviz')],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                os.path.expanduser('~'),
                'wobble_bot',
                'maps',
                'real_room_map.yaml',
            ),
            description='Path to the saved real-robot occupancy map',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='RPLiDAR serial port',
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='115200',
            description='RPLiDAR A1 serial baudrate',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description=(
                'Launch RViz on this computer (use false for a headless robot)'
            ),
        ),
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='0.0',
            description='Robot initial x position in the saved map frame',
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='0.0',
            description='Robot initial y position in the saved map frame',
        ),
        DeclareLaunchArgument(
            'initial_pose_yaw',
            default_value='0.0',
            description=(
                'Robot initial heading in the saved map frame, in radians'
            ),
        ),
        *hardware_nodes,
        *localization_nodes,
        TimerAction(period=8.0, actions=navigation_nodes),
        rviz_node,
    ])
