import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_dir = get_package_share_directory('wobble_slam')
    description_dir = get_package_share_directory('wobble_description')
    control_dir = get_package_share_directory('wobble_control')
    ekf_dir = get_package_share_directory('wobble_ekf')
    imu_dir = get_package_share_directory('wobble_imu')
    odom_dir = get_package_share_directory('wobble_odometry')
    sensor_dir = get_package_share_directory('wobble_sensor')

    sim = LaunchConfiguration('sim')
    use_rviz = LaunchConfiguration('use_rviz')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    auto_standby = LaunchConfiguration('auto_standby')

    urdf_xacro = os.path.join(description_dir, 'urdf', 'wobble.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_xacro]),
        value_type=str
    )
    motor_config = os.path.join(control_dir, 'config', 'motor.yaml')
    ekf_config = os.path.join(ekf_dir, 'config', 'ekf.yaml')
    imu_config = os.path.join(imu_dir, 'config', 'imu.yaml')
    odom_config = os.path.join(odom_dir, 'config', 'odometry.yaml')
    encoder_config = os.path.join(sensor_dir, 'config', 'encoder.yaml')
    slam_config = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'slam.rviz')

    # Simulation include (Ignition Gazebo + Bridge)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, 'launch', 'sim.launch.py')
        ),
        launch_arguments={'use_rviz': 'false'}.items(),
        condition=IfCondition(sim)
    )

    # Real Hardware Nodes (only active when sim is false)
    motor_node = Node(
        package='wobble_control',
        executable='motor_node',
        name='motor_node',
        parameters=[motor_config],
        output='screen',
        condition=UnlessCondition(sim)
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        output='screen',
        condition=UnlessCondition(sim)
    )

    encoder_node = Node(
        package='wobble_sensor',
        executable='encoder_node',
        name='encoder_node',
        parameters=[encoder_config],
        output='screen',
        condition=UnlessCondition(sim)
    )

    odom_node = Node(
        package='wobble_odometry',
        executable='odom_node',
        name='odom_node',
        parameters=[odom_config],
        output='screen',
        condition=UnlessCondition(sim)
    )

    imu_node = Node(
        package='wobble_imu',
        executable='imu_node',
        name='imu_node',
        parameters=[imu_config],
        output='screen',
        condition=UnlessCondition(sim)
    )

    ekf_node = Node(
        package='wobble_ekf',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config],
        output='screen',
        condition=UnlessCondition(sim)
    )

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'auto_standby': auto_standby,
        }],
        condition=UnlessCondition(sim)
    )

    # SLAM Toolbox Lifecycle Node
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': sim}
        ]
    )

    slam_startup_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                )
            ]
        )
    )

    slam_activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )

    # RViz2 Visualization Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': sim}],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Launch Gazebo simulation (true) or real robot hardware (false)'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 for SLAM visualization'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='RPLiDAR serial port (e.g. /dev/ttyUSB0)'
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='115200',
            description='RPLiDAR serial baudrate (115200 for A1/A2, 256000 for A3/S1)'
        ),
        DeclareLaunchArgument(
            'auto_standby',
            default_value='false',
            description='Only spin and scan when actively subscribed; stop motor when idle'
        ),

        # Simulation
        sim_launch,

        # Real hardware nodes
        motor_node,
        robot_state_publisher_node,
        encoder_node,
        odom_node,
        imu_node,
        ekf_node,
        rplidar_node,

        # SLAM
        slam_startup_handler,
        slam_activate_handler,
        slam_node,

        # Visualization
        rviz_node,
    ])
