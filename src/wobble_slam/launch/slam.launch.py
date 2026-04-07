import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState, matches_node_name
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_dir = get_package_share_directory('wobble_slam')
    ekf_dir = get_package_share_directory('wobble_ekf')
    imu_dir = get_package_share_directory('wobble_imu')
    odom_dir = get_package_share_directory('wobble_odometry')
    sensor_dir = get_package_share_directory('wobble_sensor')

    urdf_xacro = os.path.join(pkg_dir, 'urdf', 'wobble.urdf.xacro')
    robot_description = Command(['xacro ', urdf_xacro])
    ekf_config = os.path.join(ekf_dir, 'config', 'ekf.yaml')
    imu_config = os.path.join(imu_dir, 'config', 'imu.yaml')
    odom_config = os.path.join(odom_dir, 'config', 'odometry.yaml')
    encoder_config = os.path.join(sensor_dir, 'config', 'encoder.yaml')
    slam_config = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')

    # lidar
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_a1_launch.py'
            )
        ),
        launch_arguments={
            'frame_id': 'laser',
        }.items()
    )

    # slam node
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_config]
    )

    # Auto-configure and activate slam_toolbox lifecycle with sequencing.
    # A short delay avoids racing the lifecycle service before the node is ready.
    slam_startup_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_node_name('/slam_toolbox'),
                                transition_id=Transition.TRANSITION_CONFIGURE,
                            )
                        )
                    ]
                )
            ]
        )
    )

    slam_activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_node_name('/slam_toolbox'),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )

    ld_list = [
        # motor control (MUST be first for cmd_vel)
        Node(
            package='wobble_control',
            executable='motor_node',
            name='motor_node',
            output='screen'
        ),

        # robot model
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }],
            output='screen'
        ),

        # encoder - RE-ENABLED: provides encoder tick counts to odometry node
        Node(
            package='wobble_sensor',
            executable='encoder_node',
            name='encoder_node',
            parameters=[encoder_config],
            output='screen'
        ),

        # odometry - RE-ENABLED: provides odom->base_link transform (required for SLAM)
        # Without this, transform chain is broken: map->odom but no odom->base_link
        Node(
            package='wobble_odometry',
            executable='odom_node',
            name='odom_node',
            parameters=[odom_config],
            output='screen'
        ),

        # imu
        Node(
            package='wobble_imu',
            executable='imu_node',
            name='imu_node',
            parameters=[imu_config],
            output='screen'
        ),

        # fused odometry: wheel position + IMU heading, publishes odom->base_link
        Node(
            package='wobble_ekf',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config],
            output='screen'
        ),


        # lidar
        rplidar_launch,

        # slam with lifecycle management
        slam_node,
        slam_startup_handler,
        slam_activate_handler,
    ]

    return LaunchDescription(ld_list)
