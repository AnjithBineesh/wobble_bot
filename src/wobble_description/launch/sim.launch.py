import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('wobble_description')
    world_file = os.path.join(pkg_share, 'worlds', 'wobble_world.sdf')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = os.path.join(pkg_share, 'config', 'display.rviz')

    # convert xacro → urdf string (ParameterValue wrapper required in Humble)
    robot_description = ParameterValue(
        Command([
            'xacro ',
            os.path.join(pkg_share, 'urdf', 'wobble.urdf.xacro')
        ]),
        value_type=str
    )

    # start Ignition Gazebo (Fortress) with the custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    # publish TF tree from URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'publish_frequency': 50.0,
        }],
        output='screen'
    )

    # spawn robot into Ignition Gazebo just above ground (0, 0, 0.005)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'wobble',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.005'
        ],
        output='screen'
    )

    # bridge Ignition ↔ ROS2 topics (explicit directions prevent TF/topic echoes)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/world/wobble_world/model/wobble/link/laser/sensor/lidar/scan'
            '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[
            (
                '/world/wobble_world/model/wobble/link/laser/sensor/lidar/scan',
                '/scan'
            ),
        ],
        output='screen'
    )

    # optional RViz2 visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Optionally launch RViz2 alongside Gazebo'
        ),
        gazebo,
        robot_state_publisher_node,
        spawn_robot,
        bridge,
        rviz_node
    ])
