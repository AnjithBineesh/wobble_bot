import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('wobble_slam')

    # convert xacro → urdf
    robot_description = Command([
        'xacro ',
        os.path.join(
            pkg_share,
            'urdf',
            'wobble.urdf.xacro'
        )
    ])

    # start gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r -s empty.sdf'
        }.items()
    )

    # publish TF tree from URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # spawn robot into gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'wobble',
            '-topic', 'robot_description',
            '-z', '0.08'
        ],
        output='screen'
    )

    # bridge gazebo ↔ ros2 topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # velocity command
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # odometry from diff drive plugin
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # lidar scan
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_robot,
        bridge
    ])
