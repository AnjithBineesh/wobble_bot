import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('wobble_slam')
    use_joint_state_publisher_gui = LaunchConfiguration('use_joint_state_publisher_gui')

    robot_description = Command([
        'xacro',
        ' ',
        os.path.join(
            pkg_share,
            'urdf',
            'wobble.urdf.xacro'
        )
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_joint_state_publisher_gui',
            default_value='false',
            description='Launch the joint_state_publisher GUI.',
        ),

        # publishes TF tree
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }],
            output='screen'
        ),

        # publishes wheel joint values
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(use_joint_state_publisher_gui),
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_joint_state_publisher_gui),
            output='screen',
        )
    ])
