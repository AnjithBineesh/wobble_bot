import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('wobble_description')
    use_jsp_gui = LaunchConfiguration('use_joint_state_publisher_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = os.path.join(pkg_share, 'config', 'display.rviz')

    urdf_file = os.path.join(pkg_share, 'urdf', 'wobble.urdf')
    urdf_xacro = os.path.join(pkg_share, 'urdf', 'wobble.urdf.xacro')
    if os.path.exists(urdf_file):
        with open(urdf_file, 'r', encoding='utf-8') as f:
            robot_description = ParameterValue(f.read(), value_type=str)
    else:
        robot_description = ParameterValue(
            Command(['xacro ', urdf_xacro]),
            value_type=str
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_joint_state_publisher_gui',
            default_value='true',
            description='Launch the joint_state_publisher GUI.',
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2.',
        ),

        # publishes TF tree
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # publishes wheel joint values (no GUI)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(use_jsp_gui),
            output='screen',
        ),

        # publishes wheel joint values (with GUI sliders)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_jsp_gui),
            output='screen',
        ),

        # RViz2 visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(use_rviz),
            output='screen',
        ),
    ])
