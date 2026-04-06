from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory('wobble_slam')

    urdf_file = os.path.join(pkg, 'urdf', 'wobble.urdf')

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'wobble', '-topic', 'robot_description'],
        output='screen'
    )

    # Bridge (ROS ↔ Gazebo)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        bridge
    ])
