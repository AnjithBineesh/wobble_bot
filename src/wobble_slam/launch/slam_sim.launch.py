import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('wobble_slam')
    use_rviz = LaunchConfiguration('use_rviz')

    slam_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'sim': 'true',
            'use_rviz': use_rviz,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 alongside simulation SLAM'
        ),
        slam_sim_launch,
    ])

