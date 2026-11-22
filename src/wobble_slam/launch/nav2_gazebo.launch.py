import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    description_dir = get_package_share_directory('wobble_description')
    nav2_dir = get_package_share_directory('nav2_bringup')
    slam_dir = get_package_share_directory('wobble_slam')

    map_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    nav2_params = os.path.join(nav2_dir, 'params', 'nav2_params.yaml')
    nav2_navigation_params = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites={
            'controller_server.ros__parameters.progress_checker.required_movement_radius': '0.25',
            'controller_server.ros__parameters.progress_checker.movement_time_allowance': '15.0',
            'controller_server.ros__parameters.FollowPath.acc_lim_x': '1.0',
            'controller_server.ros__parameters.FollowPath.decel_lim_x': '-1.0',
            'controller_server.ros__parameters.FollowPath.acc_lim_theta': '2.0',
            'controller_server.ros__parameters.FollowPath.decel_lim_theta': '-2.0',
        },
        convert_types=True,
    )
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, 'launch', 'sim.launch.py')
        ),
        launch_arguments={'use_rviz': 'false'}.items()
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_navigation_params,
            'autostart': 'true',
            'use_composition': 'False',
        }.items()
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params,
            {'yaml_filename': map_file, 'use_sim_time': True},
        ],
    )
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            nav2_params,
            {
                'use_sim_time': True,
                'set_initial_pose': True,
                'initial_pose.x': ParameterValue(initial_pose_x, value_type=float),
                'initial_pose.y': ParameterValue(initial_pose_y, value_type=float),
                'initial_pose.z': 0.0,
                'initial_pose.yaw': ParameterValue(initial_pose_yaw, value_type=float),
            },
        ],
    )
    localization_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(slam_dir, 'config', 'nav2.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser(
                '~/wobble_bot/Map Image/wobble_map.yaml'
            ),
            description='Path to the saved occupancy map YAML file',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz with the Nav2 configuration',
        ),
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='0.0',
            description='AMCL initial x position in the map frame',
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='0.0',
            description='AMCL initial y position in the map frame',
        ),
        DeclareLaunchArgument(
            'initial_pose_yaw',
            default_value='0.0',
            description='AMCL initial yaw in the map frame, in radians',
        ),
        sim_launch,
        map_server,
        amcl,
        localization_lifecycle_manager,
        navigation_launch,
        rviz,
    ])
