from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    urdf_file = '/home/wobble/wobble_ws/src/wobble_slam/urdf/wobble.urdf'

    ekf_config = '/home/wobble/wobble_ws/src/wobble_ekf/config/ekf.yaml'


    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_a1_launch.py'
            )
        )
    )


    return LaunchDescription([

        # ---------------- ROBOT MODEL ----------------

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': open(urdf_file).read()
            }],
            output='screen'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen'
        ),


        # ---------------- ENCODER ----------------

        Node(
            package='wobble_sensor',
            executable='encoder_node',
            name='encoder_node',
            output='screen'
        ),

        # ---------------- ODOMETRY ----------------

        Node(
            package='wobble_odometry',
            executable='odom_node',
            name='odom_node',
            output='screen'
        ),


        # ---------------- IMU ----------------

        Node(
            package='wobble_imu',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),


        # ---------------- EKF ----------------

	Node(
    		package='wobble_ekf',
    		executable='ekf_node',
    		name='ekf_filter_node',
    		parameters=['/home/wobble/wobble_ws/src/wobble_ekf/config/ekf.yaml'],
    		output='screen'
		),
        # ---------------- LIDAR ----------------

        rplidar_launch,


        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.045', '0', '0.10', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),


        # ---------------- SLAM ----------------

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'map_frame': 'map',
                'scan_topic': '/scan'
            }]
        )

    ])
