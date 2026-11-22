from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    auto_standby = LaunchConfiguration('auto_standby')
    use_foxglove = LaunchConfiguration('use_foxglove')

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,
            'inverted': False,
            'angle_compensate': True,
            'auto_standby': auto_standby,
        }]
    )

    # Static transform so laser frame can be visualized relative to base_link / base_footprint
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_broadcaster',
        arguments=['--x', '0', '--y', '0', '--z', '0.2',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        output='screen'
    )

    # Foxglove WebSocket Bridge (port 8765) for browser / app visualization
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'send_buffer_limit': 10000000,
        }],
        condition=IfCondition(use_foxglove)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='RPLiDAR serial port'
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='115200',
            description='RPLiDAR serial baudrate'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser',
            description='LiDAR frame ID'
        ),
        DeclareLaunchArgument(
            'auto_standby',
            default_value='true',
            description='Only spin and scan when actively subscribed; stop motor when idle'
        ),
        DeclareLaunchArgument(
            'use_foxglove',
            default_value='true',
            description='Start Foxglove WebSocket bridge on port 8765'
        ),
        rplidar_node,
        static_tf,
        foxglove_node,
    ])
