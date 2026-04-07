#!/usr/bin/env python3

import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from tf2_ros import TransformBroadcaster


class OdomNode(Node):

    def __init__(self):
        super().__init__('odom_node')

        self.declare_parameter('wheel_radius', 0.0335)
        self.declare_parameter('left_wheel_radius', 0.0335)
        self.declare_parameter('right_wheel_radius', 0.0335)
        self.declare_parameter('wheel_base', 0.31)
        self.declare_parameter('wheel_base_scale', 1.0)
        self.declare_parameter('ticks_per_rev', 1320)
        self.declare_parameter('left_distance_scale', 1.0)
        self.declare_parameter('right_distance_scale', 1.0)
        self.declare_parameter('left_speed_scale', 1.0)
        self.declare_parameter('right_speed_scale', 1.0)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('odom_topic', '/wheel/odom')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('left_wheel_joint', 'joint_left_wheel')
        self.declare_parameter('right_wheel_joint', 'joint_right_wheel')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('max_tick_jump', 0)
        self.declare_parameter('pose_covariance_xy', 0.02)
        self.declare_parameter('pose_covariance_yaw', 0.08)
        self.declare_parameter('twist_covariance_linear', 0.05)
        self.declare_parameter('twist_covariance_angular', 0.1)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.left_wheel_radius = float(self.get_parameter('left_wheel_radius').value)
        self.right_wheel_radius = float(self.get_parameter('right_wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.wheel_base_scale = float(self.get_parameter('wheel_base_scale').value)
        self.ticks_per_rev = int(self.get_parameter('ticks_per_rev').value)
        self.left_distance_scale = float(self.get_parameter('left_distance_scale').value)
        self.right_distance_scale = float(self.get_parameter('right_distance_scale').value)
        self.max_tick_jump = int(self.get_parameter('max_tick_jump').value)
        self.pose_covariance_xy = float(self.get_parameter('pose_covariance_xy').value)
        self.pose_covariance_yaw = float(self.get_parameter('pose_covariance_yaw').value)
        self.twist_covariance_linear = float(
            self.get_parameter('twist_covariance_linear').value
        )
        self.twist_covariance_angular = float(
            self.get_parameter('twist_covariance_angular').value
        )
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.left_wheel_joint = str(self.get_parameter('left_wheel_joint').value)
        self.right_wheel_joint = str(self.get_parameter('right_wheel_joint').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        publish_period = 1.0 / publish_rate_hz if publish_rate_hz > 0.0 else 0.02
        legacy_left_scale = float(self.get_parameter('left_speed_scale').value)
        legacy_right_scale = float(self.get_parameter('right_speed_scale').value)

        if self.left_wheel_radius == self.wheel_radius:
            self.left_wheel_radius *= self.left_distance_scale * legacy_left_scale
        if self.right_wheel_radius == self.wheel_radius:
            self.right_wheel_radius *= self.right_distance_scale * legacy_right_scale

        self.effective_wheel_base = self.wheel_base * self.wheel_base_scale
        self.left_meters_per_tick = (
            2.0 * math.pi * self.left_wheel_radius / self.ticks_per_rev
        )
        self.right_meters_per_tick = (
            2.0 * math.pi * self.right_wheel_radius / self.ticks_per_rev
        )

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        self.left_ticks = 0
        self.right_ticks = 0

        self.prev_left = None
        self.prev_right = None
        self.prev_time = None

        self.left_cb_count = 0
        self.right_cb_count = 0

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Int32, '/left_encoder', self.left_cb, 10)
        self.create_subscription(Int32, '/right_encoder', self.right_cb, 10)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.joint_state_pub = self.create_publisher(
            JointState,
            self.joint_state_topic,
            10,
        )

        self.create_timer(publish_period, self.update)
        self.create_timer(1.0, self.check_callbacks)

        self.get_logger().info(
            'Odom node initialized with '
            f'left_wheel_radius={self.left_wheel_radius:.5f} m, '
            f'right_wheel_radius={self.right_wheel_radius:.5f} m, '
            f'effective_wheel_base={self.effective_wheel_base:.4f} m, '
            f'ticks_per_rev={self.ticks_per_rev}, '
            f'odom_topic={self.odom_topic}, '
            f'joint_state_topic={self.joint_state_topic}, '
            f'publish_tf={self.publish_tf}'
        )

    def left_cb(self, msg):
        self.left_ticks = msg.data
        self.left_cb_count += 1

    def right_cb(self, msg):
        self.right_ticks = msg.data
        self.right_cb_count += 1

    def check_callbacks(self):
        self.get_logger().info(
            'Encoder callbacks - '
            f'Left: {self.left_cb_count}/sec, Right: {self.right_cb_count}/sec '
            f'(ticks L:{self.left_ticks} R:{self.right_ticks})'
        )
        self.left_cb_count = 0
        self.right_cb_count = 0

    def update(self):
        now_time = self.get_clock().now()
        if self.prev_left is None:
            self.prev_left = self.left_ticks
            self.prev_right = self.right_ticks
            self.prev_time = now_time
            return

        left_tick_delta = self.left_ticks - self.prev_left
        right_tick_delta = self.right_ticks - self.prev_right

        if (
            self.max_tick_jump > 0 and
            (
                abs(left_tick_delta) > self.max_tick_jump or
                abs(right_tick_delta) > self.max_tick_jump
            )
        ):
            self.get_logger().warning(
                'Ignoring encoder jump '
                f'(left={left_tick_delta}, right={right_tick_delta}, '
                f'max_tick_jump={self.max_tick_jump})'
            )
            self.prev_left = self.left_ticks
            self.prev_right = self.right_ticks
            self.prev_time = now_time
            return

        dt = (now_time - self.prev_time).nanoseconds / 1e9 if self.prev_time else 0.0
        self.prev_left = self.left_ticks
        self.prev_right = self.right_ticks
        self.prev_time = now_time

        left_distance = left_tick_delta * self.left_meters_per_tick
        right_distance = right_tick_delta * self.right_meters_per_tick

        d_center = 0.5 * (left_distance + right_distance)
        d_theta = (
            (right_distance - left_distance) / self.effective_wheel_base
            if abs(self.effective_wheel_base) > 1e-9 else 0.0
        )

        theta_mid = self.theta + 0.5 * d_theta
        self.x += d_center * math.cos(theta_mid)
        self.y += d_center * math.sin(theta_mid)
        self.theta = self._normalize_angle(self.theta + d_theta)

        self.left_wheel_angle += left_distance / max(self.left_wheel_radius, 1e-9)
        self.right_wheel_angle += right_distance / max(self.right_wheel_radius, 1e-9)

        if dt > 1e-6:
            self.linear_velocity = d_center / dt
            self.angular_velocity = d_theta / dt
            self.left_wheel_velocity = (
                left_distance / max(self.left_wheel_radius, 1e-9) / dt
            )
            self.right_wheel_velocity = (
                right_distance / max(self.right_wheel_radius, 1e-9) / dt
            )
        else:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self.left_wheel_velocity = 0.0
            self.right_wheel_velocity = 0.0

        now = now_time.to_msg()
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity
        odom.pose.covariance = [
            self.pose_covariance_xy, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, self.pose_covariance_xy, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.pose_covariance_yaw,
        ]
        odom.twist.covariance = [
            self.twist_covariance_linear, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, self.twist_covariance_linear * 2.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.twist_covariance_angular,
        ]
        self.odom_pub.publish(odom)

        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.name = [self.left_wheel_joint, self.right_wheel_joint]
        joint_state.position = [self.left_wheel_angle, self.right_wheel_angle]
        joint_state.velocity = [self.left_wheel_velocity, self.right_wheel_velocity]
        self.joint_state_pub.publish(joint_state)

        if not self.publish_tf:
            return

        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
