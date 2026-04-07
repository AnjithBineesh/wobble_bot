#!/usr/bin/env python3

import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class EKFNode(Node):

    def __init__(self):
        super().__init__('ekf_filter_node')

        self.declare_parameter('process_noise_xy', 0.002)
        self.declare_parameter('process_noise_yaw', 0.01)
        self.declare_parameter('process_noise_velocity', 0.05)
        self.declare_parameter('process_noise_yaw_rate', 0.08)
        self.declare_parameter('wheel_odom_topic', '/wheel/odom')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('output_topic', '/odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('wheel_position_noise', 0.02)
        self.declare_parameter('wheel_yaw_noise', 0.12)
        self.declare_parameter('wheel_linear_velocity_noise', 0.08)
        self.declare_parameter('wheel_yaw_rate_noise', 0.15)
        self.declare_parameter('imu_yaw_rate_noise', 0.05)

        wheel_odom_topic = str(self.get_parameter('wheel_odom_topic').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.q_process = np.diag([
            float(self.get_parameter('process_noise_xy').value) ** 2,
            float(self.get_parameter('process_noise_xy').value) ** 2,
            float(self.get_parameter('process_noise_yaw').value) ** 2,
            float(self.get_parameter('process_noise_velocity').value) ** 2,
            float(self.get_parameter('process_noise_yaw_rate').value) ** 2,
        ])

        self.r_wheel_pose = np.diag([
            float(self.get_parameter('wheel_position_noise').value) ** 2,
            float(self.get_parameter('wheel_position_noise').value) ** 2,
            float(self.get_parameter('wheel_yaw_noise').value) ** 2,
        ])
        self.r_wheel_twist = np.diag([
            float(self.get_parameter('wheel_linear_velocity_noise').value) ** 2,
            float(self.get_parameter('wheel_yaw_rate_noise').value) ** 2,
        ])
        self.r_imu_yaw_rate = np.array([
            [float(self.get_parameter('imu_yaw_rate_noise').value) ** 2]
        ])

        # State: x, y, yaw, linear velocity, yaw rate.
        self.x = np.zeros((5, 1), dtype=float)
        self.p = np.diag([0.05, 0.05, 0.1, 0.2, 0.2])
        self.initialized = False
        self.prev_predict_time = None
        self.latest_wheel_msg = None
        self.latest_imu_msg = None
        self.last_wheel_stamp = None
        self.last_imu_stamp = None

        self.odom_pub = self.create_publisher(Odometry, self.output_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Odometry, wheel_odom_topic, self.wheel_odom_callback, 10)
        self.create_subscription(Imu, imu_topic, self.imu_callback, 10)

        publish_period = 1.0 / publish_rate_hz if publish_rate_hz > 0.0 else 0.02
        self.create_timer(publish_period, self.update_and_publish)

        self.get_logger().info(
            f'EKF fusion started: wheel={wheel_odom_topic}, imu={imu_topic}, '
            f'output={self.output_topic}, publish_tf={self.publish_tf}'
        )

    def wheel_odom_callback(self, msg):
        self.latest_wheel_msg = msg
        if self.initialized:
            return

        self.x[0, 0] = msg.pose.pose.position.x
        self.x[1, 0] = msg.pose.pose.position.y
        self.x[2, 0] = self._yaw_from_quaternion(msg.pose.pose.orientation)
        self.x[3, 0] = msg.twist.twist.linear.x
        self.x[4, 0] = msg.twist.twist.angular.z
        self.initialized = True
        self.prev_predict_time = self.get_clock().now()

    def imu_callback(self, msg):
        self.latest_imu_msg = msg

    def update_and_publish(self):
        if not self.initialized:
            return

        now = self.get_clock().now()
        dt = (now - self.prev_predict_time).nanoseconds / 1e9 if self.prev_predict_time else 0.0
        self.prev_predict_time = now
        if dt > 1e-6:
            self._predict(dt)

        wheel_stamp = self._stamp_tuple(
            self.latest_wheel_msg.header.stamp if self.latest_wheel_msg is not None else None
        )
        if self.latest_wheel_msg is not None and wheel_stamp != self.last_wheel_stamp:
            self._update_from_wheel(self.latest_wheel_msg)
            self.last_wheel_stamp = wheel_stamp

        imu_stamp = self._stamp_tuple(
            self.latest_imu_msg.header.stamp if self.latest_imu_msg is not None else None
        )
        if self.latest_imu_msg is not None and imu_stamp != self.last_imu_stamp:
            self._update_from_imu(self.latest_imu_msg)
            self.last_imu_stamp = imu_stamp

        self._publish(now.to_msg())

    def _predict(self, dt):
        yaw = self.x[2, 0]
        v = self.x[3, 0]
        w = self.x[4, 0]

        self.x[0, 0] += v * math.cos(yaw) * dt
        self.x[1, 0] += v * math.sin(yaw) * dt
        self.x[2, 0] = self._normalize_angle(self.x[2, 0] + w * dt)

        f = np.eye(5, dtype=float)
        f[0, 2] = -v * math.sin(yaw) * dt
        f[0, 3] = math.cos(yaw) * dt
        f[1, 2] = v * math.cos(yaw) * dt
        f[1, 3] = math.sin(yaw) * dt
        f[2, 4] = dt

        self.p = f @ self.p @ f.T + self.q_process

    def _update_from_wheel(self, msg):
        wheel_yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
        z_pose = np.array([
            [msg.pose.pose.position.x],
            [msg.pose.pose.position.y],
            [wheel_yaw],
        ], dtype=float)
        h_pose = np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0],
        ], dtype=float)
        self._update(z_pose, h_pose, self.r_wheel_pose, angle_rows={2})

        z_twist = np.array([
            [msg.twist.twist.linear.x],
            [msg.twist.twist.angular.z],
        ], dtype=float)
        h_twist = np.array([
            [0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0],
        ], dtype=float)
        self._update(z_twist, h_twist, self.r_wheel_twist)

    def _update_from_imu(self, msg):
        z = np.array([[msg.angular_velocity.z]], dtype=float)
        h = np.array([[0.0, 0.0, 0.0, 0.0, 1.0]], dtype=float)
        self._update(z, h, self.r_imu_yaw_rate)

    def _update(self, z, h, r, angle_rows=None):
        innovation = z - (h @ self.x)
        if angle_rows:
            for row in angle_rows:
                innovation[row, 0] = self._normalize_angle(innovation[row, 0])

        s = h @ self.p @ h.T + r
        k = self.p @ h.T @ np.linalg.inv(s)
        self.x = self.x + (k @ innovation)
        self.x[2, 0] = self._normalize_angle(self.x[2, 0])
        self.p = (np.eye(5) - k @ h) @ self.p

    def _publish(self, stamp):
        yaw = self.x[2, 0]
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = float(self.x[0, 0])
        odom_msg.pose.pose.position.y = float(self.x[1, 0])
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.twist.twist.linear.x = float(self.x[3, 0])
        odom_msg.twist.twist.angular.z = float(self.x[4, 0])
        odom_msg.pose.covariance = [
            float(self.p[0, 0]), 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, float(self.p[1, 1]), 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, float(self.p[2, 2]),
        ]
        odom_msg.twist.covariance = [
            float(self.p[3, 3]), 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 999.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, float(self.p[4, 4]),
        ]
        self.odom_pub.publish(odom_msg)

        if not self.publish_tf:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = float(self.x[0, 0])
        transform.transform.translation.y = float(self.x[1, 0])
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def _yaw_from_quaternion(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _stamp_tuple(self, stamp):
        if stamp is None:
            return None
        return (stamp.sec, stamp.nanosec)


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
