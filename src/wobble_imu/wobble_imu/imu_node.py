#!/usr/bin/env python3

import json
import math
import os
import time

from mpu6050 import mpu6050
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class IMUNode(Node):

    def __init__(self):
        super().__init__('imu_node')

        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('log_rate_hz', 1.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('calibration_file', '~/.ros2/imu_calibration.json')
        self.declare_parameter('zero_reference_on_startup', True)
        self.declare_parameter('accel_alpha', 0.2)
        self.declare_parameter('gyro_alpha', 0.35)
        self.declare_parameter('complementary_gain', 0.02)
        self.declare_parameter('mount_roll_deg', 0.0)
        self.declare_parameter('mount_pitch_deg', 0.0)
        self.declare_parameter('mount_yaw_deg', 0.0)

        self.sensor = mpu6050(0x68)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.zero_reference_on_startup = bool(
            self.get_parameter('zero_reference_on_startup').value
        )
        self.accel_alpha = float(self.get_parameter('accel_alpha').value)
        self.gyro_alpha = float(self.get_parameter('gyro_alpha').value)
        self.complementary_gain = float(self.get_parameter('complementary_gain').value)

        self.mount_rotation = self._rotation_matrix(
            math.radians(float(self.get_parameter('mount_roll_deg').value)),
            math.radians(float(self.get_parameter('mount_pitch_deg').value)),
            math.radians(float(self.get_parameter('mount_yaw_deg').value)),
        )

        calibration_file = os.path.expanduser(
            str(self.get_parameter('calibration_file').value)
        )
        if os.path.exists(calibration_file):
            self.get_logger().info(f'Loading calibration from {calibration_file}')
            with open(calibration_file, 'r', encoding='utf-8') as stream:
                cal_data = json.load(stream)
            self.gyro_offset = np.array([
                cal_data['gx_offset'],
                cal_data['gy_offset'],
                cal_data['gz_offset'],
            ], dtype=float)
            self.accel_offset = np.array([
                cal_data['ax_offset'],
                cal_data['ay_offset'],
                cal_data['az_offset'],
            ], dtype=float)
        else:
            self.get_logger().info(
                'Keeping robot still for calibration of gyro and accelerometer'
            )
            self._calibrate_gyro()
            self._calibrate_accel()
            self._save_calibration(calibration_file)

        self.filtered_accel = None
        self.filtered_gyro = None
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.yaw_ref = None
        self.prev_time = None

        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.log_period = 1.0 / max(float(self.get_parameter('log_rate_hz').value), 1e-6)
        self.last_log_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_imu)

    def _calibrate_gyro(self):
        samples = 600
        gyro_sum = np.zeros(3, dtype=float)
        for _ in range(samples):
            gyro_sum += self._raw_gyro_vector()
            time.sleep(0.01)
        self.gyro_offset = gyro_sum / samples
        self.get_logger().info(
            'Gyro offsets: '
            f'x={self.gyro_offset[0]:.4f}, '
            f'y={self.gyro_offset[1]:.4f}, '
            f'z={self.gyro_offset[2]:.4f}'
        )

    def _calibrate_accel(self):
        samples = 400
        accel_samples = []
        for _ in range(samples):
            accel_samples.append(self._raw_accel_vector())
            time.sleep(0.01)
        median_accel = np.median(np.array(accel_samples, dtype=float), axis=0)
        self.accel_offset = np.array(
            [median_accel[0], median_accel[1], median_accel[2] - 9.80665],
            dtype=float,
        )
        self.get_logger().info(
            'Accel offsets: '
            f'x={self.accel_offset[0]:.4f}, '
            f'y={self.accel_offset[1]:.4f}, '
            f'z={self.accel_offset[2]:.4f}'
        )

    def _save_calibration(self, filepath):
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        cal_data = {
            'gx_offset': float(self.gyro_offset[0]),
            'gy_offset': float(self.gyro_offset[1]),
            'gz_offset': float(self.gyro_offset[2]),
            'ax_offset': float(self.accel_offset[0]),
            'ay_offset': float(self.accel_offset[1]),
            'az_offset': float(self.accel_offset[2]),
            'timestamp': time.time(),
        }
        with open(filepath, 'w', encoding='utf-8') as stream:
            json.dump(cal_data, stream, indent=2)

    def _raw_accel_vector(self):
        accel = self.sensor.get_accel_data()
        return np.array([accel['x'], accel['y'], accel['z']], dtype=float)

    def _raw_gyro_vector(self):
        gyro = self.sensor.get_gyro_data()
        return np.array([gyro['x'], gyro['y'], gyro['z']], dtype=float)

    def publish_imu(self):
        now = self.get_clock().now()
        if self.prev_time is None:
            self.prev_time = now
            return

        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now
        if dt <= 1e-6:
            return

        accel = self.mount_rotation @ (self._raw_accel_vector() - self.accel_offset)
        gyro = self.mount_rotation @ np.radians(self._raw_gyro_vector() - self.gyro_offset)

        if self.filtered_accel is None:
            self.filtered_accel = accel
            self.filtered_gyro = gyro
        else:
            self.filtered_accel = (
                self.accel_alpha * accel + (1.0 - self.accel_alpha) * self.filtered_accel
            )
            self.filtered_gyro = (
                self.gyro_alpha * gyro + (1.0 - self.gyro_alpha) * self.filtered_gyro
            )

        gx, gy, gz = self.filtered_gyro
        ax, ay, az = self.filtered_accel

        accel_roll = math.atan2(ay, az)
        accel_pitch = math.atan2(-ax, math.sqrt(max(ay * ay + az * az, 1e-9)))

        self.roll += gx * dt
        self.pitch += gy * dt
        self.yaw += gz * dt

        self.roll = self._normalize_angle(
            (1.0 - self.complementary_gain) * self.roll +
            self.complementary_gain * accel_roll
        )
        self.pitch = self._normalize_angle(
            (1.0 - self.complementary_gain) * self.pitch +
            self.complementary_gain * accel_pitch
        )
        self.yaw = self._normalize_angle(self.yaw)

        if self.zero_reference_on_startup and self.yaw_ref is None:
            self.yaw_ref = self.yaw
        yaw_for_msg = self._normalize_angle(
            self.yaw - self.yaw_ref if self.yaw_ref is not None else self.yaw
        )

        q = self.euler_to_quaternion(self.roll, self.pitch, yaw_for_msg)

        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation.w = q[0]
        msg.orientation.x = q[1]
        msg.orientation.y = q[2]
        msg.orientation.z = q[3]
        msg.orientation_covariance = [
            0.03, 0.0, 0.0,
            0.0, 0.03, 0.0,
            0.0, 0.0, 999.0,
        ]
        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)
        msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.005,
        ]
        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)
        msg.linear_acceleration_covariance = [
            0.08, 0.0, 0.0,
            0.0, 0.08, 0.0,
            0.0, 0.0, 0.08,
        ]
        self.publisher_.publish(msg)

        if (now - self.last_log_time).nanoseconds / 1e9 >= self.log_period:
            self.last_log_time = now
            self.get_logger().info(
                f'IMU rpy deg: '
                f'{math.degrees(self.roll):6.2f}, '
                f'{math.degrees(self.pitch):6.2f}, '
                f'{math.degrees(yaw_for_msg):6.2f}'
            )

    def _rotation_matrix(self, roll, pitch, yaw):
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        return np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ], dtype=float)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        return np.array([
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ], dtype=float)

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
