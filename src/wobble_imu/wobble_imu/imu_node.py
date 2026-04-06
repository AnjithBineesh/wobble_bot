#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mpu6050 import mpu6050
from ahrs.filters import Madgwick
import numpy as np
import math
import time


class IMUNode(Node):

    def __init__(self):

        super().__init__('imu_node')

        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)

        self.sensor = mpu6050(0x68)

        self.filter = Madgwick(beta=0.05)   # lower beta = smoother output

        self.q = np.array([1.0, 0.0, 0.0, 0.0])

        self.get_logger().info("KEEP ROBOT STILL FOR 8 SECONDS")

        samples = 500

        gx_sum = 0
        gy_sum = 0
        gz_sum = 0

        ax_sum = 0
        ay_sum = 0
        az_sum = 0


        for i in range(samples):

            accel = self.sensor.get_accel_data()
            gyro = self.sensor.get_gyro_data()

            gx_sum += gyro['x']
            gy_sum += gyro['y']
            gz_sum += gyro['z']

            ax_sum += accel['x']
            ay_sum += accel['y']
            az_sum += accel['z']

            time.sleep(0.01)


        self.gx_offset = gx_sum / samples
        self.gy_offset = gy_sum / samples
        self.gz_offset = gz_sum / samples

        self.ax_offset = ax_sum / samples
        self.ay_offset = ay_sum / samples
        self.az_offset = az_sum / samples - 9.81


        self.get_logger().info("Calibration done")

        # orientation zero reference
        self.roll_ref = 0
        self.pitch_ref = 0
        self.yaw_ref = 0

        self.ref_set = False

        self.timer = self.create_timer(0.04, self.publish_imu)



    def quaternion_to_euler(self, w, x, y, z):

        t0 = 2*(w*x + y*z)
        t1 = 1 - 2*(x*x + y*y)
        roll = math.atan2(t0, t1)

        t2 = 2*(w*y - z*x)
        t2 = max(min(t2, 1), -1)
        pitch = math.asin(t2)

        t3 = 2*(w*z + x*y)
        t4 = 1 - 2*(y*y + z*z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw



    def publish_imu(self):

        accel = self.sensor.get_accel_data()
        gyro = self.sensor.get_gyro_data()


        ax = accel['x'] - self.ax_offset
        ay = accel['y'] - self.ay_offset
        az = accel['z'] - self.az_offset


        gx = math.radians(gyro['x'] - self.gx_offset)
        gy = math.radians(gyro['y'] - self.gy_offset)
        gz = math.radians(gyro['z'] - self.gz_offset)


        self.q = self.filter.updateIMU(
            self.q,
            gyr=np.array([gx, gy, gz]),
            acc=np.array([ax, ay, az])
        )


        roll, pitch, yaw = self.quaternion_to_euler(
            self.q[0], self.q[1], self.q[2], self.q[3]
        )


        # set zero reference once
        if not self.ref_set:

            self.roll_ref = roll
            self.pitch_ref = pitch
            self.yaw_ref = yaw

            self.ref_set = True


        roll -= self.roll_ref
        pitch -= self.pitch_ref
        yaw -= self.yaw_ref


        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        msg.orientation.w = self.q[0]
        msg.orientation.x = self.q[1]
        msg.orientation.y = self.q[2]
        msg.orientation.z = self.q[3]

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        self.publisher_.publish(msg)


        self.get_logger().info(
            f"Roll:{math.degrees(roll):6.3f}°  "
            f"Pitch:{math.degrees(pitch):6.3f}°  "
            f"Yaw:{math.degrees(yaw):6.3f}°"
        )



def main(args=None):

    rclpy.init(args=args)

    node = IMUNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
