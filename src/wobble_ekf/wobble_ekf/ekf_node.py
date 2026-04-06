#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import numpy as np
import math


class EKFNode(Node):

    def __init__(self):

        super().__init__('ekf_filter_node')

        self.get_logger().info("EKF fusion started")


        # -------- PARAMETERS FROM YAML --------

        self.declare_parameter('process_noise', 0.001)
        self.declare_parameter('odom_noise', 0.05)
        self.declare_parameter('imu_noise', 0.02)
        self.declare_parameter('output_topic', '/odom_filtered')


        process_noise = self.get_parameter('process_noise').value
        odom_noise = self.get_parameter('odom_noise').value
        imu_noise = self.get_parameter('imu_noise').value

        output_topic = self.get_parameter('output_topic').value


        # -------- EKF STATE --------

        # state vector [x y theta]
        self.x = np.zeros((3,1))

        # covariance matrix
        self.P = np.eye(3) * 0.01

        # process noise
        self.Q = np.eye(3) * process_noise

        # measurement noise
        self.R_odom = np.eye(3) * odom_noise
        self.R_imu = np.eye(1) * imu_noise


        # latest imu yaw
        self.imu_theta = 0.0


        # -------- SUBSCRIBERS --------

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )


        self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )


        # -------- PUBLISHER --------

        self.odom_pub = self.create_publisher(
            Odometry,
            output_topic,
            10
        )


    # -------- IMU CALLBACK --------

    def imu_callback(self, msg):

        q = msg.orientation

        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)

        self.imu_theta = math.atan2(siny, cosy)



    # -------- ODOM CALLBACK --------

    def odom_callback(self, msg):

        # measurement from encoder
        z_odom = np.array([
            [msg.pose.pose.position.x],
            [msg.pose.pose.position.y],
            [self.get_yaw(msg)]
        ])


        # prediction
        x_pred = self.x
        P_pred = self.P + self.Q


        # update from encoder
        y = z_odom - x_pred

        S = P_pred + self.R_odom

        K = P_pred @ np.linalg.inv(S)

        self.x = x_pred + K @ y

        self.P = (np.eye(3) - K) @ P_pred


        # update from IMU (orientation only)

        z_imu = np.array([[self.imu_theta]])

        H = np.array([[0,0,1]])

        y2 = z_imu - H @ self.x

        S2 = H @ self.P @ H.T + self.R_imu

        K2 = self.P @ H.T @ np.linalg.inv(S2)

        self.x = self.x + K2 @ y2

        self.P = (np.eye(3) - K2 @ H) @ self.P


        self.publish_filtered(msg)



    # -------- GET YAW FROM ODOM --------

    def get_yaw(self, msg):

        q = msg.pose.pose.orientation

        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)

        return math.atan2(siny, cosy)



    # -------- PUBLISH RESULT --------

    def publish_filtered(self, msg):

        msg.pose.pose.position.x = float(self.x[0])
        msg.pose.pose.position.y = float(self.x[1])

        theta = float(self.x[2])

        msg.pose.pose.orientation.z = math.sin(theta/2)
        msg.pose.pose.orientation.w = math.cos(theta/2)


        self.odom_pub.publish(msg)


        self.get_logger().info(

            f"x={self.x[0][0]:.3f}  "

            f"y={self.x[1][0]:.3f}  "

            f"theta={math.degrees(self.x[2][0]):.2f}°"

        )



def main(args=None):

    rclpy.init(args=args)

    node = EKFNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':

    main()
