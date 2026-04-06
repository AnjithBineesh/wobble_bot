#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomNode(Node):

    def __init__(self):
        super().__init__('odom_node')

        self.wheel_radius = 0.0325
        self.wheel_base = 0.20
        self.ticks_per_rev = 330

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.left_ticks = 0
        self.right_ticks = 0

        self.prev_left = None
        self.prev_right = None

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Int32,'/left_encoder',self.left_cb,10)
        self.create_subscription(Int32,'/right_encoder',self.right_cb,10)

        self.odom_pub = self.create_publisher(Odometry,'/odom',10)

        self.create_timer(0.02,self.update)

    def left_cb(self,msg):
        self.left_ticks = msg.data

    def right_cb(self,msg):
        self.right_ticks = msg.data

    def update(self):

        if self.prev_left is None:
            self.prev_left = self.left_ticks
            self.prev_right = self.right_ticks
            return

        dL = self.left_ticks - self.prev_left
        dR = self.right_ticks - self.prev_right

        self.prev_left = self.left_ticks
        self.prev_right = self.right_ticks

        meters_per_tick = (2*math.pi*self.wheel_radius)/self.ticks_per_rev

        dL *= meters_per_tick
        dR *= meters_per_tick

        d_center = (dL+dR)/2
        d_theta = (dR-dL)/self.wheel_base

        self.theta += d_theta

        self.x += d_center*math.cos(self.theta)
        self.y += d_center*math.sin(self.theta)

        qz = math.sin(self.theta/2)
        qw = math.cos(self.theta/2)

        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main(args=None):

    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
