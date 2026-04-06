#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import lgpio

# Raspberry Pi 5 uses gpiochip4
CHIP = 4

# Encoder pins
LEFT_A = 20
LEFT_B = 21

RIGHT_A = 26
RIGHT_B = 19


class EncoderNode(Node):

    def __init__(self):

        super().__init__('encoder_node')

        self.get_logger().info("Encoder node started")

        # publishers
        self.left_pub = self.create_publisher(Int32, '/left_encoder', 10)
        self.right_pub = self.create_publisher(Int32, '/right_encoder', 10)

        # open gpiochip
        self.h = lgpio.gpiochip_open(CHIP)

        # configure encoder pins
        for pin in [LEFT_A, LEFT_B, RIGHT_A, RIGHT_B]:
            lgpio.gpio_claim_input(self.h, pin, lgpio.SET_PULL_UP)

        # tick counters
        self.left_count = 0
        self.right_count = 0

        # previous states
        self.prev_left = lgpio.gpio_read(self.h, LEFT_A)
        self.prev_right = lgpio.gpio_read(self.h, RIGHT_A)

        # fast polling for encoders
        self.create_timer(0.001, self.read_encoders)

        # publish counts
        self.create_timer(0.05, self.publish_counts)

    def read_encoders(self):

        left_state = lgpio.gpio_read(self.h, LEFT_A)
        right_state = lgpio.gpio_read(self.h, RIGHT_A)

        # LEFT encoder
        if left_state != self.prev_left:

            b = lgpio.gpio_read(self.h, LEFT_B)

            if left_state == b:
                self.left_count += 1
            else:
                self.left_count -= 1

        # RIGHT encoder
        if right_state != self.prev_right:

            b = lgpio.gpio_read(self.h, RIGHT_B)

            if right_state == b:
                self.right_count += 1
            else:
                self.right_count -= 1

        self.prev_left = left_state
        self.prev_right = right_state

    def publish_counts(self):

        left_msg = Int32()
        right_msg = Int32()

        left_msg.data = self.left_count
        right_msg.data = self.right_count

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)


def main(args=None):

    rclpy.init(args=args)

    node = EncoderNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    lgpio.gpiochip_close(node.h)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
