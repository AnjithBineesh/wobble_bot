#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import lgpio

# Motor pins
IN1 = 17
IN2 = 18
IN3 = 27
IN4 = 22

ENA = 12
ENB = 13

CHIP = 4

# Speed tuning
SPEED_SCALE = 0.7
TURN_GAIN = 2.0
MAX_PWM = 100


class MotorNode(Node):

    def __init__(self):

        super().__init__('motor_node')

        self.get_logger().info("Motor node started")

        self.h = lgpio.gpiochip_open(CHIP)

        # Claim direction pins
        for pin in [IN1, IN2, IN3, IN4]:
            lgpio.gpio_claim_output(self.h, pin)

        # Claim PWM pins
        lgpio.gpio_claim_output(self.h, ENA)
        lgpio.gpio_claim_output(self.h, ENB)

        self.stop_motors()

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

    def stop_motors(self):

        lgpio.gpio_write(self.h, IN1, 0)
        lgpio.gpio_write(self.h, IN2, 0)
        lgpio.gpio_write(self.h, IN3, 0)
        lgpio.gpio_write(self.h, IN4, 0)

        lgpio.tx_pwm(self.h, ENA, 1000, 0)
        lgpio.tx_pwm(self.h, ENB, 1000, 0)

    def cmd_callback(self, msg):

        linear = msg.linear.x
        angular = msg.angular.z * TURN_GAIN

        # differential drive
        left = linear - angular
        right = linear + angular

        left *= SPEED_SCALE
        right *= SPEED_SCALE

        # LEFT MOTOR direction
        if left > 0:
            lgpio.gpio_write(self.h, IN1, 1)
            lgpio.gpio_write(self.h, IN2, 0)
        elif left < 0:
            lgpio.gpio_write(self.h, IN1, 0)
            lgpio.gpio_write(self.h, IN2, 1)
        else:
            lgpio.gpio_write(self.h, IN1, 0)
            lgpio.gpio_write(self.h, IN2, 0)

        # RIGHT MOTOR direction
        if right > 0:
            lgpio.gpio_write(self.h, IN3, 1)
            lgpio.gpio_write(self.h, IN4, 0)
        elif right < 0:
            lgpio.gpio_write(self.h, IN3, 0)
            lgpio.gpio_write(self.h, IN4, 1)
        else:
            lgpio.gpio_write(self.h, IN3, 0)
            lgpio.gpio_write(self.h, IN4, 0)

        pwm_left = min(int(abs(left) * MAX_PWM), MAX_PWM)
        pwm_right = min(int(abs(right) * MAX_PWM), MAX_PWM)

        lgpio.tx_pwm(self.h, ENA, 1000, pwm_left)
        lgpio.tx_pwm(self.h, ENB, 1000, pwm_right)

        self.get_logger().info(
            f"PWM Left:{pwm_left}  PWM Right:{pwm_right}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = MotorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.stop_motors()

    lgpio.gpiochip_close(node.h)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
