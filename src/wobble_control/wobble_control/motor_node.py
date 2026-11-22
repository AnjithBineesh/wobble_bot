#!/usr/bin/env python3

import math

from geometry_msgs.msg import Twist
import lgpio
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_node')

        # Parameters
        self.declare_parameter('gpio_chip', 4)
        self.declare_parameter('in1_pin', 17)
        self.declare_parameter('in2_pin', 18)
        self.declare_parameter('in3_pin', 27)
        self.declare_parameter('in4_pin', 22)
        self.declare_parameter('ena_pin', 12)
        self.declare_parameter('enb_pin', 13)
        self.declare_parameter('wheel_base', 0.14)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('linear_scale', 0.35)
        self.declare_parameter('angular_scale', 0.35)
        self.declare_parameter('min_pwm', 20)
        self.declare_parameter('max_pwm', 45)
        self.declare_parameter('pwm_frequency', 1000)
        self.declare_parameter('left_speed_trim', 1.02)
        self.declare_parameter('right_speed_trim', 1.00)
        self.declare_parameter('left_invert', False)
        self.declare_parameter('right_invert', False)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('enable_imu_heading_lock', True)
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('heading_kp', 1.2)
        self.declare_parameter('heading_kd', 0.08)
        self.declare_parameter('max_heading_correction', 0.35)

        self.gpio_chip = int(self.get_parameter('gpio_chip').value)
        self.in1_pin = int(self.get_parameter('in1_pin').value)
        self.in2_pin = int(self.get_parameter('in2_pin').value)
        self.in3_pin = int(self.get_parameter('in3_pin').value)
        self.in4_pin = int(self.get_parameter('in4_pin').value)
        self.ena_pin = int(self.get_parameter('ena_pin').value)
        self.enb_pin = int(self.get_parameter('enb_pin').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.min_pwm = int(self.get_parameter('min_pwm').value)
        self.max_pwm = int(self.get_parameter('max_pwm').value)
        self.pwm_frequency = int(self.get_parameter('pwm_frequency').value)
        self.left_speed_trim = float(self.get_parameter('left_speed_trim').value)
        self.right_speed_trim = float(self.get_parameter('right_speed_trim').value)
        self.left_invert = bool(self.get_parameter('left_invert').value)
        self.right_invert = bool(self.get_parameter('right_invert').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.enable_imu_heading_lock = bool(self.get_parameter('enable_imu_heading_lock').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.heading_kd = float(self.get_parameter('heading_kd').value)
        self.max_heading_correction = float(self.get_parameter('max_heading_correction').value)

        self.current_yaw = None
        self.current_yaw_rate = 0.0
        self.target_yaw = None

        # Open GPIO chip
        self.h = lgpio.gpiochip_open(self.gpio_chip)

        # Claim direction pins
        for pin in [self.in1_pin, self.in2_pin, self.in3_pin, self.in4_pin]:
            lgpio.gpio_claim_output(self.h, pin)

        # Claim PWM enable pins
        lgpio.gpio_claim_output(self.h, self.ena_pin)
        lgpio.gpio_claim_output(self.h, self.enb_pin)

        self.last_cmd_time = self.get_clock().now()
        self.stop_motors()

        # Subscribers
        self.sub = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_callback,
            10
        )
        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10
        )

        # Watchdog timer to stop motors if cmd_vel stream stops
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)

        self.get_logger().info(
            f'Motor node started on chip {self.gpio_chip}: '
            f'Left(ENA={self.ena_pin}, IN1={self.in1_pin}, IN2={self.in2_pin}), '
            f'Right(ENB={self.enb_pin}, IN3={self.in3_pin}, IN4={self.in4_pin}), '
            f'Subscribed to {cmd_vel_topic}, IMU heading lock={self.enable_imu_heading_lock}'
        )

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_yaw_rate = float(msg.angular_velocity.z)

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def stop_motors(self):
        self.target_yaw = None
        lgpio.gpio_write(self.h, self.in1_pin, 0)
        lgpio.gpio_write(self.h, self.in2_pin, 0)
        lgpio.gpio_write(self.h, self.in3_pin, 0)
        lgpio.gpio_write(self.h, self.in4_pin, 0)

        lgpio.tx_pwm(self.h, self.ena_pin, self.pwm_frequency, 0)
        lgpio.tx_pwm(self.h, self.enb_pin, self.pwm_frequency, 0)

    def watchdog_check(self):
        if self.cmd_timeout_sec <= 0.0:
            return
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout_sec:
            self.stop_motors()

    def _velocity_to_pwm(self, vel):
        """Map velocity (m/s) to PWM duty cycle (0-100) taking into account minimum PWM threshold."""
        abs_v = abs(vel)
        if abs_v < 0.001:
            return 0
        ratio = min(abs_v / max(self.max_linear_speed, 0.01), 1.0)
        pwm = self.min_pwm + ratio * (self.max_pwm - self.min_pwm)
        return int(min(max(pwm, 0), self.max_pwm))

    def cmd_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        linear = msg.linear.x * self.linear_scale
        angular = msg.angular.z * self.angular_scale

        # Active IMU Heading Lock: Prevents caster wheel drag or motor asymmetry from turning during forward motion
        if self.enable_imu_heading_lock and self.current_yaw is not None:
            is_driving_straight = abs(msg.angular.z) < 0.05 and abs(msg.linear.x) > 0.01
            if is_driving_straight:
                if self.target_yaw is None:
                    self.target_yaw = self.current_yaw
                yaw_error = self._normalize_angle(self.target_yaw - self.current_yaw)
                correction = (self.heading_kp * yaw_error) - (self.heading_kd * self.current_yaw_rate)
                correction = max(min(correction, self.max_heading_correction), -self.max_heading_correction)
                angular += correction
            else:
                self.target_yaw = None
        else:
            self.target_yaw = None

        # Differential drive kinematics
        # v_left = v - omega * (L / 2)
        # v_right = v + omega * (L / 2)
        half_base = self.wheel_base / 2.0
        v_left = linear - (angular * half_base)
        v_right = linear + (angular * half_base)

        if self.left_invert:
            v_left = -v_left
        if self.right_invert:
            v_right = -v_right

        # Left motor direction (IN1=0, IN2=1 is Forward)
        if v_left > 0.005:
            lgpio.gpio_write(self.h, self.in1_pin, 0)
            lgpio.gpio_write(self.h, self.in2_pin, 1)
        elif v_left < -0.005:
            lgpio.gpio_write(self.h, self.in1_pin, 1)
            lgpio.gpio_write(self.h, self.in2_pin, 0)
        else:
            lgpio.gpio_write(self.h, self.in1_pin, 0)
            lgpio.gpio_write(self.h, self.in2_pin, 0)

        # Right motor direction (IN3=0, IN4=1 is Forward)
        if v_right > 0.005:
            lgpio.gpio_write(self.h, self.in3_pin, 0)
            lgpio.gpio_write(self.h, self.in4_pin, 1)
        elif v_right < -0.005:
            lgpio.gpio_write(self.h, self.in3_pin, 1)
            lgpio.gpio_write(self.h, self.in4_pin, 0)
        else:
            lgpio.gpio_write(self.h, self.in3_pin, 0)
            lgpio.gpio_write(self.h, self.in4_pin, 0)

        pwm_left = self._velocity_to_pwm(v_left)
        pwm_right = self._velocity_to_pwm(v_right)
        if pwm_left > 0:
            pwm_left = int(min(pwm_left * self.left_speed_trim, self.max_pwm))
        if pwm_right > 0:
            pwm_right = int(min(pwm_right * self.right_speed_trim, self.max_pwm))

        lgpio.tx_pwm(self.h, self.ena_pin, self.pwm_frequency, pwm_left)
        lgpio.tx_pwm(self.h, self.enb_pin, self.pwm_frequency, pwm_right)

        self.get_logger().info(
            f'cmd_vel: vx={linear:.2f}, wz={angular:.2f} -> PWM [L:{pwm_left}, R:{pwm_right}]',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        try:
            lgpio.gpiochip_close(node.h)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
