#!/usr/bin/env python3

import lgpio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from threading import Lock


class EncoderNode(Node):

    def __init__(self):
        super().__init__('encoder_node')

        self.declare_parameter('gpio_chip', 4)
        self.declare_parameter('left_a_pin', 20)
        self.declare_parameter('left_b_pin', 21)
        self.declare_parameter('right_a_pin', 26)
        self.declare_parameter('right_b_pin', 19)
        self.declare_parameter('left_direction', 1)
        self.declare_parameter('right_direction', 1)
        self.declare_parameter('decode_mode', 'poll')
        self.declare_parameter('poll_rate_hz', 2000.0)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('diagnostics_rate_hz', 1.0)
        self.declare_parameter('stable_samples_required', 1)
        self.declare_parameter('max_still_ticks_per_sec', 2)
        self.declare_parameter('min_transition_interval_sec', 0.0005)

        self.gpio_chip = int(self.get_parameter('gpio_chip').value)
        self.left_a_pin = int(self.get_parameter('left_a_pin').value)
        self.left_b_pin = int(self.get_parameter('left_b_pin').value)
        self.right_a_pin = int(self.get_parameter('right_a_pin').value)
        self.right_b_pin = int(self.get_parameter('right_b_pin').value)
        self.left_direction = self._direction(self.get_parameter('left_direction').value)
        self.right_direction = self._direction(self.get_parameter('right_direction').value)
        self.decode_mode = str(self.get_parameter('decode_mode').value).strip().lower()
        self.poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        diagnostics_rate_hz = float(self.get_parameter('diagnostics_rate_hz').value)
        self.stable_samples_required = max(
            1, int(self.get_parameter('stable_samples_required').value)
        )
        self.max_still_ticks_per_sec = int(
            self.get_parameter('max_still_ticks_per_sec').value
        )
        self.min_transition_interval_ns = int(
            float(self.get_parameter('min_transition_interval_sec').value) * 1e9
        )

        # publishers
        self.left_pub = self.create_publisher(Int32, '/left_encoder', 10)
        self.right_pub = self.create_publisher(Int32, '/right_encoder', 10)

        # open gpiochip
        self.h = lgpio.gpiochip_open(self.gpio_chip)
        self.state_lock = Lock()
        self.left_callbacks = []
        self.right_callbacks = []

        # configure encoder pins as edge alert sources so counts are driven by
        # real transitions instead of timer sampling.
        for pin in [self.left_a_pin, self.left_b_pin, self.right_a_pin, self.right_b_pin]:
            lgpio.gpio_claim_alert(
                self.h,
                pin,
                lgpio.BOTH_EDGES,
                lgpio.SET_PULL_UP,
            )

        # tick counters
        self.left_count = 0
        self.right_count = 0

        self.left_a_level = lgpio.gpio_read(self.h, self.left_a_pin)
        self.left_b_level = lgpio.gpio_read(self.h, self.left_b_pin)
        self.right_a_level = lgpio.gpio_read(self.h, self.right_a_pin)
        self.right_b_level = lgpio.gpio_read(self.h, self.right_b_pin)
        self.left_state = (self.left_a_level << 1) | self.left_b_level
        self.right_state = (self.right_a_level << 1) | self.right_b_level
        self.left_candidate_state = self.left_state
        self.right_candidate_state = self.right_state
        self.left_candidate_count = 0
        self.right_candidate_count = 0
        self.invalid_left_transitions = 0
        self.invalid_right_transitions = 0
        self.rejected_left_bounce = 0
        self.rejected_right_bounce = 0
        now_ns = self.get_clock().now().nanoseconds
        self.last_left_transition_ns = now_ns
        self.last_right_transition_ns = now_ns

        # Valid quadrature state transitions. Opposite direction transitions are negative.
        self.transition_delta = {
            (0b00, 0b01): 1,
            (0b01, 0b11): 1,
            (0b11, 0b10): 1,
            (0b10, 0b00): 1,
            (0b00, 0b10): -1,
            (0b10, 0b11): -1,
            (0b11, 0b01): -1,
            (0b01, 0b00): -1,
        }

        publish_period = 1.0 / publish_rate_hz if publish_rate_hz > 0.0 else 0.05
        diagnostics_period = 1.0 / diagnostics_rate_hz if diagnostics_rate_hz > 0.0 else 1.0

        if self.decode_mode == 'edge':
            self.left_callbacks = [
                lgpio.callback(self.h, self.left_a_pin, lgpio.BOTH_EDGES, self._left_edge),
                lgpio.callback(self.h, self.left_b_pin, lgpio.BOTH_EDGES, self._left_edge),
            ]
            self.right_callbacks = [
                lgpio.callback(self.h, self.right_a_pin, lgpio.BOTH_EDGES, self._right_edge),
                lgpio.callback(self.h, self.right_b_pin, lgpio.BOTH_EDGES, self._right_edge),
            ]
        elif self.decode_mode == 'poll':
            poll_period = 1.0 / self.poll_rate_hz if self.poll_rate_hz > 0.0 else 0.001
            self.create_timer(poll_period, self.read_encoders)
        else:
            raise ValueError(
                f'Unsupported decode_mode={self.decode_mode!r}. Use "edge" or "poll".'
            )

        # publish counts
        self.create_timer(publish_period, self.publish_counts)

        # diagnostics
        self.last_left_count = 0
        self.last_right_count = 0
        self.create_timer(diagnostics_period, self.log_diagnostics)

        self.get_logger().info(
            'Encoder node started with '
            f'chip={self.gpio_chip}, '
            f'left=({self.left_a_pin}, {self.left_b_pin}) dir={self.left_direction}, '
            f'right=({self.right_a_pin}, {self.right_b_pin}) dir={self.right_direction}, '
            f'decode_mode={self.decode_mode}, '
            f'min_transition_interval_sec={self.min_transition_interval_ns / 1e9:.6f}'
        )
        if self.decode_mode == 'edge' and self.stable_samples_required != 1:
            self.get_logger().warning(
                'stable_samples_required is ignored in edge-driven mode; '
                'min_transition_interval_sec now handles debounce.'
            )
        if self.decode_mode == 'edge' and self.poll_rate_hz != 2000.0:
            self.get_logger().warning(
                'poll_rate_hz is ignored in edge-driven mode and is kept only '
                'for backward-compatible configs.'
            )

    def _direction(self, value):
        return 1 if int(value) >= 0 else -1

    def _left_edge(self, chip, gpio, level, timestamp_ns):
        self._handle_edge(gpio, level, timestamp_ns, is_left=True)

    def _right_edge(self, chip, gpio, level, timestamp_ns):
        self._handle_edge(gpio, level, timestamp_ns, is_left=False)

    def _read_state(self, a_pin, b_pin):
        a = lgpio.gpio_read(self.h, a_pin)
        b = lgpio.gpio_read(self.h, b_pin)
        return (a << 1) | b

    def read_encoders(self):
        with self.state_lock:
            now_ns = self.get_clock().now().nanoseconds
            new_left_state = self._filtered_state(
                self._read_state(self.left_a_pin, self.left_b_pin),
                is_left=True,
            )
            new_right_state = self._filtered_state(
                self._read_state(self.right_a_pin, self.right_b_pin),
                is_left=False,
            )

            left_delta = self._decode_delta(
                self.left_state, new_left_state, now_ns, is_left=True
            )
            right_delta = self._decode_delta(
                self.right_state, new_right_state, now_ns, is_left=False
            )

            if left_delta is None:
                self.invalid_left_transitions += 1
            else:
                self.left_count += left_delta * self.left_direction

            if right_delta is None:
                self.invalid_right_transitions += 1
            else:
                self.right_count += right_delta * self.right_direction

            self.left_state = new_left_state
            self.right_state = new_right_state

    def _handle_edge(self, gpio, level, timestamp_ns, is_left):
        if level not in (0, 1):
            return

        with self.state_lock:
            if is_left:
                if gpio == self.left_a_pin:
                    self.left_a_level = level
                elif gpio == self.left_b_pin:
                    self.left_b_level = level
                else:
                    return
                new_state = (self.left_a_level << 1) | self.left_b_level
                delta = self._decode_delta(
                    self.left_state,
                    new_state,
                    timestamp_ns,
                    is_left=True,
                )
                if delta is None:
                    self.invalid_left_transitions += 1
                else:
                    self.left_count += delta * self.left_direction
                self.left_state = new_state
                return

            if gpio == self.right_a_pin:
                self.right_a_level = level
            elif gpio == self.right_b_pin:
                self.right_b_level = level
            else:
                return
            new_state = (self.right_a_level << 1) | self.right_b_level
            delta = self._decode_delta(
                self.right_state,
                new_state,
                timestamp_ns,
                is_left=False,
            )
            if delta is None:
                self.invalid_right_transitions += 1
            else:
                self.right_count += delta * self.right_direction
            self.right_state = new_state

    def _decode_delta(self, previous_state, current_state, now_ns, is_left):
        if previous_state == current_state:
            return 0
        delta = self.transition_delta.get((previous_state, current_state))
        if delta is None:
            return None

        last_transition_ns = (
            self.last_left_transition_ns if is_left else self.last_right_transition_ns
        )
        if now_ns - last_transition_ns < self.min_transition_interval_ns:
            if is_left:
                self.rejected_left_bounce += 1
            else:
                self.rejected_right_bounce += 1
            return 0

        if is_left:
            self.last_left_transition_ns = now_ns
        else:
            self.last_right_transition_ns = now_ns
        return delta

    def _filtered_state(self, raw_state, is_left):
        if is_left:
            stable_state = self.left_state
            candidate_state = self.left_candidate_state
            candidate_count = self.left_candidate_count
        else:
            stable_state = self.right_state
            candidate_state = self.right_candidate_state
            candidate_count = self.right_candidate_count

        if raw_state == stable_state:
            candidate_state = stable_state
            candidate_count = 0
            filtered_state = stable_state
        else:
            if raw_state != candidate_state:
                candidate_state = raw_state
                candidate_count = 1
            else:
                candidate_count += 1

            if candidate_count >= self.stable_samples_required:
                filtered_state = raw_state
                candidate_state = raw_state
                candidate_count = 0
            else:
                filtered_state = stable_state

        if is_left:
            self.left_candidate_state = candidate_state
            self.left_candidate_count = candidate_count
        else:
            self.right_candidate_state = candidate_state
            self.right_candidate_count = candidate_count

        return filtered_state

    def publish_counts(self):
        left_msg = Int32()
        right_msg = Int32()

        with self.state_lock:
            left_msg.data = self.left_count
            right_msg.data = self.right_count

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

    def log_diagnostics(self):
        """Log encoder activity for debugging."""
        with self.state_lock:
            left_count = self.left_count
            right_count = self.right_count
            invalid_left_transitions = self.invalid_left_transitions
            invalid_right_transitions = self.invalid_right_transitions
            rejected_left_bounce = self.rejected_left_bounce
            rejected_right_bounce = self.rejected_right_bounce

        left_delta = left_count - self.last_left_count
        right_delta = right_count - self.last_right_count
        left_is_still_noisy = abs(left_delta) > self.max_still_ticks_per_sec
        right_is_still_noisy = abs(right_delta) > self.max_still_ticks_per_sec

        self.get_logger().info(
            'Encoder activity - '
            f'Left: {left_delta} ticks/sec, Right: {right_delta} ticks/sec '
            f'(Total L:{left_count} R:{right_count}, '
            f'Invalid L:{invalid_left_transitions} '
            f'R:{invalid_right_transitions}, '
            f'Bounce L:{rejected_left_bounce} '
            f'R:{rejected_right_bounce})'
        )

        if left_is_still_noisy or right_is_still_noisy:
            self.get_logger().warning(
                'Encoder drift detected while the robot should be still '
                f'(left_delta={left_delta}, right_delta={right_delta}).'
            )

        self.last_left_count = left_count
        self.last_right_count = right_count

    def shutdown(self):
        for callback in self.left_callbacks + self.right_callbacks:
            callback.cancel()
        if self.h is not None:
            lgpio.gpiochip_close(self.h)
            self.h = None


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
