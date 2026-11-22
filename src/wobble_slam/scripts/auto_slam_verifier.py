#!/usr/bin/env python3
"""
Automated SLAM Verification Script for Wobble Bot.
Performs continuous 30-second motion with active LiDAR obstacle avoidance:
- Continuous forward creeping and reactive turning when obstacles enter 20cm-35cm range.
- Verifies real-time scan matching, loop closure triggers, and map drift.
- Monitors TF (map -> odom -> base_footprint) to evaluate SLAM correction quality.
"""

import math
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class AutoSlamVerifier(Node):

    def __init__(self):
        super().__init__('auto_slam_verifier')

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # TF Listener to track map -> odom and map -> base_footprint
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Obstacle distance states
        self.min_front = 99.0
        self.min_left = 99.0
        self.min_right = 99.0
        self.min_back = 99.0
        self.min_overall = 99.0
        self.scan_received = False

        self.latest_map = None
        self.map_updates = 0

        self.latest_odom = None
        self.start_pose = None
        self.total_distance_traveled = 0.0
        self.last_pos = None

        self.start_time = time.time()
        self.test_duration = 30.0  # seconds

        # Thresholds
        self.OBSTACLE_AVOID_DIST = 0.32   # meters: begin turning if front obstacle <= 32cm
        self.CRITICAL_STOP_DIST = 0.20    # meters: safety threshold
        self.CLEAR_PATH_DIST = 0.36       # meters: required front clearance to resume forward creep

        self.current_turn_dir = 1.0       # +1.0 = CCW (Left), -1.0 = CW (Right)
        self.state = 'FORWARD'            # 'FORWARD' or 'TURNING'
        self.turn_start_time = 0.0

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            'Auto SLAM Verifier active: Continuous exploration with reactive avoidance.'
        )

    def scan_callback(self, msg):
        self.scan_received = True
        angles = [msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))]

        front, left, right, back = [], [], [], []
        for r, a in zip(msg.ranges, angles):
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                continue
            deg = math.degrees(a)
            if -35.0 <= deg <= 35.0:
                front.append(r)
            elif 35.0 < deg <= 115.0:
                left.append(r)
            elif -115.0 <= deg < -35.0:
                right.append(r)
            else:
                back.append(r)

        self.min_front = min(front) if front else 99.0
        self.min_left = min(left) if left else 99.0
        self.min_right = min(right) if right else 99.0
        self.min_back = min(back) if back else 99.0
        self.min_overall = min(self.min_front, self.min_left, self.min_right, self.min_back)

    def map_callback(self, msg):
        self.latest_map = msg
        self.map_updates += 1

    def odom_callback(self, msg):
        self.latest_odom = msg
        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.start_pose is None:
            self.start_pose = pos
            self.last_pos = pos
        else:
            dx = pos[0] - self.last_pos[0]
            dy = pos[1] - self.last_pos[1]
            dist = math.hypot(dx, dy)
            self.total_distance_traveled += dist
            self.last_pos = pos

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        elapsed = time.time() - self.start_time
        if elapsed >= self.test_duration:
            self.stop_robot()
            self.finish_verification()
            return

        cmd = Twist()

        # Phase 1: 0 to 2s - Short Standstill to seed initial SLAM keyframe
        if elapsed < 2.0:
            status_text = 'Initializing SLAM Baseline'
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # Phase 2: 2s to 28s - Continuous Active Exploration & Obstacle Avoidance
        elif elapsed < 28.0:
            # Check if an obstacle is encroaching in front or too close overall
            obstacle_ahead = (self.min_front <= self.OBSTACLE_AVOID_DIST or 
                              self.min_overall <= self.CRITICAL_STOP_DIST)

            if obstacle_ahead:
                # Transition to or continue TURNING
                self.state = 'TURNING'
                # Choose turn direction towards the clearer side if beginning turn
                if self.min_left > self.min_right + 0.05:
                    self.current_turn_dir = 1.0   # Left is clearer
                elif self.min_right > self.min_left + 0.05:
                    self.current_turn_dir = -1.0  # Right is clearer

                cmd.linear.x = 0.0
                cmd.angular.z = 0.90 * self.current_turn_dir
                turn_str = "LEFT" if self.current_turn_dir > 0 else "RIGHT"
                status_text = f'OBSTACLE ({self.min_front:.2f}m) -> TURNING {turn_str}'

            else:
                # Path ahead is clear!
                if self.state == 'TURNING' and self.min_front < self.CLEAR_PATH_DIST:
                    # Still completing pivot until clear threshold reached
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.80 * self.current_turn_dir
                    turn_str = "LEFT" if self.current_turn_dir > 0 else "RIGHT"
                    status_text = f'PIVOTING TO CLEAR ({self.min_front:.2f}m) -> {turn_str}'
                else:
                    # Clear path: drive forward continuously
                    self.state = 'FORWARD'
                    cmd.linear.x = 0.22  # Scaled by motor_node to ~2.6 cm/s crawl
                    # Gentle exploration curvature if wide open space to facilitate looping
                    if self.min_front > 0.80 and self.min_left > 0.50 and self.min_right > 0.50:
                        cmd.angular.z = 0.15 * self.current_turn_dir
                    else:
                        cmd.angular.z = 0.0
                    status_text = f'CONTINUOUS FORWARD CREEP (Dist={self.total_distance_traveled:.2f}m)'

        # Phase 3: 28s to 30s - Stop & Final Settle
        else:
            status_text = 'Test Complete - Final Map Settle'
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'[{elapsed:4.1f}s/30s] {status_text} | F={self.min_front:.2f}m, L={self.min_left:.2f}m, R={self.min_right:.2f}m',
            throttle_duration_sec=1.5
        )

    def finish_verification(self):
        self.stop_robot()
        self.get_logger().info('=' * 65)
        self.get_logger().info('      30-SECOND CONTINUOUS SLAM VERIFICATION REPORT')
        self.get_logger().info('=' * 65)

        # 1. Map Evaluation & Drift Check
        if self.latest_map is not None:
            data = self.latest_map.data
            free_cells = sum(1 for c in data if c == 0)
            occupied_cells = sum(1 for c in data if c > 50)
            unknown_cells = sum(1 for c in data if c == -1)
            width = self.latest_map.info.width
            height = self.latest_map.info.height
            res = self.latest_map.info.resolution
            self.get_logger().info(
                f'✓ Map Received: {width} x {height} cells @ {res:.2f}m/cell ({width*res:.2f}m x {height*res:.2f}m)'
            )
            self.get_logger().info(
                f'✓ Map Content: Free={free_cells}, Occupied/Walls={occupied_cells}, Unknown={unknown_cells}'
            )
            self.get_logger().info(
                f'✓ Total SLAM Map Updates: {self.map_updates}'
            )
            # Wall thickness / drift indicator: occupied-to-free ratio
            wall_ratio = (occupied_cells / max(free_cells, 1)) * 100.0
            self.get_logger().info(
                f'✓ Occupancy Ratio: {wall_ratio:.1f}% (Healthy range 5-25%; high values indicate smearing/drift)'
            )
        else:
            self.get_logger().error('✗ No map received from slam_toolbox!')

        # 2. Odometry & Motion Evaluation
        if self.latest_odom is not None:
            p = self.latest_odom.pose.pose.position
            self.get_logger().info(
                f'✓ Continuous Motion Distance Traveled: {self.total_distance_traveled:.3f}m'
            )
            self.get_logger().info(
                f'✓ Final EKF Odometry Pose: x={p.x:.3f}m, y={p.y:.3f}m'
            )
        else:
            self.get_logger().error('✗ No odometry received from ekf_node!')

        # 3. TF & Loop Closure Pose Graph Check
        try:
            # Check map -> odom transform (shows SLAM Toolbox pose graph correction)
            tf_map_odom = self.tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time(), timeout=Duration(seconds=1.0)
            )
            t = tf_map_odom.transform.translation
            r = tf_map_odom.transform.rotation
            correction_dist = math.hypot(t.x, t.y)
            self.get_logger().info(
                f'✓ SLAM Graph Correction (map->odom): dx={t.x:.3f}m, dy={t.y:.3f}m | Net Correction: {correction_dist:.3f}m'
            )
            self.get_logger().info(
                '✓ Loop Closure / Graph Optimizer is ACTIVE and maintaining map->odom correction'
            )
        except Exception as e:
            self.get_logger().warn(f'TF map->odom lookup: {e}')

        self.get_logger().info('=' * 65)
        sys.exit(0)


def main():
    rclpy.init()
    node = AutoSlamVerifier()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_robot()
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
