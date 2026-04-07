#!/usr/bin/env python3

"""Compute odometry calibration parameters from measured runs."""

import argparse
import math


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            'Estimate wheel radii and wheel base from measured straight and turn '
            'calibration runs.'
        )
    )
    parser.add_argument('--ticks-per-rev', type=int, required=True)
    parser.add_argument('--left-wheel-radius', type=float, default=0.0335)
    parser.add_argument('--right-wheel-radius', type=float, default=0.0335)
    parser.add_argument('--wheel-base', type=float, default=0.31)
    parser.add_argument(
        '--straight-distance',
        type=float,
        help='Measured straight-line travel in meters.',
    )
    parser.add_argument(
        '--straight-left-ticks',
        type=int,
        help='Left encoder ticks collected over the straight run.',
    )
    parser.add_argument(
        '--straight-right-ticks',
        type=int,
        help='Right encoder ticks collected over the straight run.',
    )
    parser.add_argument(
        '--turn-angle-deg',
        type=float,
        help='Measured robot heading change in degrees for a turn test.',
    )
    parser.add_argument(
        '--turn-left-ticks',
        type=int,
        help='Left encoder ticks collected over the turn test.',
    )
    parser.add_argument(
        '--turn-right-ticks',
        type=int,
        help='Right encoder ticks collected over the turn test.',
    )
    return parser.parse_args()


def radius_from_run(distance_m, ticks, ticks_per_rev):
    """Return wheel radius from a measured distance run."""
    if ticks == 0:
        raise ValueError('Ticks must be non-zero for straight calibration.')
    return abs(distance_m) * ticks_per_rev / (2.0 * math.pi * abs(ticks))


def wheel_base_from_turn(
    left_radius,
    right_radius,
    left_ticks,
    right_ticks,
    angle_rad,
    ticks_per_rev,
):
    """Return effective wheel base from an in-place turn calibration run."""
    if abs(angle_rad) < 1e-9:
        raise ValueError('Turn angle must be non-zero for wheel base calibration.')
    left_distance = 2.0 * math.pi * left_radius * left_ticks / ticks_per_rev
    right_distance = 2.0 * math.pi * right_radius * right_ticks / ticks_per_rev
    return abs((right_distance - left_distance) / angle_rad)


def main():
    args = parse_args()

    left_radius = args.left_wheel_radius
    right_radius = args.right_wheel_radius

    if (
        args.straight_distance is not None and
        args.straight_left_ticks is not None and
        args.straight_right_ticks is not None
    ):
        left_radius = radius_from_run(
            args.straight_distance,
            args.straight_left_ticks,
            args.ticks_per_rev,
        )
        right_radius = radius_from_run(
            args.straight_distance,
            args.straight_right_ticks,
            args.ticks_per_rev,
        )

    wheel_base = args.wheel_base
    if (
        args.turn_angle_deg is not None and
        args.turn_left_ticks is not None and
        args.turn_right_ticks is not None
    ):
        wheel_base = wheel_base_from_turn(
            left_radius,
            right_radius,
            args.turn_left_ticks,
            args.turn_right_ticks,
            math.radians(args.turn_angle_deg),
            args.ticks_per_rev,
        )

    print('Computed odometry calibration:')
    print(f'  left_wheel_radius:  {left_radius:.8f}')
    print(f'  right_wheel_radius: {right_radius:.8f}')
    print(f'  wheel_base:         {wheel_base:.8f}')
    print(f'  wheel_base_scale:   {wheel_base / args.wheel_base:.8f}')
    print()
    print('YAML snippet:')
    print('odom_node:')
    print('  ros__parameters:')
    print(f'    ticks_per_rev: {args.ticks_per_rev}')
    print(f'    left_wheel_radius: {left_radius:.8f}')
    print(f'    right_wheel_radius: {right_radius:.8f}')
    print(f'    wheel_base: {args.wheel_base:.8f}')
    print(f'    wheel_base_scale: {wheel_base / args.wheel_base:.8f}')


if __name__ == '__main__':
    main()
