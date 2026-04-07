#!/usr/bin/env python3
"""
Calibrate the MPU6050 for a ground robot.

This script collects gyroscope and accelerometer samples while the robot
is resting level on the floor and stores the resulting offsets in JSON.
"""

import json
import os
import sys
import time

from mpu6050 import mpu6050
import numpy as np


def collect_samples(sensor, duration=5, name=''):
    """Collect IMU samples for the requested duration."""
    print(f'\n{name} - Collecting {duration}s of data...')
    samples_per_sec = 100
    total_samples = duration * samples_per_sec

    accel_data = {'x': [], 'y': [], 'z': []}
    gyro_data = {'x': [], 'y': [], 'z': []}

    start = time.time()
    sample_count = 0

    while sample_count < total_samples:
        accel = sensor.get_accel_data()
        gyro = sensor.get_gyro_data()

        accel_data['x'].append(accel['x'])
        accel_data['y'].append(accel['y'])
        accel_data['z'].append(accel['z'])

        gyro_data['x'].append(gyro['x'])
        gyro_data['y'].append(gyro['y'])
        gyro_data['z'].append(gyro['z'])

        sample_count += 1
        time.sleep(0.01)

        if sample_count % 50 == 0:
            progress = (time.time() - start) / duration * 100
            print(f'  Progress: {progress:.1f}%', end='\r')

    print(f'  Collected {total_samples} samples')
    return accel_data, gyro_data


def calibrate_gyro(sensor):
    """Calibrate the gyroscope while the robot is still."""
    print('\n' + '=' * 60)
    print('GYROSCOPE CALIBRATION')
    print('=' * 60)
    print('\nSETUP:')
    print('   - Place robot on a flat level surface')
    print('   - Robot must stay perfectly still')
    print('   - IMU should be flat on top of the robot')
    print('\nThis calibrates gyro drift while the robot is stationary.')
    print('Waiting 3 seconds before starting collection...')
    time.sleep(3)

    _, gyro_data = collect_samples(sensor, duration=5, name='Gyro calibration')

    gx_offset = np.mean(gyro_data['x'])
    gy_offset = np.mean(gyro_data['y'])
    gz_offset = np.mean(gyro_data['z'])

    print('\nGyro offsets (deg/s) to correct:')
    print(f'  X (roll):   {gx_offset:8.4f}')
    print(f'  Y (pitch):  {gy_offset:8.4f}')
    print(f'  Z (yaw):    {gz_offset:8.4f}')

    return gx_offset, gy_offset, gz_offset


def calibrate_accel_simple(sensor):
    """Calibrate the accelerometer while the robot is level."""
    print('\n' + '=' * 60)
    print('ACCELEROMETER CALIBRATION')
    print('=' * 60)
    print('\nGROUND ROBOT MODE:')
    print('   - Place robot on a flat level surface')
    print('   - Robot should sit naturally with wheels down')
    print('   - No flipping, tilting, or rotating is needed')
    print('\nWaiting 3 seconds before starting collection...')
    time.sleep(3)

    accel_data, _ = collect_samples(sensor, duration=5, name='Accel calibration')

    ax_offset = np.mean(accel_data['x'])
    ay_offset = np.mean(accel_data['y'])
    az_offset = np.mean(accel_data['z']) - 9.81

    print('\nAccelerometer offsets (m/s^2):')
    print(f'  X (left/right):  {ax_offset:8.4f}')
    print(f'  Y (forward/back):{ay_offset:8.4f}')
    print(f'  Z (up/down):     {az_offset:8.4f}')

    return ax_offset, ay_offset, az_offset


def save_calibration(
    filepath,
    gx_offset,
    gy_offset,
    gz_offset,
    ax_offset,
    ay_offset,
    az_offset,
):
    """Save calibration values to a JSON file."""
    os.makedirs(os.path.dirname(filepath), exist_ok=True)

    cal_data = {
        'gx_offset': float(gx_offset),
        'gy_offset': float(gy_offset),
        'gz_offset': float(gz_offset),
        'ax_offset': float(ax_offset),
        'ay_offset': float(ay_offset),
        'az_offset': float(az_offset),
        'timestamp': time.time(),
    }

    with open(filepath, 'w', encoding='utf-8') as stream:
        json.dump(cal_data, stream, indent=2)

    print(f'\nCalibration saved to: {filepath}')


def main():
    print('=' * 60)
    print('IMU CALIBRATION - GROUND ROBOT')
    print('=' * 60)
    print('\nOptimized for ground robots that move on flat surfaces.')
    print('(Wheels down, no flipping or tilting.)')

    try:
        sensor = mpu6050(0x68)
        print('\nMPU6050 sensor detected at address 0x68')
    except Exception as exc:
        print(f'\nFailed to connect to MPU6050: {exc}')
        print('Make sure the sensor is connected and I2C is enabled.')
        sys.exit(1)

    print('\n' + '=' * 60)
    print('STEP 1: Gyroscope Calibration')
    print('=' * 60)

    gx_offset, gy_offset, gz_offset = calibrate_gyro(sensor)

    print('\n' + '=' * 60)
    print('STEP 2: Accelerometer Calibration')
    print('=' * 60)

    ax_offset, ay_offset, az_offset = calibrate_accel_simple(sensor)

    # Save calibration
    calibration_file = os.path.expanduser('~/.ros2/imu_calibration.json')
    save_calibration(
        calibration_file,
        gx_offset,
        gy_offset,
        gz_offset,
        ax_offset,
        ay_offset,
        az_offset,
    )

    print('\n' + '=' * 60)
    print('Calibration complete.')
    print('=' * 60)
    print('The IMU node will use these calibration values on next startup.')


if __name__ == '__main__':
    main()
