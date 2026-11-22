#!/usr/bin/env python3
"""Run SLAM launch, execute 30s verifier, and report."""

import os
import signal
import subprocess
import sys
import time

def cleanup():
    for name in ['motor_node', 'encoder_node', 'odom_node', 'imu_node', 'async_slam_toolbox_node', 'rplidar_composition', 'robot_state_publisher']:
        subprocess.run(['pkill', '-9', '-x', name], stderr=subprocess.DEVNULL)

cleanup()
time.sleep(1.0)

print("Starting wobble_slam launch...")
log_file = open("/tmp/slam_launch.log", "w")
launch_proc = subprocess.Popen(
    ["ros2", "launch", "wobble_slam", "slam.launch.py", "use_rviz:=false"],
    stdout=log_file,
    stderr=subprocess.STDOUT,
    text=True,
    preexec_fn=os.setsid
)

# Wait 8s for stack to initialize
time.sleep(8.0)

print("Starting 30-second automated SLAM verification...")
verifier_proc = subprocess.run(
    [sys.executable, "/home/squirtle_bot/wobble_bot/src/wobble_slam/scripts/auto_slam_verifier.py"]
)

print("Shutting down SLAM stack...")
try:
    os.killpg(os.getpgid(launch_proc.pid), signal.SIGTERM)
    launch_proc.wait(timeout=3)
except Exception:
    cleanup()

cleanup()
print("Automated test completed.")

