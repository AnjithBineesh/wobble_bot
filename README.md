# 🤖 Wobble — Autonomous Indoor Guide Robot

**Wobble** is a differential-drive indoor guide robot powered by a **Raspberry Pi 5** running **ROS 2 Jazzy**. Equipped with an **RPLiDAR A1 360° laser scanner**, an **MPU-6050 IMU**, and **quadrature wheel encoders**, Wobble performs real-time 2D SLAM (`slam_toolbox`) with pose-graph loop closure, robust Extended Kalman Filter (EKF) localization, and smooth teleoperation.

---

## ⚡ Quick Start: Manual SLAM Mapping

To map an indoor room, open three terminal windows:

### 1️⃣ Terminal 1: Robot Hardware Bringup
Starts the motor controller, encoders, wheel odometry, IMU, EKF fusion, RPLiDAR, and SLAM Toolbox:
```bash
cd ~/wobble_bot
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch wobble_slam slam.launch.py use_rviz:=false
```

### 2️⃣ Terminal 2: RViz Visualization
Launch RViz on your PC or laptop connected to the same Wi-Fi network (or via SSH with GUI forwarding):
```bash
source /opt/ros/jazzy/setup.bash && source ~/wobble_bot/install/setup.bash
ros2 run rviz2 rviz2 -d ~/wobble_bot/src/wobble_slam/config/slam.rviz
```

### 3️⃣ Terminal 3: Keyboard Teleoperation
Control the robot's motion from your keyboard:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- **`i`**: Drive straight forward *(Closed-loop IMU heading lock automatically counters caster wheel drag)*
- **`,`**: Drive backward
- **`j` / `l`**: Turn left / right in place
- **`k`**: Stop

### 💾 4️⃣ Save Your Finished Map
When you are done exploring the space and have closed all loops:
```bash
mkdir -p ~/wobble_bot/maps
source /opt/ros/jazzy/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/wobble_bot/maps/my_room_map
```

---

## 🛡️ Key Features & Safety Mechanisms

- **🐢 Ultra-Slow Safe Crawl Speed**:
  Motor duty cycle is hardware-capped at `20% PWM` with a `0.12` software scale factor (~1.8 cm/s forward speed). The robot moves gently and safely, ensuring dense laser point accumulation and eliminating scan-matching divergence.
- **🧭 Active IMU Heading Stabilization**:
  A passive front caster wheel often introduces mechanical pull. The `motor_node` runs closed-loop PD heading control using `/imu/data` to automatically counter caster drag and keep straight forward paths rock-solid.
- **🔄 Decoupled EKF Sensor Fusion**:
  Orientation ($\text{yaw}, \omega_z$) is governed purely by the MPU-6050 gyro, preventing wheel slip or uneven floor scrub from skewing heading. Linear velocity is measured from encoders along the true heading.
- **🔁 Ceres Loop Closure Optimization**:
  `slam_toolbox` utilizes Ceres sparse Cholesky solver with Huber loss to optimize the pose graph and continuously eliminate accumulated odometry drift.

---

## 📦 Workspace Architecture

The workspace consists of modular ROS 2 packages located in `src/`:

| Package | Purpose | Highlights |
| :--- | :--- | :--- |
| **[`wobble_slam`](src/wobble_slam)** | Hardware bringup & 2D SLAM | Unified `slam.launch.py`, Ceres loop closure config, preconfigured RViz display |
| **[`wobble_control`](src/wobble_control)** | Motor driver & heading lock | `motor_node.py` via `lgpio`, speed limits, active IMU heading correction |
| **[`wobble_sensor`](src/wobble_sensor)** | Quadrature encoder reading | Full 4-state quadrature edge/poll decoding on Pi 5 `gpiochip4` |
| **[`wobble_odometry`](src/wobble_odometry)** | Differential wheel odometry | `odom_node.py`, effective wheel base calibration (`1320 ticks/rev`) |
| **[`wobble_imu`](src/wobble_imu)** | MPU-6050 6-DOF IMU driver | Complementary filter, startup dynamic zeroing, REP-103 standard $+Z$ polarity |
| **[`wobble_ekf`](src/wobble_ekf)** | Extended Kalman Filter | 5-state filter `[x, y, yaw, v, ω]` fusing `/wheel/odom` and `/imu/data` |
| **[`wobble_description`](src/wobble_description)** | Robot URDF model & simulation | 3-tier chassis Xacro model, Ignition Gazebo Fortress simulation world |

---

## 🔌 Hardware Pinout (Raspberry Pi 5 `gpiochip4`)

| Subsystem | Function | GPIO Pin | Notes |
| :--- | :--- | :--- | :--- |
| **Left Motor** | Speed (PWM ENA) | `GPIO 12` | Frequency: `100 Hz`, PWM Cap: `20%` |
| | Direction (IN1) | `GPIO 17` | Standard H-Bridge logic |
| | Direction (IN2) | `GPIO 18` | Standard H-Bridge logic |
| **Right Motor**| Speed (PWM ENB) | `GPIO 13` | Frequency: `100 Hz`, PWM Cap: `20%` |
| | Direction (IN3) | `GPIO 27` | Standard H-Bridge logic |
| | Direction (IN4) | `GPIO 22` | Standard H-Bridge logic |
| **Left Encoder**| Channel A / B | `GPIO 20`, `GPIO 21` | Pull-up enabled, Direction: `+1` |
| **Right Encoder**| Channel A / B | `GPIO 26`, `GPIO 19` | Pull-up enabled, Direction: `-1` |
| **MPU-6050 IMU**| I2C Bus | `SDA (GPIO 2)`, `SCL (GPIO 3)` | Address: `0x68` |
| **RPLiDAR A1** | 2D LiDAR | USB Serial (`/dev/ttyUSB0`) | Baudrate: `115200`, Frame: `laser` |

---

## 📐 Calibration Parameters

All measured geometry, encoder resolutions, and sensor offsets are documented in detail in:
👉 **[CALIBRATION.md](CALIBRATION.md)**

---

## 🛠️ Build & Installation

```bash
# 1. Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# 2. Clone workspace (if not already present)
git clone https://github.com/AnjithBineesh/wobble_bot.git
cd wobble_bot

# 3. Install Python dependencies
pip install numpy scipy lgpio mpu6050 ahrs

# 4. Build packages
colcon build --symlink-install

# 5. Source the workspace overlay
source install/setup.bash
```

---

## 🧪 Automated Verification Script

To perform a safe, automated 30-second verification run with real-time obstacle avoidance and SLAM pose-graph evaluation:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 src/wobble_slam/scripts/run_auto_test.py
```
This tests continuous exploration, checks for scan matching stability, and prints a diagnostics summary.
