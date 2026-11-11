# Wobble Bot (`wobble_bot`)

A **ROS 2 Humble** workspace for **Wobble**, a differential-drive autonomous mobile robot with 2D LiDAR SLAM, wheel + IMU Extended Kalman Filter (EKF) sensor fusion, custom hardware drivers (GPIO motors/encoders and MPU-6050 IMU), RViz2 visualization, and Ignition Gazebo 6 (Fortress) simulation.

---

## Project Architecture

The workspace (`src/`) is split into seven `ament_python` ROS 2 packages:

| Package | Role | Key Files / Entry Points |
|---|---|---|
| [`wobble_description`](src/wobble_description) | Robot URDF/Xacro model, RViz2 config, Ignition Gazebo world, and visualization/sim launch files | `urdf/wobble.urdf.xacro`, `worlds/wobble_world.sdf`, `config/display.rviz`, `launch/display.launch.py`, `launch/sim.launch.py` |
| [`wobble_control`](src/wobble_control) | Differential-drive DC motor controller using Linux `lgpio` PWM and H-bridge direction pins | `motor_node` (`wobble_control/motor_node.py`) |
| [`wobble_sensor`](src/wobble_sensor) | Quadrature wheel encoder reader via `lgpio` supporting polling or edge-interrupt decoding | `encoder_node` (`wobble_sensor/encoder_node.py`), `config/encoder.yaml` |
| [`wobble_odometry`](src/wobble_odometry) | Differential-drive wheel odometry publisher and calibration calculator | `odom_node`, `calibrate_odometry`, `config/odometry.yaml` |
| [`wobble_imu`](src/wobble_imu) | MPU-6050 I2C IMU driver with complementary filtering and offline calibration tool | `imu_node`, `calibrate_imu`, `config/imu.yaml` |
| [`wobble_ekf`](src/wobble_ekf) | 5-state Extended Kalman Filter fusing `/wheel/odom` and `/imu/data` into `/odom` + `odom -> base_link` TF | `ekf_node` (`ekf_filter_node`), `config/ekf.yaml` |
| [`wobble_slam`](src/wobble_slam) | Full hardware robot bringup and lifecycle-managed `slam_toolbox` mapping launch | `launch/slam.launch.py`, `config/mapper_params_online_async.yaml` |

> **Note on `rplidar_ros`:** `src/rplidar_ros/` and `src/rplidar_sdk/` are listed in `.gitignore`. Install `rplidar_ros` via `apt` or clone it into `src/` before running hardware SLAM.

---

## Features

### 1. Robot Modeling & Visualization (`wobble_description`)
- **Parametric URDF/Xacro (`wobble.urdf.xacro`):**
  - 3-layer stacked wooden chassis (`base_link`: `0.32 × 0.26 m`, `second_layer`: `0.26 × 0.20 m`, `top_layer`: `0.22 × 0.18 m`, `0.12 m` layer spacing) connected by 8 metal spacer rods.
  - REP-120 `base_footprint` root link with `chassis_z` computed from wheel radius (`0.0335 m`) and under-plate motor offset.
  - Detailed under-mounted JGA25-370 style gear motors, brass couplers, encoder drums, saddle mounting clamps, differential wheels (`wheel_base`: `0.31 m`), and zero-friction front ball `caster`.
  - Top-mounted MPU-6050 (`imu_link`) on `base_link` and RPLiDAR A1 (`laser`) on `top_layer`.
- **RViz2 Display (`display.launch.py`):** Launches `robot_state_publisher`, `joint_state_publisher` / `joint_state_publisher_gui`, and `rviz2` preconfigured in `config/display.rviz` (`Fixed Frame: base_footprint`, Grid, RobotModel, TF, and LaserScan).

### 2. Ignition Gazebo 6 Simulation (`wobble_description`)
- **Custom SLAM Test Arena (`worlds/wobble_world.sdf`):** A `10 m × 10 m` walled enclosure with 4 interior partition walls, 3 cylindrical pillars, and 3 obstacle crates running with Ignition Fortress plugins (`Physics`, `UserCommands`, `SceneBroadcaster`, `Sensors` with `ogre2`).
- **Simulated Plugins & Sensors:**
  - `DiffDrive` plugin (`libignition-gazebo6-diff-drive-system.so`) consuming `/cmd_vel` and publishing `/odom` and `odom -> base_footprint` TF at `30 Hz`.
  - `JointStatePublisher` plugin (`libignition-gazebo6-joint-state-publisher-system.so`) for continuous wheel joint TFs.
  - `gpu_lidar` (`360` samples across `[-π, π]`, `0.15 m – 12.0 m` range, `10 Hz`, Gaussian noise `σ = 0.01`) bridged to `/scan`.
- **Clean Topic Bridging (`ros_gz_bridge`):** Explicit unidirectional bridge directions prevent `/tf` and topic echo loops.

### 3. Hardware Control & Quadrature Sensing (`wobble_control`, `wobble_sensor`)
- **GPIO Motor Driver (`motor_node`):** Subscribes to `/cmd_vel` (`geometry_msgs/msg/Twist`) and drives H-bridge pins via `lgpio` on `gpiochip4` (`IN1=17`, `IN2=18`, `IN3=27`, `IN4=22`, `ENA=12`, `ENB=13` at `1000 Hz` PWM, `SPEED_SCALE=0.7`, `TURN_GAIN=2.0`, `MAX_PWM=100`). Stops motors cleanly on shutdown.
- **Quadrature Encoder Decoder (`encoder_node`):**
  - Reads left (`GPIO 20, 21`) and right (`GPIO 26, 19`) A/B channels on `gpiochip4` with internal pull-ups.
  - Supports `poll` mode (default `2000 Hz` with `stable_samples_required`) and `edge` interrupt mode (`BOTH_EDGES` callbacks with `min_transition_interval_sec` debounce).
  - Publishes cumulative ticks on `/left_encoder` and `/right_encoder` (`std_msgs/msg/Int32` at `50 Hz`) and logs `1 Hz` diagnostics (tick rate, invalid transitions, bounce rejections, and idle drift warnings).

### 4. Wheel Odometry, IMU & EKF Sensor Fusion (`wobble_odometry`, `wobble_imu`, `wobble_ekf`)
- **Wheel Odometry (`odom_node`):** Integrates differential encoder ticks (`1320` ticks/rev full quadrature) with independent left/right radii and scaling factors. Publishes `/wheel/odom` (`nav_msgs/msg/Odometry`) and `/joint_states` (`sensor_msgs/msg/JointState`) at `50 Hz`, with optional `max_tick_jump` spike rejection and `publish_tf: false` (so the EKF owns `odom -> base_link`).
- **MPU-6050 IMU (`imu_node`):**
  - Connects to MPU-6050 at I2C address `0x68` and publishes `/imu/data` (`sensor_msgs/msg/Imu`) at `100 Hz` (`config/imu.yaml`).
  - Applies configurable mounting rotation (`mount_roll_deg`, `mount_pitch_deg`, `mount_yaw_deg`), exponential low-pass filtering (`accel_alpha`, `gyro_alpha`), complementary roll/pitch fusion, and startup yaw zeroing.
  - Loads bias offsets from `~/.ros2/imu_calibration.json` or auto-calibrates on first run.
- **Extended Kalman Filter (`ekf_node` / `ekf_filter_node`):**
  - Fuses 5-state kinematics `[x, y, yaw, v, ω]` using `/wheel/odom` pose & twist plus `/imu/data` `angular_velocity.z` at `50 Hz`.
  - Publishes fused `/odom` with updated covariances and broadcasts the `odom -> base_link` TF.
- **Calibration Tools:**
  - `calibrate_imu`: Guided stationary gyro (5s) and level accelerometer (5s) calibration saving `~/.ros2/imu_calibration.json`.
  - `calibrate_odometry`: CLI utility computing calibrated `left_wheel_radius`, `right_wheel_radius`, `wheel_base`, and `wheel_base_scale` from measured straight-line and in-place turn tests, outputting a ready-to-paste `odometry.yaml` snippet.

### 5. Online 2D SLAM (`wobble_slam`)
- **Unified Bringup (`slam.launch.py`):** Starts `motor_node`, `robot_state_publisher`, `encoder_node`, `odom_node`, `imu_node`, `ekf_filter_node`, `rplidar_ros` (`rplidar_a1_launch.py` with `frame_id: laser`), and `slam_toolbox` (`async_slam_toolbox_node`).
- **Automated Lifecycle Management:** Uses ROS 2 launch event handlers (`OnProcessStart` + `2.0s` timer and `OnStateTransition`) to automatically transition `/slam_toolbox` through `CONFIGURE` -> `ACTIVATE`.
- **Tuned Mapping (`mapper_params_online_async.yaml`):** Configured for `0.05 m` map resolution, `8.0 m` max laser range, scan matching, and loop closure (`do_loop_closing: true`).

---

## Dependencies & Tech Stack

### Core Stack
- **OS / Runtime:** Linux with **ROS 2 Humble** and **Python 3.10+** (hardware nodes target a board with `/dev/gpiochip4` and I2C enabled, such as a Raspberry Pi 5)
- **Middleware:** **ROS 2 Humble** (`rclpy`, `launch`, `launch_ros`, `ament_python`, `tf2_ros`, `xacro`)
- **Simulation & Visualization:** **Ignition Gazebo 6 (Fortress)** (`ros_gz_sim`, `ros_gz_bridge`) and **RViz2**
- **SLAM & LiDAR:** `slam_toolbox`, `rplidar_ros` (RPLiDAR A1)

### ROS 2 Package Dependencies
Installable via `rosdep` / `apt` (for ROS 2 Humble):
- `ros-humble-xacro`
- `ros-humble-robot-state-publisher`
- `ros-humble-joint-state-publisher`
- `ros-humble-joint-state-publisher-gui`
- `ros-humble-rviz2`
- `ros-humble-ros-gz-sim`
- `ros-humble-ros-gz-bridge`
- `ros-humble-slam-toolbox`
- `ros-humble-rplidar-ros` (or built from source in `src/rplidar_ros`)
- `ros-humble-tf2-ros`
- `ros-humble-nav-msgs` `ros-humble-sensor-msgs` `ros-humble-geometry-msgs` `ros-humble-std-msgs` `ros-humble-lifecycle-msgs`
- Optional utilities: `ros-humble-teleop-twist-keyboard`, `ros-humble-nav2-map-server`

### Python & Hardware Dependencies
Declared across the workspace `package.xml` files:
- `numpy`, `scipy`
- `lgpio` (Linux GPIO character device access for motor PWM and quadrature encoders on `gpiochip4`)
- `mpu6050` (I2C driver for MPU-6050 at `0x68`; ensure I2C is enabled on the robot SBC)
- `ahrs` (declared in `wobble_imu/package.xml`)

---

## Hardware Wiring & Pinout

| Subsystem | Interface / Pins | Config Location |
|---|---|---|
| Left Motor (H-Bridge) | `IN1=GPIO 17`, `IN2=GPIO 18`, `ENA=GPIO 12` (`gpiochip4`) | `src/wobble_control/wobble_control/motor_node.py` |
| Right Motor (H-Bridge) | `IN3=GPIO 27`, `IN4=GPIO 22`, `ENB=GPIO 13` (`gpiochip4`) | `src/wobble_control/wobble_control/motor_node.py` |
| Left Wheel Encoder | `A=GPIO 20`, `B=GPIO 21` (`gpiochip4`, pull-up) | `src/wobble_sensor/config/encoder.yaml` |
| Right Wheel Encoder | `A=GPIO 26`, `B=GPIO 19` (`gpiochip4`, pull-up) | `src/wobble_sensor/config/encoder.yaml` |
| MPU-6050 IMU | I2C Bus (`0x68`) | `src/wobble_imu/wobble_imu/imu_node.py` |
| RPLiDAR A1 | USB Serial (via `rplidar_ros`, `frame_id: laser`) | `src/wobble_slam/launch/slam.launch.py` |

---

## Setup & Build

```bash
# 1. Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# 2. Clone the repository (if not already present)
git clone https://github.com/AnjithBineesh/wobble_bot.git
cd wobble_bot

# 3. Install Python dependencies (on robot hardware)
pip install numpy scipy lgpio mpu6050 ahrs

# 4. Install ROS dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 5. Build all packages
colcon build

# 6. Source the workspace overlay
source install/setup.bash
```

Build a single package (example):
```bash
colcon build --packages-select wobble_description
```

---

## Commands Reference

### Visualization & Simulation

```bash
# Preview URDF model in RViz2 with interactive joint sliders
ros2 launch wobble_description display.launch.py

# Preview URDF without GUI sliders or without RViz2
ros2 launch wobble_description display.launch.py use_joint_state_publisher_gui:=false
ros2 launch wobble_description display.launch.py use_rviz:=false

# Launch Ignition Gazebo Fortress simulation (wobble_world.sdf)
ros2 launch wobble_description sim.launch.py

# Launch Ignition Gazebo simulation together with RViz2
ros2 launch wobble_description sim.launch.py use_rviz:=true
```

### Hardware SLAM Bringup

```bash
# Launch motors, robot_state_publisher, encoders, wheel odom, IMU, EKF, RPLiDAR A1, and slam_toolbox
ros2 launch wobble_slam slam.launch.py
```

### Teleoperation & Velocity Commands

```bash
# Keyboard teleoperation (requires ros-humble-teleop-twist-keyboard)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Publish a single velocity command directly
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Stop the robot
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### Calibration Commands

```bash
# Calibrate MPU-6050 (place robot flat and still; saves ~/.ros2/imu_calibration.json)
ros2 run wobble_imu calibrate_imu

# Compute wheel radii and wheel base from measured straight & in-place turn runs
ros2 run wobble_odometry calibrate_odometry \
  --ticks-per-rev 1320 \
  --straight-distance 2.0 \
  --straight-left-ticks 12500 \
  --straight-right-ticks 12480 \
  --turn-angle-deg 360.0 \
  --turn-left-ticks -3050 \
  --turn-right-ticks 3050
```

### Running Individual Nodes

```bash
# Motor controller
ros2 run wobble_control motor_node

# Wheel encoders
ros2 run wobble_sensor encoder_node --ros-args --params-file src/wobble_sensor/config/encoder.yaml

# Wheel odometry
ros2 run wobble_odometry odom_node --ros-args --params-file src/wobble_odometry/config/odometry.yaml

# MPU-6050 IMU
ros2 run wobble_imu imu_node --ros-args --params-file src/wobble_imu/config/imu.yaml

# EKF sensor fusion
ros2 run wobble_ekf ekf_node --ros-args --params-file src/wobble_ekf/config/ekf.yaml
```

### Inspection, Map Saving & Testing

```bash
# List active topics and inspect sensor/odometry streams
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /wheel/odom
ros2 topic echo /imu/data
ros2 topic echo /scan
ros2 topic echo /left_encoder
ros2 topic echo /right_encoder

# Inspect TF tree
ros2 run tf2_tools view_frames

# Save the SLAM occupancy grid map (requires nav2_map_server)
ros2 run nav2_map_server map_saver_cli -f ~/wobble_map

# Run package lint/unit tests
colcon test
colcon test-result --verbose
```

---

## ROS 2 Topics & TF Tree

| Topic / TF | Type | Publisher | Consumer(s) |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Teleop / Nav | `motor_node` (hardware) / Gazebo `DiffDrive` (sim) |
| `/left_encoder`, `/right_encoder` | `std_msgs/msg/Int32` | `encoder_node` | `odom_node` |
| `/wheel/odom` | `nav_msgs/msg/Odometry` | `odom_node` | `ekf_filter_node` |
| `/imu/data` | `sensor_msgs/msg/Imu` | `imu_node` | `ekf_filter_node` |
| `/odom` | `nav_msgs/msg/Odometry` | `ekf_filter_node` (hw) / Gazebo bridge (sim) | SLAM / Navigation |
| `/joint_states` | `sensor_msgs/msg/JointState` | `odom_node` (hw) / Gazebo bridge (sim) | `robot_state_publisher` |
| `/scan` | `sensor_msgs/msg/LaserScan` | `rplidar_ros` (hw) / Gazebo bridge (sim) | `slam_toolbox` / RViz2 |
| `map -> odom` | TF | `slam_toolbox` | TF tree |
| `odom -> base_link` / `base_footprint` | TF | `ekf_filter_node` (`base_link`) / Gazebo (`base_footprint`) | TF tree |
