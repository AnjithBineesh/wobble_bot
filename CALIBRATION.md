# Wobble Bot — Calibration & Hardware Specifications

This document records the exact physical, kinematic, sensor, and algorithmic calibration parameters for **Wobble Bot** (Raspberry Pi 5 + ROS 2 Jazzy).

---

## 1. Quick Reference: System Calibration Values

| Parameter | Calibrated Value | Notes / Purpose | Config File |
| :--- | :--- | :--- | :--- |
| **Wheel Radius** | `0.030 m` ($30\text{ mm}$) | Standard nominal wheel radius | `wobble_odometry/config/odometry.yaml` |
| **Track Width (Wheel Base)** | `0.140 m` ($140\text{ mm}$) | Measured center-to-center wheel distance | `wobble_odometry/config/odometry.yaml` |
| **Effective Wheel Base** | `0.285 m` | Compensates for tire scrub / friction during in-place turns | `wobble_odometry/config/odometry.yaml` |
| **Wheel Base Scale** | `2.035714` | $0.285 / 0.140$ effective ratio | `wobble_odometry/config/odometry.yaml` |
| **Encoder Resolution** | `1320 ticks/rev` | Full 4-state quadrature ($330\text{ pulses} \times 4$) | `wobble_odometry/config/odometry.yaml` |
| **Left / Right Distance Scale**| `1.0 / 1.0` | Direct linear scaling | `wobble_odometry/config/odometry.yaml` |
| **Min Motor PWM** | `14%` | Overcomes gearbox static friction gently | `wobble_control/config/motor.yaml` |
| **Max Motor PWM (Safety Cap)** | `20%` | Hardware speed ceiling (~2 cm/s crawl) | `wobble_control/config/motor.yaml` |
| **Linear / Angular Scale** | `0.12 / 0.12` | Scales teleop commands to safe mapping speed | `wobble_control/config/motor.yaml` |
| **Left Speed Trim Offset** | `1.02` (+2.0%) | Balances left motor gearbox resistance | `wobble_control/config/motor.yaml` |
| **Active IMU Heading Lock** | `true` ($K_p=1.2, K_d=0.08$) | Closed-loop compensation for caster wheel drag | `wobble_control/config/motor.yaml` |
| **IMU Z-Gyro Inversion** | `false` | Matches standard ROS REP-103 (CCW is $+Z$) | `wobble_imu/config/imu.yaml` |
| **Gyro Startup Deadband** | `0.5 deg/s` | Eliminates stationary odometry creep | `wobble_imu/config/imu.yaml` |
| **SLAM Map Resolution** | `0.05 m` ($5\text{ cm/cell}$) | Standard 2D occupancy grid | `wobble_slam/config/mapper_params_online_async.yaml` |
| **SLAM Travel Thresholds** | `0.05 m / 0.05 rad` | Update triggers for slow indoor crawl | `wobble_slam/config/mapper_params_online_async.yaml` |

---

## 2. Motor & Speed Control Calibration (`wobble_control`)

### Kinematics & Speed Caps
- **Ultra-Safe Crawl Limits**:
  - `min_pwm: 14` (lowest duty cycle that smoothly rotates the gearbox without motor hum or jerk).
  - `max_pwm: 20` (capped at 20% PWM to guarantee collision safety).
  - `linear_scale: 0.12` (commands map to ~1.8 cm/s forward speed).
  - `angular_scale: 0.12` (commands map to ~5 deg/s turn rate).
- **PWM Frequency**: `100 Hz` for smooth H-bridge switching via `lgpio` on Raspberry Pi 5.
- **Motor Timeout**: `0.5 s` watchdog auto-stops motors if teleop keys are released.

### Closed-Loop IMU Heading Lock (Caster Drag Correction)
Differential-drive robots with a passive caster wheel often experience small steering deflections when rolling forward. Wobble Bot solves this in real time using closed-loop IMU feedback in `motor_node.py`:
$$\Delta \omega = K_p \cdot (\theta_{\text{target}} - \theta_{\text{actual}}) - K_d \cdot \omega_z$$
- When driving straight (`msg.angular.z ≈ 0`), the node locks the current heading $\theta_{\text{target}}$ from `/imu/data`.
- If the caster wheel pulls the robot off course, the controller applies differential PWM to steer back onto the target heading.
- Heading lock automatically unlocks during intentional turning commands (`abs(msg.angular.z) >= 0.05`).

---

## 3. Wheel Encoders & Odometry (`wobble_sensor`, `wobble_odometry`)

### Hardware Pinout (`gpiochip4` on Raspberry Pi 5)
- **Left Motor**: `ENA=GPIO 12`, `IN1=GPIO 17`, `IN2=GPIO 18`
- **Right Motor**: `ENB=GPIO 13`, `IN3=GPIO 27`, `IN4=GPIO 22`
- **Left Encoder**: Channel A = `GPIO 20`, Channel B = `GPIO 21` (Direction: `+1`)
- **Right Encoder**: Channel A = `GPIO 26`, Channel B = `GPIO 19` (Direction: `-1`)

### Encoder Resolution & Math
- JGA25-370 geared motors provide **330 pulses per single-channel revolution**.
- The `encoder_node` runs full 4-state quadrature decoding on both edges:
  $$\text{Ticks per Wheel Revolution} = 330 \times 4 = 1320\text{ ticks}$$
- Linear distance per tick:
  $$\text{Distance per tick} = \frac{2 \pi \times 0.030\text{ m}}{1320} \approx 0.0001428\text{ m/tick } (0.143\text{ mm/tick})$$

---

## 4. IMU Calibration (`wobble_imu`)

### MPU-6050 Calibration Parameters (`~/.ros2/imu_calibration.json`)
The MPU-6050 is connected via I2C at address `0x68`:
- **Gyroscope Bias**:
  - `gx_offset`: `-3.715 deg/s`
  - `gy_offset`: `+0.385 deg/s`
  - `gz_offset`: `-0.703 deg/s`
- **Accelerometer Bias**:
  - `ax_offset`: `-0.132 m/s²`
  - `ay_offset`: `+0.849 m/s²`
  - `az_offset`: `+0.227 m/s²` ($a_z$ aligned to $+9.81\text{ m/s}^2$ gravity pointing UP)
- **Startup Zeroing**: A fast 150-sample dynamic calibration executes on node launch to remove thermal gyro bias.
- **REP-103 Standard Polarity**:
  - `invert_z_gyro: false`. Counter-clockwise (CCW / Left) rotation produces positive angular velocity ($+Z$).

---

## 5. Extended Kalman Filter Fusion (`wobble_ekf`)

### Decoupled Heading & Position
- **Orientation ($\text{yaw}, \omega_z$)**: Supplied solely by `/imu/data`. Wheel slip, tire scrubbing, or wheels turning in the air cannot corrupt the robot's heading.
- **Linear Position ($x, y$) & Velocity ($v_x$)**: Integrated from wheel odometry along the true IMU heading.
- **Process Noise Covariances**:
  - `process_noise_xy`: `0.001`
  - `process_noise_yaw`: `0.001`
  - `wheel_linear_velocity_noise`: `0.05`
  - `imu_yaw_noise`: `0.02`

---

## 6. SLAM & Loop Closure Tuning (`wobble_slam`)

Configured in `wobble_slam/config/mapper_params_online_async.yaml`:
- **Solver**: `solver_plugins::CeresSolver` with `HuberLoss` and `SCHUR_JACOBI` preconditioner.
- **Loop Closing**:
  - `do_loop_closing: true`
  - `loop_search_maximum_distance: 5.0 m`
  - `loop_match_minimum_chain_size: 3` (agile indoor loop detection)
  - `loop_match_minimum_response_fine: 0.35`
- **Scan Matching**:
  - `minimum_travel_distance: 0.05 m` ($5\text{ cm}$)
  - `minimum_travel_heading: 0.05 rad` ($2.8^\circ$)
  - `scan_buffer_size: 30`
- **RViz Display**:
  - Camera `Target Frame` set to `"<Fixed Frame>"` (`map`), keeping the room stationary while the robot moves inside the map.

---

## 7. How to Recalibrate (If Hardware is Changed)

### Recalibrate IMU:
Place the robot on a level, stationary surface:
```bash
source /opt/ros/jazzy/setup.bash && source ~/wobble_bot/install/setup.bash
ros2 run wobble_imu calibrate_imu
```

### Recalibrate Wheel Odometry:
Measure a straight 2.0m line and a 360-degree in-place turn:
```bash
ros2 run wobble_odometry calibrate_odometry \
  --ticks-per-rev 1320 \
  --straight-distance 2.0 \
  --straight-left-ticks <measured_ticks> \
  --straight-right-ticks <measured_ticks> \
  --turn-angle-deg 360.0 \
  --turn-left-ticks <measured_turn_ticks> \
  --turn-right-ticks <measured_turn_ticks>
```
Paste the generated values into `src/wobble_odometry/config/odometry.yaml` and rebuild.

