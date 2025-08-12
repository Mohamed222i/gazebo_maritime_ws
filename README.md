# 🚤 Maritime Simulation with Gazebo



To start our simulation In Terminal, we launch through these lines of codes :

```bash
source ~/.bashrc
colcon build --merge-install
source install/setup.bash
ros2 launch ros2_maritime display.launch.py
```




This repository contains a Gazebo simulation environment for the WAM-V vessel. You can control the vessel’s propellers by publishing thrust commands to specific Gazebo topics.

---
🛠️ Note: This project is currently under development to implement an Extended Kalman Filter (EKF) for sensor fusion and state estimation.

## 🚤 Starting the Simulation and Controlling the Propellers

To control the propellers in the Gazebo simulation, use the following commands in your terminal:

### Right Propeller Thrust Command

```bash
gz topic -t /model/wam-v/joint/right_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p 'data: 100.00'

```

### Left Propeller Thrust Command

```bash
gz topic -t /model/wam-v/joint/leftt_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p 'data: 100.00'
```













## Topics published

imu:

```bash
header:
  stamp:
    sec: 24161
    nanosec: 976000000
  frame_id: wam-v/base_link/imu_sensor
orientation:
  x: -0.003350875170302802
  y: -0.0005082356961467486
  z: 0.7511921750720882
  w: 0.660074866543588
orientation_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
angular_velocity:
  x: 0.0072490228195516565
  y: -0.013978321376039318
  z: -0.00033130862394111925
angular_velocity_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
linear_acceleration:
  x: -0.05599027993025965
  y: -0.04285368871600375
  z: 9.876385573243356
linear_acceleration_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
```
navsat
```bash
navsat topic:
header:
  stamp:
    sec: 24473
    nanosec: 196000000
  frame_id: wam-v/base_link/navsat_sensor
status:
  status: 0
  service: 0
latitude: -33.72270309594833
longitude: 150.67340769072672
altitude: -0.09707925003021955
position_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
position_covariance_type: 0

```

Altimeter
```bash

header:
  stamp:
    sec: 3019
    nanosec: 140000000
  frame_id: wam-v/base_link/altimeter_sensor
vertical_position: -0.01414530872930996
vertical_velocity: 0.10218945287522203
vertical_reference: 0.0
```
altimeter_pose
```bash
header:
  stamp:
    sec: 3098
    nanosec: 900000000
  frame_id: wam-v/base_link/altimeter_sensor
pose:
  pose:
    position:
      x: 0.0
      y: 0.0
      z: -0.14684648286852148
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  covariance:
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.1
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
```
magnetometer
```bash
header:
  stamp:
    sec: 3070
    nanosec: 188000000
  frame_id: wam-v/base_link/magnetometer_sensor
magnetic_field:
  x: 0.18329113533980168
  y: -0.188289766371513
  z: -0.4816333680406481
magnetic_field_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
```

magnetometer_imu
```bash
header:
  stamp:
    sec: 3150
    nanosec: 216000000
  frame_id: wam-v/base_link/magnetometer_sensor
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
orientation_covariance:
- -1.0
- 0.0
- 0.0
- 0.0
- -1.0
- 0.0
- 0.0
- 0.0
- 0.05
angular_velocity:
  x: 0.0
  y: 0.0
  z: 0.0
angular_velocity_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
linear_acceleration:
  x: 0.0
  y: 0.0
  z: 0.0
linear_acceleration_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0

```








