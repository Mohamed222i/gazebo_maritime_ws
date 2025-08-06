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
