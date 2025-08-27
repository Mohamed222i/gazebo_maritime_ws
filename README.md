# 🚤 Maritime Simulation with Gazebo



To start our simulation In Terminal, we launch through these lines of codes :


```bash
source ~/.bashrc
```
```bash
colcon build --merge-install
```
```bash
source install/setup.bash
```
```bash
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






---
# Sensor Fusion Logic
Primary/secondary assignments for each state component within the EKF:
- (x, y): GPS primary, accelerometer secondary.
- z: GPS primary, barometer secondary.
- (vx, vy): GPS Doppler primary, accelerometer secondary.
- vz: accelerometer primary, barometer secondary.
- Roll/Pitch: accelerometer primary, gyroscope secondary.
- Yaw: magnetometer primary, gyroscope secondary.






---



### Sensor Fusion Overview

| **Sensor**       | **Topic**             | **Type**                                | **Frame ID**                        | **Rate (Hz)** | **Data Used in EKF**               | **Current Covariance Status**                       | **Corrections Needed**                                      |
|------------------|------------------------|------------------------------------------|-------------------------------------|---------------|-------------------------------------|------------------------------------------------------|--------------------------------------------------------------|
| **GPS Odometry** | `/odometry/gps`       | `nav_msgs/Odometry`                     | `odom`                              | 20–25         | x, y (used in `odom0`)             | Zero pose/twist covariances, yaw = 0                | Remove yaw input; set pose & twist covariances               |
| **IMU**          | `/sensor/imu`         | `sensor_msgs/Imu`                       | `wam-v/base_link/imu_sensor`        | ~30           | Roll, pitch, yaw, ang. vel, lin. acc | Orientation has cov; ang vel & acc covariances = 0 | Add realistic noise estimates for angular velocity & accel. |
| **Magnetometer** | `/sensor/magnetometer`| `sensor_msgs/Imu` (converted)           | `wam-v/base_link/imu_sensor`        | 53–56         | Yaw (orientation) only (if used)   | Orientation cov ok; vel/acc covariances = 0         | Use only orientation, possibly downweight or filter yaw     |
| **Altimeter**    | `/sensor/altimeter`   | `geometry_msgs/PoseWithCovarianceStamped` | `wam-v/base_link/altimeter_sensor` | ~30           | z only                             | Only z covariance = 0.1; rest = 0                   | Set unused axes (x, y, rot) to large variances              |










---
# 🚀 Simulation Python du contrôle SMC pour WAM-V

Ce document contient un **script Python complet** qui :
- Implémente le modèle dynamique simplifié du WAM-V (surge + yaw),
- Extrait les **paramètres physiques** du modèle SDF,
- Implémente la **loi de commande SMC** validée par Lyapunov,
- Permet de simuler différents scénarios :
  - vitesse seule,
  - cap seul,
  - trajectoire rectiligne,
  - trajectoire circulaire,
  - trajectoire carrée,
  - suivi de trajectoire par **Line of Sight (LOS)**.

Toutes les options sont modifiables directement dans le code (`scenario = ...`).

---

## 📌 Script Python : `sim/smc_simulation.py

```python
#!/usr/bin/env python3
"""
smc_simulation.py

Simulation d'un WAM-V (modèle réduit surge + yaw) contrôlé par SMC.
Modes disponibles:
 - "fixed"   : consigne fixe (u_d, psi_d)
 - "circle"  : suivre un cercle
 - "square"  : suivre un carré (waypoints)
 - "waypoints_los": suivre une liste de waypoints via LOS (Line-Of-Sight)

Usage:
    python3 smc_simulation.py
ou éditer la section "USER PARAMETERS" ci-dessous.
"""

import numpy as np
import matplotlib.pyplot as plt

# -------------------------
# USER PARAMETERS (modifiable)
# -------------------------
params = {
    # physical params from SDF (default values extracted earlier)
    "m": 180.0,         # kg
    "Iz": 446.0,        # kg·m^2
    "d": 1.03,          # m (half distance between thrusters)
    "du": 51.3,         # N·s/m (linear surge damping)
    "dr": 400.0,        # N·m·s/rad (yaw damping)

    # SMC gains
    "ku": 200.0,        # surge switching gain (should exceed disturbance bound/m)
    "phi_u": 0.05,      # boundary-layer thickness for surge (sat)
    "kpsi": 50.0,       # yaw switching gain
    "phi_psi": 0.02,    # boundary-layer thickness for yaw (sat)
    "lambda_psi": 1.5,  # backstepping outer-loop gain for psi

    # simulation parameters
    "dt": 0.01,         # integration step [s]
    "T": 120.0,         # total sim time [s]

    # actuator limits
    "T_min": -400.0,    # N
    "T_max":  400.0,    # N

    # disturbance / environment
    "enable_disturb": True,
    "disturb_max_force": 300.0,   # N (wave/ wind longitudinal)
    "disturb_max_moment": 50.0,   # N·m (yaw disturbance)

    # trajectory mode: "fixed", "circle", "square", "waypoints_los"
    "mode": "waypoints_los",

    # trajectory specific
    "u_d_fixed": 1.5,   # m/s for fixed mode
    "psi_d_fixed": np.deg2rad(30),  # rad for fixed mode

    # circle
    "circle_center": (0.0, 0.0),
    "circle_radius": 20.0,
    "circle_speed": 0.8,  # tangential speed (m/s)

    # square (waypoints auto)
    "square_center": (0.0, 0.0),
    "square_side": 40.0,
    "square_speed": 1.0,

    # waypoints LOS
    "waypoints": [( -10, -0), (  10,  0), (10, 20), (-10,20), (-10, -0)],  # list of (x,y)
    "los_lookahead": 5.0,   # lookahead distance (m) for LOS
    "waypoint_speed": 1.0,  # target surge speed while following waypoints
    "waypoint_radius": 1.0, # distance to consider waypoint reached
}

# -------------------------
# Helper functions
# -------------------------
def sat(x):
    """Saturation-like soft sign: sat(x) = sign(x) for |x|>1, else x."""
    # We want a classical sat in [-1,1]
    return np.clip(x, -1.0, 1.0)

def wrap_angle(a):
    """Wrap angle to [-pi, pi]."""
    return (a + np.pi) % (2*np.pi) - np.pi

def saturate(val, vmin, vmax):
    return min(max(val, vmin), vmax)

# -------------------------
# Reference trajectory functions
# -------------------------
def ref_fixed(t, p):
    u_d = p["u_d_fixed"]
    psi_d = p["psi_d_fixed"]
    return u_d, psi_d, 0.0, 0.0, 0.0  # ud, psid, dud, psid_dot, psid_ddot

def ref_circle(t, p):
    cx, cy = p["circle_center"]
    R = p["circle_radius"]
    vt = p["circle_speed"]
    # angular speed on circle (signed)
    omega = vt / R
    theta = omega * t
    # desired position is (cx + R cos(theta), cy + R sin(theta)) but for heading we want tangent
    psi_d = theta + np.pi/2.0  # tangent direction
    u_d = vt
    # derivatives:
    dud = 0.0
    psid_dot = omega
    psid_ddot = 0.0
    return u_d, wrap_angle(psi_d), dud, psid_dot, psid_ddot

def ref_square(t, p):
    # generate waypoints of square and then reuse LOS to follow them (slowly)
    cx, cy = p["square_center"]
    s = p["square_side"] / 2.0
    way = [
        (cx - s, cy - s),
        (cx + s, cy - s),
        (cx + s, cy + s),
        (cx - s, cy + s),
        (cx - s, cy - s),
    ]
    # map time to waypoint index
    total_len = 4 * p["square_side"]
    # speed
    v = p["square_speed"]
    # compute traveled distance
    dist_traveled = v * t
    seg = int(dist_traveled // p["square_side"])
    seg = min(seg, 3)
    # compute local progress
    seg_progress = (dist_traveled - seg * p["square_side"]) / p["square_side"]
    x0, y0 = way[seg]
    x1, y1 = way[seg+1]
    x_ref = x0 + (x1-x0) * seg_progress
    y_ref = y0 + (y1-y0) * seg_progress
    # desired psi is along segment
    psi_d = np.arctan2(y1-y0, x1-x0)
    return v, wrap_angle(psi_d), 0.0, 0.0, 0.0

def ref_waypoints_los_factory(waypoints, lookahead, speed):
    """Return a closure that computes LOS desired heading and speed given current state.
       We will call it with (t, p, x_state) to compute psid and derivatives.
    """
    def ref(t, p, state):
        # state: [u, psi, r, x, y]
        u, psi, r, x, y = state
        # find current waypoint target (closest not reached)
        idx = ref._idx
        # advance index if within radius
        while idx < len(waypoints)-1 and np.hypot(waypoints[idx][0]-x, waypoints[idx][1]-y) < p["waypoint_radius"]:
            idx += 1
        ref._idx = idx
        # if last waypoint reached, hold
        if idx >= len(waypoints)-1 and np.hypot(waypoints[-1][0]-x, waypoints[-1][1]-y) < p["waypoint_radius"]:
            # hold final heading to last segment direction
            if len(waypoints) >= 2:
                x_next, y_next = waypoints[-1]
                x_prev, y_prev = waypoints[-2]
                psi_d = np.arctan2(y_next - y_prev, x_next - x_prev)
            else:
                psi_d = 0.0
            u_d = 0.0
            return u_d, wrap_angle(psi_d), 0.0, 0.0, 0.0

        # current path segment is from wp[idx] to wp[idx+1]
        wpA = np.array(waypoints[idx])
        wpB = np.array(waypoints[idx+1])
        # projected point along segment using LOS lookahead
        delta = wpB - wpA
        seg_len = np.linalg.norm(delta)
        if seg_len < 1e-6:
            # degenerate segment
            psi_d = 0.0
            u_d = speed
            return u_d, psi_d, 0.0, 0.0, 0.0

        # compute along-track parameter of projection of current pos onto segment
        ap = np.array([x, y]) - wpA
        tproj = np.dot(ap, delta) / (seg_len**2)
        # choose lookahead point at tproj + La/seg_len
        t_look = tproj + lookahead/seg_len
        t_look = np.clip(t_look, 0.0, 1.0)
        look_point = wpA + t_look * delta
        # desired heading to lookahead point
        psi_d = np.arctan2(look_point[1] - y, look_point[0] - x)
        u_d = speed
        # derivatives approximated zero (we'll assume slowly varying)
        return u_d, wrap_angle(psi_d), 0.0, 0.0, 0.0
    ref._idx = 0
    return ref

# -------------------------
# Dynamics and Controller
# -------------------------
def dynamics_dot(state, TL, TR, p, dist_force=0.0, dist_moment=0.0):
    """
    state: [u, psi, r, x, y]
    TL, TR: thrusts (N)
    returns derivative of state
    """
    m = p["m"]; Iz = p["Iz"]; d = p["d"]; du = p["du"]; dr = p["dr"]
    u, psi, r, x, y = state
    X = TL + TR
    N = d * (TR - TL)
    # external disturbances
    X_ext = dist_force
    N_ext = dist_moment
    u_dot = (X - du * u + X_ext) / m
    r_dot = (N - dr * r + N_ext) / Iz
    psi_dot = r
    x_dot = u * np.cos(psi)
    y_dot = u * np.sin(psi)
    return np.array([u_dot, psi_dot, r_dot, x_dot, y_dot])

def smc_control(state, ref_signals, p):
    """
    Compute TL, TR using SMC law described in the LaTeX proof.
    ref_signals: (u_d, psi_d, dud, psid_dot, psid_ddot)
    """
    m = p["m"]; Iz = p["Iz"]; d = p["d"]; du = p["du"]; dr = p["dr"]
    ku = p["ku"]; phi_u = p["phi_u"]
    kpsi = p["kpsi"]; phi_psi = p["phi_psi"]; lambda_psi = p["lambda_psi"]

    u, psi, r, x, y = state
    u_d, psi_d, dud, psid_dot, psid_ddot = ref_signals

    # SURGE SMC (degree 1) : s_u = e_u
    e_u = u - u_d
    s_u = e_u
    # control X  (eq. X = m(ud_dot - k sat(s/phi)) + d_u u)
    # Here we use dud (time derivative of ud) as provided (often 0)
    X = m * (dud - ku * sat(s_u / phi_u)) + du * u

    # YAW control via backstepping:
    # r_d = psid_dot - lambda * e_psi
    e_psi = wrap_angle(psi - psi_d)
    r_d = psid_dot - lambda_psi * e_psi
    # derivative of r_d:
    # dot r_d = psid_ddot - lambda*(r - psid_dot)
    r_d_dot = psid_ddot - lambda_psi * (r - psid_dot)

    # surface on r:
    s_psi = r - r_d

    # control moment N
    N = Iz * (r_d_dot - kpsi * sat(s_psi / phi_psi)) + dr * r

    # convert to TL, TR
    TL = 0.5 * (X - N / d)
    TR = 0.5 * (X + N / d)

    # saturate thrusters
    TL = saturate(TL, p["T_min"], p["T_max"])
    TR = saturate(TR, p["T_min"], p["T_max"])

    # return thrusters and diagnostic terms
    diagnostics = {
        "e_u": e_u, "s_u": s_u,
        "e_psi": e_psi, "r_d": r_d, "s_psi": s_psi,
        "X": X, "N": N
    }
    return TL, TR, diagnostics

# -------------------------
# Integration (RK4)
# -------------------------
def rk4_step(state, TL, TR, dt, p, dist_force=0.0, dist_moment=0.0):
    f = lambda s: dynamics_dot(s, TL, TR, p, dist_force, dist_moment)
    k1 = f(state)
    k2 = f(state + 0.5*dt*k1)
    k3 = f(state + 0.5*dt*k2)
    k4 = f(state + dt*k3)
    return state + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)

# -------------------------
# Main simulation
# -------------------------
def run_sim(p):
    dt = p["dt"]
    T = p["T"]
    Nsim = int(T / dt)
    t_hist = np.zeros(Nsim)
    # initial state: [u, psi, r, x, y]
    state = np.array([0.0, 0.0, 0.0, -20.0, -5.0])  # start offset so LOS makes sense

    # choose reference handler
    mode = p["mode"]
    if mode == "fixed":
        ref_func = lambda t, st: ref_fixed(t, p)
    elif mode == "circle":
        ref_func = lambda t, st: ref_circle(t, p)
    elif mode == "square":
        ref_func = lambda t, st: ref_square(t, p)
    elif mode == "waypoints_los":
        ref_func = ref_waypoints_los_factory(p["waypoints"], p["los_lookahead"], p["waypoint_speed"])
    else:
        raise ValueError("mode unknown")

    # storage
    hist = {
        "t": [], "u": [], "psi": [], "r": [], "x": [], "y": [],
        "TL": [], "TR": [], "e_u": [], "e_psi": [], "X": [], "N": []
    }

    for k in range(Nsim):
        t = k * dt
        # compute reference signals (some refs need current state; unify API)
        if p["mode"] == "waypoints_los":
            u_d, psi_d, dud, psid_dot, psid_ddot = ref_func(t, p, state)
        else:
            u_d, psi_d, dud, psid_dot, psid_ddot = ref_func(t, p)

        ref_signals = (u_d, psi_d, dud, psid_dot, psid_ddot)

        # disturbance sample (constant over dt)
        if p["enable_disturb"]:
            # random bounded disturbance (you can change to deterministic)
            dist_force = np.random.uniform(-p["disturb_max_force"], p["disturb_max_force"])
            dist_moment = np.random.uniform(-p["disturb_max_moment"], p["disturb_max_moment"])
        else:
            dist_force = 0.0
            dist_moment = 0.0

        # compute control
        TL, TR, diag = smc_control(state, ref_signals, p)

        # integrate
        state = rk4_step(state, TL, TR, dt, p, dist_force, dist_moment)

        # store
        hist["t"].append(t)
        hist["u"].append(state[0])
        hist["psi"].append(wrap_angle(state[1]))
        hist["r"].append(state[2])
        hist["x"].append(state[3])
        hist["y"].append(state[4])
        hist["TL"].append(TL)
        hist["TR"].append(TR)
        hist["e_u"].append(diag["e_u"])
        hist["e_psi"].append(diag["e_psi"])
        hist["X"].append(diag["X"])
        hist["N"].append(diag["N"])

    return hist

# -------------------------
# Plotting helpers
# -------------------------
def plot_results(hist, p):
    t = np.array(hist["t"])
    fig1, axs = plt.subplots(3, 1, figsize=(9, 10))
    axs[0].plot(t, hist["u"], label="u (actual)")
    axs[0].set_ylabel("u [m/s]")
    axs[0].grid(True)

    psi_deg = np.array(hist["psi"]) * 180/np.pi
    axs[1].plot(t, psi_deg, label="psi (deg)")
    axs[1].set_ylabel("psi [deg]")
    axs[1].grid(True)

    axs[2].plot(t, hist["TL"], label="T_L")
    axs[2].plot(t, hist["TR"], label="T_R")
    axs[2].set_ylabel("Thrust [N]")
    axs[2].grid(True)
    axs[2].legend()

    fig1.suptitle("States & Actuators")

    # errors
    fig2, ax2 = plt.subplots(2, 1, figsize=(9,6))
    ax2[0].plot(t, hist["e_u"], label="e_u")
    ax2[0].set_ylabel("e_u [m/s]")
    ax2[0].grid(True)
    ax2[1].plot(t, np.array(hist["e_psi"]) * 180/np.pi, label="e_psi (deg)")
    ax2[1].set_ylabel("e_psi [deg]")
    ax2[1].grid(True)
    fig2.suptitle("Errors")

    # trajectory
    fig3, ax3 = plt.subplots(1,1, figsize=(8,8))
    ax3.plot(hist["x"], hist["y"], label="trajectory")
    # plot waypoints if present
    if "waypoints" in p and p["mode"] == "waypoints_los":
        wps = np.array(p["waypoints"])
        ax3.plot(wps[:,0], wps[:,1], 'ro--', label="waypoints")
    if p["mode"] == "circle":
        cx, cy = p["circle_center"]
        R = p["circle_radius"]
        th = np.linspace(0, 2*np.pi, 200)
        ax3.plot(cx + R*np.cos(th), cy + R*np.sin(th), '--', label="circle ref")
    if p["mode"] == "square":
        cx, cy = p["square_center"]
        s = p["square_side"]/2.0
        sq = np.array([
            [cx-s, cy-s],
            [cx+s, cy-s],
            [cx+s, cy+s],
            [cx-s, cy+s],
            [cx-s, cy-s]
        ])
        ax3.plot(sq[:,0], sq[:,1], 'k--', label="square ref")
    ax3.set_xlabel("x [m]"); ax3.set_ylabel("y [m]")
    ax3.axis('equal'); ax3.grid(True); ax3.legend()
    fig3.suptitle("Trajectory")

    plt.show()

# -------------------------
# Entry point
# -------------------------
if __name__ == "__main__":
    print("SMC Simulation starting. Mode:", params["mode"])
    # optional: you can modify params here programmatically before run
    hist = run_sim(params)
    plot_results(hist, params)

```
## 📝 Notes & conseils d’utilisation

### Personnalisation rapide
- Modifie directement la section **USER PARAMETERS** en haut du script pour ajuster :
  - Gains SMC (`ku`, `kpsi`, `lambda_u`, `lambda_psi`)  
  - Limites des propulseurs (`T_min`, `T_max`)  
  - Mode de trajectoire (`mode`)  
  - Liste de waypoints et lookahead pour LOS  
  - Activation ou non des perturbations (`enable_disturb`)  

### Choix des gains SMC
- Les gains doivent être assez grands pour compenser les perturbations :
\[
k_u > \frac{\Delta_u}{m}, \quad k_\psi > \frac{\Delta_r}{I_z}
\]
  - Exemple : avec \(m=180\,\text{kg}\), \(\Delta_u = 300\,\text{N}\), il faut \(k_u > 1.67\), mais en pratique on choisit un gain plus élevé (ex. 200-500) pour de meilleures performances.  
  - Pour le yaw : \(k_\psi\) doit dépasser \(\Delta_r/I_z\) avec \(\Delta_r\) perturbation maximale sur le moment.

### Anti-chattering
- On utilise la fonction **sat(s/phi)** pour limiter le chattering numérique de la loi SMC.  
- Ajuste **phi_u** et **phi_psi** pour lisser le signal sans perdre la robustesse.

### Conversion commande → propulseur
- Le script calcule directement les forces **T_L** et **T_R**.  
- Dans Gazebo avec `gz-sim-thruster-system` : il faut convertir ces forces en vitesses d’hélice selon :
\[
F = \text{thrust\_coefficient} \times \rho \times n^2 \times D^4
\]
ou créer un wrapper ROS qui publie directement la commande adaptée.

### Stratégie LOS
- La stratégie **Line-of-Sight (LOS)** est simple et robuste pour suivre des waypoints.  
- Possibilités d’amélioration :
  - Filtrage du cap désiré \(\psi_d\)
  - Transition douce entre segments
  - Maintien d’altitude et priorisation de trajectoire

### Validation
1. Commence par `enable_disturb = False` pour vérifier le comportement nominal.  
2. Active ensuite `enable_disturb = True` pour tester la robustesse face aux perturbations.  

### Trucs pratiques
- Vérifie que les intégrations ne saturent pas les propulseurs.  
- Pour des trajectoires complexes, visualise la trajectoire XY avec les waypoints pour valider le suivi.  
- Ajuste `dt` pour un compromis précision / vitesse de simulation.




