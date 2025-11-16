# Deployment Guide - Complete Reference

**Last updated**: 2025-11-16

---

## 📋 TABLE OF CONTENTS

1. [Architecture](#architecture)
2. [Network Setup](#network-setup)
3. [File Distribution](#file-distribution)
4. [Configuration](#configuration)
5. [Scaling to N Robots](#scaling-to-n-robots)
6. [Performance Tuning](#performance-tuning)
7. [Latency Monitoring](#latency-monitoring)

---

## 🏗️ ARCHITECTURE

### Distributed Multi-Robot System

```
┌─────────────────────── MASTER PC (192.168.1.100) ────────────────────────┐
│                                                                            │
│  roscore  │  Map Server  │  AMCL (N×)  │  Model Inference (N×)  │  RViz  │
│                                                                            │
└────────────────────────────────────┬───────────────────────────────────────┘
                                     │ WiFi (10-50ms)
            ┌────────────────────────┴────────────────────────┐
            │                                                  │
    ┌───────▼─────────┐                              ┌────────▼────────┐
    │ JETSON #0       │                              │ JETSON #1       │
    │ (.101)          │                              │ (.102)          │
    │                 │                              │                 │
    │ • LiDAR         │                              │ • LiDAR         │
    │ • IMU           │                              │ • IMU           │
    │ • UKF Fusion    │                              │ • UKF Fusion    │
    │ • Motor Control │                              │ • Motor Control │
    │                 │                              │                 │
    └─────────────────┘                              └─────────────────┘
       Robot 0                                          Robot 1
```

### Why Centralized Inference?

**Jetson Limitations**:
- 4GB RAM, weak GPU (Maxwell architecture)
- ARM CPU not optimized for PyTorch
- Cannot handle multiple robot inferences simultaneously

**Master PC Advantages**:
- Powerful CPU/GPU (can be desktop with NVIDIA GPU)
- Can batch inference for N robots
- 16GB+ RAM for model loading

**Trade-off**:
- **Benefit**: Computational power, batch processing
- **Cost**: Network latency 10-50ms (acceptable vs 100ms control period)

---

## 🌐 NETWORK SETUP

### IP Configuration

| Device | IP | Role |
|--------|-----------|------|
| Master PC | 192.168.1.100 | ROS Master, inference |
| Jetson #0 | 192.168.1.101 | Robot 0 sensors/motors |
| Jetson #1 | 192.168.1.102 | Robot 1 sensors/motors |

### ROS Environment Variables

**Master PC** (`~/.bashrc`):
```bash
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_HOSTNAME=192.168.1.100
export ROS_IP=192.168.1.100
```

**Jetson #0** (`~/.bashrc`):
```bash
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_HOSTNAME=192.168.1.101
export ROS_IP=192.168.1.101
```

**Jetson #1** (`~/.bashrc`):
```bash
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_HOSTNAME=192.168.1.102
export ROS_IP=192.168.1.102
```

### Verify Network

```bash
# From Master PC
ping 192.168.1.101  # Should be < 5ms
ping 192.168.1.102

# From Jetson
ping 192.168.1.100

# Test ROS connectivity (after roscore started)
rostopic list  # Should work from all devices
```

---

## 📁 FILE DISTRIBUTION

### Master PC (Needs)

```
~/thesis/deployment/
├── src/robot_controller/src/
│   ├── run_model_safe.py       ⭐ Model inference
│   ├── robot_env.py
│   ├── latency_monitor.py
│   ├── model/
│   │   ├── net.py
│   │   └── ppo.py
│   └── policy/
│       └── cnn_modern_best_71pct.pth  (7MB)
│
├── launch/
│   ├── master_navigation.launch  ⭐ Master-side
│   └── amcl_single.launch
│
├── config/
│   └── amcl.yaml
│
└── maps/
    ├── map_final.yaml
    └── map_final.pgm
```

### Jetson (Needs)

```
~/catkin_ws/src/robot_controller/
├── src/
│   ├── run_robot.py           ⭐ Motor controller
│   └── robot_control.py
│
├── launch/
│   └── jetson_bringup.launch  ⭐ Jetson-side
│
└── config/
    └── ukf.yml
```

### Jetson (Does NOT need - save space!)

- ❌ `policy/*.pth` (model files)
- ❌ `maps/` (map files)
- ❌ `run_model*.py`
- ❌ `model/` (network architecture)

### Sync Files to Jetson

```bash
# On Master PC
rsync -avz --exclude='*.pth' --exclude='maps/' \
  ~/thesis/deployment/src/ \
  jetson@192.168.1.101:~/catkin_ws/src/

# Then on Jetson
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## ⚙️ CONFIGURATION

### Critical Parameters (run_model_safe.py)

```python
# Safety
MIN_OBSTACLE_DISTANCE = 0.25  # m - Emergency stop threshold

# Velocity limits - MUST MATCH TRAINING!
MAX_LINEAR_VEL = 0.7          # m/s
MAX_ANGULAR_VEL = 1.0         # rad/s

# Control frequency
CONTROL_RATE = 10             # Hz - Match training

# Timeout
MAX_STEPS_PER_GOAL = 500      # 50 seconds at 10 Hz
```

⚠️ **CRITICAL**: Do NOT reduce MAX_LINEAR_VEL or MAX_ANGULAR_VEL below training values (0.7 / 1.0). This creates distribution mismatch → poor performance. See [TROUBLESHOOTING.md](TROUBLESHOOTING.md#distribution-mismatch).

### AMCL Configuration (config/amcl.yaml)

```yaml
# Odometry model
odom_model_type: diff
odom_alpha1: 0.2
odom_alpha2: 0.2
odom_alpha3: 0.2
odom_alpha4: 0.2

# Laser model
laser_model_type: likelihood_field
laser_max_beams: 60
laser_z_hit: 0.95

# Particles
min_particles: 100
max_particles: 500

# Update thresholds
update_min_d: 0.2    # m
update_min_a: 0.5    # rad
```

**Tuning tips**:
- Poor localization → Increase `max_particles` (500 → 1000)
- AMCL too slow → Decrease `laser_max_beams` (60 → 30)
- Jumpy localization → Increase `update_min_d/a`

### UKF Sensor Fusion (config/ukf.yml)

```yaml
odom0: /robot_0/odom          # Encoder odometry
imu0: /robot_0/imu/data       # IMU orientation

# Sensor config
odom0_config: [false, false, false,  # x, y, z
               false, false, false,  # roll, pitch, yaw
               true,  true,  false,  # vx, vy, vz
               false, false, true,   # vroll, vpitch, vyaw
               false, false, false]  # ax, ay, az

imu0_config: [false, false, false,
              false, false, true,    # yaw from IMU
              false, false, false,
              false, false, true,    # vyaw from IMU
              false, false, false]
```

---

## 📈 SCALING TO N ROBOTS

### 1 Robot → 2 Robots

**Changes needed**:

1. **Master PC** - Update launch file:
```bash
roslaunch robot_controller master_navigation.launch num_robots:=2
```

2. **Jetson #1** - Setup network and start bringup:
```bash
# Set ROS env (see Network Setup section)
roslaunch robot_controller jetson_bringup.launch robot_id:=1
```

3. **Master PC** - Start second model:
```bash
python3 run_model_safe.py --robot_id 1
```

### Scaling Considerations

| # Robots | Model Inference | AMCL Instances | Expected Latency |
|----------|-----------------|----------------|------------------|
| 1 | ~10ms | 1 | 30-80ms |
| 2 | ~15ms (batch) | 2 | 40-100ms |
| 4 | ~25ms (batch) | 4 | 60-120ms |
| 8 | ~45ms (batch) | 8 | 80-150ms |

**Bottlenecks**:
- **CPU**: AMCL localization (N instances)
- **GPU**: Model inference (can batch)
- **Network**: Bandwidth (~1 Mbps per robot)
- **Control frequency**: May need to reduce 10Hz → 5Hz for N > 4

### Batch Inference Optimization

**Current**: Sequential inference
```python
for robot_id in range(N):
    model.forward(state[robot_id])
```

**Optimized**: Batch inference (future work)
```python
states_batch = torch.cat([state[i] for i in range(N)])
actions_batch = model.forward(states_batch)  # Single forward pass!
```

**Expected speedup**: ~2-3x for N=4-8 robots

---

## 🎯 PERFORMANCE TUNING

### Expected Performance

| Metric | Simulation | Real Robot (Target) | Real Robot (Realistic) |
|--------|------------|---------------------|------------------------|
| Success rate | 88% (test) | 70-80% | 60-70% |
| Collision rate | 12% | 15-20% | 20-30% |
| Avg time to goal | ~15s | ~20s | ~25s |

**Sim-to-real gap factors**:
1. Sensor noise (LiDAR, IMU)
2. Motor response lag
3. Localization error (AMCL uncertainty)
4. Floor type, lighting, obstacles
5. Action execution gap

### Tuning Strategies

**If collision rate too high** (> 30%):
1. Increase `MIN_OBSTACLE_DISTANCE` (0.25 → 0.35 m)
2. Lower control frequency (10 → 5 Hz) for smoother control
3. Check AMCL localization accuracy
4. Verify laser scan quality

**If success rate too low** (< 60%):
1. Check AMCL covariance (should be < 0.5)
2. Verify goals are reachable
3. Increase timeout (MAX_STEPS_PER_GOAL)
4. Check for hardware issues (motor lag, encoder drift)

**If robot too slow**:
1. Verify MAX_LINEAR_VEL = 0.7 (match training!)
2. Check motor PWM limits
3. Monitor battery voltage (low battery → slow motors)

**If robot too fast/aggressive**:
- Do NOT reduce MAX_LINEAR_VEL (creates distribution mismatch!)
- Instead: Increase `MIN_OBSTACLE_DISTANCE` for earlier slowdown
- Or: Retrain model with lower max velocity

---

## 📊 LATENCY MONITORING

### Why Critical?

Training assumed ~0ms latency (simulation). Real deployment has:
- Sensor → Network → Master: 10-30ms
- AMCL localization: 10-30ms
- Model inference: 10-50ms
- Network → Jetson → Motors: 10-50ms

**Total**: 40-160ms (vs training: ~0ms)

If latency > 150ms → Model receives outdated state → poor decisions!

### How to Monitor

```bash
# Terminal 1: Run latency monitor
cd ~/thesis/deployment/src/robot_controller/src
python3 latency_monitor.py --robot_id 0

# Expected output:
# [Robot 0] Latency (ms) - avg: 45.3 ± 12.1, min: 28.5, max: 89.2, p95: 67.8
```

**Warnings**:
- ⚠️ avg > 100ms: Moderate latency
- ⚠️ avg > 150ms: High latency - reduce control freq to 5Hz
- ❌ avg > 200ms: Critical - check network/WiFi

### Diagnose High Latency

```bash
# 1. Network ping (should be < 5ms)
ping 192.168.1.101

# 2. WiFi signal (on Jetson)
ssh jetson@192.168.1.101
iwconfig wlan0 | grep Signal  # Should be > -60 dBm

# 3. Bandwidth test
# On Master: iperf3 -s
# On Jetson: iperf3 -c 192.168.1.100 -t 10
# Should be > 50 Mbps

# 4. CPU usage (on Master)
top  # Check if AMCL or model using > 80% CPU
```

### Reduce Latency

1. **Move Master PC closer** to robots (< 10m)
2. **Use 5GHz WiFi** instead of 2.4GHz
3. **Reduce num_robots** (test with 1 first)
4. **Lower control frequency** (10Hz → 5Hz in run_model_safe.py)
5. **Optimize AMCL**: Reduce `laser_max_beams`, `max_particles`

---

## 🔗 NEXT STEPS

**After reading this**:
- Setup network → [QUICK_START.md](QUICK_START.md)
- Having issues → [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
- Quick reference → [README.md](README.md)

---

**Author**: Claude AI Assistant
**Last tested**: 2025-11-16
