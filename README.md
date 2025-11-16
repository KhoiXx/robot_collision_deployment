# Multi-Robot RL Navigation - Deployment Guide

**Project**: Collision Avoidance with Deep Reinforcement Learning
**Author**: Nguyễn Tấn Khôi
**Model**: PPO-trained CNN (71% train, 88% test success rate)
**Last updated**: 2025-11-16

---

## 🚀 QUICK START

### First-time deployment (1 robot):

```bash
# 1. On Master PC: Start ROS
cd ~/thesis/deployment
source devel/setup.bash
roscore

# 2. On Jetson: Start sensors + motors
roslaunch robot_controller jetson_bringup.launch robot_id:=0

# 3. On Master PC: Start navigation
roslaunch robot_controller master_navigation.launch num_robots:=1

# 4. Set initial pose in RViz (click "2D Pose Estimate")

# 5. On Master PC: Start model
python3 src/robot_controller/src/run_model_safe.py --robot_id 0

# 6. Set goal in RViz (click "2D Nav Goal") - start with 0.5m distance!
```

**Emergency stop**: `rostopic pub /robot_0/emergency_stop std_msgs/Bool true`

→ **For detailed instructions**: See [QUICK_START.md](QUICK_START.md)

---

## 📚 DOCUMENTATION

| File | Purpose | When to read |
|------|---------|--------------|
| **[QUICK_START.md](QUICK_START.md)** | Step-by-step startup commands | Every deployment session |
| **[DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md)** | Complete deployment guide | Setup, architecture, tuning |
| **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** | Bugs, fixes, debugging | When problems occur |

---

## 🏗️ ARCHITECTURE OVERVIEW

```
┌─────────────────────────────────────────────────────────────┐
│                        MASTER PC                             │
│  ┌──────────┐  ┌──────┐  ┌────────────┐  ┌──────────────┐  │
│  │ roscore  │  │ Map  │  │ AMCL (n×)  │  │ Model (n×)   │  │
│  └──────────┘  └──────┘  └────────────┘  └──────────────┘  │
│                                    ↕ WiFi                    │
└─────────────────────────────────────────────────────────────┘
                                     ↕
            ┌────────────────────────┴────────────────────────┐
            │                                                  │
    ┌───────────────┐                              ┌───────────────┐
    │  JETSON #0    │                              │  JETSON #1    │
    │  ┌─────────┐  │                              │  ┌─────────┐  │
    │  │ LiDAR   │  │                              │  │ LiDAR   │  │
    │  │ IMU     │  │                              │  │ IMU     │  │
    │  │ Motors  │  │                              │  │ Motors  │  │
    │  │ UKF     │  │                              │  │ UKF     │  │
    │  └─────────┘  │                              │  └─────────┘  │
    └───────────────┘                              └───────────────┘
       Robot 0                                         Robot 1
```

**Why distributed?**
- Jetson (4GB RAM, weak GPU) can't run model inference for multiple robots
- Master PC has powerful CPU/GPU for batch inference
- Trade-off: Network latency (10-50ms) vs computational power

---

## ⚙️ KEY PARAMETERS

### Safety settings (`run_model_safe.py`):
```python
MIN_OBSTACLE_DISTANCE = 0.25  # m - Emergency stop threshold
MAX_LINEAR_VEL = 0.7          # m/s - MUST match training!
MAX_ANGULAR_VEL = 1.0         # rad/s - MUST match training!
CONTROL_RATE = 10             # Hz - Match training frequency
```

⚠️ **CRITICAL**: Max velocities MUST match training (0.7 m/s, 1.0 rad/s). Reducing creates distribution mismatch → poor performance!

### Network setup:
```bash
# Master PC (192.168.1.100)
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_HOSTNAME=192.168.1.100

# Jetson #0 (192.168.1.101)
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_HOSTNAME=192.168.1.101
```

---

## 🎯 FILE STRUCTURE

```
deployment/
├── README.md                          ← YOU ARE HERE
├── QUICK_START.md                     ← Startup commands
├── DEPLOYMENT_GUIDE.md                ← Complete guide
├── TROUBLESHOOTING.md                 ← Bug fixes & debugging
│
├── src/robot_controller/
│   ├── src/
│   │   ├── run_model_safe.py         ← USE THIS (Master PC)
│   │   ├── robot_env.py               ← Environment wrapper
│   │   ├── latency_monitor.py         ← Monitor control latency
│   │   ├── run_robot.py               ← Motor controller (Jetson)
│   │   └── settings.py                ← Configuration
│   │
│   ├── launch/
│   │   ├── jetson_bringup.launch     ← Jetson startup
│   │   ├── master_navigation.launch  ← Master PC startup
│   │   └── amcl_single.launch        ← Helper
│   │
│   ├── config/
│   │   ├── amcl.yaml                 ← AMCL localization
│   │   └── ukf.yml                   ← Sensor fusion
│   │
│   └── policy/
│       └── cnn_modern_best_71pct.pth ← Trained model
│
└── maps/
    ├── map_final.yaml
    └── map_final.pgm
```

---

## 🔧 CRITICAL FIXES APPLIED

### 🔴 Robot ran uncontrollably ("chạy như điên")
**Root causes**:
1. `state_GT` / `speed_GT` could be `None` → crash → robot continues last cmd
2. Control loop 200 Hz (should be 10 Hz like training)
3. No safety checks

**Fix**: Use `run_model_safe.py` with:
- Sensor ready check (5s timeout)
- 10 Hz control rate
- Obstacle emergency stop (< 25cm)
- Timeout protection (50s)
- Emergency stop topic

### ⚠️ Distribution mismatch
**Problem**: Initially reduced max velocity to 0.35 m/s for "safety"
**Why wrong**: Model trained with 0.7 m/s → expects robot to go 0.7 m/s → poor performance
**Fix**: Match training exactly (0.7 m/s linear, 1.0 rad/s angular)

→ See [TROUBLESHOOTING.md](TROUBLESHOOTING.md) for details

---

## 📊 MONITORING

### Essential topics:
```bash
# Sensors (should be ~10 Hz)
rostopic hz /robot_0/scan
rostopic hz /robot_0/odometry/filtered

# Control commands
rostopic echo /robot_0/cmd_vel

# Latency (should be < 100ms)
python3 src/robot_controller/src/latency_monitor.py --robot_id 0
```

### Expected performance:
| Metric | Target | Acceptable | Poor |
|--------|--------|------------|------|
| Latency | < 50ms | 50-100ms | > 150ms |
| Success rate | 70%+ | 60-70% | < 60% |
| Control freq | 10 Hz | 5-10 Hz | < 5 Hz |

---

## ⚠️ COMMON ISSUES

### "Sensor timeout!" when starting model
→ AMCL not converged. Set initial pose in RViz, wait 5-10s.

### Robot doesn't move
```bash
rostopic echo /robot_0/cmd_vel  # Commands published?
rosnode list | grep motor       # Motor controller running?
```

### High latency (>150ms)
```bash
ping 192.168.1.101              # Should be < 5ms
# Fix: Move closer, use 5GHz WiFi, reduce control freq to 5Hz
```

### Robot navigation poor
- Check AMCL particle cloud not spread out
- Verify max velocities match training (0.7 / 1.0)
- Start with SHORT goals (0.5m) first

→ See [TROUBLESHOOTING.md](TROUBLESHOOTING.md) for complete list

---

## 📈 TESTING PROGRESSION

**Phase 1**: 1 robot, 0.5m goal, open space
**Phase 2**: 1 robot, 2-3m goal, with obstacles
**Phase 3**: 2 robots, separate areas (verify no interference)
**Phase 4**: 2 robots, crossing paths (test collision avoidance)

---

## 🔗 REFERENCES

- **Training code**: `/home/khoint/thesis/catkin_ws/rl-collision-avoidance/`
- **Thesis spec**: `specs/001-thesis-latex-docs/spec.md`
- **Original paper**: Long et al. 2018 (arXiv:1709.10082)

---

## 🆘 NEED HELP?

1. Check [QUICK_START.md](QUICK_START.md) for startup commands
2. Check [TROUBLESHOOTING.md](TROUBLESHOOTING.md) for error messages
3. Check [DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md) for architecture details

**Emergency stop**:
```bash
rostopic pub /robot_0/emergency_stop std_msgs/Bool true
# OR
pkill -f run_model_safe.py
```

---

**Status**: Ready for deployment ✅
**Last tested**: 2025-11-16
**Next step**: Deploy on real robot, monitor latency, collect data
