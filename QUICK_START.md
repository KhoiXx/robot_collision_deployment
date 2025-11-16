# Quick Start - Deployment Commands

**Goal**: Start robot navigation in < 5 minutes

---

## 🔌 ONE-TIME NETWORK SETUP

### Master PC (192.168.1.100):
```bash
echo 'export ROS_MASTER_URI=http://192.168.1.100:11311' >> ~/.bashrc
echo 'export ROS_HOSTNAME=192.168.1.100' >> ~/.bashrc
echo 'export ROS_IP=192.168.1.100' >> ~/.bashrc
source ~/.bashrc
```

### Jetson #0 (192.168.1.101):
```bash
echo 'export ROS_MASTER_URI=http://192.168.1.100:11311' >> ~/.bashrc
echo 'export ROS_HOSTNAME=192.168.1.101' >> ~/.bashrc
echo 'export ROS_IP=192.168.1.101' >> ~/.bashrc
source ~/.bashrc
```

---

## 🚀 STARTUP (1 Robot)

### Terminal 1 - Master PC: ROS Master
```bash
cd ~/thesis/deployment
source devel/setup.bash
roscore
```

### Terminal 2 - Jetson: Sensors + Motors
```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch robot_controller jetson_bringup.launch robot_id:=0
```

**✅ Expected**: LiDAR ~10 Hz, IMU ~100 Hz, motor controller ready

### Terminal 3 - Master PC: Navigation
```bash
cd ~/thesis/deployment
source devel/setup.bash
roslaunch robot_controller master_navigation.launch num_robots:=1
```

**✅ Expected**: Map loaded, AMCL started, RViz opens

### Terminal 4 - Master PC: Set Initial Pose

**In RViz**:
1. Click "2D Pose Estimate"
2. Click robot's current position on map
3. Drag to set orientation

**OR via command**:
```bash
rostopic pub /robot_0/initialpose geometry_msgs/PoseWithCovarianceStamped '{
  header: {frame_id: "map"},
  pose: {
    pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}},
    covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.068]
  }
}' --once
```

**✅ Verify AMCL converged**:
```bash
rostopic echo /robot_0/amcl_pose -n 1
# Check covariance < 0.5
```

### Terminal 5 - Master PC: Start Model
```bash
cd ~/thesis/deployment/src/robot_controller/src
python3 run_model_safe.py --robot_id 0
```

**✅ Expected**:
```
====================================================
Loading Trained Model (SAFE MODE)
====================================================
Model loaded: .../cnn_modern_best_71pct.pth
Waiting for sensor data...
✅ All sensors ready!
====================================================
robot_0_model_safe node READY
```

### Terminal 6 - Master PC: Set Goal

**In RViz**:
1. Click "2D Nav Goal"
2. Click destination (start with 0.5-1m distance!)
3. Drag to set final orientation

**OR via command**:
```bash
# Go 0.5m forward
rostopic pub /robot_0/move_base_simple/goal geometry_msgs/PoseStamped '{
  header: {frame_id: "map"},
  pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
}' --once
```

**✅ Robot should start moving!**

---

## 🚀 STARTUP (2 Robots)

Same as above, but:

**Jetson #1**:
```bash
roslaunch robot_controller jetson_bringup.launch robot_id:=1
```

**Master navigation**:
```bash
roslaunch robot_controller master_navigation.launch num_robots:=2
```

**Master model (Terminal 5)**:
```bash
python3 run_model_safe.py --robot_id 0
```

**Master model (Terminal 6)**:
```bash
python3 run_model_safe.py --robot_id 1
```

**Set goals** for both robots via RViz or commands.

---

## 📊 MONITORING

### Terminal 7 - Master PC: Latency Monitor
```bash
cd ~/thesis/deployment/src/robot_controller/src
python3 latency_monitor.py --robot_id 0

# For robot 1:
# python3 latency_monitor.py --robot_id 1
```

**✅ Expected**: Latency < 100ms

### Check Topics
```bash
# Sensors OK?
rostopic hz /robot_0/scan               # ~10 Hz
rostopic hz /robot_0/odometry/filtered  # ~10 Hz

# Commands published?
rostopic echo /robot_0/cmd_vel

# Latency?
rostopic echo /robot_0/control_latency
```

---

## 🛑 SHUTDOWN

```bash
# 1. Stop model (Ctrl+C in Terminals 5, 6)
# 2. Stop navigation (Ctrl+C in Terminal 3)
# 3. Stop Jetson (Ctrl+C on Jetson)
# 4. Stop roscore (Ctrl+C in Terminal 1)
```

---

## 🚨 EMERGENCY STOP

```bash
# Method 1: Topic
rostopic pub /robot_0/emergency_stop std_msgs/Bool "data: true"

# Method 2: Kill process
pkill -f run_model_safe.py

# Method 3: Zero velocity
rostopic pub /robot_0/cmd_vel geometry_msgs/Twist "{}" --once

# Method 4: Battery (last resort!)
```

---

## ⚠️ COMMON STARTUP ISSUES

### "Sensor timeout!"
→ AMCL not converged. Set initial pose, wait 5-10s, retry.

### Robot doesn't move
```bash
rostopic echo /robot_0/cmd_vel  # Check if commands published
rostopic list | grep robot_0    # Check if Jetson topics exist
```

### High latency
```bash
ping 192.168.1.101  # Should be < 5ms
# Move Master PC closer, use 5GHz WiFi
```

### Can't connect to roscore
```bash
# On Jetson, check:
echo $ROS_MASTER_URI  # Should be http://192.168.1.100:11311
ping 192.168.1.100    # Master PC reachable?
```

---

## 📝 TESTING CHECKLIST

**Before first run**:
- [ ] Network setup done (ROS_MASTER_URI, ROS_HOSTNAME)
- [ ] Model file exists (`policy/cnn_modern_best_71pct.pth`)
- [ ] Map file exists (`maps/map_final.yaml`)
- [ ] Tested in open space (no obstacles)

**During run**:
- [ ] All sensors publishing (scan, odom)
- [ ] AMCL converged (covariance < 0.5)
- [ ] Model loaded without errors
- [ ] Latency < 100ms
- [ ] Robot moving smoothly

**After run**:
- [ ] Robot stopped cleanly (no runaway)
- [ ] Goal reached (or timeout/emergency stop)
- [ ] No collisions

---

**For troubleshooting**: See [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
**For architecture details**: See [DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md)
