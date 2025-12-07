# CHECKLIST - Navigation Deployment
Updated: 2025-11-23

## 1. Map file
- [x] `map_final.yaml` -> `map_test_3_2.pgm`
- [x] Origin: [-15, -15, 0]

## 2. AMCL config (amcl.yaml)
- [x] odom_alpha1: 0.2 -> 0.1
- [x] odom_alpha2: 0.2 -> 0.15
- [x] odom_alpha3: 0.2 -> 0.1
- [x] odom_alpha4: 0.2 -> 0.1
- [x] laser_sigma_hit: 0.2 -> 0.075
- [x] laser_likelihood_max_dist: 2.0 -> 5.0
- [x] min_particles: 100 -> 50 (nhanh hon)
- [x] max_particles: 500 -> 200 (nhanh hon)
- [x] update_min_d: 0.1 -> 0.05 (update thuong xuyen hon)
- [x] update_min_a: 0.1 -> 0.05 (update thuong xuyen hon)

## 3. Luong dieu khien
- [x] Goal (RViz) -> run_model_safe.py -> policy -> cmd_vel -> Jetson

## 4. Sync model/net.py voi training (ppo_stage2_enhanced.py)
- [x] Them orthogonal_init function
- [x] Them use_layer_norm parameter
- [x] Sua forward() return order: action, v, logprob, mean (match training)
- [x] Them layer norm trong forward(), policy(), value()
- [x] Sua run_model_safe.py: _, _, _, mean = self.policy(...)

## 5. Gioi han toc do deployment (run_model_safe.py)
- [x] MAX_LINEAR_VEL: 0.7 -> 0.4 m/s (an toan hon)
- [x] MAX_STEPS_PER_GOAL: 500 -> 800 (tang timeout vi robot cham hon)

## 6. Trajectory Logging & Visualization
- [x] trajectory_logger.py - ROS node ghi lai duong di
  - [x] Subscribe: /robot_0/amcl_pose, /robot_0/cmd_vel, /robot_0/move_base_simple/goal
  - [x] Publish: /robot_0/trajectory (nav_msgs/Path) cho RViz
  - [x] Export CSV: ~/trajectory_logs/trajectory_YYYYMMDD_HHMMSS.csv
  - [x] Auto-start khi nhan goal tu RViz
- [x] plot_trajectory_on_map.py - Script ve hinh cho bao cao
  - [x] Doc map PGM lam background (vat can, free space)
  - [x] Ve trajectory voi mau gradient (time/velocity/distance)
  - [x] Danh dau Start, End, Goal
  - [x] Hien thi statistics: duration, path length, velocity
  - [x] Export PNG/PDF
- [x] Update master_navigation.launch - them trajectory_logger node

### 6.1 Cach su dung
```bash
# 1. Chay navigation (logger tu dong chay)
roslaunch robot_controller master_navigation.launch num_robots:=1

# 2. Xem realtime trong RViz: Add Path display -> /robot_0/trajectory

# 3. Sau khi test, ve hinh:
python3 ~/thesis/deployment/src/robot_controller/scripts/plot_trajectory_on_map.py \
    --csv ~/trajectory_logs/trajectory_xxx.csv \
    --output report.png \
    --colorby time  # hoac velocity, distance
```

---

## 7. [TODO] Xu ly robot bi mac ket (Stuck Recovery)

### 7.1 Phat hien mac ket
- [ ] Kiem tra robot khong di chuyen trong N giay (position delta < threshold)
- [ ] Kiem tra robot xoay tai cho qua nhieu (angular vel cao, linear vel thap)
- [ ] Kiem tra obstacle phia truoc qua gan (LiDAR front < 0.3m)
- [ ] Dem so lan bi mac ket lien tiep

### 7.2 Hanh dong khi mac ket
- [ ] Dung robot hoan toan truoc khi di lui
- [ ] Kiem tra LiDAR phia sau truoc khi di lui (an toan)
- [ ] Di lui cham (v = -0.1 m/s) trong 1-2 giay
- [ ] Thu lai navigation sau khi thoat mac ket

### 7.3 Cac tham so can them (run_model_safe.py)
- [ ] STUCK_TIME_THRESHOLD = 3.0  # giay - thoi gian khong di chuyen
- [ ] STUCK_DISTANCE_THRESHOLD = 0.05  # m - di chuyen toi thieu
- [ ] REVERSE_SPEED = -0.1  # m/s - toc do di lui
- [ ] REVERSE_DURATION = 1.5  # giay - thoi gian di lui
- [ ] MAX_STUCK_RETRIES = 3  # so lan thu thoat mac ket

### 7.4 Logic can implement
- [ ] _check_stuck(): kiem tra robot co bi mac ket khong
- [ ] _reverse_safely(): di lui an toan (check LiDAR phia sau)
- [ ] _rotate_to_escape(): xoay de tim duong moi
- [ ] Tich hop vao main loop cua run()

---

## 8. [TODO] Tu xac dinh vi tri robot (Auto Localization)

### 8.1 Phat hien mat vi tri
- [ ] Kiem tra AMCL covariance qua lon (uncertainty cao)
- [ ] Kiem tra particle spread qua rong
- [ ] Kiem tra LiDAR scan khong match voi map

### 8.2 Hanh dong khi mat vi tri
- [ ] Dung robot tai cho
- [ ] Tang so particles tam thoi (global localization)
- [ ] Xoay tai cho 360 do de thu thap LiDAR data
- [ ] Cho AMCL converge truoc khi tiep tuc navigation

### 8.3 Cac tham so can them
- [ ] MAX_COVARIANCE_THRESHOLD = 0.5  # nguong uncertainty
- [ ] LOCALIZATION_SPIN_SPEED = 0.3  # rad/s - toc do xoay
- [ ] LOCALIZATION_TIMEOUT = 30.0  # giay - timeout cho localization

### 8.4 Service/Topic can subscribe
- [ ] /robot_0/amcl_pose (PoseWithCovarianceStamped) - lay covariance
- [ ] /robot_0/particlecloud (PoseArray) - kiem tra particle spread
- [ ] Service: /global_localization - trigger global localization
