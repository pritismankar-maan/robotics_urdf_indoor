# Differential Drive Robot – ROS 2 Localization 

> Complete notes for the assignment: building, simulating, and fusing indivisual odometry for a differential-drive robot in a simulated Indoor Gazebo Environment, with EKF fusion.

---

## 1️⃣ Project Overview
- **Goal:** 👉 _Explore odometry from indivisual sensors, conduct drift analysis from each of them, sensor fusion for localizing robot and analysis._ 
- **Robot type:** Differential-drive (“diffbot”)
- **Major dimensions:**
  - Wheel radius: 👉 _0.05 meters_
  - Wheel separation: 👉 _0.3 meters_
  - Base length / width / height: 👉 _0.5/0.3/0.1 meters_
- **Kinematics:** Unicycle model → wheel velocity commands.

---

## 2️⃣ World & Simulation
- World file: [`apartment.world`](src/my_simulation/worlds/apartment.world)  
  👉 _Describe obstacles, walls, furniture Its an indoor apartment with an cross-section of 7x7 meters with multiple static objects (Sofa, Table, Chair and Box) laying around. This room has a door of 1 meter length and all objects are cuboid shape for simplicity._
- Physics: Gazebo Classic / Ignition
- Controllers: [`diffbot_controllers.yaml`](src/my_simulation/config/diffbot_controllers.yaml)

---

## 3️⃣ Sensors & Topics
| Sensor / Descritpion | Plugin / Source | Topics | Notes |
|--------|-----------------|--------|-------|
| LiDAR |`libgazebo_ros_ray_sensor.so` | `/scan` | 2-D ranges |
| IMU | `libgazebo_ros_imu_sensor.so` | `/imu/data` | Linear accel + angular vel |
| Wheel encoders | NA | `/dynamic_joint_states` | Integrated velocities |
| Velocity Commands | `libgazebo_ros2_control.so` | `/diff_cont/cmd_vel_unstamped` | Command vel to robot |
| Est Pose from Encoder | NA | `/odom_est` | Processed in my_odometry node |
| Est Pose from IMU | NA | `/odom_imu_est` | Processed in my_odometry node |
| Est Pose from Lidar (WIP) | NA | `/odom_lidar_est` | Processed in my_odometry node |
| Fused Pose from Encoders and IMU | NA | `/odom_fused` | Processed in my_fusion node |

---

## 4️⃣ Packages
| Package | Purpose |
|---------|---------|
| **my_robot_description** | URDF, materials, RViz config |
| **my_simulation** | Gazebo world, controllers, launch scripts for static/in-motion Rviz and Gazebo visualization |
| **my_odometry** | Wheel odometry node, Display and play around with Rviz visualization |
| **my_fusion** | EKF fusion of odom + IMU, Bag file for post analysis, Plot pose from each sensor along with fused pose |

---

## 5️⃣ Setup & Build
```bash
# Build workspace
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 1. View robot model
ros2 launch my_robot_description display.launch.py

# 2. Start simulation
ros2 launch my_simulation sim.launch.py

# 3. Run odometry
ros2 launch my_odometry odom.launch.py

# 4. Start EKF fusion
ros2 launch my_fusion fusion.launch.py

```

## 6️⃣ Launch Files

| File                  | Location                  | Purpose                  | Run Command |
|-----------------------|---------------------------|--------------------------|-------------|
| `display.launch.py`   | `my_robot_description/launch` | View URDF in RViz         | ```bash<br>ros2 launch my_robot_description display.launch.py<br>``` |
| `sim.launch.py`       | `my_simulation/launch`    | Spawn robot in `apartment.world` | ```bash<br>ros2 launch my_simulation sim.launch.py<br>``` |
| `sim_open_control.launch.py` | `my_simulation/launch` | Same as above + open loop control   | ```bash<br>ros2 launch my_simulation sim_open_control.launch.py<br>``` |
| `odom.launch.py`      | `my_odometry/launch`      | Convert Indivisual sensors to world frame + RViz Marker visualization | ```bash<br>ros2 launch my_odometry odom.launch.py<br>``` |
| `fusion.launch.py`    | `my_fusion/launch`        | Start EKF fusion node to fuse Encoders and IMU, Log pose with ROS Bag for post analysis with plots    | ```bash<br>ros2 launch my_fusion fusion.launch.py<br>``` |

## 7️⃣ Odometry & EKF Fusion

- Odometry from **wheel encoders** → `/odom_est`  
- IMU **angular velocity** → `/imu/data`  
- EKF fuses both sources to estimate robot pose → `/odom_fused`  
- `visualization_msgs/Marker` is used to plot the fused trajectory in RViz. `/odom_fused`  

---

## 📂 Repository Structure
ros2_ws/
├── src/
    ├── my_robot_description/
    ├── my_simulation/
    ├── my_odometry/
    └── my_fusion/

