# Differential Drive Robot – ROS 2 Localization 

> Building, simulating, and fusing indivisual odometry/pose for a differential-drive robot in a simulated Indoor Gazebo Environment, with EKF fusion.

---

## Project Overview
- **Goal:** _Explore odometry from indivisual sensors, conduct drift analysis from each of them, fuse the pose from different sensors for localizing the robot and conduct different analysis._ 
- **Robot type:** Differential-drive (“diffbot”)
- **Major dimensions:**
  - Wheel radius: _0.05 meters_
  - Wheel separation: _0.3 meters_
  - Base length / width / height: _0.5/0.3/0.1 meters_
- **Kinematics:** Unicycle model → wheel velocity commands.

---

## World & Simulation
- World file: [`apartment.world`](src/my_simulation/worlds/apartment.world)  
  _Its an indoor apartment with an cross-section of 7x7 meters with multiple static objects (Sofa, Table, Chair and Box) laying around. This room has a door of 1 meter length and all objects are cuboid shape for simplicity._
- Physics: Gazebo Classic / Ignition
- Controllers: [`diffbot_controllers.yaml`](src/my_simulation/config/diffbot_controllers.yaml)

---

## Sensors & Topics
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

## Packages
| Package | Purpose |
|---------|---------|
| **my_robot_description** | URDF, materials, RViz config |
| **my_simulation** | Gazebo world, controllers, launch scripts for static/in-motion Rviz and Gazebo visualization |
| **my_odometry** | Covert indivisual sensor reading (Encoder, IMU, Lidar) to estimated pose, Send markers for additional visualization in Rviz |
| **my_fusion** | EKF fusion of estimated Encoder + IMU poses, Corresponding marker visualization, Bag file for post analysis, Plot pose from each sensor along with fused pose in a post python script |

---

## Setup & Build
```bash
cd ~/ros2_ws

# delete only if absolute necessary, orelse skip this step
rm -rf build install log [Optional]

# make post analysis script executable
chmod +x src/my_simulation/scripts/velocity_driver.py

# Build workspace
colcon build --symlink-install
source /opt/ros/foxy/setup.bash
source install/setup.bash

# 1. View robot model
ros2 launch my_robot_description display.launch.py

# 2. Start simulation
ros2 launch my_simulation sim.launch.py

# 3. Run odometry
ros2 launch my_odometry odom.launch.py

# 4. Start EKF fusion
ros2 launch my_fusion fusion.launch.py

# 5. To drive the Robot manually in our Environment while Fusion / Odometry node are running. In a 2nd terminal -
cd ~/ros2_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_cont/cmd_vel_unstamped

# 6. [Optional - Example] Running post analysis python script to plot poses from Bag file, After running EKF fusion
python3 src/my_fusion/scripts/plot_odom.py install/my_fusion/share/my_fusion/bags/<path_to_latest_db3_file.db3>
i.e. python3 src/my_fusion/scripts/plot_odom.py install/my_fusion/share/my_fusion/bags/session_20250920_193145/session_20250920_193145_0.db3
 
```

## Launch Files

| File                  | Location                  | Purpose                  | Run Command |
|-----------------------|---------------------------|--------------------------|-------------|
| `display.launch.py`   | `my_robot_description/launch` | View URDF in RViz         | `ros2 launch my_robot_description display.launch.py` |
| `sim.launch.py`       | `my_simulation/launch`    | Spawn robot in `apartment.world` | `ros2 launch my_simulation sim.launch.py` |
| `sim_open_control.launch.py` | `my_simulation/launch` | Same as above + open loop control   | `ros2 launch my_simulation sim_open_control.launch.py` |
| `odom.launch.py`      | `my_odometry/launch`      | Convert Indivisual sensors to world frame + RViz Marker visualization | `ros2 launch my_odometry odom.launch.py` |
| `fusion.launch.py`    | `my_fusion/launch`        | Start EKF fusion node to fuse Encoders and IMU, Log pose with ROS Bag for post analysis with plots    | `ros2 launch my_fusion fusion.launch.py` |

## Odometry & EKF Fusion

- Odometry from **wheel encoders** → `/odom_est`  
- Odometry from IMU **angular velocity** → `/imu/data`  
- Odometry from LIDAR **Lidar scan (not used for fusion since not tuned yet)**  → `/scan`  
- `visualization_msgs/Marker` is used to plot the estimated pose from wheel encoders in RViz (with Global frame set to 'odom'). `/odom_points`  
- EKF fuses both sources to estimate robot pose → `/odom_fused`  
- `visualization_msgs/Marker` is used to plot the fused trajectory in RViz (with Global frame set to 'odom'). `/fused_points`  
- Through our implementation, we are only trusting IMU sensor pose for yaw calculation.  
- Currently, the implementation doesn't account for slip. It means the fused pose will keep growing even when it gets stucked/blocked. This is planned for future implementation. 
---

## Rviz Configuration for visualizing Estimated Pose

- Add relevant markers (i.e. By Topic->`/odom_points` or By Topic->`/fused_points`) first into Rviz window simulation.  
- Move the Global frame to `odom`  
- As one moves the robot manually using teleops command, these green and red MARKER points will start popping up on Rviz window dipicting estimated poses.  
- One can visualize `/odom_points` after running Odometry node and `/fused_points`after running fusion node. The Rviz and Gazebo are already linked in current launch files.  

## Results and Screenshot

## Drift Analysis
