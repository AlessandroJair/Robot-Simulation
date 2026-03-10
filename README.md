# Comparative Study: Rocker-Bogie vs. Differential vs. Tracked Robots in Underground Mine Environments

Comparative analysis of three robotic locomotion platforms evaluating mechanical stability and SLAM performance during autonomous navigation in a simulated underground mine tunnel network.

## Overview

This project implements a complete simulation and evaluation pipeline to compare:

| Platform | Description |
|----------|-------------|
| **Rocker-Bogie** | 6-wheel passive suspension mechanism |
| **Differential Drive** | Husky A200 equivalent, 4-wheel skid-steer |
| **Tracked** | Continuous track system |

Each robot traverses a ~60m route through an underground mine environment while collecting IMU, odometry, and SLAM data for post-run comparative analysis.

### Key Findings

- Rocker-bogie achieves **54% reduction in vertical shock** vs. tracked vehicle
- Rocker-bogie achieves **lowest ATE (0.72m)** — 50% lower than Husky, 59% lower than tracked
- Rocker-bogie maintains the lowest roll angular acceleration (22.69 rad/s²)

## Project Structure

```
jounal_comparison/
├── src/
│   ├── rocker_bogie/           # 6-wheel rocker-bogie robot package
│   │   ├── urdf/               # URDF/XACRO robot model
│   │   ├── launch/             # Gazebo, SLAM, navigation launch files
│   │   ├── scripts/            # Control and odometry scripts
│   │   ├── config/             # Navigation and controller parameters
│   │   ├── meshes/             # STL 3D meshes
│   │   └── world/              # Gazebo mine environment
│   ├── differential/           # Husky A200 equivalent package
│   │   ├── launch/
│   │   ├── scripts/
│   │   ├── urdf_xacro/
│   │   └── config/
│   ├── tracked/                # Tracked vehicle package
│   │   ├── gazebo_continuous_track/
│   │   └── gazebo_continuous_track_example/
│   └── robot_metrics/          # Metrics logging and analysis
│       └── scripts/
│           ├── metrics_logger.py
│           └── analyze_metrics.py
├── plots/                      # Generated CSV data and EPS plots
├── generate_sdf.py             # SDF model generator (mass normalization)
├── plot_rocker_bogie.py        # 2D rocker-bogie geometry visualization
├── plot_rocker_bogie.m         # MATLAB visualization script
└── temp_stats.py               # Statistics computation across trials
```

## Technologies

- **ROS Noetic** — Robot Operating System framework
- **Gazebo 11** — 3D physics simulation
- **RTAB-Map** — Real-Time Appearance-Based SLAM
- **URDF/Xacro** — Robot modeling
- **move_base** — Autonomous navigation stack (DWA local planner + navfn global planner)
- **Python 3** — Control scripts, data analysis (pandas, numpy, matplotlib, scipy)

### Simulated Sensors

| Sensor | Specs |
|--------|-------|
| Velodyne VLP-16 LiDAR | 16 channels, 360° FOV, 10 Hz, 130m range |
| IMU (6-axis) | 100 Hz, angular velocity + linear acceleration |
| RGB Camera | 640×480 px, 15–30 Hz |

## Prerequisites

- **Ubuntu 20.04 LTS**
- **ROS Noetic**
- **Gazebo 11+**
- **Python 3.8+** with `rospy`, `pandas`, `numpy`, `matplotlib`, `scipy`, `pyyaml`, `actionlib`
- ROS packages: `gazebo_ros`, `gazebo_ros_control`, `controller_manager`, `move_base`, `rtabmap_ros`, `robot_state_publisher`

### Launch Parameters

| Parameter | Values | Description |
|-----------|--------|-------------|
| `paused` | `true/false` | Start Gazebo paused |
| `gui` | `true/false` | Run with/without GUI |
| `use_move_base` | `true/false` | Navigation stack or direct control |
| `config_file` | path | Waypoint YAML file |

## Results

### Mechanical Stability

| Metric | Husky | Tracked | Rocker-Bogie |
|--------|------:|--------:|-------------:|
| Max vertical accel. (m/s²) | 13.12 | 16.85 | **7.68** |
| Max roll angle (rad) | 0.577 | 0.503 | **0.459** |
| Max roll angular accel. (rad/s²) | 42.20 | 80.65 | **22.69** |

### SLAM Performance

| Metric | Husky | Tracked | Rocker-Bogie |
|--------|------:|--------:|-------------:|
| ATE (m) | 1.46 | 1.76 | **0.72** |
| Max ATE (m) | 2.34 | 3.04 | **1.23** |
| RPE (m) | 0.034 | **0.027** | 0.034 |

## License

This project is intended for academic and research purposes.
