# Quadrotor UAV-Based Search and Rescue System

A complete quadrotor UAV search and rescue system integrating real-time trajectory planning, visual-inertial navigation, swarm coordination, and embedded wireless communication. Based on EGO-Planner with B-spline optimization, supporting both Gazebo simulation and real-world flight experiments.

## System Architecture

```
Quadrotor UAV-Based-Search-and-Rescue-System/
├── core code/core_code/          # ROS-based planning, navigation, and simulation
│   ├── planner/                  # EGO-Planner trajectory planning
│   ├── uav_simulator/            # Quadrotor simulation environment
│   ├── realflight_modules/       # VINS-Fusion, PX4Ctrl, RealSense
│   └── utils/                    # Shared utilities and messages
└── bearpi-pico_h3863-master/     # BearPi-Pico H3863 embedded firmware (LiteOS)
```

## Core Code (`core code/core_code`)

### Planner

| Package | Description |
|---------|-------------|
| `plan_manage` | Main planner manager & EGO-Planner FSM — orchestrates global/local planning, trajectory execution, emergency stop, and swarm trajectory broadcasting |
| `bspline_opt` | B-spline trajectory optimization with rebound constraints, swarm clearance, smoothness/feasibility costs via L-BFGS |
| `path_searching` | Dynamic A* path searching on the occupancy grid for initial collision-free path |
| `plan_env` | Grid map with probabilistic occupancy, depth image raycasting, and moving object prediction |
| `traj_utils` | Trajectory container, polynomial trajectory, B-spline messaging (`Bspline`, `MultiBsplines`, `DataDisp`), and planning visualization |
| `drone_detect` | Drone-to-drone detection using depth images — erases detected drones from depth maps to avoid false obstacles |
| `rosmsg_tcp_bridge` | TCP bridge for ROS message forwarding in distributed setups |

### UAV Simulator

| Package | Description |
|---------|-------------|
| `so3_quadrotor_simulator` | SO3 dynamics quadrotor simulator |
| `so3_control` | SO3 attitude controller for the simulated drone |
| `map_generator` | Random forest / obstacle map generator for simulation |
| `local_sensing` | Depth camera rendering (CUDA-accelerated) for simulated sensing |
| `mockamap` | Mock map generation (Perlin noise, maze, 2D/3D) |
| `odom_visualization` | Odometry visualization with drone mesh |
| `fake_drone` | Fake drone for debugging and testing |

### Real Flight Modules

| Package | Description |
|---------|-------------|
| `VINS-Fusion` | Visual-inertial state estimation (stereo + GPS fusion, loop closure) |
| `px4ctrl` | PX4 flight controller interface with thrust calibration |
| `realsense-ros` | Intel RealSense camera driver (D435/D455/T265) |

### Utilities

| Package | Description |
|---------|-------------|
| `quadrotor_msgs` | Custom ROS message definitions for position commands |
| `uav_utils` | UAV utility functions (geometry, coordinate conversion, TF) |
| `pose_utils` | Pose manipulation utilities |
| `rviz_plugins` | Custom RViz visualization plugins |
| `DecompROS` | Convex decomposition for safe corridor generation |
| `catkin_simple` | Simplified catkin build macros |
| `cmake_utils` | CMake helper utilities |

## Embedded Firmware (`bearpi-pico_h3863-master`)

BearPi-Pico H3863 firmware running on Huawei LiteOS, providing:

- **Peripheral Drivers**: UART, SPI, I2C, PWM, ADC, GPIO, Timer, Watchdog
- **Wireless Communication**: WiFi (STA/AP), BLE (server/client), SLE (Super Low Energy)
- **Application Framework**: LiteOS task scheduling, device management
- **Network Modules**: UDP client/server, SLE gateway, BLE-UART bridge

### SLE (StarLight / 星闪) Application Layer

星闪(SLE)是新一代短距无线通信技术，具备低功耗、低时延、高可靠的特点。在本搜救系统中，星闪用于无人机与地面设备、无人机集群之间的近距离数据透传和指令下发。

#### 应用场景与数据流

```
搜救指挥中心(UDP服务器)
        ↑ WiFi/UDP
   [SLE Gateway]
        ↑ SLE星闪
   ┌────┴────┐
   │ UAV 1   │ ←→ PX4飞控 ←→ 传感器(雷达/相机)
   │(Server) │
   └────┬────┘
        │ SLE星闪 (集群内通信)
   ┌────┴────┐
   │ UAV 2   │
   │(Client) │
   └─────────┘
```

- **无人机 ↔ 飞控板**：BearPi-Pico 作为 SLE Server，通过 UART 接收飞控板的里程计、传感器数据，经星闪透传至地面站
- **地面站 ↔ 无人机**：地面站 BearPi-Pico 作为 SLE Client，扫描并连接无人机 Server，下发航点指令、接收状态回传
- **无人机集群**：多架无人机通过 SLE 进行集群内状态广播与协同避障
- **远程监控**：SLE Gateway 模式将星闪数据桥接到 WiFi/UDP，指挥中心可远程监控搜救进度

#### SLE 示例程序

| 示例 | 角色 | 效果 |
|------|------|------|
| `sle_uart` | Server / Client | 基础点对点串口透传，UART ↔ SLE ↔ UART 双向转发 |
| `sle_uart_echo` | Server + Client | 自动回连双角色适配器，上电自动扫描→连接→配对，断开后自动重连，LED 指示连接状态 |
| `sle_gateway` | Server / Client+WiFi | 星闪到 WiFi 网关，Client 端连接 WiFi 后通过 UDP socket 转发数据至远程服务器 |

#### 技术参数

| 参数 | 值 |
|------|-----|
| UART 波特率 | 115200 bps |
| 数据格式 | 8N1 |
| SLE MTU | 520 字节 |
| 扫描间隔/窗口 | 100ms / 100ms |

## System Data Flow

The complete data flow in the search and rescue scenario:

```
[RealSense D435] ──depth/color──→ [VINS-Fusion] ──odom──→ [EGO-Planner]
                                                            │
                                                     replan / trajectory
                                                            │
                                                            ↓
[Ground Station] ←──SLE/WiFi/UDP──→ [BearPi-Pico] ←──UART──→ [PX4 Controller]
                                                            │
                                                          motor cmd
                                                            │
                                                            ↓
                                                      [Quadrotor UAV]
```

## EGO-Planner Pipeline

The planning pipeline follows a reactive FSM design:

```
INIT → WAIT_TARGET → GEN_NEW_TRAJ → EXEC_TRAJ → REPLAN_TRAJ
                                                 ↕
                                          EMERGENCY_STOP
```

**Planning Flow:**
1. **Global Planning**: Piecewise minimum-snap trajectory through waypoints
2. **Path Searching**: Dynamic A* on inflated occupancy grid
3. **Local Optimization**: B-spline optimization with distance, smoothness, feasibility, and swarm costs
4. **Rebound**: Automatic replanning when new obstacles are detected
5. **Emergency Stop**: Triggered when trajectory becomes infeasible

## Quick Start

### Prerequisites

- ROS Melodic / Noetic
- OpenCV, PCL, Eigen, cv_bridge
- CUDA (optional, for GPU-accelerated depth rendering)

### Build

```bash
# Clone into catkin workspace
cd ~/catkin_ws/src
git clone https://github.com/pluto2006c/Quadrotor-UAV-Based-Search-and-Rescue-System.git

# Build
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Run in Simulation

```bash
# Single drone simulation
roslaunch ego_planner single_run_in_sim.launch

# Multi-drone simulation
roslaunch ego_planner run_in_sim.launch
```

### Run on Real Drone

```bash
# Start VINS-Fusion
roslaunch vins_estimator fast_drone_250.launch

# Start PX4 controller
roslaunch px4ctrl run_ctrl.launch

# Start planner
roslaunch ego_planner single_run_in_exp.launch
```

### Set Goal

In RViz, use **2D Nav Goal** to set a target position, or configure waypoints in the launch file.

## Key Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `fsm/flight_type` | 1: Manual goal, 2: Preset waypoints | 1 |
| `fsm/planning_horizon` | Local planning horizon (m) | 7.5 |
| `fsm/thresh_replan_time` | Replan threshold (s) | 1.0 |
| `fsm/fail_safe` | Enable emergency stop | true |
| `grid_map/resolution` | Grid map resolution (m) | 0.1 |
| `optimization/max_vel` | Maximum velocity (m/s) | 2.0 |
| `optimization/max_acc` | Maximum acceleration (m/s^2) | 6.0 |

## References

- EGO-Planner: Fast, Gradient-Based Local Planner for Quadrotors
- VINS-Fusion: Robust Visual-Inertial State Estimation with Multiple Sensor Fusion
- PX4 Autopilot: Open Source Flight Controller Firmware

## License

MIT License
