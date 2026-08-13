# LVI-SLAM Tank Inspection System — Complete Documentation

**System Owner:** Jose Solares (FIU)
**Last Updated:** August 2026
**ROS Distribution:** ROS 2 Humble (Compute Node) / ROS 2 Jazzy (Edge Node)

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Inventory](#2-hardware-inventory)
3. [Network Architecture](#3-network-architecture)
4. [New Computer Setup (Migration Guide)](#4-new-computer-setup-migration-guide)
5. [Edge Node (SBC) Setup](#5-edge-node-sbc-setup)
6. [Building the Workspace](#6-building-the-workspace)
7. [Environment Variables](#7-environment-variables)
8. [Launching the System](#8-launching-the-system)
9. [Operating Modes](#9-operating-modes)
10. [Configuration Files Reference](#10-configuration-files-reference)
11. [Persistent Mapping (Save / Load / Expand)](#11-persistent-mapping-save--load--expand)
12. [RViz Visualization Guide](#12-rviz-visualization-guide)
13. [Custom Nodes Reference](#13-custom-nodes-reference)
14. [TF Tree & URDF Geometry](#14-tf-tree--urdf-geometry)
15. [Camera Calibration](#15-camera-calibration)
16. [Known Issues & Fixes](#16-known-issues--fixes)
17. [Troubleshooting](#17-troubleshooting)
18. [File Manifest](#18-file-manifest)

---

## 1. System Overview

The LVI-SLAM system is a **Lidar-Visual-Inertial Simultaneous Localization and Mapping** pipeline built for autonomous tank inspection. It fuses data from three sensor modalities:

- **LiDAR** (Ouster OS-0-128): 128-channel 3D point clouds for geometric mapping
- **Monocular Cameras** (Front & Back): Visual feature tracking for high-frequency odometry
- **IMU** (Ouster internal): 100 Hz inertial measurements for gravity alignment and motion prediction

The software runs on **ROS 2 Humble** using the following core frameworks:
- **Kitware LidarSLAM**: ICP-based LiDAR odometry and mapping
- **VINS-Fusion**: Visual-inertial odometry (one instance per camera)
- **robot_localization (EKF)**: Fuses dual VINS outputs into a single pose estimate
- **Open3D**: Dense point cloud accumulation with voxel downsampling
- **CycloneDDS**: DDS middleware for real-time data transport

### Architecture Diagram

```
               +--------------------------+
               |     Compute Node         | (Laptop: ROS 2 Humble)
               |  IP: 10.42.0.1           |
               |  Runs: LidarSLAM, VINS,  |
               |  EKF, RViz, Dense Map,   |
               |  Map Manager GUI         |
               +-----------+--------------+
                           |
                  (Tethered Ethernet)
                           |
               +-----------+--------------+
               |     Edge Node            | (SBC: ROS 2 Jazzy)
               |  Bridge IP: 10.42.0.149  |
               |  Runs: Camera drivers,   |
               |  Network bridge (br0)    |
               +-----+-----------+--------+
                     |           |
         (Dual CSI)  |           | (Onboard Ethernet)
                     |           |
             +-------+-------+  +-------+-------+
             | 2x Monocular  |  | Ouster LiDAR  |
             |   Cameras     |  | IP: 10.42.0.58|
             +---------------+  +---------------+
```

---

## 2. Hardware Inventory

| Component | Model | Connection | Notes |
|-----------|-------|------------|-------|
| LiDAR | Ouster OS-0-128 (Rev C, FW v3.1.0) | Ethernet to Edge Node `eth0` | S/N: 122451000259, 128 channels, 512x10 mode |
| Front Camera | Pi Camera Module 3 NoIR | CSI ribbon to Edge Node | 640x480, JPEG-compressed transport |
| Back Camera | Pi Camera Module 3 NoIR | CSI ribbon to Edge Node | 640x480, JPEG-compressed transport |
| Edge Node (SBC) | Raspberry Pi 5 | Ethernet to Compute Node | Runs ROS 2 Jazzy, acts as network bridge |
| Compute Node | GPU-equipped laptop | Ethernet to Edge Node | Runs ROS 2 Humble, all SLAM processing |
| IMU | Ouster internal (in LiDAR) | Integrated | 100 Hz accel + gyro |

### Physical Wiring

```
[Compute Node Laptop]
    |-- USB-to-Ethernet adapter --(Cat6)--> [Edge Node SBC eth0 OR br0 uplink]

[Edge Node SBC]
    |-- Onboard Ethernet (eth0) --(Cat6)--> [Ouster LiDAR Ethernet port]
    |-- USB-to-Ethernet adapter --(Cat6)--> [Compute Node]
    |-- CSI Port 0 --(ribbon)--> [Front Camera]
    |-- CSI Port 1 --(ribbon)--> [Back Camera]
```

> [!CAUTION]
> The USB-to-Ethernet adapter on the edge node requires **active cooling** (fans in the enclosure). Without it, the adapter will thermally throttle after ~17 minutes of sustained operation at 150+ Mbps and silently drop all packets while LEDs continue to flash.

---

## 3. Network Architecture

### IP Assignments

| Device | IP Address | Interface |
|--------|-----------|-----------|
| Compute Node (Laptop) | `10.42.0.1` | `enx4cea416cc906` (USB Ethernet) |
| Edge Node (SBC) | `10.42.0.149` | `br0` (Linux bridge) |
| Ouster LiDAR | `10.42.0.58` | Hardware Ethernet |

### Software Bridge Topology

The edge node runs a **Linux bridge** (`br0`) that merges its onboard Ethernet interface with a USB Ethernet adapter, creating a transparent Layer 2 passthrough. This allows the compute node's DHCP server to assign an IP directly to the LiDAR through the SBC.

**Why not a hardware switch?** Unmanaged Gigabit switches have a hardcoded IGMP Snooping feature that incorrectly classifies CycloneDDS UDP multicast discovery traffic as a "broadcast storm." The switch permanently shuts down the affected ports exactly 2 seconds after the LiDAR begins transmitting. The software bridge avoids this entirely.

### DDS Configuration

Both nodes use **CycloneDDS** bound to the Ethernet-only interface to prevent ROS 2 multicast from leaking over Wi-Fi:

- **ROS Domain ID:** `27`
- **Middleware:** `rmw_cyclonedds_cpp`
- **Compute Node interface:** `enx4cea416cc906` *(this changes per USB adapter — see migration section)*
- **Edge Node interface:** `br0`

---

## 4. New Computer Setup (Migration Guide)

### 4.1 Install ROS 2 Humble

Follow the official installation: https://docs.ros.org/en/humble/Installation.html

Install `ros-humble-desktop` (full desktop including RViz).

### 4.2 Install System Dependencies

```bash
# Core ROS 2 packages needed by the workspace
sudo apt install -y \
  ros-humble-robot-localization \
  ros-humble-nav2-planner \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-costmap-2d \
  ros-humble-tf2-ros \
  ros-humble-tf2-eigen \
  ros-humble-pcl-conversions \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-image-transport-plugins \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-rmw-cyclonedds-cpp

# Build dependencies
sudo apt install -y \
  libpcl-dev \
  libeigen3-dev \
  libceres-dev \
  libgoogle-glog-dev \
  libjsoncpp-dev \
  libtins-dev \
  libspdlog-dev \
  libzip-dev \
  libcurl4-openssl-dev \
  python3-colcon-common-extensions

# Python dependencies for custom nodes
pip install open3d numpy opencv-python
```

### 4.3 Clone the Workspace

Copy the entire `sensor_ws` directory from the old machine:

```bash
# Option A: rsync from old machine
rsync -avz robotics@OLD_MACHINE_IP:/home/robotics/sensor_ws/ ~/sensor_ws/

# Option B: If using git
cd ~
git clone <repository-url> sensor_ws
```

### 4.4 Create the Maps Directory

```bash
mkdir -p ~/maps
```

### 4.5 Identify Your Ethernet Interface Name

Plug in your USB-to-Ethernet adapter and run:

```bash
ip link show
```

Find the interface name (e.g., `enx4cea416cc906`, `enp0s20f0u1`, etc.). You will need this for the CycloneDDS and DHCP configuration.

### 4.6 Configure Network Sharing (DHCP Server)

The compute node must act as a DHCP server on the Ethernet interface so it can assign IPs to the edge node and LiDAR.

**Using NetworkManager (Ubuntu 22.04+):**

1. Open **Settings > Network > Wired Connection**
2. Click the gear icon on your USB Ethernet adapter
3. Go to **IPv4 Settings**
4. Set Method to **Shared to other computers**
5. The interface will automatically get `10.42.0.1`

**Or via command line:**
```bash
nmcli connection add type ethernet con-name "Robot-Ethernet" \
  ifname YOUR_INTERFACE_NAME ipv4.method shared
nmcli connection up "Robot-Ethernet"
```

### 4.7 Update the CycloneDDS Interface

Edit `~/.bashrc` and update the interface name:

```bash
# Replace enx4cea416cc906 with YOUR interface name
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>YOUR_INTERFACE_NAME</NetworkInterfaceAddress><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>'
```

### 4.8 Update Hard-Coded Paths

If your username is NOT `robotics`, you must update these paths:

1. **VINS config paths** in `viz_lvi_slam.launch.py` (lines 118, 129):
   ```python
   vins_config_front = '/home/YOUR_USER/sensor_ws/src/vins_fusion_ros2/config/ouster_dual_cam/front_mono_imu_config.yaml'
   vins_config_back = '/home/YOUR_USER/sensor_ws/src/vins_fusion_ros2/config/ouster_dual_cam/back_mono_imu_config.yaml'
   ```

2. **VINS binary path** in `viz_lvi_slam.launch.py` (lines 121, 132):
   ```python
   '/home/YOUR_USER/sensor_ws/install/vins/lib/vins/vins_node'
   ```

3. **Map save/load path** in `slam_config_indoor.yaml`:
   ```yaml
   initial_maps: "/home/YOUR_USER/maps/slam_map"
   ```

4. **Dense map path** in `dense_map_accumulator.py` and `map_autosave.py`:
   - Search for `/home/robotics/maps/` and replace with `/home/YOUR_USER/maps/`

### 4.9 Build the Workspace

```bash
cd ~/sensor_ws
colcon build --symlink-install
source install/setup.bash
```

> [!WARNING]
> The first build of `kitware_slam` and `vins_fusion_ros2` can take 15-30 minutes due to Ceres optimization library compilation.

---

## 5. Edge Node (SBC) Setup

### 5.1 Operating System

The edge node runs **Ubuntu 24.04 Server (arm64)** with **ROS 2 Jazzy** installed.

### 5.2 Network Bridge Configuration

File: `/etc/netplan/50-cloud-init.yaml` on the edge node

```yaml
network:
  version: 2
  ethernets:
    eth0:
      optional: true
      dhcp4: false
      dhcp6: false
    enx00e04c68344f:        # USB-to-Ethernet adapter (may differ)
      optional: true
      dhcp4: false
      dhcp6: false
  bridges:
    br0:
      interfaces:
        - eth0
        - enx00e04c68344f
      dhcp4: true
      dhcp6: false
      macaddress: 88:a2:9e:84:af:42   # Set to match one of the interfaces
      parameters:
        forward-delay: 0
        stp: false
```

Apply with: `sudo netplan apply`

> [!IMPORTANT]
> If you replace the USB-to-Ethernet adapter on the SBC, the interface name (`enx00e04c68344f`) will change. Run `ip link show` on the SBC to find the new name and update this file accordingly.

### 5.3 Camera Driver Service

The cameras auto-start on boot via systemd:

**Service file:** `/etc/systemd/system/ros2_cameras.service`

**Startup script:** `/home/robotics/start_cameras.sh`

```bash
#!/bin/bash
export ROS_DOMAIN_ID=27
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>br0</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
source /opt/ros/jazzy/setup.bash
taskset -c 2,3 ros2 launch /home/robotics/camera_launch.py
```

> [!NOTE]
> The `taskset -c 2,3` is critical. It pins camera drivers to dedicated CPU cores to prevent the DDS network threads from starving them during high-bandwidth LiDAR data bursts. **Do not remove this.**

### 5.4 Camera Topics Published by Edge Node

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/front/front_camera/image_raw` | `sensor_msgs/Image` | Raw front camera (uncompressed) |
| `/camera/front/front_camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | JPEG-compressed front feed |
| `/camera/back/back_camera/image_raw` | `sensor_msgs/Image` | Raw back camera |
| `/camera/back/back_camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | JPEG-compressed back feed |

### 5.5 SSH Access

```bash
ssh robotics@10.42.0.149
```

### 5.6 Managing the Camera Service

```bash
# Check status
ssh robotics@10.42.0.149 "systemctl status ros2_cameras.service"

# Restart cameras
ssh robotics@10.42.0.149 "sudo systemctl restart ros2_cameras.service"

# View live logs
ssh robotics@10.42.0.149 "journalctl -u ros2_cameras.service -f"
```

### 5.7 Custom libcamera Build

The edge node uses a **custom-compiled `libcamera`** with the Raspberry Pi 5 Image Signal Processor (PiSP) pipeline handler enabled. The system `libcamera` libraries at `/opt/ros/jazzy/lib/` are symlinked to the custom build at `/usr/lib/aarch64-linux-gnu/libcamera.so.0.7.1`. If you ever reinstall ROS 2 Jazzy on the SBC, you must re-link these libraries.

---

## 6. Building the Workspace

### Full Build (First Time)

```bash
cd ~/sensor_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Incremental Build (After Editing Config/Launch Files)

```bash
cd ~/sensor_ws
colcon build --packages-select ouster_ros lidar_slam
source install/setup.bash
```

### Build Specific Packages

| Package | When to Rebuild |
|---------|----------------|
| `ouster_ros` | After editing launch files, RViz configs, EKF/nav2 params, or custom Python nodes |
| `lidar_slam` | After editing `slam_config_indoor.yaml` or `slam_config_outdoor.yaml` |
| `vins` | After modifying VINS-Fusion source code (rare) |
| `sensor_pkg` | After modifying the URDF geometry |
| `lidar_conversions` | After changing point cloud format conversion params |

---

## 7. Environment Variables

Add these to your `~/.bashrc`:

```bash
# ROS 2 Humble
source /opt/ros/humble/setup.bash

# DDS Configuration
export ROS_DOMAIN_ID=27
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# CycloneDDS — CHANGE THE INTERFACE NAME to match your USB Ethernet adapter
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>enx4cea416cc906</NetworkInterfaceAddress><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>'
```

After editing, run `source ~/.bashrc`.

> [!IMPORTANT]
> The `CYCLONEDDS_URI` interface name **must exactly match** your USB-to-Ethernet adapter's interface name. If you use a different adapter or port, this value will change. Run `ip link show` to find it.

---

## 8. Launching the System

### Step 1: Verify Hardware Connections

1. Edge node is powered on and connected via Ethernet
2. LiDAR is powered on and connected to edge node's onboard Ethernet
3. Cameras are connected via CSI ribbons to the edge node

### Step 2: Verify Edge Node

```bash
# Ping the edge node
ping 10.42.0.149

# Ping the LiDAR (through the bridge)
ping 10.42.0.58

# Verify cameras are running
ssh robotics@10.42.0.149 "systemctl status ros2_cameras.service"
```

### Step 3: Launch the Full Stack

```bash
source ~/sensor_ws/install/setup.bash
ros2 launch ouster_ros viz_lvi_slam.launch.py
```

This single command launches **24 nodes**:

| # | Node | Purpose |
|---|------|---------|
| 1 | Ouster LiDAR driver | Publishes `/ouster/points`, `/ouster/imu` |
| 2 | RViz2 | 3D visualization |
| 3 | `robot_state_publisher` | Publishes URDF transforms |
| 4-6 | Static TF publishers (x3) | `base_link<>os_sensor`, camera optical frames |
| 7-8 | `image_transport` republish (x2) | Decompresses JPEG camera feeds |
| 9-10 | VINS-Fusion (x2) | Front and back visual-inertial odometry |
| 11 | EKF (`robot_localization`) | Fuses dual VINS outputs |
| 12 | `odom_to_pose` | Converts EKF odometry to PoseStamped for LidarSLAM |
| 13 | `path_publisher` | Publishes `/slam_path` for RViz trajectory line |
| 14 | `ouster_conversion` | Converts Ouster point format to LidarSLAM format |
| 15 | Kitware LidarSLAM | Point cloud registration and mapping |
| 16-17 | Nav2 planner + lifecycle | Path planning (proof of concept) |
| 18 | `waypoint_manager` | Waypoint click-to-navigate via RViz |
| 19 | `map_autosave` | Auto-saves SLAM maps every 2 minutes |
| 20 | Map Manager GUI | Floating toolbar: New Map / Save Now / Delete Map |
| 21 | `saved_map_loader` | Loads and publishes saved PCD maps on startup |
| 22 | `dense_map_accumulator` | Builds voxelized dense 3D environment map |
| 23 | `camera_rotate` | Rotates camera feeds 180 degrees for RViz display |

### Step 4: Verify Topics

In a second terminal:

```bash
source ~/sensor_ws/install/setup.bash
ros2 topic list | grep -E 'ouster|slam|vins|dense|camera'
```

Expected topics:
```
/ouster/points
/ouster/imu
/slam_registered_points
/slam_odom
/slam_path
/vins_front/vins_estimator/odometry
/vins_back/vins_estimator/odometry
/odometry/filtered_vision
/dense_map
/camera/front/front_camera/image_decompressed
/camera/back/back_camera/image_decompressed
```

### Shutting Down

Press `Ctrl+C` in the launch terminal. The `map_autosave` node performs a final map save on shutdown.

---

## 9. Operating Modes

### Mode A: Full LVI-SLAM (Default — Cameras + LiDAR)

This is the default configuration. Uses cameras for visual odometry + LiDAR for mapping.

**Requirements:**
- Edge node running with cameras active
- All nodes in `viz_lvi_slam.launch.py` uncommented

**SLAM config (`slam_config_indoor.yaml`):**
```yaml
external_poses:
  enable: true        # Use VINS poses as prior
imu:
  enable: false       # VINS handles IMU internally
```

### Mode B: Pure LiDAR SLAM (No Cameras / Bypass Mode)

Use this to bypass the edge node entirely and plug the LiDAR directly into the compute node.

**Changes needed:**

1. In `viz_lvi_slam.launch.py`, comment out these nodes in the `nodes` list:
   - `front_decompress_node`, `back_decompress_node`
   - `vins_front_node`, `vins_back_node`
   - `ekf_node`
   - `odom_to_pose_node`
   - `camera_rotate_node`

2. In `slam_config_indoor.yaml`:
   ```yaml
   external_poses:
     enable: false       # No external poses available
   imu:
     enable: true        # LidarSLAM handles IMU directly for gravity alignment
   ```

3. In the `lidar_driver` section of the launch file, change `sensor_hostname`:
   ```python
   'sensor_hostname': 'os-122119100588.local'  # Use hostname instead of IP
   ```

4. Rebuild: `colcon build --packages-select ouster_ros lidar_slam && source install/setup.bash`

> [!TIP]
> When connecting the LiDAR directly to the laptop (bypassing the SBC), the LiDAR's IP may change depending on the laptop's DHCP assignment. Using the hostname (`os-SERIALNUMBER.local`) instead of an IP ensures the driver can always find the sensor.

### Switching Between Modes

After making changes to the launch file or YAML configs, always rebuild:

```bash
cd ~/sensor_ws
colcon build --packages-select ouster_ros lidar_slam
source install/setup.bash
```

---

## 10. Configuration Files Reference

### 10.1 SLAM Configuration

**File:** `src/kitware_slam/ros2_wrapping/lidar_slam/params/slam_config_indoor.yaml`

Key parameters:

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `odometry_frame` | `"odom"` | World-fixed reference frame |
| `tracking_frame` | `"base_link"` | Robot body frame |
| `initial_maps` | `"/home/robotics/maps/slam_map"` | Path prefix for persistent map load |
| `override_timestamp` | `true` | Aligns loaded keypoint timestamps with current session |
| `update_maps` | `1` | Expansion mode: freeze loaded map, add new areas only |
| `external_poses.enable` | `true` | Accept VINS pose priors (set `false` for pure LiDAR mode) |
| `external_poses.weight` | `2.0` | Weight of external pose constraint in optimization |
| `imu.enable` | `false` | IMU handled by VINS (set `true` for pure LiDAR mode) |
| `slam.ego_motion` | `1` | Extrapolate from 2 previous poses |
| `slam.undistortion` | `2` | Iteratively refined rolling-shutter correction |
| `slam.n_threads` | `5` | Parallel processing threads |
| `voxel_grid.leaf_size.edges` | `0.2` m | Edge keypoint map resolution |
| `voxel_grid.leaf_size.planes` | `0.3` m | Plane keypoint map resolution |
| `voxel_grid.size` | `100` | Voxel grid dimension (100 cubed voxels) |
| `voxel_grid.resolution` | `5.0` m/voxel | Voxel physical size |

### 10.2 EKF Configuration

**File:** `src/ouster-ros/ouster-ros/config/ekf.yaml`

```yaml
ekf_filter_node:
    ros__parameters:
        frequency: 30.0              # Output rate (Hz)
        publish_tf: false            # LidarSLAM publishes TF, not EKF
        odom_frame: odom
        base_link_frame: base_link
        world_frame: odom

        # Input 1: VINS Front odometry
        odom0: /vins_front/vins_estimator/odometry
        odom0_config: [true, true, true,     # x, y, z
                       true, true, true,     # roll, pitch, yaw
                       false, false, false,  # vx, vy, vz
                       false, false, false,  # vroll, vpitch, vyaw
                       false, false, false]  # ax, ay, az

        # Input 2: VINS Back odometry
        odom1: /vins_back/vins_estimator/odometry
        odom1_config: [true, true, true,
                       true, true, true,
                       false, false, false,
                       false, false, false,
                       false, false, false]
```

### 10.3 VINS-Fusion Configuration

**Front camera:** `src/vins_fusion_ros2/config/ouster_dual_cam/front_mono_imu_config.yaml`
**Back camera:** `src/vins_fusion_ros2/config/ouster_dual_cam/back_mono_imu_config.yaml`

Key parameters (same for both, except topics and extrinsics):

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `imu_topic` | `/ouster/imu` | IMU data from Ouster LiDAR |
| `image0_topic` | `/camera/front/front_camera/image_decompressed` | Decompressed camera feed |
| `image_width` / `image_height` | `640` / `480` | Camera resolution |
| `max_cnt` | `150` | Max features to track |
| `freq` | `10` | VIO output frequency (Hz) |
| `estimate_extrinsic` | `1` | Online extrinsic calibration refinement |
| `estimate_td` | `1` | Online time offset estimation |
| `acc_n` / `gyr_n` | `0.1` / `0.01` | IMU noise parameters |

### 10.4 Nav2 Planner (Proof of Concept)

**File:** `src/ouster-ros/ouster-ros/config/nav2_planner_params.yaml`

This is a basic A* path planner using Nav2. It is functional but purely proof-of-concept since the crawler has no motor control integration yet.

---

## 11. Persistent Mapping (Save / Load / Expand)

### How It Works

1. **Auto-Load on Startup:** SLAM loads saved keypoint maps from `/home/robotics/maps/slam_map_*.pcd` at boot. If no maps exist, it starts fresh.

2. **Auto-Save Every 2 Minutes:** The `map_autosave.py` node calls the SLAM save service periodically and on shutdown.

3. **Expansion Mode (`update_maps: 1`):** Previously mapped areas are frozen. New keypoints are added only in unexplored areas. Over multiple sessions, the map grows.

### Saved Map Files

Located in `/home/robotics/maps/`:

| File | Description |
|------|-------------|
| `slam_map_edges.pcd` | Edge keypoints (corners, object boundaries) |
| `slam_map_planes.pcd` | Planar surface keypoints (walls, floors) |
| `slam_map_intensity_edges.pcd` | Intensity-based edge keypoints |
| `dense_map.pcd` | Full voxel-downsampled 3D environment map |

### Manual Map Operations

**Save immediately:**
```bash
ros2 service call /lidar_slam/save_pc lidar_slam/srv/SavePc \
  "{output_prefix_path: /home/robotics/maps/slam_map, format: 2, filtered: true, fixed: true}"
```

**Start a completely fresh map:**
```bash
rm /home/robotics/maps/slam_map_*.pcd
rm /home/robotics/maps/dense_map.pcd
# Then relaunch
```

**Or use the Map Manager GUI** — the floating toolbar has "New Map" (wipes and resets) and "Save Now" buttons.

> [!IMPORTANT]
> **Start from a consistent location.** SLAM initializes at pose (0,0,0) every boot. For the loaded map to align correctly, always power on the robot in the same physical spot where you first started mapping. If you start from a different location, the robot will build a disconnected local map until it physically moves into a previously mapped area, at which point ICP scan matching will lock on.

---

## 12. RViz Visualization Guide

The custom RViz config (`slam_viz.rviz`) loads automatically with the launch file.

### Critical Setting

**Fixed Frame must be `odom`**. Setting it to `base_link` or `os_sensor` will cause the map to appear attached to the robot instead of the world.

### Display Configuration

| Display | Topic | Color | Purpose |
|---------|-------|-------|---------|
| RobotModel | `/robot_description` | — | Physical robot geometry |
| PointCloud2 (Live) | `/ouster/points` | White (FlatColor) | Real-time LiDAR scan |
| PointCloud2 (SLAM Map) | `/slam_registered_points` | Green | Accumulating SLAM keypoint map |
| PointCloud2 (Dense Map) | `/dense_map` | RGB | Full 3D environment reconstruction |
| PointCloud2 (Saved Map) | `/saved_map/edges` | Cyan | Loaded persistent map overlay |
| Path | `/slam_path` | Red | Robot trajectory line |
| Image | `/vins_front/vins_estimator/tracking_image` | — | Front camera with feature tracking |
| Image | `/vins_back/vins_estimator/tracking_image` | — | Back camera with feature tracking |
| TF | — | — | Coordinate frame visualization |

### Manual RViz Launch (if auto-launch crashes)

```bash
rviz2 -d ~/sensor_ws/src/ouster-ros/ouster-ros/config/slam_viz.rviz
```

---

## 13. Custom Nodes Reference

All custom Python nodes are located in:
`~/sensor_ws/src/ouster-ros/ouster-ros/launch/`

### 13.1 `dense_map_accumulator.py`

Accumulates every LiDAR scan into a persistent voxel-downsampled point cloud using Open3D. Publishes `/dense_map` and auto-saves/loads `~/maps/dense_map.pcd`.

**Key features:**
- Voxel downsampling (configurable resolution, default 0.05m)
- Z-range cropping to remove floor reflections (`z_min = -0.6m`, `z_max = 5.0m`)
- Transform lookup from `odom` to `os_lidar` via TF2
- Auto-loads previous dense map on startup

### 13.2 `map_autosave.py`

Periodically calls the SLAM save service to persist keypoint maps. Default interval: 2 minutes. Also saves on Ctrl+C shutdown.

### 13.3 `map_manager.py` (GUI)

A Tkinter floating toolbar with three buttons:
- **Save Now** — triggers an immediate map save
- **New Map** — deletes all saved PCD files and resets SLAM state
- **Delete Map** — removes saved maps from disk

### 13.4 `saved_map_loader.py`

On startup, reads saved PCD files from `~/maps/` and publishes them as latched PointCloud2 topics so the previously-mapped environment is visible in RViz immediately, before SLAM starts processing.

### 13.5 `odom_to_pose.py`

Bridges VINS-Fusion/EKF output (`nav_msgs/Odometry`) to the format expected by Kitware LidarSLAM (`geometry_msgs/PoseStamped` on `/ext_poses`).

### 13.6 `path_publisher.py`

Subscribes to `/slam_odom` and accumulates poses into a `nav_msgs/Path` for a clean trajectory line in RViz on `/slam_path`.

### 13.7 `camera_rotate.py`

Subscribes to raw camera images and publishes 180-degree-rotated versions for optional RViz display. This is for visual inspection only — nothing in the SLAM pipeline subscribes to the rotated feeds.

### 13.8 `waypoint_manager.py`

RViz "2D Nav Goal" click handler. Receives `PoseStamped` goals and sends them to the Nav2 planner for path computation.

---

## 14. TF Tree and URDF Geometry

### Transform Tree

```
odom  (world-fixed, published by LidarSLAM)
 |-- base_link  (robot body center)
      |-- os_sensor  (Ouster LiDAR mount point, offset z=-0.04926m)
      |    |-- os_lidar  (LiDAR scan frame, published by Ouster driver)
      |    |-- os_imu   (IMU frame, published by Ouster driver)
      |-- cam1_link  (back camera, xyz=[-0.07669, 0, 0.015], pitch=-90 deg)
      |    |-- camera_back_optical_frame  (optical convention)
      |-- cam2_link  (front camera, xyz=[0.07669, 0, 0.015], pitch=-90 deg, roll=180 deg)
           |-- camera_front_optical_frame  (optical convention)
```

### URDF File

**Location:** `src/sensor_pkg/urdf/sensor_pkg.urdf`

This file defines the physical geometry of the sensor package (3D-printed mount). The STL meshes are in `src/sensor_pkg/meshes/`.

### Extrinsic Transforms (Camera to IMU)

Defined in the VINS config files as `body_T_cam0` matrices:

**Front camera:**
```
[ 0.0,  0.0,  1.0, -0.07669]
[ 1.0,  0.0,  0.0, -0.03426]
[ 0.0,  1.0,  0.0,  0.0    ]
```

**Back camera:**
```
[ 0.0,  0.0, -1.0,  0.07669]
[-1.0,  0.0,  0.0, -0.03426]
[ 0.0,  1.0,  0.0,  0.0    ]
```

These values represent the rigid transformation from the IMU frame (Ouster-internal) to each camera's optical frame, in meters. They were determined through measurement of the CAD model and refined during online calibration (`estimate_extrinsic: 1`).

---

## 15. Camera Calibration

### Current Calibration Files

| Camera | File | Model |
|--------|------|-------|
| Front | `config/ouster_dual_cam/front_camera.yaml` | PINHOLE |
| Back | `config/ouster_dual_cam/back_camera.yaml` | PINHOLE |

### Calibration Parameters

```yaml
model_type: PINHOLE
image_width: 640
image_height: 480
distortion_parameters:
   k1: 0.069608
   k2: -0.084763
   p1: -0.005188
   p2: 0.023848
projection_parameters:
   fx: 587.30902
   fy: 581.931
   cx: 449.68032
   cy: 244.80127
```

### Recalibration

If you replace a camera, recalibrate using the provided scripts:

```bash
# Front camera
bash ~/sensor_ws/src/ouster-ros/ouster-ros/launch/calibrate_front.sh

# Back camera
bash ~/sensor_ws/src/ouster-ros/ouster-ros/launch/calibrate_back.sh
```

These scripts use a checkerboard pattern and the `camera_calibration` ROS 2 package.

---

## 16. Known Issues and Fixes

### 16.1 The 17-Minute Freeze (Thermal Throttling)

**Symptom:** System freezes after ~17 minutes of continuous operation. LiDAR data stops but adapter LEDs keep blinking.

**Cause:** Network adapter on the edge node overheats at sustained >150 Mbps throughput.

**Fix:**
1. Replace the degraded adapter with a fresh Gigabit USB-C Ethernet adapter
2. Install active ventilation (dual-fan push-pull) inside the SBC enclosure

### 16.2 Camera Freezing Under Load

**Symptom:** One or both camera feeds stop publishing while LiDAR continues working.

**Cause:** DDS network threads consuming all CPU cores, starving camera drivers.

**Fix:** Use `taskset -c 2,3` to pin camera drivers to dedicated CPU cores. This is already configured in `start_cameras.sh`. **Do not remove it.**

### 16.3 IGMP Snooping Switch Incompatibility

**Symptom:** LiDAR data appears for ~2 seconds then stops completely.

**Cause:** Unmanaged Gigabit switches classify CycloneDDS UDP multicast as a broadcast storm and shut down ports.

**Fix:** Use the software bridge on the SBC instead of a hardware switch. If you must use a switch, use a managed switch with IGMP snooping disabled, or a robotics-grade switch (e.g., BotBlox).

### 16.4 LiDAR Multipath Reflections

**Symptom:** Phantom geometry appears below the floor plane in shiny/reflective environments.

**Fix:** The `dense_map_accumulator.py` applies a Z-range crop filter: `z_min = -0.6m`, `z_max = 5.0m`. Adjust these values in the script if your environment differs.

### 16.5 Ouster Driver Crash on FW 3.1.0

**Symptom:** Driver crashes with "WINDOW field not found" error.

**Fix:** Already patched in `src/ouster-ros/ouster-ros/src/point_cloud_compose.h`. The patch dynamically checks if firmware-specific fields exist before reading them.

---

## 17. Troubleshooting

| Problem | Diagnosis | Fix |
|---------|-----------|-----|
| "odom frame does not exist" in RViz | LidarSLAM hasn't received its first point cloud yet | Wait for LiDAR data. Check: `ros2 topic hz /ouster/points` |
| RViz shows floating/drifting points | Fixed Frame is wrong | Set RViz Fixed Frame to `odom` |
| Can't ping 10.42.0.58 (LiDAR) | Bridge not active on SBC | SSH to SBC, check `ip addr show br0`, verify netplan |
| Can't ping 10.42.0.149 (SBC) | Ethernet not connected or DHCP not sharing | Check NetworkManager "Shared" mode on compute node |
| VINS tracking drops frequently | Poor lighting, excessive motion blur, or cameras blocked | Check tracking images in RViz |
| Cameras not publishing | systemd service not running on SBC | SSH and restart: `sudo systemctl restart ros2_cameras.service` |
| Dense map not appearing | No TF from `odom` to `os_lidar` yet | SLAM must process at least one scan first |
| Map doesn't align after restart | Robot not started at same location | Always power on at the original mapping start position |
| Build fails: "Ceres not found" | Missing dependency | `sudo apt install libceres-dev` |
| Build fails: "Eigen not found" | Missing dependency | `sudo apt install libeigen3-dev` |
| `colcon build` very slow | First time building VINS/Kitware | Normal. Ceres compilation takes 15-30 min |

---

## 18. File Manifest

### Compute Node — Key Files

```
~/sensor_ws/
|-- src/
|   |-- ouster-ros/ouster-ros/
|   |   |-- launch/
|   |   |   |-- viz_lvi_slam.launch.py        # MASTER LAUNCH FILE
|   |   |   |-- dense_map_accumulator.py      # Dense 3D map builder
|   |   |   |-- map_autosave.py               # Periodic map saver
|   |   |   |-- map_manager.py                # GUI toolbar
|   |   |   |-- saved_map_loader.py           # PCD to PointCloud2 loader
|   |   |   |-- odom_to_pose.py               # Odometry to PoseStamped bridge
|   |   |   |-- path_publisher.py             # SLAM trajectory publisher
|   |   |   |-- camera_rotate.py              # 180 degree camera rotation
|   |   |   |-- waypoint_manager.py           # Nav2 waypoint handler
|   |   |   |-- calibrate_front.sh            # Camera calibration script
|   |   |   |-- calibrate_back.sh             # Camera calibration script
|   |   |-- config/
|   |   |   |-- slam_viz.rviz                 # RViz configuration
|   |   |   |-- ekf.yaml                      # EKF fusion parameters
|   |   |   |-- nav2_planner_params.yaml      # Nav2 planner config
|   |   |   |-- sensor_pkg.urdf               # Robot geometry (copy)
|   |
|   |-- kitware_slam/ros2_wrapping/
|   |   |-- lidar_slam/params/
|   |   |   |-- slam_config_indoor.yaml       # SLAM parameters (indoor)
|   |   |   |-- slam_config_outdoor.yaml      # SLAM parameters (outdoor)
|   |
|   |-- vins_fusion_ros2/config/ouster_dual_cam/
|   |   |-- front_mono_imu_config.yaml        # VINS front config
|   |   |-- back_mono_imu_config.yaml         # VINS back config
|   |   |-- front_camera.yaml                 # Front camera intrinsics
|   |   |-- back_camera.yaml                  # Back camera intrinsics
|   |
|   |-- sensor_pkg/
|       |-- urdf/sensor_pkg.urdf              # URDF robot model
|       |-- meshes/                           # STL mesh files
|
|-- install/                                  # Built packages (auto-generated)
|-- build/                                    # Build artifacts
|-- log/                                      # Build/run logs

~/maps/                                       # Persistent map storage
|-- slam_map_edges.pcd
|-- slam_map_planes.pcd
|-- slam_map_intensity_edges.pcd
|-- dense_map.pcd
```

### Edge Node — Key Files

```
/home/robotics/
|-- camera_launch.py                          # Dual camera ROS 2 launch file
|-- start_cameras.sh                          # Systemd startup script (with taskset)

/etc/
|-- netplan/50-cloud-init.yaml                # Network bridge configuration
|-- systemd/system/ros2_cameras.service       # Auto-start service

/opt/ros/jazzy/lib/
|-- libcamera*.so*                            # Symlinked to custom PiSP build
```

---

## Quick Reference Card

```
===================== QUICK LAUNCH =======================
  source ~/sensor_ws/install/setup.bash
  ros2 launch ouster_ros viz_lvi_slam.launch.py

===================== REBUILD =============================
  cd ~/sensor_ws
  colcon build --packages-select ouster_ros lidar_slam
  source install/setup.bash

===================== FRESH MAP ===========================
  rm ~/maps/slam_map_*.pcd ~/maps/dense_map.pcd

===================== SSH TO EDGE NODE ====================
  ssh robotics@10.42.0.149

===================== RESTART CAMERAS =====================
  ssh robotics@10.42.0.149 \
    "sudo systemctl restart ros2_cameras.service"

===================== KEY IP ADDRESSES ====================
  Compute Node:  10.42.0.1
  Edge Node:     10.42.0.149
  Ouster LiDAR:  10.42.0.58
===============================================================
```
