# Sensor Workspace: LiDAR SLAM & Dual Camera Navigation

This workspace (`sensor_ws`) contains the packages and configuration files to build a robot navigation stack using an **Ouster OS0-128 LiDAR** and **two Raspberry Pi Camera Module 3 NoIR cameras** connected to a Raspberry Pi 5.

---

## 🛠 System Architecture & Folder Layout

The system is split between a **Main Computer (Host)** and a **Raspberry Pi 5 (Pi)** connected via a tethered Ethernet cable.

```
                  +-----------------------+
                  |     Main Computer     | (Host: ROS2 Humble)
                  |  IP: 10.42.0.1        |
                  +-----------+-----------+
                              |
                     (Tethered Ethernet)
                              |
                  +-----------+-----------+
                  |     Raspberry Pi 5    | (Pi: ROS2 Jazzy)
                  |  Bridge IP: 10.42.0.149|
                  +-----+-----------+-----+
                        |           |
            (Dual CSI)  |           | (Onboard Ethernet)
            +-----------+           +-----------+
            |                                   |
    +-------+-------+                   +-------+-------+
    |  CSI Cameras  |                   | Ouster LiDAR  |
    | (Front & Back)|                   | IP: 10.42.0.58|
    +---------------+                   +---------------+
```

### Created Files & Folders

#### 💻 Main Computer (Host)
*   **`/home/robotics/sensor_ws/`**: Your main ROS2 Humble workspace.
    *   `src/ouster-ros/`: The Ouster ROS2 driver package.
        *   *Patched file:* `src/ouster-ros/ouster-ros/src/point_cloud_compose.h` (Modified to prevent crashes on firmware `3.1.0` by dynamically checking if fields like `WINDOW` exist before reading).
        *   *New launch file:* `src/ouster-ros/ouster-ros/launch/viz_dual_cam.launch.py` (Launches LiDAR driver, static TFs, the URDF `robot_state_publisher`, and RViz preconfigured with cameras).
        *   *New RViz config:* `src/ouster-ros/ouster-ros/config/viz_dual_cam.rviz` (RViz configuration preloaded with PointCloud2, trajectory, map, and dual camera panels).
        *   *Workspace URDF:* `src/ouster-ros/ouster-ros/config/sensor_pkg.urdf` (Your mount's physical design file defining how the LiDAR and cameras are physically placed).
    *   `src/kiss-icp/`: The KISS-ICP LiDAR SLAM package.
*   **`/home/robotics/sensor_ws/README.md`**: This documentation file.

#### 🍓 Raspberry Pi 5 (`robotics@10.42.0.149`)
*   **`/etc/netplan/50-cloud-init.yaml`**: Modified to create a network bridge (`br0`) between the onboard Ethernet (`eth0` -> LiDAR) and the USB3-to-Ethernet adapter (`enx00e04c68344f` -> Host). This allows the LiDAR data to bypass the Pi's CPU and stream directly to the host computer at Layer 2.
*   **`/usr/lib/aarch64-linux-gnu/libcamera.so.0.7.1`**: Custom compiled `libcamera` built from source with Raspberry Pi 5 Image Signal Processor (`PiSP`) pipeline handler enabled.
*   **`/opt/ros/jazzy/lib/libcamera*.so*`**: Overridden with symlinks pointing to our custom compiled libraries to force ROS2 nodes to use the PiSP-enabled backend.
*   **`/home/robotics/camera_launch.py`**: A ROS2 launch file that starts both CSI cameras. Camera 0 publishes under `/camera/front/front_camera/image_raw` and Camera 1 publishes under `/camera/back/back_camera/image_raw`.
*   **`/home/robotics/start_cameras.sh`**: Helper shell script used by systemd to initialize the environment, set CycloneDDS parameters, and launch the cameras.
*   **`/etc/systemd/system/ros2_cameras.service`**: Systemd startup service that runs the cameras automatically on boot.

---

## 🚀 How to Launch Everything (LiDAR + Dual Cameras + SLAM)

We configure **CycloneDDS** on both the host and the Pi to bind only to the Ethernet interface. This prevents ROS2 packages from routing multicast packets over Wi-Fi and solves the point cloud latency/lag issues.

### Step 1: Start the Entire Cockpit (Host)
Run this command on your **host computer** to configure CycloneDDS and launch the LiDAR, static camera TFs, the URDF `robot_state_publisher`, RViz, and the KISS-ICP SLAM mapping node all in one go:

```bash
# Set GUI display variables
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority

# Configure CycloneDDS for the host's USB-to-Ethernet interface
export ROS_DOMAIN_ID=27
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>enx4cea416cc906</NetworkInterfaceAddress></General></Domain></CycloneDDS>'

# Source and launch
source /home/robotics/sensor_ws/install/setup.bash
ros2 launch ouster_ros viz_dual_cam.launch.py
```
*Note: If you ever want to run the LiDAR and cameras without SLAM, simply run `ros2 launch ouster_ros viz_dual_cam.launch.py run_slam:=false`.*

### Step 2: Verify / Start the Cameras (Raspberry Pi 5)
The cameras start automatically on boot via the systemd service (preconfigured to use CycloneDDS on `br0`).

To check the service status or view live camera logs on the Pi:
```bash
# Check status of the service
systemctl status ros2_cameras.service

# To restart it:
sudo systemctl restart ros2_cameras.service

# To view live camera logs:
journalctl -u ros2_cameras.service -f

ssh robotics@10.42.0.149
```

---

## 📊 Visualizing inside RViz

By running `ros2 launch ouster_ros viz_dual_cam.launch.py`, RViz will open preloaded with:
1.  **LiDAR Point Cloud** (Topic: `/ouster/points`): Real-time points colored by Z-axis height.
2.  **Trajectory Path** (Topic: `/kiss/path`): A **Red** line mapping your robot's path over time.
3.  **SLAM Map** (Topic: `/kiss/map`): A persistent **Green** point cloud showing the accumulated map of your environment.
4.  **Front/Back Camera Views**: Floating panels showing live, compressed video feeds from the Pi module cameras.
5.  **Fixed Frame**: Preconfigured to `odom_lidar` so your robot moves relative to the accumulated map!
