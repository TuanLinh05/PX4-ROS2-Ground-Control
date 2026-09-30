# PX4 ROS 2 Ground Control Station (GCS)

<a id="english"></a>**🇬🇧 English** · [🇻🇳 Tiếng Việt](#tieng-viet)

A **Ground Control Station (GCS)** for **PX4 SITL/Gazebo** built on **ROS 2**. It lets you monitor and control the drone and design flight trajectories visually, through a web interface and **RViz2**.

## 👨‍💻 Authors and credits
The project combines original core control logic with a separately developed control interface (GUI) and 3D visualisation module.

*   **Web interface (GUI / `px4_gcs`) and 3D visualisation (`px4_trajectory_visualizer`)**: designed, developed and optimised by **TuanLinh05**.
*   **Core backend (PID and LQR control, test bridge, etc.)**: the original source code belongs to its original author.

## 📂 Source structure

*   `px4_gcs/`: (Author: TuanLinh05) Web GUI package for real-time telemetry monitoring, process management and trajectory design (Trajectory Designer).
*   `px4_trajectory_visualizer/`: (Author: TuanLinh05) Node that renders the actual flight path, setpoints, waypoints and the drone's 3D frames (TF) in RViz2.
*   `px4_gen_trajectory/`: Node that generates and publishes the trajectory for the drone to follow. It was adapted to hover by default and to accept trajectories drawn in the GUI.
*   `px4_position_controller/`: Position controller using PID.
*   `px4_lqr_controller/`: Position controller using LQR.
*   `px4_msgs/`: PX4 message definitions (uORB), the bridge between ROS 2 and PX4 over DDS.
*   `px4_test_bridge/`: Node for basic communication tests.

## 🚀 Getting started

### Requirements
*   Ubuntu 22.04 (or WSL2)
*   ROS 2 Humble
*   PX4-Autopilot (with the Micro XRCE-DDS Agent configured)

### Build and run
1. **Build the workspace:**
   ```bash
   cd src/..
   colcon build
   source install/setup.bash
   ```

2. **Start the GCS manager node:**
   ```bash
   ros2 run px4_gcs gcs_node
   ```

3. **Open the control interface (GUI):**
   * Open a browser at `http://localhost:8085`.
   * Click **Launch All** in the "Process Manager" to start PX4 SITL, the Agent and the controllers, and open the 3D view in **RViz2**.

---
*Project 1 (Đồ án 1) – semester HK252*

---

<a id="tieng-viet"></a>

## 🇻🇳 Tiếng Việt

[🇬🇧 English](#english) · **🇻🇳 Tiếng Việt**

Dự án này là hệ thống Ground Control Station (GCS) dành cho PX4 SITL/Gazebo trên nền tảng ROS 2. Cho phép giám sát, điều khiển, và thiết kế quỹ đạo bay cho drone một cách trực quan trên nền web và RViz2.

### 👨‍💻 Tác giả và Bản quyền
Dự án là sự kết hợp giữa các core logic điều khiển (original) và giao diện điều khiển (GUI) cùng với module giám sát trực quan 3D (visualizer) được phát triển riêng.

*   **Phần Giao diện Web (GUI / `px4_gcs`) và Trực quan hóa 3D (`px4_trajectory_visualizer`)**: Được thiết kế, phát triển và tối ưu bởi **TuanLinh05**.
*   **Phần backend core (Điều khiển PID, LQR, Test Bridge, v.v...)**: Mã nguồn gốc (src) thuộc về tác giả nguyên bản.

### 📂 Cấu trúc mã nguồn

*   `px4_gcs/`: (Tác giả: TuanLinh05) Gói Web GUI cung cấp giao diện giám sát Realtime Telemetry, quản lý process, và thiết kế quỹ đạo (Trajectory Designer).
*   `px4_trajectory_visualizer/`: (Tác giả: TuanLinh05) Node giao tiếp với RViz2 để render 3D quỹ đạo đường bay thực tế, điểm đích (setpoint), điểm theo dõi (waypoint), và hệ tọa độ 3D (TF) của drone.
*   `px4_gen_trajectory/`: Node tạo và phát quỹ đạo bay để drone theo dõi. (Đã được điều chỉnh để hỗ trợ Hover mặc định và nhận lệnh vẽ quỹ đạo từ GUI).
*   `px4_position_controller/`: Bộ điều khiển vị trí dùng thuật toán PID.
*   `px4_lqr_controller/`: Bộ điều khiển vị trí dùng thuật toán LQR.
*   `px4_msgs/`: Các file định nghĩa message (uORB) của PX4, cầu nối giữa ROS 2 và PX4 qua DDS.
*   `px4_test_bridge/`: Node kiểm thử giao tiếp cơ bản.

### 🚀 Hướng dẫn khởi chạy

#### Yêu cầu hệ thống
*   Ubuntu 22.04 (hoặc WSL2)
*   ROS 2 Humble
*   PX4-Autopilot (cùng cấu hình Micro XRCE-DDS Agent)

#### Build và Chạy
1. **Build dự án:**
   ```bash
   cd src/..
   colcon build
   source install/setup.bash
   ```

2. **Chạy Node GCS Manager:**
   ```bash
   ros2 run px4_gcs gcs_node
   ```

3. **Mở giao diện điều khiển (GUI):**
   * Mở trình duyệt và truy cập: `http://localhost:8085`
   * Bấm **Launch All** trong "Process Manager" để hệ thống tự động gọi PX4 SITL, Agent, Controllers và mở sẵn không gian 3D trên **RViz2**.

---
*Đồ án 1 - Khóa HK252*
