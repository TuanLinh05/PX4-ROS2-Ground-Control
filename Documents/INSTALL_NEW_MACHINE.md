# Hướng Dẫn Cài Đặt Đồ Án Trên Máy Tính Mới
*(Dành cho máy Ubuntu 22.04 / WSL2 chưa cài đặt gì)*

Nếu bạn copy thư mục `DoAn1` sang một máy tính hoàn toàn mới để chạy mô phỏng và bảo vệ đồ án, bạn cần cài đặt môi trường cơ bản (ROS 2, PX4, DDS Agent) trước khi chạy.

## BƯỚC 1: Chạy Script Tự Động Cài Đặt Môi Trường

Tôi đã chuẩn bị sẵn một script tự động tải và cài đặt toàn bộ những thành phần cần thiết (thay vì phải gõ từng lệnh thủ công).

1. Mở Terminal (hoặc WSL) trên máy mới.
2. Di chuyển vào thư mục `DoAn1` (thư mục chứa file `setup_env.sh`).
3. Chạy lệnh sau:
   ```bash
   bash setup_env.sh
   ```
4. **Lưu ý trong quá trình chạy:** Script có thể sẽ yêu cầu bạn nhập mật khẩu `sudo` của máy tính. Hãy chú ý màn hình Terminal để nhập khi được hỏi. Quá trình này sẽ tải ROS 2, clone `PX4-Autopilot` (khá nặng), và build các tool liên quan nên sẽ mất khoảng **15 - 30 phút** tùy tốc độ mạng.

5. Sau khi Terminal báo **"CÀI ĐẶT HOÀN TẤT!"**, hãy **KHỞI ĐỘNG LẠI MÁY TÍNH** (hoặc tắt mở lại WSL) để các thiết lập ROS 2 có hiệu lực.

## BƯỚC 2: Build Đồ Án

Sau khi máy tính đã có môi trường, bạn cần build mã nguồn của đồ án (`DoAn1/src`).

1. Mở Terminal và di chuyển vào thư mục `DoAn1`:
   ```bash
   cd /đường/dẫn/đến/thư/mục/DoAn1
   ```
2. Build code bằng `colcon`:
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build
   ```

## BƯỚC 3: Chạy Hệ Thống GCS

Mỗi khi bạn muốn bật GCS để điều khiển drone, bạn chỉ cần dùng 2 lệnh sau:

```bash
cd /đường/dẫn/đến/thư/mục/DoAn1
source install/setup.bash
ros2 run px4_gcs gcs_node
```

Sau đó:
1. Mở trình duyệt Web tại địa chỉ: **http://localhost:8085**
2. Bấm nút **🚀 Launch All** ở menu bên trái. Hệ thống sẽ tự động bật Gazebo (mô phỏng Drone) và RViz2 (quan sát đồ thị 3D).

---
*Chúc bạn bảo vệ đồ án thành công!*
