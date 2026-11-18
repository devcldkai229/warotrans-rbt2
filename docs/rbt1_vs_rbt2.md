# RBT-001 và RBT-002: tham số khác nhau

Chỉ ghi những gì khác nhau. Mọi giá trị RBT-002 nằm trong overlay `ros2_ws/src/warotrans_rbt002_bringup/config/` hoặc tham số launch. Core package không bị sửa (xem `UPSTREAM.md`).

Dấu "chưa đo" nghĩa là giá trị lấy từ bản vẽ hoặc thước. Giá trị đó chưa được kiểm bằng bài đo chạy thật (Pha 1).

## Phần cứng

| | RBT-001 | RBT-002 |
|---|---|---|
| Khung | tự chế, 4 bánh skid-steer | Hiwonder, 4 bánh mecanum (chạy như differential, vy = 0) |
| Mạch điều khiển | ESP32, `esp32_bridge_node`, watchdog lệnh 300 ms | RRC Lite (firmware patch v2), `rrc_lite` + `base_controller` |
| IMU | BNO055 (có hướng) | QMI8658 trên RRC Lite, chỉ fuse gyro z |
| Lidar | RPLIDAR, x 0.10 m, cao khoảng 0.175 m, xoay +90° | RPLIDAR C1, x −0.018 m, cao 0.18 m, xoay 0 |
| Nút xác nhận | có (GPIO) | chưa có (`human_confirm_enabled: false`) |

## Hình học và odometry

| | RBT-001 | RBT-002 |
|---|---|---|
| Tâm quay so với tâm thân | lùi sau 0.05 m | trùng tâm thân |
| Footprint Nav2 (`base_footprint`) | `[[0.21,0.13],[0.21,-0.13],[-0.11,-0.13],[-0.11,0.13]]` | `[[0.15,0.132],[0.15,-0.132],[-0.15,-0.132],[-0.15,0.132]]` |
| `recovery_guard` nửa dài × nửa rộng | 0.16 × 0.13 | 0.15 × 0.132 |
| Bánh | r 0.033 m, 2481.9 / 2468.6 tick/vòng | r 0.0485 m (bản vẽ, chưa đo), 5764 count/vòng (preset JGB528, chưa đếm tay) |
| Vết bánh | hiệu dụng 0.4535 m (đã đo) | nửa chiều dài cơ sở 0.0975, nửa vết 0.1075 (thước, hệ số quay chưa đo) |

## Tốc độ

| | RBT-001 | RBT-002 |
|---|---|---|
| Chạy thường (RPP, velocity smoother) | 0.35 m/s | 0.30 m/s (giới hạn bánh 1.0 rps ≈ 0.30 m/s ở fleet) |
| Lùi | 0.18 m/s (core) | 0.18 m/s |
| Xoay tối đa | core | 0.6 rad/s |

## Fleet và backend

| | RBT-001 | RBT-002 |
|---|---|---|
| `robot_code` | RBT-001 | RBT-002 |
| `ROS_DOMAIN_ID` | 0 (`pi_run_semantic.sh`) | 31 (service, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`) |
| Chạy stack | `tools/pi_run_semantic.sh` | systemd `warotrans-rbt002`, `tools/rbt2_mode.sh fleet <broker>` |
| Motor mặc định | `MOTORS=false` | `MOTORS_ENABLED=false` |
| Home (backend `Traffic:RobotHomeEndpoints`) | EP-PARKING-A | EP-PARKING-B |
| Camera (backend `RobotCamera:Robots`) | `warotrans.local:8081` | `warotrans-rbt002.local:8081` |
| Footprint traffic backend (chung) | 0.32 × 0.27 m, padding 0.02 | như RBT-001 |
| Bản đồ | MAP-ONE-V013 | MAP-ONE-V013 (cùng bundle, cùng fingerprint) |
