# Nhật ký hiệu chuẩn RBT-002

Quy tắc: chỉ ghi số **đã đo**, kèm ngày, phương pháp và người đo. Chưa đo thì ghi **CHƯA XÁC NHẬN**. `config/rbt002.yaml` chỉ được lấy số từ file này.

## 0. Đã xác nhận

| Ngày | Mục | Giá trị | Phương pháp |
|---|---|---|---|
| 2026-11-15 | Cổng RRC Lite | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE2006352-if00` → ttyACM0 | `udevadm info` trên Pi |
| 2026-11-15 | Cổng LiDAR C1 | `/dev/serial/by-id/usb-Silicon_Labs_CP2102N_..._8edd7c2b506eef1198b2dcc2c169b110-if00-port0` → ttyUSB0 | `udevadm info` |
| 2026-11-15 | Baud RRC Lite | 1 000 000 | source FW + probe 10 s, 0 lỗi CRC |
| 2026-11-15 | Đơn vị IMU | accel g, gyro deg/s (\|a\| = 0.946 khi đứng yên) | `tools/pi_probe.py` |
| 2026-11-15 | Tần số IMU / pin | khoảng 38 Hz / 1051 ms | probe |
| 2026-11-15 | IMU có frame toàn 0 | có (az min = 0.0000) | probe |
| 2026-11-15 | Motor preset gửi khi kết nối | `motor_type 1` (JGB37). Một gói này làm 2 bánh sau đang tự quay dừng lại (anh xác nhận) | `pi_init_motors.py` |
| 2026-11-15 | Pin | 2S2P Li-ion, tối đa 8.4 V (anh cung cấp) → ngưỡng 6.6–8.6 V | — |
| 2026-11-15 | Board đọc pin | 3.94 V kể cả khi bật công tắc. Motor chỉ kêu ở 0.1 rps, áp sụt 3936 xuống 3893 mV | `pi_spin_test.py` (kê bánh, có đồng ý) |
| 2026-11-15 | LiDAR C1 | `sllidar_ros2` @3430009, 460800 baud, `scan_mode Standard`, `frame_id laser` | `~/ros2_ws/src/sllidar_ros2/launch/sllidar_c1_launch.py` trên Pi |

## 1. Hình học bánh: CHƯA XÁC NHẬN

| Thông số | Giá trị | Cách đo |
|---|---|---|
| `wheel_radius_m` | CHƯA XÁC NHẬN | thước kẹp đo đường kính bánh mecanum (chỗ con lăn tiếp đất) chia 2. Hoặc lăn 10 vòng, đo quãng đường rồi chia cho 20π |
| `half_wheelbase_m` | CHƯA XÁC NHẬN | khoảng cách tâm trục trước và trục sau, chia 2 |
| `half_track_m` | CHƯA XÁC NHẬN | khoảng cách tâm bánh trái và bánh phải, chia 2 |

## 2. Port motor và chiều

- 2026-11-15, anh xác nhận cổng → bánh: **M1 = FL, M2 = RL, M3 = FR, M4 = RR**.
- Dấu `port_signs = [+1, +1, −1, −1]` (bên trái +, bên phải −) lấy theo quy ước mecanum của Hiwonder (`jetauto_set_motors` trong `chassis_porting.c`), **chưa kiểm chứng trên robot**. Nếu bấm Tiến mà một bên lăn lùi thì đảo dấu bên đó.
- Map được lưu ở `~/.config/warotrans/rbt002_calibration.yaml` trên Pi. Launch nạp file này sau `rbt002.yaml`.

Lịch sử:

- 2026-11-15: đã quay thử M1..M4 ở +0.1 rps, mỗi motor 1 s. **Không bánh nào quay, chỉ nghe tiếng rè** (thiếu nguồn, xem §4), nên chưa xác định được.
- Khi đã có nguồn pin: vào UI → Calibration → giữ **+** từng cổng → chọn bánh và chiều trong form "Map cổng → bánh" → **Lưu**. Map được áp dụng ngay và lưu vào `~/.config/warotrans/rbt002_calibration.yaml` trên Pi.

## 3. Motor JGB36-520: CHƯA XÁC NHẬN

Cần biết **tỉ số truyền** (in trên nhãn motor, ví dụ 1:30) và **PPR encoder** (thường là 11 xung/vòng trục motor với dòng 520). Từ đó tính counts/vòng trục ra = PPR × 4 × tỉ số.

So với preset của firmware:

| type | counts |
|---|---|
| JGB520 | 3960 (tỉ số 90) |
| JGB37 | 1980 (tỉ số 45) |
| JGA27 | 1040 |
| JGB528 | 5764 (tỉ số 131) |

Nếu không khớp preset nào, PID và thang rps của firmware gốc sẽ sai. Khi đó cần firmware patch (MOTOR sub 0x06) hoặc chấp nhận sai số thang tốc độ.

## 4. Pin: CHƯA XÁC NHẬN

- 2S2P Li-ion, tối đa 8.4 V (anh cung cấp). Ngưỡng cho phép chạy: 6.6–8.6 V.
- 2026-11-15: board luôn đọc **3.94 V**, kể cả khi bật công tắc. Pin không vào được board, motor ăn nguồn USB dội ngược (`docs/safety.md`).
- **Cần đo bằng đồng hồ:** điện áp pin, và điện áp tại cổng nguồn RRC Lite khi bật công tắc.

## 5. Vị trí cảm biến (geometry.yaml): CHƯA XÁC NHẬN

`base_footprint→base_link`, LiDAR, IMU, ultrasonic, camera: tất cả `null`.

## 6. IMU noise: CHƯA XÁC NHẬN

Đo variance khi robot đứng yên 60 s (`ros2 bag record /imu/data`), rồi điền `imu_accel_variance` và `imu_gyro_variance`. Bias tĩnh của gyro trong probe là khoảng +0.25 / −0.52 / +0.22 deg/s; chỉ là một lần đo, chưa phải số hiệu chuẩn.
