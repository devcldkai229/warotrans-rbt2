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

## 1. Hình học bánh và vị trí cảm biến (2026-11-15)

| Thông số | Giá trị | Nguồn |
|---|---|---|
| `wheel_radius_m` | 0.0485 | bản vẽ Hiwonder: bánh mecanum Ø97 × 48 mm. Chưa kiểm bằng cách lăn 10 vòng rồi đo quãng đường |
| `half_wheelbase_m` | 0.0975 | anh đo bằng thước: tâm trục trước tới tâm trục sau 195 mm, hai bên như nhau |
| `half_track_m` | 0.1075 | anh đo: tâm bánh trái tới tâm bánh phải 215 mm |
| `base_footprint → base_link` | (0, 0, 0.0485) | tâm xoay = tâm khung; base_link đặt ở chiều cao trục bánh |
| laser | (−0.018, 0, 0.1315), yaw 0 | anh đo: lùi 1.8 cm so với tâm khung, mặt quét cao 18 cm, hướng 0° của C1 đã nhìn về phía trước. TF từ `base_footprint` kiểm trên Pi: (−0.018, 0, 0.180) |
| imu | x 0.041, y 0, z 0.0515 (board cao 10 cm); **yaw CHƯA XÁC NHẬN** (sửa khi chạy) | anh đo: tâm board lệch 4.1 cm về phía trước. Board đặt phẳng (trọng lực chỉ trên trục +z) |
| ultrasonic_front | (0.125, 0, 0.0565), yaw 0 (cao 10.5 cm) | anh đo: lùi 2.5 cm từ mép trước khung; mép trước = 0.30/2 theo wiki |
| camera | (0.125, 0, 0.1015), yaw 0 (cao 15 cm, ngay trên ultrasonic); **pitch CHƯA XÁC NHẬN** | anh đo |
| khung (chỉ để vẽ) | 0.30 × 0.245 × 0.10 m | Hiwonder wiki, Black Mecanum Wheel Chassis |
| footprint bao ngoài (cho Nav2 sau này) | dài 0.30 m, rộng 0.263 m (215 + 48) | tính từ số đo |

Motor theo wiki của khung này: **JGB37-528R131-08**, tỉ số 1:131, encoder 11 cực, 100 vòng/phút không tải. Số xung kỳ vọng = 11 × 4 × 131 = 5764 counts mỗi vòng trục ra, trùng preset firmware `JGB528` (type 3). Hiện đang dùng preset JGB37 (1980): **cần xác nhận bằng cách đếm encoder** (§3). Anh có ghi "JGB36-520", nên chưa chốt.

## 2. Port motor và chiều

- 2026-11-15, anh xác nhận cổng → bánh: **M1 = FL, M2 = RL, M3 = FR, M4 = RR**.
- Dấu `port_signs = [+1, +1, −1, −1]` (bên trái +, bên phải −) lấy theo quy ước mecanum của Hiwonder (`jetauto_set_motors` trong `chassis_porting.c`), **chưa kiểm chứng trên robot**. Nếu bấm Tiến mà một bên lăn lùi thì đảo dấu bên đó.
- Map được lưu ở `~/.config/warotrans/rbt002_calibration.yaml` trên Pi. Launch nạp file này sau `rbt002.yaml`.

Lịch sử:

- 2026-11-15: đã quay thử M1..M4 ở +0.1 rps, mỗi motor 1 s. **Không bánh nào quay, chỉ nghe tiếng rè** (thiếu nguồn, xem §4), nên chưa xác định được.
- Khi đã có nguồn pin: vào UI → Calibration → giữ **+** từng cổng → chọn bánh và chiều trong form "Map cổng → bánh" → **Lưu**. Map được áp dụng ngay và lưu vào `~/.config/warotrans/rbt002_calibration.yaml` trên Pi.

## 3. Motor: JGB37-528R131-08 (anh xác nhận 2026-11-15)

- Datasheet (Hiwonder wiki): tỉ số 1:131, encoder AB 11 cực, 100 vòng/phút không tải, 90 vòng/phút định mức.
- **counts_per_rev = 11 × 4 × 131 = 5764**, trùng preset firmware `JGB528` → `motor_type: 3` (rps_limit 1.1, PID 300/2/12).
- `wheel_odom.counts_per_rev: 5764`. `/odom` đã chạy 50 Hz trên Pi.
- **Chưa kiểm bằng tay:** quay một bánh đúng 10 vòng thì phải ra khoảng 57 640 counts.
- Tốc độ: `max_wheel_rps` hiện là 0.5, tức 0.5 × π × 0.097 ≈ **0.15 m/s**. Firmware cho tối đa 1.1 vòng/s (≈ 0.34 m/s). Chỉ nâng sau khi chạy thử thật.

## 4. Pin: CHƯA XÁC NHẬN

- 2S2P Li-ion, tối đa 8.4 V (anh cung cấp). Ngưỡng cho phép chạy: 6.6–8.6 V.
- 2026-11-15: board luôn đọc **3.94 V**, kể cả khi bật công tắc. Pin không vào được board, motor ăn nguồn USB dội ngược (`docs/safety.md`).
- **Cần đo bằng đồng hồ:** điện áp pin, và điện áp tại cổng nguồn RRC Lite khi bật công tắc.

## 5. Vị trí cảm biến (geometry.yaml): CHƯA XÁC NHẬN

`base_footprint→base_link`, LiDAR, IMU, ultrasonic, camera: tất cả `null`.

## 6. IMU noise: CHƯA XÁC NHẬN

Đo variance khi robot đứng yên 60 s (`ros2 bag record /imu/data`), rồi điền `imu_accel_variance` và `imu_gyro_variance`. Bias tĩnh của gyro trong probe là khoảng +0.25 / −0.52 / +0.22 deg/s; chỉ là một lần đo, chưa phải số hiệu chuẩn.
