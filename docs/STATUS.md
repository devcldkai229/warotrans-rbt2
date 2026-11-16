# Trạng thái RBT-002 (chỉ ghi những gì có bằng chứng)

Cập nhật: 2026-11-15

## Đã xác minh trên phần cứng
- [x] Pi 5, Ubuntu 24.04.5, ROS 2 Jazzy, hostname `warotrans-rbt002`, user `waro` (nhóm dialout)
- [x] RRC Lite trên `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE2006352-if00`. Stream đọc thụ động 10 s: 0 lỗi CRC
- [x] IMU khoảng 38 Hz, đơn vị g / deg/s; báo pin 1 Hz
- [x] LiDAR C1 qua `sllidar_ros2` (anh đã kiểm tra trên Foxglove trước đó)
- [ ] Điện áp pack: chưa đọc được (3.95 V, board đang ăn nguồn USB)
- [x] 2026-11-15: driver chạy trên Pi với `motors_enabled:=false`.
  - Kết quả: link_ok, profile `stock`, IMU 50 Hz, `/scan` 10 Hz, 0 lỗi CRC.
  - ENABLE bị từ chối đúng với 4 blocker.
  - Mở cổng lần thứ hai bị từ chối (`Errno 11`). Token sai bị từ chối. Không có `/odom`.
- [x] **Sự cố 2 bánh sau tự quay** đã tìm ra nguyên nhân và đã xử lý.
  - Nguyên nhân: RRC Lite chưa nhận gói MOTOR nào sau khi có điện.
  - Một gói `set_motor_type(JGB37)` làm 2 bánh dừng ngay. Driver giờ gửi gói này đầu tiên mỗi khi kết nối (`docs/safety.md`).
- [x] 2026-11-15 11:04: stack chạy trên Pi (phiên tạm, không phải systemd), UI tại `http://192.168.137.200:8080`.
  - Từ laptop: HTTP 200, WebSocket, token OK. ENABLE và CALIBRATION bị từ chối đúng (pin 3.94 V, map motor chưa có). STOP trả về SAFE.
  - Mỗi node chỉ có một bản. Một bug crash `base_controller` khi STOP (logger rclpy) đã được sửa và kiểm chứng lại trên robot.
- [x] Nguồn pin: 8.04 V sau khi anh xử lý, `battery_ok` = true.
- [x] **Firmware patch đã nạp** (sao lưu bản gốc, verify, có cách rollback): có encoder, ultrasonic, khởi tạo motor ngay lúc boot, watchdog MCU (`docs/firmware/patch_proposal.md`).
- [x] Autostart systemd đã cài và kiểm chứng qua 2 lần reboot. DDS dùng localhost, nên các node thấy nhau ngay cả khi mạng lên muộn.
- [x] UI: `http://warotrans-rbt002.local:8080/?token=…`. IP Ethernet do DHCP của laptop cấp, đổi sau mỗi reboot (.200 → .129 → .51).
- [x] Map motor M1 = FL, M2 = RL, M3 = FR, M4 = RR (anh cung cấp); dấu theo quy ước Hiwonder, chưa kiểm chứng. Không còn blocker, ENABLE sẵn sàng.
- [x] UI đơn giản: giữ nút Tiến / Lùi / Xoay trái / Xoay phải (hoặc phím mũi tên), thanh tốc độ, STOP. Lần bấm đầu tự nhận quyền và tự ENABLE.
- [x] Reboot lần 3: stack tự chạy, nạp map, phục vụ UI mới.
- [ ] Autostart systemd: script `tools/install_autostart.sh` có sẵn nhưng chưa cài (cần anh duyệt, vì là thay đổi hệ thống lúc boot).
- [ ] Bất kỳ chuyển động motor nào: **chưa thử**, người vận hành sẽ làm theo `docs/first_motion_procedure.md`

- [x] 2026-11-15: firmware v2 đã nạp (đèn siêu âm đổi màu: xanh > 50, vàng 20–50, đỏ < 20 cm). Sau khi nạp: 0 lỗi CRC, siêu âm 10 Hz.
- [x] 2026-11-15: camera chạy (imx708_wide, libcamera và rpicam-apps của Raspberry Pi build từ source vào /usr/local). MJPEG 1280×720 @ 15 fps ở `:8081/stream`. Snapshot ra ảnh đúng chiều. Pi 54 °C, CPU khoảng 28 % khi stream.

## Đưa RBT-002 về cùng chức năng RBT-001 (plan 2026-11-15)
- [x] **Pha 0:** copy core (navigation, localization, slam, lane_planner, fleet, map V013) bằng `tools/sync_from_rbt1.py`, nguồn `fix/final-rbt1-precise-arrival@d44bd23` cùng working tree chưa commit. `colcon test` 553 test pass. Pi đã cài Nav2, slam_toolbox, robot_localization và build đủ 13 package.
- [x] **Overlay RBT-002** (`warotrans_rbt002_bringup/config/rbt002/`):
  - Nav2: footprint 0.30 × 0.264 m, tốc độ 0.30 m/s;
  - EKF: chỉ fuse gyro z, vì QMI8658 không có orientation;
  - AMCL: theo C1, alpha cao hơn cho mecanum;
  - gateway: RBT-002, không có nút xác nhận;
  - `fleet_base.yaml`: `/cmd_vel_motor` chỉ nhận từ `cmd_vel_authority_gate`, `auto_enable`.
- [x] `fleet.launch.py` và service một-chế-độ (`MODE=manual|fleet`, `tools/rbt2_mode.sh`).
- [x] **Chạy thử fleet trên Pi (motor tắt, chưa nối MQTT), 2026-11-15:**
  - toàn bộ lifecycle Nav2 `active`;
  - đủ các action `navigate_to_pose`, `follow_path`, `guarded_spin` / `guarded_backup`;
  - `/cmd_vel_motor` chỉ có gate publish;
  - TF `map→odom→base_footprint→laser/imu_link` đầy đủ;
  - EKF 22–28 Hz.
  - Load khoảng 9 trên 4 nhân: cần theo dõi.
- [x] Sửa bug: `rrc_lite_node` crash do deque bị sửa đồng thời giữa 2 thread khi CPU tải cao.
- [ ] Pha 1 (cần anh ở robot): đếm encoder 10 vòng, hiệu chuẩn odom thẳng/xoay, hướng IMU, tốc độ tối đa, watchdog khi đang chạy.
- [ ] Pha 2–3: AMCL ±90°, NavigateToPose thật, các leg V013, docking.
- [ ] Pha 4: đăng ký RBT-002 trong backend (cần anh cho phép ghi dev DB), `Traffic:Footprint` 0.32 × 0.27, home `EP-OUTBOUND`, camera.
- [ ] Pha 5: harness 2 robot, rồi chạy tuần tự, rồi đồng thời.

## Đã xác minh trong mô phỏng (mock board, WSL Jazzy)
- [x] `colcon build`: 7 package. `colcon test`: tất cả pass (xem lần chạy gần nhất). Web: vitest 16 test, `npm run build` OK
- [x] E2E qua WebSocket thật (`tools/mock_e2e.py`), 27/27 bước cho cả profile `stock` và `patched`: lease, ENABLE bị chặn khi chưa có lease, tiến/xoay không trộn, deadman, giới hạn tốc độ, STOP từ client khác, calibration 1 motor, đóng socket thì dừng, `/odom` và `/ultrasonic/front` chỉ có với firmware patch

## Chưa có / bị chặn
- E-stop phần cứng: chưa có (RBT-001 cũng chưa)
- Pin motor riêng: chưa có. Thử không motor dùng `tools/rbt2_mode.sh sim-battery 80` (telemetry báo `SIMULATED`), tắt bằng `sim-battery off`
- Pha 1 (đo ở robot): encoder 10 vòng, odom thẳng/xoay, trục IMU, tốc độ tối đa, watchdog khi đang chạy

## Sẵn sàng điều phối nhiều robot (2026-11-17)
- [x] `recovery_guard` nhận kích thước qua tham số (sửa ở warotrans, sync sang). Overlay `nav2_rbt002.yaml` đặt 0.15 × 0.132
- [x] `max_wheel_rps` chế độ manual 1.1 (giới hạn preset JGB528), fleet 1.0
- [x] `ROS_DOMAIN_ID=31` (service), `MOTORS_ENABLED=false` mặc định khi cài autostart
- [x] `tools/rbt2_readiness.py`: nodes, chỉ gate publish `/cmd_vel_motor`, TF, AMCL, pin, broker, lệch giờ ≤ 2 s
- [x] `tools/set_initial_pose.py` (copy từ RBT-001); bình thường dùng LOCALIZE trên web
- [x] Backend: footprint traffic chung 0.32 × 0.27 m, home RBT-002 = `EP-PARKING-B`, camera `warotrans-rbt002.local:8081`
- Khác biệt tham số với RBT-001: `docs/rbt1_vs_rbt2.md`
