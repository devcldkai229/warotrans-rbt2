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

## Đã xác minh trong mô phỏng (mock board, WSL Jazzy)
- [x] `colcon build`: 7 package. `colcon test`: tất cả pass (xem lần chạy gần nhất). Web: vitest 16 test, `npm run build` OK
- [x] E2E qua WebSocket thật (`tools/mock_e2e.py`), 27/27 bước cho cả profile `stock` và `patched`: lease, ENABLE bị chặn khi chưa có lease, tiến/xoay không trộn, deadman, giới hạn tốc độ, STOP từ client khác, calibration 1 motor, đóng socket thì dừng, `/odom` và `/ultrasonic/front` chỉ có với firmware patch

## Chưa có / bị chặn
- Wheel odometry, ultrasonic: cần firmware patch (đề xuất, chưa nạp)
- ENABLED: bị chặn cho tới khi đo hình học, map port/chiều, ngưỡng pin
- E-stop phần cứng: chưa có
- Fleet (MQTT) và Nav2: chưa tích hợp (cần odom)
