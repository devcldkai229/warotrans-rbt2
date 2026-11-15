# Chạy motor lần đầu và hiệu chuẩn port/chiều (người vận hành thực hiện)

> Tài liệu này dành cho người đứng cạnh robot. Firmware gốc không có command timeout (`docs/safety.md`).

## 0. Chuẩn bị

- [ ] **Thứ tự bật nguồn:** bật Pi và chờ stack chạy (UI mở được) **trước**, rồi mới bật pin RRC Lite. Nếu RRC Lite chưa nhận gói motor, 2 bánh sau có thể tự quay (`docs/safety.md`).

- [ ] **Kê robot lên giá, 4 bánh không chạm đất.**
- [ ] Tay đặt trên công tắc pin RRC Lite trong suốt quá trình test.
- [ ] Pin RRC Lite đã bật. Kiểm tra `ros2 topic echo /rrc_lite/battery --field voltage` hiển thị điện áp pack thật, **không phải khoảng 3.9 V** (khoảng 3.9 V là board đang ăn nguồn USB).
- [ ] Ghi cell count và điện áp đầy/cạn của pack vào `docs/calibration.md` §4. Đặt `battery_min_v` và `battery_max_v` trong `config/rbt002.yaml`.

## 1. Chọn motor type (chưa chạy motor)

Xem `docs/calibration.md` §3. Khi `motor_type: -1`, driver **không gửi gói motor nào**, và mọi bước dưới đây đều bị chặn.

**Không bao giờ để firmware chạy với preset mặc định JGA27.** Sự cố 2026-11-15 (`docs/safety.md`): preset này có PID âm, và 2 bánh sau đã tự quay hết tốc độ khi bật pin. Vì vậy:

- Chọn preset có PID dương, gần nhất với JGB36-520 (JGB520 hoặc JGB37, tuỳ tỉ số truyền đo được). Ghi lý do chọn vào `docs/calibration.md`.
- Lần đầu bật pin sau khi chọn: **kê bánh**, đặt tay sẵn trên công tắc pin.
- Nếu vẫn có bánh tự quay khi setpoint = 0, thì encoder của motor đó đang đảo pha so với chiều motor. **Tắt pin ngay.** Cần đảo dây encoder (A/B) hoặc dây motor của port đó. Đây là việc phần cứng.

## 2. Bật driver với motor

```bash
source ~/ros2_ws/install/setup.bash        # sllidar_ros2
source ~/warotrans_rbt2_ws/install/setup.bash
ros2 launch warotrans_rbt002_bringup bringup.launch.py motors_enabled:=true
```

Mở UI `http://<pi>:8080/?token=<WARO_UI_TOKEN>`, bấm **Nhận quyền điều khiển**, rồi bấm **Calibration**.

## 3. Hiệu chuẩn port và chiều từng motor

> 2026-11-15: map đã được đặt (`docs/calibration.md` §2). UI hiện chỉ còn các nút lái; panel Calibration đã bỏ. Muốn sửa map thì dùng lệnh WebSocket `save_motor_map` (xem `app.py`). Phần dưới chỉ giữ để tham khảo.

Với từng port M1..M4:

1. Đặt slider tốc độ ở **0.05 rps**.
2. Giữ nút **+** trong khoảng 1 s rồi thả.
3. Ghi lại hai điều: **bánh nào quay** (FL/FR/RL/RR) và **bánh đó lăn về phía trước hay phía sau** của robot.
4. Nếu bánh quay mạnh bất thường, không dừng khi thả nút, hoặc kêu rít ở 100 %, **tắt pin**. Đó là dấu hiệu encoder không đếm hoặc đảo pha. Ghi lại port đó, đặc biệt là M4, vì firmware có bug encoder M4.

| Port | Bánh | `+` lăn về | `port_signs` |
|---|---|---|---|
| M1 | | trước / sau | +1 / −1 |
| M2 | | | |
| M3 | | | |
| M4 | | | |

Điền kết quả vào `config/rbt002.yaml` (`port_fl…port_rr`, `port_signs`), ghi ngày và người đo vào `docs/calibration.md` §2. Sau đó rebuild hoặc khởi động lại.

## 4. Đo hình học

Đo `wheel_radius_m`, `half_wheelbase_m`, `half_track_m` (`docs/calibration.md` §1). Thiếu các số này thì nút ENABLE bị khoá.

## 5. Chạy thử ENABLED, vẫn kê bánh

- Đặt slider giới hạn tốc độ ở **10 %**, rồi bấm ENABLE.
- Joystick **Tiến**: 4 bánh phải lăn về phía trước.
- **Xoay trái**: bánh trái lùi, bánh phải tiến.
- Thả tay: bánh dừng trong khoảng 0.5 s.
- Bấm **STOP**: chế độ chuyển về SAFE.
- Rút cáp USB RRC trong khi đang chạy: **motor sẽ KHÔNG dừng** với firmware gốc. Chỉ thử nếu muốn tự xác nhận điều này, và phải có tay trên công tắc pin.

## 6. Chạy trên sàn

Chỉ làm sau khi bước 5 đạt. Bắt đầu ở 10 %, khu vực trống, có người đứng cạnh công tắc pin.
