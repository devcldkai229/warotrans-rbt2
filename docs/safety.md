# An toàn RBT-002

## Sự cố 2026-11-15: 2 bánh sau tự quay khi bật pin

**Diễn biến:**
1. Chạy thử driver trên Pi với `motors_enabled:=false`. Driver chỉ gửi STOP-all (lúc kết nối và khi shutdown) và một truy vấn version.
2. Driver tắt. MCU vẫn được nuôi qua USB của Pi.
3. Người vận hành bật công tắc pin RRC Lite. **Hai bánh sau quay** mà không có lệnh chạy nào.

**Chẩn đoán ban đầu (SAI):** nghi gói STOP đã bật PID mặc định. Bản sửa đầu tiên (không gửi gói MOTOR nào khi chưa có `motor_type`) đã bị **đảo lại**.

**Bằng chứng thật (VERIFIED-HW, 10:35–10:40):**
1. USB được rút/cắm lại lúc 10:35:45. Sau đó Pi **không gửi byte nào** xuống board (không có tiến trình mở cổng), nhưng anh vẫn thấy 2 bánh sau quay.
2. Lúc 10:40, gửi **một** gói `set_motor_type(JGB37)` và một gói STOP-all. **Hai bánh sau dừng ngay** (anh xác nhận bằng mắt).

**Nguyên nhân (suy từ source FW-25 và khớp với 2 bằng chứng trên):**
- FW-25 khởi tạo motor theo kiểu lười: `motors_init()` chỉ chạy ở gói MOTOR đầu tiên. Bản FW-24 gọi hàm này ngay khi boot.
- Trước gói đó, chân PWM (AF, không pull) chưa được kích hoạt. Đầu vào driver motor M3/M4 (TIM9, TIM10/11) có thể bị thả nổi, và/hoặc PID đang dùng preset JGA27 có hệ số âm.
- Kết quả: **khi RRC Lite có điện mà chưa nhận gói MOTOR nào, motor có thể tự chạy.**

**Đã sửa:**
- Bắt buộc có `motor_type`. Trên phần cứng, node từ chối khởi động nếu thiếu.
- Lúc kết nối, driver gửi `set_motor_type` **trước**, rồi mới STOP.
- `config/rbt002.yaml` đặt `motor_type: 1` (JGB37, PID dương).
- Có test `test_motor_type_sent_on_connect_before_any_stop`.

**Quy trình bật nguồn:**
- Bật Pi và để stack chạy **trước**, rồi mới bật pin RRC Lite.
- Nếu bật pin khi stack chưa chạy, motor có thể tự quay cho tới khi driver kết nối.
- Sửa triệt để cần patch firmware gọi `motors_init()` ngay khi boot (`docs/firmware/patch_proposal.md`, mục 8).

**Bài học:** đừng gọi một thao tác ghi xuống board là "an toàn" khi chưa kiểm chứng hiệu ứng phía firmware. Và đừng sửa theo chẩn đoán trước khi có bằng chứng.

## Nguồn điện RRC Lite (2026-11-15, chưa giải quyết)

- Board luôn đọc khoảng **3.94 V**, kể cả khi gạt công tắc pin. Pin của robot là 2S2P Li-ion, tối đa 8.4 V.
- Khi quay thử 0.1 rps (kê bánh, có sự đồng ý của anh), **4 motor chỉ kêu rè rè, không quay**, và điện áp sụt từ 3936 xuống 3893 mV.
- Kết luận: motor đang ăn nguồn **5 V từ USB của Pi dội ngược qua board**, còn pin không vào được board. Pi có thể bị sụt áp hoặc reset.
- Cần đo bằng đồng hồ: điện áp pin, và điện áp tại cổng nguồn RRC Lite khi bật công tắc.
- Cho tới khi sửa xong, ngưỡng pin 6.6–8.6 V sẽ chặn ENABLED và CALIBRATION.

## Các lớp bảo vệ hiện có (firmware gốc)

| Lớp | Ở đâu | Hành vi |
|---|---|---|
| UI | `web/` | Thả joystick thì gửi (0,0). Nút STOP luôn bấm được. Tab bị ẩn thì dừng |
| Gateway | `control_gateway` | Chỉ một client có control lease. Deadman 300 ms. WebSocket đóng thì STOP. Lượng tử hoá 4 hướng ở phía server. Ai cũng bấm STOP được |
| Base controller | `base_controller` | Mặc định là SAFE. Bỏ `linear.y`. Kẹp vận tốc và gia tốc. Deadman `/cmd_vel` 300 ms. Chỉ nhận `/cmd_vel` từ `control_gateway`. Pin phải mới và nằm trong ngưỡng. Mất link thì về SAFE, ghi lỗi lại, và **không tự bật lại** |
| Driver | `rrc_lite` | Duy nhất một tiến trình giữ cổng (TIOCEXCL + flock). Lúc kết nối gửi **`set_motor_type` trước, rồi STOP**. Thiếu `motor_type` thì không chạy trên phần cứng. Chỉ gửi STOP khi `motors_enabled=false`. Watchdog 200 ms. STOP khi connect, khi idle (2 Hz), khi shutdown và khi nhận SIGINT/SIGTERM. Kẹp `max_wheel_rps`. Từ chối id sai hoặc NaN. Chỉ nhận setpoint từ `base_controller` |
| MCU | firmware gốc | **Không có command timeout.** IWDG chỉ xử lý trường hợp MCU treo |

## Các trường hợp CHƯA được bảo vệ

1. **Mất USB, Pi treo, kernel panic, hoặc `kill -9` driver:** motor giữ setpoint cuối và **tiếp tục chạy**.
2. **Encoder không đếm hoặc đảo pha:** PID của firmware đẩy PWM lên 100 %. Lệnh nhỏ vẫn có thể làm motor chạy hết tốc độ.
3. **Chọn sai motor type:** JGA27 có PID âm, nên chọn nhầm có thể gây phản hồi dương. Hiện tượng này **đã xảy ra** với preset mặc định (xem sự cố ở trên).
4. **Không có E-stop phần cứng.**

Cho tới khi có firmware patch (`docs/firmware/patch_proposal.md`) **và** E-stop phần cứng cắt nguồn pin motor:

- Chỉ chạy motor khi **bánh được kê khỏi mặt đất**, hoặc có người đứng cạnh, tay đặt trên công tắc pin RRC Lite.
- Mặc định `motors_enabled:=false`. Chỉ bật khi đang có người giám sát trực tiếp.
- Không chạy Nav2, không chạy fleet, không để robot tự hành.

## Quy tắc vận hành

- Khi Claude hoặc tự động hoá làm việc trên Pi, **không gửi lệnh chạy motor**. Người vận hành tự chạy thử.
- Không flash STM32 khi chưa được duyệt.
- Mọi thông số vật lý chưa đo đều để 0 (CHƯA XÁC NHẬN). Các node từ chối chức năng cần đến chúng.

## Sự cố 2026-11-15 (chiều): UI "timed out", motor không chạy

- **Triệu chứng:** UI báo timeout khi bấm điều khiển. `base_controller` thấy `link_ok=false` và `battery_ok=false`, dù `rrc_lite` vẫn nhận dữ liệu board 50 Hz, 0 lỗi CRC.
- **Nguyên nhân:** Pi không có pin RTC, nên đồng hồ nhảy khoảng 4 giờ khi NTP đồng bộ, 19 s sau boot. Stack đã khởi động trước cú nhảy. Khoảng 5 phút sau, các node mất liên lạc DDS với nhau; node mới mở thì vẫn thấy dữ liệu.
- **Khôi phục:** restart service. Theo dõi sau đó 9 phút, không còn lỗi.
- **Đã sửa:**
  - `warotrans-rbt002.service` chờ NTP tối đa 90 s trước khi khởi động.
  - `warotrans-rbt002-clock-guard.service` (`tools/rbt2_clock_guard.sh`) tự restart stack khi đồng hồ nhảy quá 30 s. Phần này chưa thử bằng cú nhảy thật.
- **Liên quan:** điện thoại mở UI không có `?token=` sẽ bị 403 và hiện "Mất kết nối".
