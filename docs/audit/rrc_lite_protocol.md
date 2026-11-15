# Audit: Hiwonder RRC Lite — giao thức serial và firmware

Ngày audit: 2026-11-15. Board: RRC Lite (STM32F407VET6), USB CH9102 `1a86:55d4`, trên RBT-002.

Ký hiệu độ tin cậy:
- **VERIFIED-SRC**: đọc thấy trong source.
- **VERIFIED-HW**: đo trên board thật của RBT-002.
- **UNCERTAIN**: chưa xác nhận được.

## 1. Nguồn đã đọc

| Mã | Nguồn | Commit | Ghi chú |
|---|---|---|---|
| SDK | Hiwonder/LanderPi `src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py` và `..._node.py`, `scripts/99-ttyACM0.rules` | `e93be9a` | Repo **chính thức** của Hiwonder; LanderPi dùng RRC Lite, rule udev ghi `1a86:55d4` |
| FW-25 | `RosRobotControllerLite_ros_250811` trong OPEN-SOURCE-UPIITA/C-ROS (`AutoModelCar/stm/...`) | `9396c71` | Firmware RRC Lite, header tác giả Hiwonder; **repo bên thứ ba** |
| FW-24 | Aroge21/SIU-ROSLite-Board | `7bf8455` | Bản port RRC Lite 2024 kèm nhật ký `Doc/log.txt`; bên thứ ba |
| WIKI | wiki.hiwonder.com "RosRobot Controller Program Analysis" | — | Tài liệu cho board RRC **bản đầy đủ**, không phải Lite |

Hiwonder không công bố repo firmware RRC Lite chính thức. **Không đọc được flash của board** (không có ST-Link), nên không biết build nào đang chạy. Kết quả dò trên board thật (mục 9) khớp với FW-25 về đơn vị IMU và về các gói được gửi.

Không dùng SDK của các board Hiwonder khác làm chuẩn. Ví dụ, `set_motor_duty` (sub 0x05) của TurboPi SDK **mâu thuẫn** với RRC Lite, vì trên RRC Lite sub 0x05 là "set motor type".

## 2. Lớp đường truyền: VERIFIED-SRC + VERIFIED-HW

| Mục | Giá trị | Bằng chứng |
|---|---|---|
| Baud | **1 000 000**, 8N1 | FW-25 `usart.c`: `huart1.Init.BaudRate = 1000000;` · SDK `Board(baudrate=1000000)` · probe: 0 lỗi CRC |
| Frame | `0xAA 0x55 \| func u8 \| len u8 \| data[len] \| crc8` | FW-25 `packet.h` (`struct PacketRawFrame`), `packet.c` (state machine) |
| CRC | CRC-8/MAXIM: poly phản xạ 0x8C (0x31), init 0, không xorout. Tính trên **func, len, data** | `checksum_crc8(&frame.function, len + 2)`. Bảng `crc8_table` trong SDK trùng với bảng sinh từ 0x8C (test `test_crc_table_matches_sdk`) |
| Endianness | little-endian | struct `#pragma pack(1)` trên Cortex-M |
| Version | **không có** field hay lệnh version | không có trong source |
| Lỗi CRC | firmware **bỏ qua im lặng** | `packet.c`: chỉ gọi handler khi CRC khớp |
| func ≥ NONE (12) | firmware bỏ, quay về tìm header | `packet.c` state FUNCTION |

Comment trong SDK ghi "Length Function" là **sai thứ tự**; code thực tế gửi Function trước Length.

## 3. Function ID: VERIFIED-SRC

`SYS=0, LED=1, BUZZER=2, MOTOR=3, PWM_SERVO=4, BUS_SERVO=5, KEY=6, IMU=7, GAMEPAD=8, SBUS=9, OLED=10, RGB=11, NONE=12`

GAMEPAD và SBUS đã bị bỏ trên RRC Lite (log FW-24: "砍掉了SBUS"). Board thật chỉ gửi func 0 (SYS) và 7 (IMU) khi idle (mục 9).

## 4. Điều khiển motor: VERIFIED-SRC

`packet_motor_handle()` trong FW-25 `Hiwonder/System/packet_handle.c`:

| sub | Payload | Ý nghĩa |
|---|---|---|
| 0x00 | `<BBf>` sub, id, rps | đặt 1 motor |
| 0x01 | `[1, n, (id u8, rps f32)*n]` | đặt nhiều motor (SDK dùng lệnh này) |
| 0x02 | `[2, id]` | dừng 1 motor |
| 0x03 | `[3, mask]` | dừng theo bitmask (bit i = motor i) |
| 0x05 | `[5, type]` | motor type cho **cả 4** motor, chỉ lưu RAM |

**Đơn vị tốc độ:** rev/s của **trục ra hộp số**, có dấu, điều khiển **vòng kín** trên MCU:

- TIM7 chạy mỗi 10 ms: `encoder_update(motor, 0.01, counter)` rồi `encoder_motor_control()` (`stm32f4xx_it.c` `TIM7_IRQHandler`).
- PID kiểu incremental, output PWM kẹp ±1000 (100 %). Vùng `|pwm| < 250` bị ép về 0 (`encoder_motor.c`).
- Không có chế độ duty open-loop trên RRC Lite.

**Lỗi và rủi ro trong firmware gốc:**

1. **Không kiểm tra id:** `motors[mscc->motor_id]` với id ≥ 4 ghi ra ngoài mảng. Driver luôn kiểm tra 0..3.
2. **Đường serial bỏ qua `rps_limit`:** gán thẳng `pid_controller.set_point`, không gọi `encoder_motor_set_speed()`. Host phải tự kẹp (`max_wheel_rps`).
3. **Motor được khởi tạo lười:** `motors_init()` chỉ chạy ở gói MOTOR **đầu tiên** (`motor_init_flag`).
4. **Bug encoder M4:** `HAL_TIM_Encoder_Start(&htim4, ...)` được gọi lần thứ hai thay vì `&htim3` (`motor_porting.c`). Bug này có cả ở FW-24 và FW-25. Encoder M4 có đếm hay không: UNCERTAIN.
5. **PID có thể chạy hết tốc độ:** nếu encoder không đếm (dây lỏng, bug 4) hoặc pha encoder ngược so với chiều motor, PID sẽ đẩy PWM tới ±1000. Hệ quả là **motor chạy 100 %** ngay cả khi setpoint nhỏ. Xem `docs/first_motion_procedure.md`.
6. **Gõ sai motor type thì bị âm thầm thay bằng JGB520.**
7. **Gói STOP cũng khởi động bộ điều khiển** (vì việc khởi tạo lười ở mục 3), với preset mặc định JGA27 có **PID âm**. **Đã xảy ra trên RBT-002 ngày 2026-11-15:** sau vài gói STOP, bật pin thì 2 bánh sau tự quay (`docs/safety.md`).

**Motor type** (`motors_param.h`, FW-25):

| type | Tên | counts / vòng trục ra | rps_limit | PID |
|---|---|---|---|---|
| 0 | JGB520 | 3960 (11×4×90) | 1.5 | 63 / 2.6 / 2.4 |
| 1 | JGB37 | 1980 (11×4×45) | 3.0 | 40 / 2 / 2 |
| 2 | JGA27 | 1040 (13×4×20) | 6.0 | −36 / −1 / −1 |
| 3 | JGB528 | 5764 (11×4×131) | 1.1 | 300 / 2 / 12 |
| mặc định lúc bật | = JGA27 | 1040 | 6.0 | −36 / −1 / −1 |

> RBT-002 dùng **JGB36-520** (theo khai báo). Tên này không khớp chắc chắn với preset nào. Tỉ số truyền và số xung encoder là **CHƯA XÁC NHẬN**. Nếu preset sai, thang tốc độ sai theo tỉ lệ counts. Riêng JGA27 dùng PID âm: chọn nhầm sẽ đảo chiều phản hồi và có thể làm motor chạy hết tốc độ. Cách xác định nằm ở `docs/calibration.md` §3. Driver **không gửi** motor type cho tới khi `motor_type` được cấu hình.

## 5. Encoder và feedback tốc độ: VERIFIED-SRC + VERIFIED-HW

**Firmware gốc không gửi encoder hay tốc độ motor về host.** Biến `counter` và `rps` có trong RAM nhưng không có `packet_transmit(PACKET_FUNC_MOTOR, ...)` nào trong FW-24, FW-25 hay bản RRC đầy đủ. Bảng parser của SDK cũng không có entry MOTOR. Probe 10 s trên board thật: 0 gói func 3.

Hệ quả: RBT-002 với firmware gốc **không có wheel odometry**. Driver không suy odom từ lệnh, mà báo `UNSUPPORTED_BY_FIRMWARE`.

## 6. IMU, pin, ultrasonic

**IMU: VERIFIED-SRC + VERIFIED-HW.**
- Chip QMI8658, đọc bằng I2C bit-bang trên PB10/PB11.
- func 7, payload `<6f>` = ax, ay, az, gx, gy, gz.
- FW-25: accel tính bằng g (`raw/ssvt_a`), gyro tính bằng deg/s. Dải ±4 g, **±128 dps** (gyro bão hoà khi xoay nhanh hơn 128 °/s ≈ 2.2 rad/s).
- Khoảng 50 Hz (`osDelay(20)`). Board thật đo được ~38 Hz.
- Board thật **thỉnh thoảng gửi frame toàn số 0**, nên driver loại bỏ chúng.

**Pin: VERIFIED-SRC + VERIFIED-HW.**
- func 0, payload `<BH>` = `0x04, mV`, khoảng 1 Hz.
- Cầu chia 11:1, lọc EMA. Đo đúng 1051 ms/gói.
- Comment trong SDK ghi "mAh" là **sai**; giá trị là mV.
- Ngưỡng `SYS sub 0x01` chỉ điều khiển còi báo pin yếu (mặc định 6300 mV), **không dừng motor**.

**Ultrasonic: VERIFIED-SRC (là không có).**
- Không có function ID, không có code đọc sonar trong firmware RRC Lite.
- Trên RBT-002, sonar cắm vào cổng I2C mở rộng của RRC Lite (anh xác nhận), nên **firmware gốc không đọc được**.
- FW-25 có khởi tạo I2C1 (PB6/PB7, 100 kHz) và một hàm `i2c_scan` không được dùng. Việc I2C1 chính là cổng mở rộng: UNCERTAIN, cần xem schematic.
- Sonar Hiwonder: địa chỉ 0x77, thanh ghi 0, 2 byte LE tính bằng mm, tối đa 5000 (TurboPi `Sonar.py`).

## 7. Watchdog, timeout, mất USB: VERIFIED-SRC

- **Không có command timeout.** Handler chỉ đặt setpoint, PID TIM7 chạy mãi. Motor **giữ setpoint cuối** cho tới khi nhận lệnh khác.
- FW-25 có **IWDG** (prescaler 32, reload 19, khoảng 20 ms), refresh trong vòng lặp `app_task_entry` mỗi 10 ms. IWDG chỉ reset MCU khi firmware **treo**. Sau reset, `motor_init_flag = 0` và PWM không chạy, tức là an toàn. **IWDG không phải command timeout.**
- Timeout 100 ms nêu trong WIKI thuộc task chassis/Bluetooth của board RRC đầy đủ, và task đó đã bị comment out trên RRC Lite.
- **Mất USB hoặc Pi treo:** MCU dùng pin riêng nên vẫn chạy, và **motor tiếp tục chạy theo setpoint cuối**. Watchdog phía Pi không cứu được trường hợp này. Chỉ có firmware patch hoặc E-stop phần cứng cứu được.

## 8. Port mapping và chiều quay: VERIFIED-SRC

| Port | Wire id | PWM | Encoder timer |
|---|---|---|---|
| M1 | 0 | TIM1 CH3/CH4 | TIM5 |
| M2 | 1 | TIM1 CH1/CH2 | TIM2 |
| M3 | 2 | TIM9 CH1/CH2 | TIM4 |
| M4 | 3 | TIM10/TIM11 | TIM3 (xem bug ở §4) |

- SDK nhận id 1..4 và tự trừ 1.
- **Không có lệnh đảo chiều** từng motor, nên chiều được đảo bằng dấu phía host (`port_signs`).
- Bánh nào cắm vào port nào trên RBT-002: **CHƯA XÁC NHẬN**.
- Quy trình hiệu chuẩn: `docs/first_motion_procedure.md` §3.

## 9. Probe trên board thật (VERIFIED-HW, 2026-11-15)

Chạy `tools/pi_probe.py`, nghe 10 s, **không ghi byte nào** lên serial:

```
port=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE2006352-if00 baud=1000000
raw_bytes=11121 crc_errors=0
  func  0 SYS   n=9    median_period=1051 ms
  func  7 IMU   n=381  (≈38 Hz)
battery_mV min=3945 max=3955
  ax +0.0007  ay +0.0511  az +0.9442   (min az 0.0000 → frame toàn 0)
  gx +0.25  gy -0.52  gz +0.22 (deg/s, bias tĩnh)
accel |a| mean=0.9456 → đơn vị g / deg/s (giống FW-25)
```

**Pin 3.95 V:** pin RRC Lite có lẽ đang tắt, board được cấp nguồn qua USB. Với điện áp này, mọi chế độ chạy motor đều bị chặn.

## 10. Hệ quả cho thiết kế driver

| Phát hiện | Cách driver xử lý |
|---|---|
| Không có command timeout | Watchdog trên Pi 200 ms. STOP khi connect, khi idle (500 ms), khi shutdown. Deadman `/cmd_vel` 300 ms |
| Gói MOTOR đầu tiên bật PID với gain JGA27 | Khi `motor_type` chưa đặt, **không gửi gói MOTOR nào** |
| Không có feedback encoder | Không publish `/odom` và encoder. UI chỉ hiển thị setpoint. Profile `patched` được nhận diện tự động |
| Firmware không kẹp rps, không kiểm tra id | `max_wheel_rps` kẹp trong `protocol.encode_motor_speeds`, id 0..3 |
| Frame IMU toàn 0 | `ImuConverter` loại bỏ |
| Pin không dừng motor | `base_controller` chặn ENABLED/CALIBRATION khi điện áp ngoài `[battery_min_v, battery_max_v]` hoặc cũ hơn 3 s |
| Motor type mặc định JGA27 | Không gửi `motor_type` cho tới khi xác định đúng |

Đề xuất sửa firmware: `docs/firmware/patch_proposal.md`.
