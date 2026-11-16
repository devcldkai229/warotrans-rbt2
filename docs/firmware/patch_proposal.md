# Patch firmware RRC Lite: ĐÃ NẠP 2026-11-15

**Trạng thái:** đã build bằng GCC và đã nạp lên RBT-002 theo yêu cầu của anh (pin tắt, bánh kê).

| Mục | Giá trị |
|---|---|
| Firmware gốc (sao lưu 512 KB, không bị khoá đọc, đọc lại 2 lần khớp nhau) | `rrc_lite_stock_20261115_044959.bin`, SHA256 `fb58ae8be195c075cc12049671a0fdd85253eeb10f37e2856bed39e5a2ac38b5`. Nằm ở `~/warotrans_rbt2_backups/` trên Pi và `firmware/backups/` trên laptop (gitignore) |
| Firmware mới | `rrc_lite_warotrans.bin`, 61 108 byte, SHA256 `f7f8e9370ff282af8f7879015dd51e911374400b7741de4614690ef369e19354` |
| Build | `firmware/rrc_lite_patch/build.sh` (arm-none-eabi-gcc 13.2, `apply_patch.py` + `gen_sources.py` + `Makefile` + `STM32F407VETx_FLASH.ld`) |
| Vào bootloader qua USB | `stm32flash -i "-rts&dtr,-dtr,,,,dtr,,,,:rts&dtr,-dtr,,,,dtr"` (6/6 lần). Chuỗi ngắn hơn thì không ổn định |
| Kiểm chứng sau nạp | gói encoder 50 Hz ngay từ boot, ultrasonic 10 Hz (44–55 mm), driver nhận profile `patched`, 0 lỗi CRC. Bật pin thì bánh đứng yên |

**v2 (2026-11-15, đã nạp):** thêm mục 9, đèn cảm biến siêu âm đổi màu theo khoảng cách.
- Ngưỡng: xanh > 50 cm, vàng 20–50 cm, đỏ < 20 cm, trễ 2 cm.
- Ghi một lần 7 byte vào thanh ghi 2–8 của sonar, chỉ khi đổi vùng màu. Đọc lỗi (0 mm) thì giữ nguyên màu.
- Đã kiểm logic chuyển vùng bằng gcc host (14 mẫu, 5 lần ghi).
- `rrc_lite_warotrans.bin` 61 912 byte, SHA256 `3ff5006695c08cba23dcd755f3d7c402bb6e9fcdbd40eb0765f9dfa917c719dc`.
- Bản v1 lưu ở `~/warotrans_rbt2_backups/rrc_lite_warotrans_v1_f7f8e937.bin`.

**Rollback:** dừng service, tắt pin, rồi chạy trên Pi:

```
stm32flash -b 115200 -i "-rts&dtr,-dtr,,,,dtr,,,,:rts&dtr,-dtr,,,,dtr" -w ~/warotrans_rbt2_backups/rrc_lite_stock_20261115_044959.bin -v -S 0x08000000 /dev/serial/by-id/usb-1a86_*
```

**Chưa kiểm chứng:** watchdog MCU 250 ms khi motor đang quay (rút USB lúc đang chạy), và PID với preset JGB37 trên motor JGB36-520.

Lưu ý: mở cổng serial sẽ reset MCU (mạch DTR/RTS). Với firmware mới thì vô hại, vì motor được khởi tạo về 0 ngay lúc boot.

- Diff: [`firmware/rrc_lite_patch/warotrans_rrc_lite_fw25.patch`](../../firmware/rrc_lite_patch/warotrans_rrc_lite_fw25.patch)
- Base: FW-25 `RosRobotControllerLite_ros_250811` (OPEN-SOURCE-UPIITA/C-ROS @ `9396c71`)
- Patch sửa 5 file.

## Vì sao cần patch

Firmware gốc có ba thiếu sót, phần mềm trên Pi không bù được:

1. **Không có command timeout:** mất USB hoặc Pi treo thì motor chạy mãi.
2. **Không gửi encoder:** không có odometry, nên chưa chạy được Nav2 hay fleet.
3. **Không đọc sonar** cắm ở cổng I2C mở rộng.

## Nội dung patch

| # | Thay đổi | File | Wire protocol |
|---|---|---|---|
| 1 | **MCU watchdog:** không nhận gói MOTOR trong 250 ms thì mọi setpoint về 0 | `packet_handle.c` (ghi tick), `app.c` (kiểm tra mỗi 10 ms) | không đổi. Host đã gửi setpoint 50 Hz và STOP 2 Hz khi idle |
| 2 | **Encoder feedback** 50 Hz | `app.c`, `packet_reports.h` | MOTOR sub `0x80` board→host `<BI4i4f>`: tick_ms, counts[4], rps[4] |
| 3 | **Ultrasonic** 10 Hz qua `hi2c1` địa chỉ 0x77, timeout 3 ms | `app.c`, `packet.h` | func mới `ULTRASONIC=12` `<H>` mm (0 = không đọc được). `NONE` dời từ 12 lên 13 |
| 4 | **Version** | `packet_handle.c` | SYS sub `0x10`: host gửi không có tham số, board trả `<BBBHI>` sub, major, minor, flags, build |
| 5 | Kiểm tra id 0..3 và độ dài gói. NaN thành 0. Đường serial áp `rps_limit` qua `encoder_motor_set_speed()` | `packet_handle.c` | không đổi |
| 6 | MOTOR sub `0x06` đặt `ticks_per_circle` và `rps_limit` tuỳ chỉnh (giữ nguyên PID của preset) cho JGB36-520 | `packet_handle.c` | `<Bff>` |
| 7 | Sửa bug encoder M4: `HAL_TIM_Encoder_Start(&htim3)` | `motor_porting.c` | không đổi |
| 8 | **Gọi `motors_init()` khi boot** (giống FW-24) và nạp preset có PID dương. **Chưa có trong file diff, cần thêm.** Lý do: RBT-002 tự quay 2 bánh sau khi chưa nhận gói MOTOR (`docs/safety.md`) | `app.c` | không đổi |

Driver Pi (`warotrans_rrc_lite`) đã hỗ trợ sẵn các mục 2, 3, 4 và được test bằng mock profile `patched`. Firmware gốc vẫn chạy bình thường với driver này.

## Rủi ro và điểm cần xác minh trước khi build

- **I2C1 (PB6/PB7) có phải cổng I2C mở rộng không:** UNCERTAIN, cần xem schematic RRC Lite. Nếu sai bus, sonar báo 0 nhưng không ảnh hưởng gì khác.
- **Thời gian vòng lặp app:** `HAL_I2C_Mem_Read` blocking tối đa 3 ms. IWDG khoảng 20 ms và được refresh mỗi 10 ms, nên còn biên. Cần đo trên bàn.
- **`packet_transmit` dùng heap lwmem.** Thêm gói 50 Hz làm tăng áp lực bộ nhớ và DMA TX. Cần theo dõi xem có mất gói IMU không.
- **Watchdog 250 ms:** host phải gửi lệnh đều. `base_controller` gửi 50 Hz khi ENABLED, `rrc_lite` gửi STOP 2 Hz khi idle. Gói STOP cũng làm mới tick, và điều này không gây hại vì STOP đặt setpoint về 0.
- **Thay đổi `NONE` từ 12 lên 13:** SDK Hiwonder cũ vẫn chạy được, vì chỉ thêm một function ID mới.

## Quy trình đề xuất (mỗi bước chờ anh duyệt)

1. **Backup trước:** dùng ST-Link đọc toàn bộ flash hiện tại, ví dụ `st-flash read rrc_lite_stock_<date>.bin 0x08000000 0x80000`. Lưu ngoài repo và ghi SHA256. Đây là đường rollback duy nhất, vì Hiwonder không công bố binary.
2. Build FW-25 **gốc** trước, so sánh hành vi với board hiện tại (IMU g/dps, pin 1 Hz). Mục đích là chứng minh source khớp với build đang chạy.
3. Apply patch, build, rồi test trên bàn **khi bánh được kê khỏi mặt đất**:
   - `tools/pi_probe.py` thấy func 3 sub 0x80, func 12 và version.
   - Rút USB khi motor đang quay: motor phải dừng trong ≤ 300 ms.
   - Encoder M4 phải đếm.
4. Chạy E2E thật với `motors_enabled:=true` theo `docs/first_motion_procedure.md`.
5. **Rollback:** nạp lại file `.bin` đã backup ở bước 1.
