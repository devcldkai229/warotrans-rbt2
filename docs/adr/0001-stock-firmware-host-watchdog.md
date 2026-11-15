# ADR 0001: Chạy firmware RRC Lite gốc, watchdog phía host, patch là đề xuất

Ngày: 2026-11-15 · Trạng thái: chấp nhận

## Bối cảnh
Audit (`docs/audit/rrc_lite_protocol.md`) cho thấy firmware RRC Lite gốc không có command timeout, không gửi encoder và không đọc sonar trên cổng I2C. Người vận hành yêu cầu giữ firmware gốc, không flash khi chưa duyệt.

## Quyết định
- Driver chạy với firmware gốc. Watchdog nhiều lớp phía Pi (UI, gateway, base_controller, rrc_lite).
- Không publish odom, encoder hay ultrasonic khi firmware không cung cấp. Không suy odom từ lệnh.
- Viết patch firmware (watchdog MCU, encoder feedback, sonar, version) dưới dạng diff + tài liệu, chưa build.
- Driver tự nhận diện firmware đã patch (version reply hoặc gói feedback) để không phải đổi code khi nâng cấp.

## Hệ quả
- Chưa có bảo vệ khi mất USB hoặc Pi treo, nên mọi test motor phải kê bánh hoặc có người giữ công tắc pin.
- Chưa chạy được Nav2 hay fleet.
