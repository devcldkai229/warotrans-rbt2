# ADR 0002: Core của RBT-001 được copy nguyên trạng, khác biệt RBT-002 chỉ nằm trong overlay

Ngày: 2026-11-15 · Trạng thái: chấp nhận (anh chọn "copy vào repo rbt2")

## Bối cảnh
RBT-002 phải có đủ chức năng của RBT-001: SLAM, AMCL kèm EKF, Nav2 cùng lane planner, costmap filter, docking, robot gateway semantic traffic và authority gate. Phần cứng hai robot khác nhau, nhưng các package trên không phụ thuộc phần cứng.

## Quyết định
- Copy `warotrans_navigation` (chỉ bundle bản đồ đang vận hành), `warotrans_localization`, `warotrans_slam`, `warotrans_lane_planner`, `warotrans_fleet` cùng `warotrans_simulation/scripts/semantic_map_adapter.py` bằng `tools/sync_from_rbt1.py`. Mỗi lần sync ghi lại commit nguồn trong `UPSTREAM.md`.
- **Không sửa file core trong repo này.** Khác biệt của RBT-002 (footprint, tốc độ, AMCL, EKF, gateway) đặt trong `warotrans_rbt002_bringup/config/rbt002/*.yaml` và tham số launch.
- Bringup RBT-002 include trực tiếp `localization.launch.py`, `ekf.launch.py` và `navigation.launch.py` của core. Base (RRC Lite, base_controller, sensor_adapter, wheel_odom, C1) là của RBT-002.

## Hệ quả
- Mỗi khi RBT-001 sửa core, chạy `python tools/sync_from_rbt1.py`, rồi review diff và chạy `colcon test`.
- Nếu một giá trị bị hardcode trong core (ví dụ `recovery_guard.GuardConfig`) thì sửa ở repo warotrans: chuyển thành tham số, giữ mặc định của RBT-001, rồi sync lại.
