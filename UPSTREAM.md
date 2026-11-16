# Code lấy từ RBT-001

Nguồn: repo `warotrans` (RBT-001) @ `d44bd23e18abb43b7148ef8fa33f845afc638c22`.

## Copy nguyên (giữ tên, không sửa logic)

| RBT-002 | RBT-001 | Thay đổi |
|---|---|---|
| `ros2_ws/src/warotrans_msgs/` | `ros2_ws/src/warotrans_msgs/` | từ 2026-11-17 đồng bộ bằng `tools/sync_from_rbt1.py` như các core package |
| `warotrans_rbt002_hardware/battery_soc.py` | `warotrans_hardware/warotrans_hardware/battery_soc.py` | không đổi. Cấu hình `divider_gain=1` vì RRC Lite báo thẳng mV của pack |
| `warotrans_rbt002_hardware/test/test_battery_soc.py` | `warotrans_hardware/test/test_battery_soc.py` | chỉ đổi đường import |

## Phỏng theo (adapted)

| RBT-002 | Lấy gì từ RBT-001 |
|---|---|
| `warotrans_rbt002_hardware/wheel_odom.py` | `wheel_odom_node.py`: tích phân midpoint, delta theo đồng hồ MCU, xử lý wrap và MCU reset, từ chối tham số ≤ 0, `base_footprint` |
| `warotrans_rrc_lite/rrc_lite_node.py`, `base_controller_node.py` | `esp32_bridge_node.py`: thread đọc, reconnect, kiểm tra publisher duy nhất, `motors_enabled`, gửi 0 khi shutdown |
| `warotrans_rbt002_description` | `warotrans_description`: geometry.yaml với `null` = chưa đo, báo lỗi rõ ràng. Thay xacro bằng `urdf_gen.py` vì Pi không cài xacro |
| `warotrans_rbt002_control_gateway` | `demo/backend/app/ros_gateway.py` + `main.py`: rclpy spin trên thread, FastAPI, WebSocket |
| `docs/*` | quy tắc "CHƯA XÁC NHẬN", thứ tự nguồn sự thật, TF contract |

## Chưa dùng
- `warotrans_teleop`: thay bằng gateway mới (WebSocket, lease, calibration)

## Core package dùng chung (copy nguyên trạng, đồng bộ bằng `tools/sync_from_rbt1.py`)

`warotrans_navigation` (chỉ mang bundle bản đồ đang vận hành), `warotrans_localization`, `warotrans_slam`, `warotrans_lane_planner`, `warotrans_fleet`, `warotrans_msgs`. `tools/set_initial_pose.py` copy từ `warotrans/tools`.

Quy tắc:
- **Không sửa các package này ở đây.**
- Mọi khác biệt của RBT-002 nằm trong `warotrans_rbt002_bringup/config/rbt002/*.yaml` (overlay) và trong tham số launch.
- Nếu RBT-002 cần một giá trị mà RBT-001 đang hardcode, sửa ở repo warotrans (chuyển thành tham số) rồi sync lại.

| Ngày | Việc | Nguồn | Thay đổi | Ghi chú |
|---|---|---|---|---|
| 2026-11-15 | core sync | `fix/final-rbt1-precise-arrival@d44bd23` + working tree chưa commit (bản precise-arrival đang chạy trên RBT-001) | +156 ~0 -0 | warotrans_navigation, warotrans_localization, warotrans_slam, warotrans_lane_planner, warotrans_fleet; maps map_one_v013 |
| 2026-11-15 | core sync | `fix/final-rbt1-precise-arrival@d44bd23` + working tree chưa commit | +0 ~1 -0 | warotrans_navigation, warotrans_localization, warotrans_slam, warotrans_lane_planner, warotrans_fleet; maps map_one_v013 |
| 2026-11-17 | core sync | `feature/rbt2-params@6579cf3` + working tree chưa commit | +2 ~2 -0 | warotrans_navigation, warotrans_localization, warotrans_slam, warotrans_lane_planner, warotrans_fleet, warotrans_msgs; maps map_one_v013 |
