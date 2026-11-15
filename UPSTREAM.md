# Code lấy từ RBT-001

Nguồn: repo `warotrans` (RBT-001) @ `d44bd23e18abb43b7148ef8fa33f845afc638c22`.

## Copy nguyên (giữ tên, không sửa logic)

| RBT-002 | RBT-001 | Thay đổi |
|---|---|---|
| `ros2_ws/src/warotrans_msgs/` | `ros2_ws/src/warotrans_msgs/` | bỏ thư mục `action/`, `srv/` vì cả hai đều trống. Wire-compatible với backend / fleet |
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
- `warotrans_fleet` (MQTT gateway): cần `/odom` và Nav2, sẽ dùng khi có firmware patch
- `warotrans_teleop`: thay bằng gateway mới (WebSocket, lease, calibration)
