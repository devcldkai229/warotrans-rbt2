# Interfaces RBT-002

## Node và topic

```
                 /cmd_vel (Twist)               /rrc_lite/motor_setpoints
control_gateway ─────────────────► base_controller ──────────────────────► rrc_lite ══ USB ══ RRC Lite
   ▲  │  (FastAPI :8080, /ws)        │  services ~/set_mode, ~/calibrate_motor,   │
   │  └── services ──────────────────┘           ~/set_speed_limit                │
   │                                                                              │
   └── telemetry ◄── /safety_state /motor_state ◄── base_controller               │
                     /rrc_lite/status /rrc_lite/battery ◄─────────────────────────┤
                     /imu/data /battery_state /battery/estimate /ultrasonic/front ◄── sensor_adapter ◄─ /rrc_lite/imu_raw, /rrc_lite/ultrasonic
                     /odom (+TF tuỳ chọn) ◄── wheel_odom ◄── /rrc_lite/encoders (chỉ firmware patch)
```

| Topic | Type | Publisher | Ghi chú |
|---|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | control_gateway | base_controller chỉ nhận từ `required_cmd_vel_publisher`. `linear.y` bị bỏ |
| `/rrc_lite/motor_setpoints` | warotrans_rbt002_msgs/MotorSetpoints | base_controller | index = port M1..M4, rps trục ra. rrc_lite chỉ nhận từ `base_controller` |
| `/rrc_lite/status` | RrcLiteStatus | rrc_lite | 5 Hz |
| `/rrc_lite/imu_raw` | ImuRaw | rrc_lite | đơn vị firmware (g, deg/s) |
| `/rrc_lite/battery` | sensor_msgs/BatteryState | rrc_lite | chỉ có voltage, khoảng 1 Hz |
| `/rrc_lite/encoders` | EncoderFeedback | rrc_lite | **chỉ firmware patch** |
| `/rrc_lite/ultrasonic` | UltrasonicRaw | rrc_lite | **chỉ firmware patch** |
| `/imu/data` | sensor_msgs/Imu | sensor_adapter | SI, `imu_link`, không có orientation (cov[0] = −1) |
| `/battery_state` | sensor_msgs/BatteryState | sensor_adapter | SOC NaN (chưa hiệu chuẩn) |
| `/battery/estimate` | warotrans_msgs/BatteryEstimate | sensor_adapter | contract của RBT-001 |
| `/ultrasonic/front` | sensor_msgs/Range | sensor_adapter | chỉ firmware patch |
| `/odom` | nav_msgs/Odometry | wheel_odom | chỉ firmware patch **và** khi tham số đã đo |
| `/safety_state` | SafetyState | base_controller | 10 Hz |
| `/motor_state` | MotorState | base_controller | 10 Hz, `measured_rps` NaN khi không có feedback |
| `/scan` | sensor_msgs/LaserScan | sllidar_node | `frame_id laser` |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | rrc_lite, base_controller | |

## TF

`map → odom → base_footprint → base_link → {laser, imu_link, ultrasonic_front, camera_link}`

- `odom → base_footprint`: chỉ `wheel_odom` (`publish_tf`) hoặc EKF, không bao giờ cả hai. Hiện tại **không có ai publish** (firmware gốc không có encoder).
- Các cạnh tĩnh lấy từ `warotrans_rbt002_description/config/geometry.yaml`. Cạnh nào chưa đo thì không có TF. Hiện tại chưa có cạnh nào được đo, nên robot_state_publisher không chạy trên robot thật.

## WebSocket `/ws`

Xem docstring trong `warotrans_rbt002_control_gateway/app.py`. Token qua `?token=` khi đặt `WARO_UI_TOKEN`.

## Serial

Xem `docs/audit/rrc_lite_protocol.md`.

## Camera (warotrans_camera)

- `http://<robot>:8081/stream`: MJPEG `multipart/x-mixed-replace; boundary=frame`, 1280×720 @ 15 fps. Cùng hợp đồng với `StreamRobotCamera` của warotrans-system (`RobotCamera:Robots:RBT-002`).
- `http://<robot>:8081/snapshot.jpg`, `/health`, `/` (trang xem nhanh).
- `rpicam-vid` chỉ chạy khi có người xem. Cần libcamera và rpicam-apps của Raspberry Pi (`tools/build_camera_stack.sh`).
