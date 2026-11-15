# WaroTrans RBT-002

Robot kho vận thứ hai: Raspberry Pi 5 + Hiwonder **RRC Lite** (STM32F407), 4 motor encoder bánh mecanum (**chỉ tiến, lùi, xoay**, không đi ngang), RPLIDAR C1, ROS 2 Jazzy.

> **Đọc trước:** [docs/safety.md](docs/safety.md). Firmware RRC Lite gốc **không có command timeout và không gửi encoder**. Mất USB thì motor vẫn chạy.

## Cấu trúc

```
ros2_ws/src/
  warotrans_msgs/                    copy từ RBT-001 (wire-compatible)
  warotrans_rbt002_msgs/             MotorSetpoints, RrcLiteStatus, SafetyState, SetMode, CalibrateMotor…
  warotrans_rrc_lite/                giao thức RRC Lite, giữ cổng serial duy nhất, watchdog, mock board
  warotrans_rbt002_hardware/         kinematics mecanum (không ngang), state machine an toàn, IMU/pin/odom
  warotrans_rbt002_description/      URDF từ geometry.yaml (null = chưa đo)
  warotrans_rbt002_bringup/          launch + config/rbt002.yaml (nơi duy nhất chứa hằng số robot)
  warotrans_rbt002_control_gateway/  FastAPI + WebSocket cho UI điện thoại
web/                                 React 19 + Vite + TypeScript + Tailwind 4
firmware/rrc_lite_patch/             đề xuất patch (CHƯA build, CHƯA nạp)
tools/                               pi_probe.py, mock_e2e.py, build_web.sh, deploy.sh, doctor.sh, check.sh
docs/                                audit, safety, calibration, interfaces, STATUS
```

## Chạy mô phỏng (không cần phần cứng)

```bash
tools/build_web.sh
cd ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash
ros2 launch warotrans_rbt002_bringup bringup.launch.py mock:=true motors_enabled:=true mock_profile:=stock
# mở http://localhost:8080   ·   kiểm thử tự động:
python3 ../tools/mock_e2e.py --expect-profile stock
```

Cần `fastapi`, `uvicorn`, `websockets` (trên Ubuntu: `sudo apt install python3-fastapi python3-uvicorn python3-websockets`).

## Chạy trên robot

```bash
tools/deploy.sh                    # từ WSL/Linux: đồng bộ, build trên Pi
# trên Pi:
source ~/ros2_ws/install/setup.bash           # sllidar_ros2
source ~/warotrans_rbt2_ws/install/setup.bash
export WARO_UI_TOKEN=<chuỗi bí mật>
ros2 launch warotrans_rbt002_bringup bringup.launch.py           # motor bị giữ ở STOP
```

Mobile control UI: `http://warotrans-rbt002.local:8080/?token=<WARO_UI_TOKEN>` (dùng tên này, vì IP Ethernet do DHCP cấp và đổi sau mỗi reboot). Token nằm trong `~/.config/warotrans/rbt002.env` trên Pi. Điện thoại phải cùng mạng với Pi.

Tự chạy khi boot: **đã cài** (`systemctl status warotrans-rbt002`). Script cài lại: `~/warotrans_rbt2_ws/tools/install_autostart.sh`.

Thứ tự bật nguồn: **Pi và stack trước, pin RRC Lite sau** (`docs/safety.md`).

Chạy motor: chỉ theo [docs/first_motion_procedure.md](docs/first_motion_procedure.md).

## Kiểm thử

```bash
cd ros2_ws && colcon test && colcon test-result --verbose
cd web && npm test && npm run build
tools/check.sh
```

## Nguồn sự thật (khi hai chỗ nói khác nhau)

1. Đo trên robot thật: `docs/calibration.md`
2. `docs/STATUS.md`
3. `docs/interfaces.md`, `docs/audit/rrc_lite_protocol.md`
4. Code

Giá trị chưa đo thì ghi **CHƯA XÁC NHẬN** (trong YAML ROS là `0`), không điền số đoán.
