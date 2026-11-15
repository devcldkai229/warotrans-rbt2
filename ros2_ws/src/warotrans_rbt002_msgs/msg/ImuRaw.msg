# IMU sample exactly as reported by the firmware (QMI8658). Units depend on the firmware build:
# stock FW-25 build = accel in g, gyro in deg/s (verified on RBT-002, docs/calibration.md).
std_msgs/Header header
float32[3] accel
float32[3] gyro
