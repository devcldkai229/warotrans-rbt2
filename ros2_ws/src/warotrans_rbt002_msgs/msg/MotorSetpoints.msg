# Wheel speed setpoints for the RRC Lite, index = motor PORT M1..M4 (wire id 0..3).
# Signed output-shaft revolutions per second, exactly as sent to the firmware (sign/port mapping
# already applied by warotrans_rbt002_hardware). Only base_controller publishes this topic.
std_msgs/Header header
float32[4] rps
