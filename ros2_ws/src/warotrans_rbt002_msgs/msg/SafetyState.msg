# Drive safety state machine (warotrans_rbt002_hardware/base_controller).
uint8 MODE_SAFE=0
uint8 MODE_ENABLED=1
uint8 MODE_CALIBRATION=2
std_msgs/Header header
string units               # mps (geometry measured) | percent (fraction of max_wheel_rps, no geometry) | none
uint8 mode
string mode_name
string[] blockers          # why ENABLED / CALIBRATION is refused right now (empty = allowed)
string[] faults            # latched events that forced SAFE since the last mode request
bool link_ok
bool battery_ok
float32 battery_voltage    # NaN if unknown
float32 battery_age_s
bool motors_enabled_param
float32 cmd_age_s          # age of the last /cmd_vel, +inf if none
float32 max_linear_mps     # effective limit after speed_scale (fraction 0..1 when units == percent)
float32 max_angular_radps
float32 speed_scale        # 0..1 of the configured limits
uint32 lateral_rejected    # /cmd_vel messages whose linear.y was dropped
float32 cmd_linear_mps     # command after limits
float32 cmd_angular_radps
uint8 calibration_motor    # 1..4, 0 = none
float32 calibration_rps
