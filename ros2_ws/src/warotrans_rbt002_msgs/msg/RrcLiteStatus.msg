# Link and firmware state of the RRC Lite driver (warotrans_rrc_lite).
std_msgs/Header header
bool connected            # port open
bool link_ok              # connected and valid frames arriving (rx_age_s <= rx_stale_s)
string port
string firmware_profile   # unknown | stock | patched
string firmware_version   # empty for stock firmware (no version query)
uint64 rx_frames
uint64 crc_errors
uint64 malformed_frames
uint64 discarded_bytes
float32 rx_age_s          # +inf if nothing received yet
float32 imu_rate_hz
float32 setpoint_age_s    # +inf when idle
bool watchdog_active      # true = host is holding motors at STOP
uint32 watchdog_trips     # nonzero setpoints that went stale and were stopped
uint32 rejected_setpoints
uint32 reconnects
bool motors_allowed       # driver parameter motors_enabled
bool encoder_feedback     # firmware reports encoders (patched firmware only)
bool ultrasonic_available # firmware reports ultrasonic (patched firmware only)
float32[4] last_sent_rps
string last_error
