# Per-port motor state (warotrans_rbt002_hardware). Index = motor port M1..M4.
std_msgs/Header header
string[4] wheel             # wheel at each port: FL / FR / RL / RR, or "?" while the map is unconfigured
float32[4] setpoint_rps     # last setpoint sent (host side)
bool feedback_valid         # true only with patched firmware and fresh encoder reports
float32[4] measured_rps     # NaN when feedback_valid is false
int64[4] encoder_counts     # 0 when feedback_valid is false
float32 feedback_age_s      # +inf when never received
