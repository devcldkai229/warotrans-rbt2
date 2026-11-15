# PATCHED FIRMWARE ONLY. The stock RRC Lite firmware never reports encoders.
# Index = motor port M1..M4. counts = cumulative encoder counts, rps = firmware-filtered speed.
std_msgs/Header header
uint32 mcu_ms
int32[4] counts
float32[4] rps
