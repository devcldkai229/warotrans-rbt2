// Mirrors warotrans_rbt002_control_gateway/bridge.py snapshot(). Non-finite numbers arrive as null.

export type Num = number | null;
export type Mode = "SAFE" | "ENABLED" | "CALIBRATION";
export type Direction = "STOP" | "FORWARD" | "REVERSE" | "ROTATE_LEFT" | "ROTATE_RIGHT";

export interface SafetyTelemetry {
  units: "mps" | "percent" | "none";
  mode: Mode;
  blockers: string[];
  faults: string[];
  link_ok: boolean;
  battery_ok: boolean;
  battery_voltage: Num;
  motors_enabled_param: boolean;
  cmd_age_s: Num;
  max_linear_mps: number;
  max_angular_radps: number;
  speed_scale: number;
  lateral_rejected: number;
  cmd_linear_mps: number;
  cmd_angular_radps: number;
  calibration_motor: number;
  calibration_rps: number;
}

export interface MotorTelemetry {
  wheel: string[];
  setpoint_rps: Num[];
  feedback_valid: boolean;
  measured_rps: Num[];
  encoder_counts: number[];
  feedback_age_s: Num;
}

export interface LinkTelemetry {
  connected: boolean;
  link_ok: boolean;
  port: string;
  firmware_profile: "unknown" | "stock" | "patched";
  firmware_version: string;
  rx_frames: number;
  crc_errors: number;
  malformed_frames: number;
  rx_age_s: Num;
  imu_rate_hz: number;
  setpoint_age_s: Num;
  watchdog_active: boolean;
  watchdog_trips: number;
  rejected_setpoints: number;
  reconnects: number;
  motors_allowed: boolean;
  encoder_feedback: boolean;
  ultrasonic_available: boolean;
  last_sent_rps: Num[];
  last_error: string;
}

export interface BatteryTelemetry {
  raw_voltage?: Num;
  filtered_voltage?: Num;
  percentage?: Num;
  estimate_mode?: string;
  estimate_fault?: string;
}

export interface DiagnosticItem {
  name: string;
  level: "OK" | "WARN" | "ERROR" | "STALE" | "?";
  level_num: number;
  message: string;
}

export interface GatewayInfo {
  client_id: string;
  has_control: boolean;
  controlled_by_other: boolean;
  drive: Direction;
  drive_magnitude: number;
  auth: boolean;
  clients: number;
}

export interface Telemetry {
  type: "telemetry";
  safety?: SafetyTelemetry;
  motors?: MotorTelemetry;
  link?: LinkTelemetry;
  battery?: BatteryTelemetry;
  imu?: { accel: Num[]; gyro: Num[]; rate_hz?: number };
  ultrasonic?: { range_m: Num; min_range_m: Num; max_range_m: Num };
  diagnostics?: DiagnosticItem[];
  age_s?: Record<string, number>;
  calibration?: { file: string; saved: Record<string, unknown> | null };
  gateway: GatewayInfo;
}

export interface Ack {
  type: "ack";
  req: string;
  ok: boolean;
  message: string;
}
