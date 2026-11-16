import type { Num, Telemetry } from "./types";

export function fmt(v: Num | undefined, digits = 2, unit = ""): string {
  if (v === null || v === undefined || !Number.isFinite(v)) return "—";
  return `${v.toFixed(digits)}${unit ? " " + unit : ""}`;
}

export type Severity = "error" | "warn" | "info";
export interface Issue {
  severity: Severity;
  source: string;
  text: string;
}

/** Collect serial / watchdog / motor / safety problems into one list, worst first. */
export function collectIssues(t: Telemetry | null, wsConnected: boolean): Issue[] {
  const out: Issue[] = [];
  if (!wsConnected) out.push({ severity: "error", source: "UI", text: "Mất kết nối tới gateway" });
  if (!t) return out;
  const l = t.link;
  if (!l) {
    out.push({ severity: "error", source: "Serial", text: "Chưa nhận trạng thái từ driver RRC Lite" });
  } else {
    if (!l.connected) out.push({ severity: "error", source: "Serial", text: `Cổng serial chưa kết nối: ${l.last_error || l.port}` });
    else if (!l.link_ok) out.push({ severity: "error", source: "Serial", text: "Board không gửi dữ liệu" });
    if (l.crc_errors > 0 || l.malformed_frames > 0)
      out.push({ severity: "warn", source: "Serial", text: `Lỗi CRC ${l.crc_errors}, frame hỏng ${l.malformed_frames}` });
    if (l.watchdog_trips > 0)
      out.push({ severity: "warn", source: "Watchdog", text: `Watchdog đã dừng motor ${l.watchdog_trips} lần (lệnh bị trễ)` });
    if (l.rejected_setpoints > 0 && l.motors_allowed)
      out.push({ severity: "warn", source: "Motor", text: `${l.rejected_setpoints} lệnh motor bị từ chối` });
    if (l.firmware_profile === "stock")
      out.push({ severity: "info", source: "Firmware", text: "Firmware gốc: không có encoder, không có watchdog trên MCU" });
    if (l.reconnects > 0) out.push({ severity: "info", source: "Serial", text: `Đã kết nối lại ${l.reconnects} lần` });
  }
  const s = t.safety;
  if (s) {
    for (const f of s.faults) out.push({ severity: "error", source: "An toàn", text: f });
    if (s.lateral_rejected > 0)
      out.push({ severity: "info", source: "Motor", text: `Đã bỏ ${s.lateral_rejected} lệnh có thành phần ngang` });
  }
  for (const d of t.diagnostics ?? []) {
    if (d.level === "ERROR") out.push({ severity: "error", source: d.name, text: d.message });
  }
  const rank: Record<Severity, number> = { error: 0, warn: 1, info: 2 };
  return out.sort((a, b) => rank[a.severity] - rank[b.severity]);
}

// Same zones as the sonar LEDs in the RRC Lite firmware (item 9): green > 50 cm, yellow 20-50 cm, red < 20 cm.
export function sonarZone(rangeM: number | null): "far" | "mid" | "near" | null {
  if (rangeM === null || !Number.isFinite(rangeM) || rangeM <= 0) return null;
  return rangeM > 0.5 ? "far" : rangeM >= 0.2 ? "mid" : "near";
}
