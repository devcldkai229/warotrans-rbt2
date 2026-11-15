// Motor port -> wheel mapping entered in Calibration mode, validated before it is sent to the gateway.

export const WHEELS = ["FL", "FR", "RL", "RR"] as const;
export type Wheel = (typeof WHEELS)[number];

export interface PortRow {
  wheel: Wheel | "";
  plusForward: boolean | null; // does "+" roll this wheel forward?
}

export interface MotorMapPayload {
  ports: Record<Wheel, number>;
  signs: number[]; // per port M1..M4
}

/** Rows are indexed by port (0 = M1). Returns the payload or an error message (Vietnamese, for the UI). */
export function buildMotorMap(rows: PortRow[]): MotorMapPayload | string {
  if (rows.length !== 4) return "Cần đủ 4 cổng";
  const ports: Partial<Record<Wheel, number>> = {};
  const signs: number[] = [];
  for (let i = 0; i < 4; i++) {
    const r = rows[i];
    if (!r.wheel) return `M${i + 1}: chưa chọn bánh`;
    if (r.plusForward === null) return `M${i + 1}: chưa chọn chiều`;
    if (ports[r.wheel] !== undefined) return `Bánh ${r.wheel} bị gán cho 2 cổng`;
    ports[r.wheel] = i + 1;
    signs.push(r.plusForward ? 1 : -1);
  }
  return { ports: ports as Record<Wheel, number>, signs };
}

/** Pre-fill rows from a saved calibration (ROS params) if present. */
export function rowsFromSaved(saved: Record<string, unknown> | null | undefined): PortRow[] {
  const rows: PortRow[] = [0, 1, 2, 3].map(() => ({ wheel: "", plusForward: null }));
  if (!saved) return rows;
  const signs = Array.isArray(saved.port_signs) ? (saved.port_signs as number[]) : [];
  for (const w of WHEELS) {
    const port = Number(saved[`port_${w.toLowerCase()}`]);
    if (port >= 1 && port <= 4) rows[port - 1].wheel = w;
  }
  signs.forEach((s, i) => {
    if (i < 4 && (s === 1 || s === -1)) rows[i].plusForward = s === 1;
  });
  return rows;
}
