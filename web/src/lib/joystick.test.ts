import { describe, expect, it } from "vitest";
import { collectIssues } from "./format";
import { quantize, snappedKnob, stickVector } from "./joystick";
import type { Telemetry } from "./types";

describe("quantize (must match gateway session.py)", () => {
  it.each([
    [0, 1, "FORWARD"],
    [0, -1, "REVERSE"],
    [-1, 0, "ROTATE_LEFT"],
    [1, 0, "ROTATE_RIGHT"],
    [0.4, 0.6, "FORWARD"],
    [0.7, -0.5, "ROTATE_RIGHT"],
    [0.05, 0.05, "STOP"],
    [Number.NaN, 1, "STOP"],
  ])("(%s, %s) -> %s", (x, y, d) => {
    expect(quantize(x as number, y as number).direction).toBe(d);
  });

  it("magnitude is deadzone-rescaled and bounded", () => {
    expect(quantize(0, 1).magnitude).toBe(1);
    expect(quantize(0, 0.9).magnitude).toBeCloseTo((0.9 - 0.15) / 0.85);
    expect(quantize(0, 5).magnitude).toBe(1);
  });
});

describe("stick geometry", () => {
  it("normalises and clamps to the unit circle, screen y inverted", () => {
    expect(stickVector(0, -50, 100)).toEqual({ x: 0, y: 0.5 });
    const v = stickVector(300, 0, 100);
    expect(v.x).toBeCloseTo(1);
    expect(stickVector(1, 1, 0)).toEqual({ x: 0, y: 0 });
  });

  it("snaps the knob onto one axis", () => {
    expect(snappedKnob(0.3, 0.8)).toEqual({ x: 0, y: 0.8 });
    expect(snappedKnob(-0.8, 0.3)).toEqual({ x: -0.8, y: 0 });
  });
});

describe("collectIssues", () => {
  const base = {
    type: "telemetry",
    gateway: { client_id: "a", has_control: false, controlled_by_other: false, drive: "STOP", drive_magnitude: 0, auth: false, clients: 1 },
  } as Telemetry;

  it("reports a lost gateway first", () => {
    expect(collectIssues(null, false)[0].source).toBe("UI");
  });

  it("reports serial, watchdog and stock firmware", () => {
    const t = {
      ...base,
      link: {
        connected: true, link_ok: false, port: "/dev/x", firmware_profile: "stock", firmware_version: "", rx_frames: 1,
        crc_errors: 2, malformed_frames: 0, rx_age_s: 3, imu_rate_hz: 0, setpoint_age_s: null, watchdog_active: true,
        watchdog_trips: 1, rejected_setpoints: 0, reconnects: 0, motors_allowed: false, encoder_feedback: false,
        ultrasonic_available: false, last_sent_rps: [0, 0, 0, 0], last_error: "",
      },
    } as Telemetry;
    const issues = collectIssues(t, true);
    expect(issues[0].severity).toBe("error");
    expect(issues.map((i) => i.source)).toEqual(expect.arrayContaining(["Serial", "Watchdog", "Firmware"]));
  });
});

import { buildMotorMap, rowsFromSaved } from "./motorMap";

describe("motor map", () => {
  it("builds ports and signs", () => {
    const m = buildMotorMap([
      { wheel: "RL", plusForward: true },
      { wheel: "RR", plusForward: false },
      { wheel: "FL", plusForward: true },
      { wheel: "FR", plusForward: false },
    ]);
    expect(m).toEqual({ ports: { RL: 1, RR: 2, FL: 3, FR: 4 }, signs: [1, -1, 1, -1] });
  });

  it("rejects duplicates and missing fields", () => {
    expect(typeof buildMotorMap([
      { wheel: "FL", plusForward: true }, { wheel: "FL", plusForward: true },
      { wheel: "RL", plusForward: true }, { wheel: "RR", plusForward: true },
    ])).toBe("string");
    expect(typeof buildMotorMap([
      { wheel: "FL", plusForward: null }, { wheel: "FR", plusForward: true },
      { wheel: "RL", plusForward: true }, { wheel: "RR", plusForward: true },
    ])).toBe("string");
  });

  it("round-trips saved ROS params", () => {
    const rows = rowsFromSaved({ port_fl: 3, port_fr: 4, port_rl: 1, port_rr: 2, port_signs: [1, -1, 1, -1] });
    expect(rows[0]).toEqual({ wheel: "RL", plusForward: true });
    expect(rows[3]).toEqual({ wheel: "FR", plusForward: false });
  });
});
