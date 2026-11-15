// Joystick geometry and the same 4-direction quantisation the gateway applies (session.py).
// The server is authoritative; this copy only drives the on-screen snap so the user sees what is sent.

import type { Direction } from "./types";

export const DEADZONE = 0.15;

/** Pointer offset (px, screen y down) inside a pad of `radius` px -> stick vector (x right+, y up+), |v| <= 1. */
export function stickVector(dx: number, dy: number, radius: number): { x: number; y: number } {
  if (!(radius > 0) || !Number.isFinite(dx) || !Number.isFinite(dy)) return { x: 0, y: 0 };
  let x = dx / radius;
  let y = -dy / radius;
  const m = Math.hypot(x, y);
  if (m > 1) {
    x /= m;
    y /= m;
  }
  return { x, y };
}

export function quantize(x: number, y: number, deadzone = DEADZONE): { direction: Direction; magnitude: number } {
  if (!Number.isFinite(x) || !Number.isFinite(y)) return { direction: "STOP", magnitude: 0 };
  x = Math.max(-1, Math.min(1, x));
  y = Math.max(-1, Math.min(1, y));
  if (Math.hypot(x, y) < deadzone) return { direction: "STOP", magnitude: 0 };
  const scale = (v: number) => Math.max(0, Math.min(1, (Math.abs(v) - deadzone) / (1 - deadzone)));
  if (Math.abs(y) >= Math.abs(x)) return { direction: y > 0 ? "FORWARD" : "REVERSE", magnitude: scale(y) };
  return { direction: x > 0 ? "ROTATE_RIGHT" : "ROTATE_LEFT", magnitude: scale(x) };
}

/** Knob position snapped onto the axis of the quantised direction (what the robot will do). */
export function snappedKnob(x: number, y: number): { x: number; y: number } {
  const { direction } = quantize(x, y);
  switch (direction) {
    case "FORWARD":
    case "REVERSE":
      return { x: 0, y };
    case "ROTATE_LEFT":
    case "ROTATE_RIGHT":
      return { x, y: 0 };
    default:
      return { x, y };
  }
}

export const DIRECTION_LABEL: Record<Direction, string> = {
  STOP: "Dừng",
  FORWARD: "Tiến",
  REVERSE: "Lùi",
  ROTATE_LEFT: "Xoay trái",
  ROTATE_RIGHT: "Xoay phải",
};
