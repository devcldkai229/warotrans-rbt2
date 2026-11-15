import { useCallback, useEffect, useRef, useState } from "react";
import type { Ack, Telemetry } from "./types";

export type Outgoing =
  | { type: "claim" }
  | { type: "release" }
  | { type: "drive"; x: number; y: number }
  | { type: "stop" }
  | { type: "set_mode"; mode: "SAFE" | "ENABLED" | "CALIBRATION" }
  | { type: "set_speed_limit"; scale: number }
  | { type: "calibrate"; motor: number; rps: number; duration_ms: number }
  | { type: "save_motor_map"; ports: Record<string, number>; signs: number[] };

function readToken(): string {
  const fromUrl = new URLSearchParams(window.location.search).get("token");
  try {
    if (fromUrl) sessionStorage.setItem("waro_ui_token", fromUrl);
    return fromUrl ?? sessionStorage.getItem("waro_ui_token") ?? "";
  } catch {
    return fromUrl ?? "";
  }
}

/** WebSocket to the gateway with automatic reconnect. Telemetry older than 1.5 s counts as disconnected. */
export function useGateway() {
  const [telemetry, setTelemetry] = useState<Telemetry | null>(null);
  const [connected, setConnected] = useState(false);
  const [lastAck, setLastAck] = useState<Ack | null>(null);
  const wsRef = useRef<WebSocket | null>(null);
  const lastMsg = useRef(0);

  useEffect(() => {
    let closed = false;
    let retry: ReturnType<typeof setTimeout> | undefined;
    let backoff = 500;
    const connect = () => {
      const proto = window.location.protocol === "https:" ? "wss" : "ws";
      const token = readToken();
      const url = `${proto}://${window.location.host}/ws${token ? `?token=${encodeURIComponent(token)}` : ""}`;
      const ws = new WebSocket(url);
      wsRef.current = ws;
      ws.onopen = () => {
        backoff = 500;
      };
      ws.onmessage = (ev) => {
        lastMsg.current = Date.now();
        setConnected(true);
        try {
          const msg = JSON.parse(ev.data as string);
          if (msg.type === "telemetry") setTelemetry(msg as Telemetry);
          else if (msg.type === "ack") setLastAck(msg as Ack);
        } catch {
          /* ignore malformed */
        }
      };
      ws.onclose = () => {
        setConnected(false);
        wsRef.current = null;
        if (!closed) {
          retry = setTimeout(connect, backoff);
          backoff = Math.min(backoff * 2, 5000);
        }
      };
      ws.onerror = () => ws.close();
    };
    connect();
    const stale = setInterval(() => {
      if (Date.now() - lastMsg.current > 1500) setConnected(false);
    }, 500);
    return () => {
      closed = true;
      clearTimeout(retry);
      clearInterval(stale);
      wsRef.current?.close();
    };
  }, []);

  const send = useCallback((msg: Outgoing): boolean => {
    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) return false;
    ws.send(JSON.stringify(msg));
    return true;
  }, []);

  return { telemetry, connected, lastAck, send };
}
