import { useCallback, useEffect, useRef, useState } from "react";
import { collectIssues, fmt } from "./lib/format";
import { useGateway } from "./lib/useGateway";

// Simple manual control: hold a button to move (forward / backward / rotate left / rotate right), release to stop.
// The first press takes control and switches the robot to ENABLED; STOP returns it to SAFE.

type Dir = "forward" | "backward" | "left" | "right";
const VECTOR: Record<Dir, { x: number; y: number }> = {
  forward: { x: 0, y: 1 },
  backward: { x: 0, y: -1 },
  left: { x: -1, y: 0 },
  right: { x: 1, y: 0 },
};
const KEYS: Record<string, Dir> = { ArrowUp: "forward", ArrowDown: "backward", ArrowLeft: "left", ArrowRight: "right" };
const SEND_MS = 50;

export default function App() {
  const { telemetry: t, connected, lastAck, send } = useGateway();
  const [active, setActive] = useState<Dir | null>(null);
  const [speed, setSpeed] = useState<number | null>(null);
  const [toast, setToast] = useState<string | null>(null);
  const timer = useRef<ReturnType<typeof setInterval> | null>(null);

  const s = t?.safety;
  const mode = s?.mode;
  const hasControl = !!t?.gateway?.has_control;
  const shownSpeed = speed ?? s?.speed_scale ?? 0.5;
  const issues = collectIssues(t, connected).filter((i) => i.severity === "error");

  useEffect(() => {
    if (lastAck && !lastAck.ok && lastAck.req !== "drive") {
      setToast(lastAck.message);
      const id = setTimeout(() => setToast(null), 5000);
      return () => clearTimeout(id);
    }
  }, [lastAck]);

  const stopMoving = useCallback(() => {
    if (timer.current) clearInterval(timer.current);
    timer.current = null;
    setActive(null);
    send({ type: "drive", x: 0, y: 0 });
  }, [send]);

  const startMoving = useCallback(
    (dir: Dir) => {
      if (!connected || timer.current) return;
      if (!hasControl) send({ type: "claim" });
      if (mode !== "ENABLED") send({ type: "set_mode", mode: "ENABLED" });
      const v = VECTOR[dir];
      setActive(dir);
      send({ type: "drive", ...v });
      timer.current = setInterval(() => send({ type: "drive", ...v }), SEND_MS);
    },
    [connected, hasControl, mode, send],
  );

  const emergencyStop = useCallback(() => {
    if (timer.current) clearInterval(timer.current);
    timer.current = null;
    setActive(null);
    send({ type: "stop" });
  }, [send]);

  // keyboard (laptop): arrows hold to move, space = STOP
  useEffect(() => {
    const down = (e: KeyboardEvent) => {
      if (e.code === "Space") {
        e.preventDefault();
        emergencyStop();
      } else if (KEYS[e.key] && !e.repeat) {
        e.preventDefault();
        startMoving(KEYS[e.key]);
      }
    };
    const up = (e: KeyboardEvent) => {
      if (KEYS[e.key]) stopMoving();
    };
    const hide = () => timer.current && stopMoving();
    window.addEventListener("keydown", down);
    window.addEventListener("keyup", up);
    window.addEventListener("blur", hide);
    document.addEventListener("visibilitychange", hide);
    return () => {
      window.removeEventListener("keydown", down);
      window.removeEventListener("keyup", up);
      window.removeEventListener("blur", hide);
      document.removeEventListener("visibilitychange", hide);
    };
  }, [startMoving, stopMoving, emergencyStop]);

  useEffect(() => {
    if (!connected && timer.current) stopMoving();
  }, [connected, stopMoving]);

  const applySpeed = (v: number) => {
    if (!hasControl) send({ type: "claim" });
    send({ type: "set_speed_limit", scale: v });
  };

  const btn = (dir: Dir, label: string, icon: string) => (
    <button
      onPointerDown={(e) => {
        (e.target as Element).setPointerCapture(e.pointerId);
        startMoving(dir);
      }}
      onPointerUp={stopMoving}
      onPointerCancel={stopMoving}
      onLostPointerCapture={() => timer.current && stopMoving()}
      onContextMenu={(e) => e.preventDefault()}
      disabled={!connected}
      className={`touch-none-select flex aspect-square w-full flex-col items-center justify-center rounded-2xl text-lg font-bold shadow-lg transition-colors disabled:opacity-30 ${
        active === dir ? "bg-sky-500 text-white" : "bg-slate-800 text-slate-100 active:bg-sky-600"
      }`}
    >
      <span className="text-4xl leading-none">{icon}</span>
      <span className="mt-1 text-sm">{label}</span>
    </button>
  );

  return (
    <div className="mx-auto flex min-h-full max-w-md flex-col gap-4 p-4 pb-32">
      <header className="flex items-center gap-2">
        <h1 className="mr-auto text-xl font-bold">RBT-002</h1>
        <span className={`rounded-full px-3 py-1 text-xs font-semibold ${connected ? "bg-emerald-700" : "bg-red-700"}`}>
          {connected ? "Đã kết nối" : "Mất kết nối"}
        </span>
        <span
          className={`rounded-full px-3 py-1 text-xs font-bold ${
            mode === "ENABLED" ? "bg-red-600" : mode === "SAFE" ? "bg-emerald-800" : "bg-slate-700"
          }`}
        >
          {mode ?? "—"}
        </span>
      </header>

      <div className="num flex justify-between text-sm text-slate-400">
        <span>Pin {fmt(t?.battery?.raw_voltage, 2, "V")}</span>
        <span>
          {t?.link?.link_ok ? "Board OK" : "Board mất tín hiệu"} · firmware {t?.link?.firmware_profile ?? "—"}
        </span>
      </div>

      {(toast || issues.length > 0) && (
        <div className="rounded-xl bg-red-900/70 px-3 py-2 text-sm">
          {toast ?? `${issues[0].source}: ${issues[0].text}`}
        </div>
      )}
      {s && s.blockers.length > 0 && mode !== "ENABLED" && (
        <div className="rounded-xl bg-amber-900/50 px-3 py-2 text-xs text-amber-100">Chưa chạy được: {s.blockers.join("; ")}</div>
      )}

      <section className="grid grid-cols-3 gap-3">
        <div />
        {btn("forward", "Tiến", "▲")}
        <div />
        {btn("left", "Xoay trái", "⟲")}
        <div className="flex items-center justify-center text-xs text-slate-500">{active ? "đang chạy" : "giữ để chạy"}</div>
        {btn("right", "Xoay phải", "⟳")}
        <div />
        {btn("backward", "Lùi", "▼")}
        <div />
      </section>

      <section className="rounded-2xl bg-slate-900/70 p-4">
        <div className="mb-2 flex justify-between text-sm">
          <span className="text-slate-300">Tốc độ</span>
          <span className="num font-semibold">{Math.round(shownSpeed * 100)}%</span>
        </div>
        <input
          type="range"
          min={0.05}
          max={1}
          step={0.05}
          value={shownSpeed}
          disabled={!connected}
          onChange={(e) => setSpeed(Number(e.target.value))}
          onPointerUp={(e) => applySpeed(Number((e.target as HTMLInputElement).value))}
          onKeyUp={(e) => applySpeed(Number((e.target as HTMLInputElement).value))}
          className="w-full accent-sky-500"
        />
      </section>

      <button
        onClick={emergencyStop}
        className="fixed inset-x-4 bottom-4 z-50 mx-auto max-w-md rounded-2xl bg-red-600 py-5 text-2xl font-black tracking-widest text-white shadow-2xl active:bg-red-800"
      >
        STOP
      </button>
    </div>
  );
}
