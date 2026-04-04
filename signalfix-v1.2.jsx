import { useState, useEffect, useRef, useCallback } from "react";
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ReferenceLine, ResponsiveContainer, Legend } from "recharts";

// ─── CONSTANTS ────────────────────────────────────────────────────────────────
const EPS = 1e-9;
const Q_MIN = 1e-9;
const R_MIN = 1e-9;
const P_MIN = 1e-9;
const HISTORY_LEN = 200;

// ─── DEFAULT PARAMS ───────────────────────────────────────────────────────────
const DEFAULT_PARAMS = {
  Q: 0.01,
  R_nominal: 1.0,
  R_noisy: 10.0,
  T_enter: 12.0,
  T_exit: 4.0,
  T_gate: 25.0,
  beta: 5.0,
  N: 3,
  M: 15,
  alpha_init: 100.0,
};

// ─── SIGNAL REGIME GENERATOR ─────────────────────────────────────────────────
function generateMeasurement(tick) {
  const trueVal = Math.sin(tick * 0.05) * 10;
  const regime = Math.floor(tick / 80) % 4;
  let noise = 0;
  switch (regime) {
    case 0: noise = (Math.random() - 0.5) * 2;    break; // clean
    case 1: noise = (Math.random() - 0.5) * 20;   break; // noise burst
    case 2: noise = Math.random() < 0.3 ? (Math.random() - 0.5) * 80 : (Math.random() - 0.5) * 2; break; // spikes
    case 3: noise = (Math.random() - 0.5) * 200;  break; // sensor death
  }
  return { z: trueVal + noise, trueVal, regime };
}

const REGIME_LABELS = ["CLEAN", "NOISE BURST", "SPIKE TRAIN", "SENSOR DEATH"];

// ─── KALMAN STEP ─────────────────────────────────────────────────────────────
function kalmanStep(state, z, dt, params) {
  const { Q, R_nominal, R_noisy, T_enter, T_exit, T_gate, beta, N, M } = params;

  let { x, P, noise_state, rejection_count, system_state } = state;

  // Safe dt
  const safeDt = Math.max(dt, 0.001);
  // dt-scaled Q (physically correct for constant velocity model)
  // Q_pos ~ Q*dt^3/3, Q_vel ~ Q*dt  (discrete-time process noise)
  const Q_pos = Math.max(Q * (safeDt * safeDt * safeDt) / 3, Q_MIN);
  const Q_vel = Math.max(Q * safeDt, Q_MIN);
  const Q_cross = Q * (safeDt * safeDt) / 2;

  // F matrix (constant velocity 2-state: [pos, vel])
  const F = [[1, safeDt], [0, 1]];
  const H = [1, 0]; // observe position only

  // ── PREDICT ──
  // x- = F x
  let xp = [
    F[0][0] * x[0] + F[0][1] * x[1],
    F[1][0] * x[0] + F[1][1] * x[1],
  ];

  // P- = F P F^T + Q  (Q matrix dt-scaled for constant velocity model)
  const Qmat = [[Q_pos, Q_cross], [Q_cross, Q_vel]];
  const Pp = matAdd(matMul(matMul(F, P), transpose(F)), Qmat);
  const Pp_sym = symmetrize(applyFloor(Pp));

  // ── INNOVATION ──
  const y = z - (H[0] * xp[0] + H[1] * xp[1]);

  // ── NOMINAL S and NIS ──
  const S_nom = Math.max(hPhT(H, Pp_sym) + Math.max(R_nominal, R_MIN), EPS);
  const NIS_nom = (y * y) / S_nom;

  // ── NOISE STATE SWITCHING (hysteresis) ──
  let new_noise_state = noise_state;
  if (noise_state === "NOMINAL" && NIS_nom > T_enter) new_noise_state = "NOISY";
  if (noise_state === "NOISY"   && NIS_nom < T_exit)  new_noise_state = "NOMINAL";

  // ── SELECT R ──
  const R = new_noise_state === "NOMINAL" ? Math.max(R_nominal, R_MIN) : Math.max(R_noisy, R_MIN);

  // ── RECOMPUTE S and NIS with selected R (MANDATORY) ──
  const S = Math.max(hPhT(H, Pp_sym) + R, EPS);
  const NIS = (y * y) / S;

  // ── GATING ──
  // rejection_count accumulates continuously.
  // Inflation fires every N rejections (modulo check — no reset).
  // Hard fail fires when total consecutive rejections exceed M.
  // Only an accepted measurement resets rejection_count to 0.
  let new_rejection_count = rejection_count;
  let accepted = false;
  let new_system_state = system_state;

  if (system_state === "SAFE_MODE") {
    // prediction only — no gating, no count change
  } else if (NIS > T_gate) {
    new_rejection_count = rejection_count + 1;
    accepted = false;
  } else {
    new_rejection_count = 0; // reset only on acceptance
    accepted = true;
  }

  // ── P INFLATION ──
  // Fires every N consecutive rejections (checked before hard fail so P
  // can recover before we declare sensor death, but count is NOT reset).
  let Pp_final = Pp_sym;
  if (new_rejection_count > 0 && new_rejection_count % N === 0 && new_rejection_count <= M) {
    Pp_final = scalarMat(Pp_sym, beta);
    // DO NOT reset new_rejection_count — hard fail must still be reachable
  }

  // ── HARD FAIL ──
  // Fires when rejection_count exceeds M without ever being reset by acceptance.
  if (new_rejection_count > M) {
    new_system_state = "SAFE_MODE";
  }

  // ── UPDATE (Joseph Form) ──
  let x_new = xp;
  let P_new = Pp_final;

  if (accepted && new_system_state === "NORMAL") {
    // K = P- H^T S^-1
    const K = [Pp_final[0][0] * H[0] / S + Pp_final[0][1] * H[1] / S,
               Pp_final[1][0] * H[0] / S + Pp_final[1][1] * H[1] / S];

    // x = x- + K y
    x_new = [xp[0] + K[0] * y, xp[1] + K[1] * y];

    // P = (I - KH) P- (I - KH)^T + K R K^T  (Joseph form)
    const IKH = [[1 - K[0] * H[0], -K[0] * H[1]],
                 [-K[1] * H[0],    1 - K[1] * H[1]]];
    const IKHP = matMul(IKH, Pp_final);
    const joseph = matMul(IKHP, transpose(IKH));
    const KRKt = [[K[0] * R * K[0], K[0] * R * K[1]],
                  [K[1] * R * K[0], K[1] * R * K[1]]];
    P_new = symmetrize(applyFloor(matAdd(joseph, KRKt)));
  }

  // ── NaN guard ──
  if (isNaN(x_new[0]) || isNaN(x_new[1]) || isNaN(P_new[0][0])) {
    x_new = xp;
    P_new = [[params.alpha_init, 0], [0, params.alpha_init]];
  }

  return {
    x: x_new,
    P: P_new,
    noise_state: new_noise_state,
    rejection_count: new_rejection_count,
    system_state: new_system_state,
    diagnostics: { NIS, NIS_nom, S, R, y, accepted, innovation: y, inflation_fired: new_rejection_count > 0 && new_rejection_count % N === 0 && new_rejection_count <= M },
  };
}

// ─── MATRIX UTILS ─────────────────────────────────────────────────────────────
function matMul(A, B) {
  return A.map((row, i) => B[0].map((_, j) => row.reduce((s, _, k) => s + A[i][k] * B[k][j], 0)));
}
function matAdd(A, B) { return A.map((r, i) => r.map((v, j) => v + B[i][j])); }
function transpose(A) { return A[0].map((_, j) => A.map(r => r[j])); }
function symmetrize(A) { return A.map((r, i) => r.map((v, j) => 0.5 * (v + A[j][i]))); }
function scalarMat(A, s) { if (s === undefined) return A; return A.map(r => r.map(v => v * s)); }
function applyFloor(P) { return P.map((r, i) => r.map((v, j) => i === j ? Math.max(v, P_MIN) : v)); }
function hPhT(H, P) { return H[0] * (H[0] * P[0][0] + H[1] * P[1][0]) + H[1] * (H[0] * P[0][1] + H[1] * P[1][1]); }

// ─── INITIAL STATE ────────────────────────────────────────────────────────────
function initState(params) {
  return {
    x: [0, 0],
    P: [[params.alpha_init, 0], [0, params.alpha_init]],
    noise_state: "NOMINAL",
    rejection_count: 0,
    system_state: "NORMAL",
  };
}

// ─── PARAM SLIDER ─────────────────────────────────────────────────────────────
function ParamSlider({ label, value, min, max, step, onChange, color = "#94a3b8" }) {
  return (
    <div style={{ display: "flex", flexDirection: "column", gap: 2, minWidth: 120 }}>
      <div style={{ display: "flex", justifyContent: "space-between", fontSize: 10, color }}>
        <span style={{ fontFamily: "monospace", letterSpacing: 1 }}>{label}</span>
        <span style={{ fontFamily: "monospace", color: "#e2e8f0" }}>{value}</span>
      </div>
      <input type="range" min={min} max={max} step={step} value={value}
        onChange={e => onChange(parseFloat(e.target.value))}
        style={{ width: "100%", accentColor: color, height: 3, cursor: "pointer" }} />
    </div>
  );
}

// ─── MAIN APP ─────────────────────────────────────────────────────────────────
export default function SignalFixV1() {
  const [params, setParams] = useState(DEFAULT_PARAMS);
  const [running, setRunning] = useState(true);
  const [history, setHistory] = useState([]);
  const [currentState, setCurrentState] = useState(() => initState(DEFAULT_PARAMS));
  const [tick, setTick] = useState(0);

  const stateRef = useRef(initState(DEFAULT_PARAMS));
  const tickRef = useRef(0);
  const lastTimeRef = useRef(null);
  const paramsRef = useRef(DEFAULT_PARAMS);
  const runningRef = useRef(true);
  const animRef = useRef(null);

  useEffect(() => { paramsRef.current = params; }, [params]);
  useEffect(() => { runningRef.current = running; }, [running]);

  const reset = useCallback(() => {
    stateRef.current = initState(paramsRef.current);
    tickRef.current = 0;
    lastTimeRef.current = null;
    setTick(0);
    setHistory([]);
    setCurrentState(initState(paramsRef.current));
  }, []);

  useEffect(() => {
    const loop = (timestamp) => {
      animRef.current = requestAnimationFrame(loop);
      if (!runningRef.current) return;
      if (lastTimeRef.current === null) { lastTimeRef.current = timestamp; return; }
      const dt = Math.min((timestamp - lastTimeRef.current) / 1000, 0.1);
      lastTimeRef.current = timestamp;

      const t = tickRef.current;
      const { z, trueVal, regime } = generateMeasurement(t);
      const p = paramsRef.current;

      // enforce hysteresis
      const safeParams = { ...p, T_enter: Math.max(p.T_enter, p.T_exit + 0.1) };
      const result = kalmanStep(stateRef.current, z, dt, safeParams);
      stateRef.current = { x: result.x, P: result.P, noise_state: result.noise_state, rejection_count: result.rejection_count, system_state: result.system_state };
      tickRef.current = t + 1;

      const point = {
        t,
        raw: +z.toFixed(3),
        filtered: +result.x[0].toFixed(3),
        truth: +trueVal.toFixed(3),
        NIS: +result.diagnostics.NIS.toFixed(3),
        P00: +result.P[0][0].toFixed(4),
        noise_state: result.noise_state,
        system_state: result.system_state,
        accepted: result.diagnostics.accepted ? 1 : 0,
        rejection_count: result.rejection_count,
        regime,
      };

      setHistory(prev => {
        const next = [...prev, point];
        return next.length > HISTORY_LEN ? next.slice(-HISTORY_LEN) : next;
      });
      setCurrentState({ ...stateRef.current, diagnostics: result.diagnostics });
      setTick(t + 1);
    };
    animRef.current = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(animRef.current);
  }, []);

  const sp = params;
  const cur = currentState;
  const regime = history.length > 0 ? history[history.length - 1].regime : 0;
  const isNormal = cur.system_state === "NORMAL";
  const isNoisy = cur.noise_state === "NOISY";

  const regimeColor = ["#22d3ee", "#f59e0b", "#f97316", "#ef4444"][regime] || "#22d3ee";

  return (
    <div style={{
      background: "#060b14",
      minHeight: "100vh",
      fontFamily: "'JetBrains Mono', 'Fira Code', monospace",
      color: "#e2e8f0",
      padding: "16px",
      boxSizing: "border-box",
    }}>
      {/* HEADER */}
      <div style={{ display: "flex", alignItems: "center", justifyContent: "space-between", marginBottom: 16, borderBottom: "1px solid #1e293b", paddingBottom: 12 }}>
        <div style={{ display: "flex", alignItems: "center", gap: 16 }}>
          <div style={{ fontSize: 18, fontWeight: 700, letterSpacing: 2, color: "#38bdf8" }}>
            SIGNAL<span style={{ color: "#f97316" }}>FIX</span>
            <span style={{ fontSize: 12, color: "#475569", marginLeft: 8 }}>V1</span>
          </div>
          <div style={{
            padding: "3px 10px", borderRadius: 3, fontSize: 11, fontWeight: 700, letterSpacing: 1.5,
            background: isNormal ? "#052e16" : "#450a0a",
            color: isNormal ? "#4ade80" : "#f87171",
            border: `1px solid ${isNormal ? "#166534" : "#7f1d1d"}`,
          }}>{cur.system_state}</div>
          <div style={{
            padding: "3px 10px", borderRadius: 3, fontSize: 11, fontWeight: 700, letterSpacing: 1.5,
            background: isNoisy ? "#431407" : "#0f172a",
            color: isNoisy ? "#fb923c" : "#64748b",
            border: `1px solid ${isNoisy ? "#7c2d12" : "#1e293b"}`,
          }}>{cur.noise_state}</div>
          <div style={{ padding: "3px 10px", borderRadius: 3, fontSize: 11, background: "#0f172a", border: `1px solid ${regimeColor}`, color: regimeColor }}>
            {REGIME_LABELS[regime]}
          </div>
        </div>
        <div style={{ display: "flex", gap: 8 }}>
          <button onClick={() => setRunning(r => !r)} style={{
            padding: "5px 16px", borderRadius: 4, border: "1px solid #334155",
            background: running ? "#1e293b" : "#052e16", color: running ? "#94a3b8" : "#4ade80",
            cursor: "pointer", fontSize: 11, letterSpacing: 1, fontFamily: "monospace",
          }}>{running ? "⏸ PAUSE" : "▶ RUN"}</button>
          <button onClick={reset} style={{
            padding: "5px 16px", borderRadius: 4, border: "1px solid #334155",
            background: "#1e293b", color: "#94a3b8", cursor: "pointer", fontSize: 11, letterSpacing: 1, fontFamily: "monospace",
          }}>↺ RESET</button>
        </div>
      </div>

      {/* STATS ROW */}
      <div style={{ display: "grid", gridTemplateColumns: "repeat(6, 1fr)", gap: 8, marginBottom: 16 }}>
        {[
          { label: "TICK", value: tick },
          { label: "NIS", value: cur.diagnostics?.NIS?.toFixed(2) ?? "—", warn: cur.diagnostics?.NIS > sp.T_gate },
          { label: "P₀₀", value: cur.P?.[0]?.[0]?.toFixed(3) ?? "—" },
          { label: "x̂ pos", value: cur.x?.[0]?.toFixed(2) ?? "—" },
          { label: "REJECTIONS", value: `${cur.rejection_count} / ${sp.M}`, warn: cur.rejection_count > 0 },
          { label: "ACCEPTED", value: cur.diagnostics?.accepted ? "YES" : "NO", warn: !cur.diagnostics?.accepted },
        ].map(({ label, value, warn }) => (
          <div key={label} style={{
            background: "#0f172a", border: `1px solid ${warn ? "#7c2d12" : "#1e293b"}`,
            borderRadius: 6, padding: "8px 12px",
          }}>
            <div style={{ fontSize: 9, color: "#475569", letterSpacing: 1.5, marginBottom: 4 }}>{label}</div>
            <div style={{ fontSize: 16, fontWeight: 700, color: warn ? "#f87171" : "#e2e8f0" }}>{value}</div>
            {label === "REJECTIONS" && (
              <div style={{ marginTop: 4, height: 3, background: "#1e293b", borderRadius: 2 }}>
                <div style={{ height: "100%", borderRadius: 2, width: `${Math.min((cur.rejection_count / sp.M) * 100, 100)}%`, background: cur.rejection_count > sp.N ? "#ef4444" : "#f59e0b", transition: "width 0.1s" }} />
              </div>
            )}
            {label === "ACCEPTED" && cur.diagnostics?.inflation_fired && (
              <div style={{ fontSize: 9, color: "#a78bfa", marginTop: 2 }}>↑ P INFLATED</div>
            )}
          </div>
        ))}
      </div>

      {/* CHART 1: Signal */}
      <div style={{ background: "#0a1628", border: "1px solid #1e293b", borderRadius: 8, padding: "12px 8px", marginBottom: 12 }}>
        <div style={{ fontSize: 10, color: "#475569", letterSpacing: 2, marginBottom: 8, paddingLeft: 8 }}>SIGNAL — RAW vs FILTERED vs TRUTH</div>
        <ResponsiveContainer width="100%" height={160}>
          <LineChart data={history} margin={{ top: 4, right: 16, bottom: 0, left: -16 }}>
            <CartesianGrid strokeDasharray="3 3" stroke="#1e293b" />
            <XAxis dataKey="t" tick={{ fill: "#475569", fontSize: 9 }} />
            <YAxis tick={{ fill: "#475569", fontSize: 9 }} />
            <Tooltip contentStyle={{ background: "#0f172a", border: "1px solid #334155", fontSize: 11 }}
              labelStyle={{ color: "#64748b" }} />
            <Line type="monotone" dataKey="raw" stroke="#334155" dot={false} strokeWidth={1} name="raw" />
            <Line type="monotone" dataKey="truth" stroke="#64748b" dot={false} strokeWidth={1} strokeDasharray="4 2" name="truth" />
            <Line type="monotone" dataKey="filtered" stroke="#38bdf8" dot={false} strokeWidth={2} name="filtered" />
          </LineChart>
        </ResponsiveContainer>
      </div>

      {/* CHART 2: NIS */}
      <div style={{ background: "#0a1628", border: "1px solid #1e293b", borderRadius: 8, padding: "12px 8px", marginBottom: 12 }}>
        <div style={{ fontSize: 10, color: "#475569", letterSpacing: 2, marginBottom: 8, paddingLeft: 8 }}>NIS — NORMALIZED INNOVATION SQUARED</div>
        <ResponsiveContainer width="100%" height={120}>
          <LineChart data={history} margin={{ top: 4, right: 16, bottom: 0, left: -16 }}>
            <CartesianGrid strokeDasharray="3 3" stroke="#1e293b" />
            <XAxis dataKey="t" tick={{ fill: "#475569", fontSize: 9 }} />
            <YAxis tick={{ fill: "#475569", fontSize: 9 }} />
            <Tooltip contentStyle={{ background: "#0f172a", border: "1px solid #334155", fontSize: 11 }} />
            <ReferenceLine y={sp.T_enter} stroke="#f59e0b" strokeDasharray="3 3" label={{ value: "T_enter", fill: "#f59e0b", fontSize: 9 }} />
            <ReferenceLine y={sp.T_exit} stroke="#22d3ee" strokeDasharray="3 3" label={{ value: "T_exit", fill: "#22d3ee", fontSize: 9 }} />
            <ReferenceLine y={sp.T_gate} stroke="#ef4444" strokeDasharray="3 3" label={{ value: "T_gate", fill: "#ef4444", fontSize: 9 }} />
            <Line type="monotone" dataKey="NIS" stroke="#a78bfa" dot={false} strokeWidth={1.5} name="NIS" />
          </LineChart>
        </ResponsiveContainer>
      </div>

      {/* CHART 3: P diagonal */}
      <div style={{ background: "#0a1628", border: "1px solid #1e293b", borderRadius: 8, padding: "12px 8px", marginBottom: 16 }}>
        <div style={{ fontSize: 10, color: "#475569", letterSpacing: 2, marginBottom: 8, paddingLeft: 8 }}>COVARIANCE P₀₀ — UNCERTAINTY</div>
        <ResponsiveContainer width="100%" height={100}>
          <LineChart data={history} margin={{ top: 4, right: 16, bottom: 0, left: -16 }}>
            <CartesianGrid strokeDasharray="3 3" stroke="#1e293b" />
            <XAxis dataKey="t" tick={{ fill: "#475569", fontSize: 9 }} />
            <YAxis tick={{ fill: "#475569", fontSize: 9 }} />
            <Tooltip contentStyle={{ background: "#0f172a", border: "1px solid #334155", fontSize: 11 }} />
            <Line type="monotone" dataKey="P00" stroke="#4ade80" dot={false} strokeWidth={1.5} name="P₀₀" />
          </LineChart>
        </ResponsiveContainer>
      </div>

      {/* PARAMS */}
      <div style={{ background: "#0a1628", border: "1px solid #1e293b", borderRadius: 8, padding: 16 }}>
        <div style={{ fontSize: 10, color: "#475569", letterSpacing: 2, marginBottom: 12 }}>PARAMETERS — LIVE TUNING</div>
        <div style={{ display: "grid", gridTemplateColumns: "repeat(4, 1fr)", gap: 16 }}>
          <ParamSlider label="Q" value={sp.Q} min={0.001} max={1} step={0.001} color="#4ade80"
            onChange={v => setParams(p => ({ ...p, Q: v }))} />
          <ParamSlider label="R_nominal" value={sp.R_nominal} min={0.1} max={10} step={0.1} color="#38bdf8"
            onChange={v => setParams(p => ({ ...p, R_nominal: v }))} />
          <ParamSlider label="R_noisy" value={sp.R_noisy} min={1} max={100} step={1} color="#f97316"
            onChange={v => setParams(p => ({ ...p, R_noisy: v }))} />
          <ParamSlider label="T_enter" value={sp.T_enter} min={1} max={50} step={0.5} color="#f59e0b"
            onChange={v => setParams(p => ({ ...p, T_enter: Math.max(v, p.T_exit + 0.5) }))} />
          <ParamSlider label="T_exit" value={sp.T_exit} min={0.5} max={30} step={0.5} color="#22d3ee"
            onChange={v => setParams(p => ({ ...p, T_exit: Math.min(v, p.T_enter - 0.5) }))} />
          <ParamSlider label="T_gate" value={sp.T_gate} min={5} max={100} step={1} color="#ef4444"
            onChange={v => setParams(p => ({ ...p, T_gate: v }))} />
          <ParamSlider label="β (inflation)" value={sp.beta} min={1.1} max={10} step={0.1} color="#a78bfa"
            onChange={v => setParams(p => ({ ...p, beta: v }))} />
          <ParamSlider label="N (inflate@)" value={sp.N} min={1} max={20} step={1} color="#fb923c"
            onChange={v => setParams(p => ({ ...p, N: v }))} />
        </div>
        {/* Hysteresis guard display */}
        <div style={{ marginTop: 12, fontSize: 10, color: sp.T_enter > sp.T_exit ? "#4ade80" : "#ef4444" }}>
          {sp.T_enter > sp.T_exit
            ? `✓ Hysteresis OK: T_enter (${sp.T_enter}) > T_exit (${sp.T_exit})`
            : `✗ HYSTERESIS VIOLATED: T_enter must > T_exit`}
        </div>
      </div>

      {/* LEGEND */}
      <div style={{ display: "flex", gap: 20, marginTop: 12, fontSize: 10, color: "#475569" }}>
        {[["#334155","RAW"], ["#64748b","TRUTH"], ["#38bdf8","FILTERED"], ["#a78bfa","NIS"], ["#4ade80","P₀₀"]].map(([c, l]) => (
          <div key={l} style={{ display: "flex", alignItems: "center", gap: 5 }}>
            <div style={{ width: 20, height: 2, background: c }} />
            <span>{l}</span>
          </div>
        ))}
        <div style={{ display: "flex", alignItems: "center", gap: 5 }}><div style={{ width: 20, height: 1, background: "#f59e0b", borderTop: "1px dashed #f59e0b" }} /><span>T_ENTER</span></div>
        <div style={{ display: "flex", alignItems: "center", gap: 5 }}><div style={{ width: 20, height: 1, background: "#22d3ee", borderTop: "1px dashed #22d3ee" }} /><span>T_EXIT</span></div>
        <div style={{ display: "flex", alignItems: "center", gap: 5 }}><div style={{ width: 20, height: 1, background: "#ef4444", borderTop: "1px dashed #ef4444" }} /><span>T_GATE</span></div>
      </div>
    </div>
  );
}
