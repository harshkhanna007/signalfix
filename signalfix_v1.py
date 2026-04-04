"""
signalfix_v1.py
───────────────────────────────────────────────────────────────────────────────
SignalFix V1 — Robust Kalman Filter (Python/NumPy port)

Direct migration of the JavaScript V1 architecture. No redesign.
Architecture:
  - 2-state constant velocity model: x = [position, velocity]
  - dt-scaled Q matrix (Van Loan discretisation: dt³/3, dt²/2, dt terms)
  - Dual-state R with hysteresis switching (T_enter / T_exit)
  - NIS-based measurement gating
  - P covariance inflation every N consecutive rejections (counter NOT reset)
  - SAFE_MODE (prediction-only) after M consecutive rejections
  - Joseph-form covariance update for numerical stability
  - Covariance symmetrisation after every predict and update
  - Numerical floors: EPS, P_min, Q_min, R_min
  - NaN guard with state recovery

Requires: numpy
"""

from __future__ import annotations
import math
import numpy as np
from dataclasses import dataclass, field
from typing import Literal


# ──────────────────────────────────────────────────────────────────────────────
# NUMERICAL CONSTANTS  (mirrors JS constants block)
# ──────────────────────────────────────────────────────────────────────────────
EPS:   float = 1e-9
Q_MIN: float = 1e-9
R_MIN: float = 1e-9
P_MIN: float = 1e-9

NoiseState  = Literal["NOMINAL", "NOISY"]
SystemState = Literal["NORMAL", "SAFE_MODE"]


# ──────────────────────────────────────────────────────────────────────────────
# FILTER CONFIGURATION
# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class SignalFixParams:
    """All tunable parameters in one place. Mirrors DEFAULT_PARAMS in JS."""
    Q:          float = 0.01   # process noise intensity (scaled by dt inside filter)
    R_nominal:  float = 1.0    # measurement noise variance — nominal conditions
    R_noisy:    float = 10.0   # measurement noise variance — noisy conditions
    T_enter:    float = 12.0   # NIS threshold: NOMINAL → NOISY  (must be > T_exit)
    T_exit:     float = 4.0    # NIS threshold: NOISY → NOMINAL
    T_gate:     float = 25.0   # NIS gate: measurements above this are rejected
    beta:       float = 5.0    # P inflation factor (applied every N rejections)
    N:          int   = 3      # inflate P every N consecutive rejections
    M:          int   = 15     # hard-fail threshold: SAFE_MODE after M rejections
    alpha_init: float = 100.0  # initial P diagonal scale (large = wide prior)

    def validate(self) -> None:
        assert self.T_enter > self.T_exit, (
            f"Hysteresis violated: T_enter ({self.T_enter}) must be > T_exit ({self.T_exit})"
        )
        assert self.M > self.N,  "M must be > N so inflation can fire before hard fail"
        assert self.beta > 1.0,  "beta must be > 1 for inflation to expand P"


# ──────────────────────────────────────────────────────────────────────────────
# DIAGNOSTIC OUTPUT  (returned from every step)
# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class StepDiagnostics:
    NIS:             float
    NIS_nom:         float
    S:               float           # innovation covariance (with selected R)
    R_used:          float           # which R was active this step
    innovation:      float           # y = z - H x-
    accepted:        bool
    inflation_fired: bool
    noise_state:     NoiseState
    system_state:    SystemState
    rejection_count: int
    x_est:           np.ndarray      # filtered state [pos, vel]
    P_diag:          np.ndarray      # diagonal of P (uncertainty)


# ──────────────────────────────────────────────────────────────────────────────
# CORE FILTER
# ──────────────────────────────────────────────────────────────────────────────
class SignalFixV1:
    """
    Robust Kalman filter with adaptive noise switching, gating, P inflation,
    and hard-fail protection. Single scalar sensor, 2-state constant velocity
    model.

    State vector:  x = [position, velocity]  (2×1)
    Transition:    F = [[1, dt], [0, 1]]
    Observation:   H = [1, 0]  (position measured directly)
    """

    def __init__(self, params: SignalFixParams | None = None) -> None:
        self.params = params or SignalFixParams()
        self.params.validate()
        self._reset_state()

    # ── PUBLIC ────────────────────────────────────────────────────────────────

    def reset(self) -> None:
        """Reset filter to initial conditions."""
        self._reset_state()

    def step(self, z: float, dt: float) -> StepDiagnostics:
        """
        Process one measurement.

        Args:
            z:  scalar measurement
            dt: elapsed time since last step (seconds)

        Returns:
            StepDiagnostics with all outputs and internal telemetry.
        """
        p = self.params
        safe_dt = max(dt, 0.001)

        # ── Step A: Build F(dt) ───────────────────────────────────────────────
        F = np.array([[1.0, safe_dt],
                      [0.0, 1.0]])

        # H is fixed: observe position only. Shape (2,) for clean scalar math.
        H = np.array([1.0, 0.0])     # shape (2,)

        # ── Step B: Predict ───────────────────────────────────────────────────
        x_prior = F @ self.x                          # (2,)
        Q_mat   = self._build_Q(safe_dt)              # dt-scaled process noise
        P_prior = F @ self.P @ F.T + Q_mat            # (2×2)
        P_prior = self._symmetrize(self._apply_floor(P_prior))

        # ── Step C: Innovation ────────────────────────────────────────────────
        y = z - float(H @ x_prior)                    # scalar

        # ── Step D: Nominal S and NIS (using R_nominal always) ────────────────
        S_nom   = max(float(H @ P_prior @ H) + max(p.R_nominal, R_MIN), EPS)
        NIS_nom = (y * y) / S_nom

        # ── Step E: Noise state switching (hysteresis) ────────────────────────
        if self.noise_state == "NOMINAL" and NIS_nom > p.T_enter:
            self.noise_state = "NOISY"
        elif self.noise_state == "NOISY" and NIS_nom < p.T_exit:
            self.noise_state = "NOMINAL"

        # ── Step F: Select R ──────────────────────────────────────────────────
        R = max(p.R_nominal if self.noise_state == "NOMINAL" else p.R_noisy, R_MIN)

        # ── Step G: Recompute S and NIS with selected R (MANDATORY) ──────────
        S   = max(float(H @ P_prior @ H) + R, EPS)
        NIS = (y * y) / S

        # ── Step H: Gating ────────────────────────────────────────────────────
        # rejection_count accumulates continuously.
        # Only an accepted measurement resets it to 0.
        # SAFE_MODE: skip gating, preserve count, prediction-only.
        accepted = False
        if self.system_state == "SAFE_MODE":
            pass  # frozen — no count change
        elif NIS > p.T_gate:
            self.rejection_count += 1
        else:
            self.rejection_count = 0   # reset ONLY on acceptance
            accepted = True

        # ── Step I: P Inflation ───────────────────────────────────────────────
        # Fires every N consecutive rejections.
        # Counter is NOT reset here — hard fail must remain reachable.
        inflation_fired = (
            self.rejection_count > 0
            and self.rejection_count % p.N == 0
            and self.rejection_count <= p.M
        )
        P_inflated = P_prior * p.beta if inflation_fired else P_prior

        # ── Step J: Hard Fail ─────────────────────────────────────────────────
        if self.rejection_count > p.M:
            self.system_state = "SAFE_MODE"

        # ── Step K: Update (Joseph Form) ─────────────────────────────────────
        if accepted and self.system_state == "NORMAL":
            # Kalman gain: K = P- H^T S^-1  → shape (2,) since H is 1D
            K = (P_inflated @ H) / S          # (2,)

            # State update
            x_new = x_prior + K * y           # (2,)

            # Joseph form: P = (I - KH) P- (I - KH)^T + K R K^T
            # K outer H gives (2,2); K outer K scaled by R gives (2,2)
            I_KH  = np.eye(2) - np.outer(K, H)            # (2, 2)
            P_new = I_KH @ P_inflated @ I_KH.T + R * np.outer(K, K)
            P_new = self._symmetrize(self._apply_floor(P_new))
        else:
            # Rejected or SAFE_MODE — carry forward predicted state
            x_new = x_prior
            P_new = P_inflated

        # ── NaN guard ─────────────────────────────────────────────────────────
        if not np.isfinite(x_new).all() or not np.isfinite(P_new).all():
            x_new = x_prior
            P_new = np.eye(2) * p.alpha_init

        # ── Step L: Post-update stabilisation ────────────────────────────────
        P_new = self._symmetrize(self._apply_floor(P_new))

        # Persist state
        self.x = x_new
        self.P = P_new

        # ── Step M: Output ────────────────────────────────────────────────────
        return StepDiagnostics(
            NIS             = NIS,
            NIS_nom         = NIS_nom,
            S               = S,
            R_used          = R,
            innovation      = y,
            accepted        = accepted,
            inflation_fired = inflation_fired,
            noise_state     = self.noise_state,
            system_state    = self.system_state,
            rejection_count = self.rejection_count,
            x_est           = self.x.copy(),
            P_diag          = np.diag(self.P).copy(),
        )

    # ── PRIVATE ───────────────────────────────────────────────────────────────

    def _reset_state(self) -> None:
        p = self.params
        self.x:               np.ndarray  = np.zeros(2)
        self.P:               np.ndarray  = np.eye(2) * p.alpha_init
        self.noise_state:     NoiseState  = "NOMINAL"
        self.system_state:    SystemState = "NORMAL"
        self.rejection_count: int         = 0

    def _build_Q(self, dt: float) -> np.ndarray:
        """
        Discrete-time process noise matrix for constant velocity model.
        Van Loan discretisation:
          Q_pos   = Q * dt³ / 3
          Q_cross = Q * dt² / 2
          Q_vel   = Q * dt
        """
        q  = self.params.Q
        q_pos   = max(q * dt**3 / 3,  Q_MIN)
        q_cross = q * dt**2 / 2
        q_vel   = max(q * dt,          Q_MIN)
        return np.array([[q_pos,   q_cross],
                         [q_cross, q_vel  ]])

    @staticmethod
    def _symmetrize(M: np.ndarray) -> np.ndarray:
        """Enforce exact symmetry: P = 0.5 * (P + P^T)."""
        return 0.5 * (M + M.T)

    @staticmethod
    def _apply_floor(M: np.ndarray) -> np.ndarray:
        """Apply P_MIN floor to diagonal elements."""
        out = M.copy()
        idx = np.arange(min(out.shape))
        out[idx, idx] = np.maximum(out[idx, idx], P_MIN)
        return out


# ──────────────────────────────────────────────────────────────────────────────
# SIGNAL GENERATOR  (mirrors JS generateMeasurement)
# ──────────────────────────────────────────────────────────────────────────────
REGIME_LABELS = ["CLEAN", "NOISE BURST", "SPIKE TRAIN", "SENSOR DEATH"]

def generate_measurement(tick: int, rng: np.random.Generator) -> tuple[float, float, int]:
    """
    Produce a synthetic measurement cycling through 4 signal regimes
    every 80 ticks: clean → noise burst → spike train → sensor death.

    Returns:
        (z, true_val, regime_index)
    """
    true_val = math.sin(tick * 0.05) * 10.0
    regime   = (tick // 80) % 4

    if regime == 0:
        noise = (rng.random() - 0.5) * 2.0          # clean ±1
    elif regime == 1:
        noise = (rng.random() - 0.5) * 20.0         # sustained noise burst ±10
    elif regime == 2:
        # 30% chance of large spike, 70% clean
        noise = ((rng.random() - 0.5) * 80.0
                 if rng.random() < 0.3
                 else (rng.random() - 0.5) * 2.0)
    else:
        noise = (rng.random() - 0.5) * 200.0        # sensor death — massive noise

    return float(true_val + noise), true_val, regime


# ──────────────────────────────────────────────────────────────────────────────
# SIMULATION RUNNER
# ──────────────────────────────────────────────────────────────────────────────
def run_simulation(
    n_steps:  int                   = 500,
    dt:       float                 = 0.05,
    params:   SignalFixParams | None = None,
    seed:     int                   = 42,
    verbose:  bool                  = True,
) -> list[dict]:
    """
    Run a fixed-dt simulation for n_steps, exercising all four signal regimes.

    Args:
        n_steps:  number of filter steps
        dt:       fixed timestep in seconds
        params:   filter parameters (uses defaults if None)
        seed:     RNG seed for reproducibility
        verbose:  print per-step diagnostics

    Returns:
        List of result dicts (one per step) for downstream analysis.
    """
    filt = SignalFixV1(params)
    rng  = np.random.default_rng(seed)

    results: list[dict] = []

    # Event counters
    n_accepted      = 0
    n_rejected      = 0
    n_inflations    = 0
    n_noise_switch  = 0
    prev_noise_state: NoiseState = "NOMINAL"

    if verbose:
        print("=" * 80)
        print("SIGNALFIX V1 — 500-STEP SIMULATION")
        print("=" * 80)
        print(f"{'Tick':>5}  {'Regime':<13}  {'Raw':>8}  {'Filt':>8}  "
              f"{'NIS':>8}  {'Rej':>4}  {'Noise':>8}  {'Sys':>10}  {'Notes'}")
        print("-" * 80)

    for tick in range(n_steps):
        z, true_val, regime = generate_measurement(tick, rng)
        diag = filt.step(z, dt)

        # Track state changes
        notes = []
        if diag.noise_state != prev_noise_state:
            notes.append(f"→{diag.noise_state}")
            n_noise_switch += 1
            prev_noise_state = diag.noise_state

        if diag.inflation_fired:
            notes.append("P_INFLATED")
            n_inflations += 1

        if diag.system_state == "SAFE_MODE" and (not results or results[-1]["system_state"] != "SAFE_MODE"):
            notes.append("*** SAFE_MODE ENTERED ***")

        if diag.accepted:
            n_accepted += 1
        else:
            n_rejected += 1

        row = {
            "tick":            tick,
            "regime":          REGIME_LABELS[regime],
            "z":               z,
            "true_val":        true_val,
            "x_pos":           float(diag.x_est[0]),
            "x_vel":           float(diag.x_est[1]),
            "P00":             float(diag.P_diag[0]),
            "P11":             float(diag.P_diag[1]),
            "NIS":             diag.NIS,
            "NIS_nom":         diag.NIS_nom,
            "R_used":          diag.R_used,
            "accepted":        diag.accepted,
            "rejection_count": diag.rejection_count,
            "noise_state":     diag.noise_state,
            "system_state":    diag.system_state,
            "inflation_fired": diag.inflation_fired,
            "innovation":      diag.innovation,
        }
        results.append(row)

        if verbose:
            note_str = " | ".join(notes) if notes else ""
            print(
                f"{tick:>5}  {REGIME_LABELS[regime]:<13}  "
                f"{z:>8.3f}  {diag.x_est[0]:>8.3f}  "
                f"{diag.NIS:>8.3f}  {diag.rejection_count:>4}  "
                f"{diag.noise_state:>8}  {diag.system_state:>10}  "
                f"{note_str}"
            )

    if verbose:
        print("=" * 80)
        print("SUMMARY")
        print(f"  Total steps    : {n_steps}")
        print(f"  Accepted       : {n_accepted}  ({100*n_accepted/n_steps:.1f}%)")
        print(f"  Rejected       : {n_rejected}  ({100*n_rejected/n_steps:.1f}%)")
        print(f"  P inflations   : {n_inflations}")
        print(f"  Noise switches : {n_noise_switch}")
        print(f"  Final state    : system={filt.system_state}  noise={filt.noise_state}")
        print(f"  Final x̂       : pos={filt.x[0]:.4f}  vel={filt.x[1]:.4f}")
        print(f"  Final P diag   : {np.diag(filt.P)}")

        # RMSE over accepted steps only
        accepted_rows = [r for r in results if r["accepted"]]
        if accepted_rows:
            rmse = math.sqrt(
                sum((r["x_pos"] - r["true_val"])**2 for r in accepted_rows)
                / len(accepted_rows)
            )
            print(f"  RMSE (accepted): {rmse:.4f}")
        print("=" * 80)

    return results


# ──────────────────────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────────────────────
def main() -> None:
    params = SignalFixParams(
        Q         = 0.01,
        R_nominal = 1.0,
        R_noisy   = 10.0,
        T_enter   = 12.0,
        T_exit    = 4.0,
        T_gate    = 25.0,
        beta      = 5.0,
        N         = 3,
        M         = 15,
        alpha_init= 100.0,
    )

    run_simulation(
        n_steps = 500,
        dt      = 0.05,
        params  = params,
        seed    = 42,
        verbose = True,
    )


if __name__ == "__main__":
    main()
