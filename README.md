SignalFix AI
A real-time signal integrity system for noisy industrial sensor data. Monitors statistical drift, detects persistent anomalies, and maintains temporal context to separate genuine degradation from measurement noise.

The Problem
Industrial sensors fail gradually, not suddenly. A temperature probe doesn't jump from 100°C to 500°C — it drifts by 0.3% per hour over three days. By the time traditional threshold-based monitoring catches it, you've lost a $2M pharmaceutical batch.
Real-world sensor data has:

Clock jitter — timestamps drift, samples arrive late or out of order
Dropouts — sensors go silent for milliseconds to seconds
Gradual drift — calibration degrades slowly, below alert thresholds
Transient noise — EMI spikes, vibration, thermal effects

Fixed thresholds can't distinguish between a sensor dying and a noisy measurement environment. You either tolerate false alarms or miss real failures.

The Solution
SignalFix sits between raw sensor streams and downstream systems (process controllers, predictive models, SCADA). It provides:

Adaptive thresholding — Learns baseline statistics per channel, adjusts limits dynamically
Temporal reasoning — Tracks persistence (how long), not just magnitude (how much)
Multi-stage validation — Calibration → gap detection → plausibility → ROC monitoring → drift arbitration
State classification — NOMINAL / DRIFT_EXCEEDED with confidence scoring
Timestamp correction — PLL-based clock alignment to handle jitter and dropouts

The system doesn't just flag anomalies — it differentiates between "sensor spiked once" and "sensor has been degrading for 90 seconds."

What Makes It Different
Adaptive + Temporal + System Thinking

Not rule-based: Thresholds adjust to local conditions (mean, variance, ROC history)
Not instant-only: Persistence accumulators distinguish transient noise from sustained drift
Not isolated: Multi-stage pipeline validates data integrity before statistical analysis

Design Philosophy: "AI assists, math decides"

Adaptive algorithms tune parameters (deadbands, thresholds)
Deterministic state machines enforce decisions (NOMINAL ↔ DRIFT transitions)
No black-box inference — every decision is traceable to measurable signal properties


Current Capabilities (Module 1 Prototype)
Signal Preprocessing Pipeline (9 stages, ~3,200 lines C++)
StageFunctionS0Input validation, arrival timestampingS1Linear calibration (gain/offset correction)S2aGap pre-detection (missing samples)S2bPLL-based timestamp correctionS3Plausibility bounds (hard/soft limits)S4Gap classification (minor/major/critical)S5Rate-of-change monitoring (adaptive ROC thresholds)S5.5Drift persistence arbitration (time-based hysteresis)S7Output packaging, quality classification
Key Features Implemented:

CUSUM-style drift accumulator (drift_gsigma)
Adaptive deadband expansion (noise-floor tracking)
Asymmetric hysteresis (4-level drift classification: NOISE → BUILDUP → CONFIRMED → CRITICAL)
Confidence scoring from persistence + ROC + statistical deviation
Monotonic timestamp enforcement
20/20 invariant tests passing, 14/14 trust tier tests passing


Example Behavior
Scenario: Temperature sensor drifts 2.5°C over 3 minutes
[S5] ch=0x00010203 seq=1823: rr=0.0082°C/s T_roc=0.012 NOMINAL (streak=0)
[S5] ch=0x00010203 seq=1824: rr=0.0091°C/s T_roc=0.012 NOMINAL (streak=0)
[S5] ch=0x00010203 seq=1825: rr=0.0098°C/s T_roc=0.012 NOMINAL (streak=0)
...
[S5] ch=0x00010203 seq=1912: rr=0.0134°C/s T_roc=0.013 ⚠ DRIFT (streak=1, persist=0.5s)
[S5] ch=0x00010203 seq=1913: rr=0.0141°C/s T_roc=0.013 ⚠ DRIFT (streak=2, persist=1.0s)
...
[S5.5] BUILDUP → CONFIRMED (persist=2.8s, conf=0.67, gsigma=4.2)
What this shows:

rr = instantaneous rate of change (°C/s)
T_roc = adaptive threshold (grows with local variance)
streak = consecutive samples with ROC > threshold
persist = wall-clock duration of sustained drift
gsigma = CUSUM accumulator (fractional standard deviations)

The system flags DRIFT only after sustained elevation (2.8 seconds), not from a single noisy sample. Confidence increases as persistence + statistical evidence accumulate.
Status
Prototype, not production.

✅ Core algorithms implemented and tested
✅ Passes invariant + trust tier test suites
✅ Hardened against NaN/Inf, clock anomalies, missing samples
⚠ No customer deployments yet
⚠ No ground-truth validation on real industrial datasets
⚠ C-MAPSS turbofan data used for development (aerospace, not pharma/food/chemical)
