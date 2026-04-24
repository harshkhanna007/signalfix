# SignalFix AI

A real-time signal integrity system for noisy industrial sensor data.

SignalFix monitors statistical drift, detects persistent anomalies, and maintains temporal context to separate genuine system degradation from measurement noise.

---

## The Problem

Industrial sensors rarely fail instantly. They degrade slowly.

A temperature probe might drift 0.3% per hour over several days. By the time fixed-threshold systems detect it, the damage is already done.

Real-world sensor data suffers from:

* Clock jitter (irregular timestamps, out-of-order samples)
* Dropouts (missing data for milliseconds to seconds)
* Gradual drift (calibration degradation below alert thresholds)
* Transient noise (EMI spikes, vibration, thermal effects)

Traditional monitoring systems rely on static thresholds. These cannot distinguish between a failing sensor and a noisy environment.

---

## The Solution

SignalFix sits between raw sensor streams and downstream systems (controllers, models, SCADA).

It provides:

* Adaptive thresholding based on real-time signal statistics
* Temporal reasoning (tracks persistence, not just magnitude)
* Multi-stage validation pipeline for signal integrity
* Stateful classification (NOMINAL / DRIFT_EXCEEDED)
* Timestamp correction using PLL-based alignment

The system does not just detect anomalies.
It determines whether the signal is **temporarily noisy or consistently degrading**.

---

## What Makes It Different

### Adaptive + Temporal + System-Level Thinking

* Thresholds evolve with signal behavior (mean, variance, ROC history)
* Detection is based on **persistence over time**, not instant spikes
* Multi-stage pipeline validates data before statistical decisions

### Design Philosophy

> AI assists, math decides

* Adaptive logic tunes thresholds and sensitivity
* Deterministic state machines enforce final decisions
* No black-box inference — all outputs are traceable

---

## Current Capabilities (Module 1 Prototype)

Signal Preprocessing Pipeline (9 stages, ~3,200 lines C++):

* Input validation and timestamping
* Calibration (gain/offset correction)
* Gap detection and classification
* PLL-based timestamp correction
* Plausibility validation (hard/soft bounds)
* Adaptive rate-of-change monitoring
* Drift persistence arbitration
* Output classification and quality tagging

### Key Features

* CUSUM-style drift accumulation (gsigma)
* Adaptive deadband expansion (noise-floor tracking)
* Asymmetric hysteresis (NOISE → BUILDUP → CONFIRMED → CRITICAL)
* Confidence scoring based on persistence + statistical deviation
* Monotonic timestamp enforcement

Validation:

* 20/20 invariant tests passed
* 14/14 trust-tier tests passed

---

## Example Behavior

**Scenario:** Temperature sensor drifts 2.5°C over 3 minutes

* Initial samples remain NOMINAL as drift is small
* Adaptive threshold evolves with local variance
* Sustained increase in rate-of-change triggers drift detection
* System transitions from BUILDUP → CONFIRMED based on persistence

Key signals:

* `rr` → instantaneous rate of change
* `T_roc` → adaptive threshold
* `streak` → consecutive threshold violations
* `persist` → duration of sustained deviation
* `gsigma` → cumulative statistical drift

SignalFix flags drift only after **consistent deviation over time**, not from isolated spikes.

---

## Status

Prototype system, not production-ready.

* Core algorithms implemented and validated
* Robust against NaN/Inf, missing data, and clock anomalies
* No real industrial deployment yet
* Developed using C-MAPSS turbofan dataset (aerospace domain)

---

## Why This Exists

Most systems focus on making better decisions.

SignalFix focuses on something more fundamental:

**ensuring the data those decisions rely on is actually trustworthy.**
