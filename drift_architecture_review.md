# 🛑 Architecture Review: SignalFix Drift Detection

## 1. Architecture Review & Brutal Truths

Your proposed direction implies squashing multiple distinct dimensions (energy, rate, time, and probability) into a single scalar formula `decision_score = f(drift_gsigma, momentum, confidence, persistence)`. 

**This is a trap.** It is mathematically unsound and operationally brittle.

Mixing time (`persistence`), physical magnitude (`gsigma`), statistical quality (`confidence`), and rate (`momentum`) into one equation leads to **dimension mismatch**. You will spend months tuning arbitrary weights, and it will fail as soon as an engine exhibits a different noise profile. 

If `f()` is linear (e.g., `w1*gsigma + w2*momentum + ...`), a massive momentary spike (high momentum, low persistence) can produce the identical score as a long, slow drift (low momentum, high persistence). To the operator, these are fundamentally different events, but your system will treat them the same.

### Where it will fail in the real world:
* **The "Noisy Recovery" false positive:** System stays latched in drift for hours because `drift_gsigma` is still draining, even though `momentum` is strongly negative (engine is recovering) and `confidence` is high. 
* **The "Low Confidence" hallucination:** A bad sensor drops out, creating massive `momentum` and `gsigma`. Even if `persistence` is met, triggering an alert on bad data destroys operator trust.

---

## 2. Key Flaws in Current Design

1. **Missing Exit Condition:** CUSUM measures accumulation, but doesn't inherently decay unless explicitly driven by a negative shift or drain factor. Without an explicit definition of "Normal", you have a sticky state.
2. **Dimension Bleeding:** `persistence` is a temporal property. `gsigma` is an energy/spatial property. Treating them as inputs to the same equation mixes spatial and temporal domains.
3. **Alert/Logic Coupling:** The system treats "is it drifting currently" (state) as interchangeable with "should I alert the user" (edge). This causes spam.
4. **Veto Bypass:** `confidence` is weakly used. Low confidence shouldn't just lower a score; it should act as an absolute veto against new state transitions.

---

## 3. Improved Design: The Orthogonal Decision Architecture

To achieve "Silence by default. Speak only when it matters", we must decouple **Evidence Generation** (magnitude/rate) from **Time Validation** (persistence/recovery).

**The Principles:**
* **Confidence is a Gate:** Do not trust high drift if confidence is low.
* **Momentum is Directional Context:** Only use momentum to accelerate detection of worsening states, not to trigger them standalone.
* **Separation of Space and Time:** Compute a `Severity Score` instantly every frame. Then use a **State Machine** to enforce `Persistence` across time.
* **Hysteresis on Both Axes:** Use different spatial thresholds AND different temporal thresholds for triggering vs. recovering.

---

## 4. Final Decision Logic (Formula + Pseudocode)

### Part 1: The Instantaneous Severity Score (Space Domain)

We compute a clean `severity_score` that represents the *believability and magnitude* of drift at this exact moment.

```python
# 1. Gate: If confidence is garbage, the signal is garbage.
if confidence < CONFIDENCE_MIN_THRESH:
    severity_score = 0.0
else:
    # 2. Base Evidence: Scale accumulated drift by our confidence in it.
    base_evidence = drift_gsigma * confidence
    
    # 3. Momentum Accelerator: If drift is actively worsening, amplify it. 
    # If recovering (negative momentum), it dampens the score.
    # We clip momentum so an unphysical spike doesn't blow up the score.
    m_factor = 1.0 + (K_MOMENTUM * max(0.0, min(momentum, MAX_M_LIMIT)))
    
    severity_score = base_evidence * m_factor
```
*Why this works:* It normalizes out time. The score simply answers: "Right now, how bad is the true signal?"

### Part 2: The State Machine (Time Domain)

Here is where we enforce "Sustained" behavior and handle Exit/Recovery.

```python
# State variables:
# current_state (NOMINAL, DRIFT_CONFIRMED)
# persist_timer, recover_timer

if current_state == NOMINAL:
    if severity_score > THRESH_TRIGGER:
        persist_timer += 1
        if persist_timer >= REQUIRED_PERSIST_FRAMES:
            current_state = DRIFT_CONFIRMED
            emit_alert(Event.DRIFT_STARTED, severity=severity_score)
    else:
        # Rapid decay of timer if signal drops back below threshold (noise rejection)
        persist_timer = max(0, persist_timer - 2) 

elif current_state == DRIFT_CONFIRMED:
    # RECOVERY REQUIRES 3 THINGS:
    # 1. Score drops below lower hysteresis threshold
    # 2. Momentum is not actively positive (things aren't getting worse again)
    # 3. Time sustained
    
    if severity_score < THRESH_RECOVER and momentum <= 0.0:
        recover_timer += 1
        if recover_timer >= REQUIRED_RECOVER_FRAMES:
            current_state = NOMINAL
            persist_timer = 0
            emit_alert(Event.DRIFT_RESOLVED)
    else:
        recover_timer = 0
```

---

## 5. Exit / Recovery Logic In-Depth

**Premature Recovery Prevention:**
Notice `THRESH_RECOVER < THRESH_TRIGGER` (Amplitude Hysteresis) AND `REQUIRED_RECOVER_FRAMES > REQUIRED_PERSIST_FRAMES` (Temporal Hysteresis). 
Additionally, recovery is instantly aborted if `momentum > 0`. If the signal is trending up even slightly, we freeze the recovery timer. This ensures the engine must truly settle out before we declare it healthy.

---

## 6. Alerting Behavior (Avoiding Spam)

**Rule: Never emit state. Emit Edges and Escalations.**

* **Silent by Default:** While in `NOMINAL` (even if `severity_score` is rising), print nothing. Write to telemetry databases for silent tracing, but emit zero logs/alerts.
* **The "Speak" Events:**
  1. `DRIFT_STARTED`: Triggered exactly *once* when state transitions to `DRIFT_CONFIRMED`.
  2. `DRIFT_ESCALATED`: While in `DRIFT_CONFIRMED`, if `severity_score` crosses a massive secondary threshold (e.g., `severity > THRESH_CRITICAL`) and a cool-down timer has expired, emit an update.
  3. `DRIFT_RESOLVED`: Triggered exactly *once* upon returning to `NOMINAL`.

---

## 7. Why this is Production-Safe

1. **Deterministic:** No hidden ML weights. If it triggers, you can look at the log and trace exactly why (Threshold X crossed, sustained for Y frames).
2. **Noise Immunity:** The rapid decrement of `persist_timer` (e.g., `persist_timer - 2`) means that an oscillating signal that dips below the threshold frequently will never trigger the alarm. It absolutely *must* be sustained.
3. **No Ghost Alarms:** The `confidence` multiplier mathematically terminates spurious outputs caused by sensor resets or telemetry dropouts.
4. **Physical Realism:** Using `momentum <= 0` as a hard gate for recovery mirrors the physical world—you shouldn't consider a system "recovering" if the degradation vector is still pointing up, even if the absolute accumulation dropped slightly.
