// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : src/module1/pipeline.cpp
// Spec   : SFX-M1-TDS-001  Revision 2.1
// =============================================================================
//
// IngestionPipeline implementation.
//
// This file implements the pipeline orchestrator only. No signal processing
// algorithms are implemented here. Stage algorithms (S0–S7) are implemented
// in separate translation units and registered at pipeline construction time.
//
// =============================================================================

#include "signalfix/module1/pipeline.hpp"

#include <cassert>   // assert (debug-only invariant checks)
#include <cmath>     // std::isfinite (raw input sanity check)
#include <cstdio>    // std::fprintf (diagnostic output only, not in hot path)
#include <cstring>   // std::memset

// ---------------------------------------------------------------------------
// Internal diagnostics macro
//
// Diagnostic output is routed to stderr and is conditional on the
// SIGNALFIX_PIPELINE_DIAG preprocessor flag. In production builds this
// should be defined as 0 or left undefined to eliminate all fprintf calls.
// The macro evaluates to a no-op in that case with zero runtime cost.
//
// Deliberately using fprintf rather than a logger to avoid a dependency on
// any logging infrastructure in this core infrastructure file.
// ---------------------------------------------------------------------------
#ifndef SIGNALFIX_PIPELINE_DIAG
#  define SIGNALFIX_PIPELINE_DIAG 0
#endif

#if SIGNALFIX_PIPELINE_DIAG
#  define PIPELINE_LOG(fmt, ...) \
       std::fprintf(stderr, "[Pipeline] " fmt "\n", ##__VA_ARGS__)
#else
#  define PIPELINE_LOG(fmt, ...) ((void)0)
#endif


namespace signalfix {

// =============================================================================
// Construction
// =============================================================================

IngestionPipeline::IngestionPipeline(OutputBusFn output_bus_fn) noexcept
    : output_bus_fn_(output_bus_fn)
{
    // [FIX-1] Debug-only: catch null output bus at construction time.
    assert(output_bus_fn_ != nullptr &&
           "IngestionPipeline: output_bus_fn must not be null");

    // Zero-initialise the stage registry to null pointers.
    stages_.fill(nullptr);

    // Mark all channel table slots as empty.
    // kChannelIdEmpty (0xFFFF'FFFF) is the sentinel for an unoccupied slot.
    channel_ids_.fill(kChannelIdEmpty);

    // channel_states_ is default-initialised (all bytes zero via std::array).
    // channel_configs_ likewise.
    //
    // Note: we do NOT call init_channel_state() here; that is done per-channel
    // in register_channel() when actual parameters are known.
}


// =============================================================================
// Stage registration
// =============================================================================

bool IngestionPipeline::register_stage(IStage* stage) noexcept
{
    // Reject null stage pointers — a null in the stage array would cause a
    // null dereference in the hot path, which is unacceptable.
    if (stage == nullptr)
    {
        PIPELINE_LOG("register_stage: rejected null stage pointer at slot %u",
                     static_cast<unsigned>(stage_count_));
        return false;
    }

    if (stage_count_ >= kMaxPipelineStages)
    {
        PIPELINE_LOG("register_stage: stage table full (%u stages registered, max %u)",
                     static_cast<unsigned>(stage_count_),
                     static_cast<unsigned>(kMaxPipelineStages));
        return false;
    }

    stages_[stage_count_] = stage;
    ++stage_count_;

    // [FIX-5] Debug-level stage ordering validation.
    // If the stage declares an expected registration slot, verify match.
    const uint8_t actual_slot = stage_count_ - 1u;
    const uint8_t expected_slot = stage->expected_registration_index();
    if (expected_slot != IStage::kNoExpectedIndex &&
        expected_slot != actual_slot)
    {
        PIPELINE_LOG("register_stage: WARNING — '%s' expected slot %u "
                     "but was registered at slot %u (order mismatch)",
                     stage->stage_name(),
                     static_cast<unsigned>(expected_slot),
                     static_cast<unsigned>(actual_slot));
    }

    PIPELINE_LOG("register_stage: registered '%s' at slot %u",
                 stage->stage_name(),
                 static_cast<unsigned>(stage_count_ - 1u));
    return true;
}


// =============================================================================
// Channel registration
// =============================================================================

bool IngestionPipeline::register_channel(uint32_t channel_id,
                                          uint32_t calibration_version,
                                          uint64_t nominal_delta_t_us,
                                          uint16_t roc_window_size) noexcept
{
    // Refuse to register kChannelIdEmpty as a real channel_id to prevent
    // it from colliding with the sentinel value used in the lookup table.
    if (channel_id == kChannelIdEmpty)
    {
        PIPELINE_LOG("register_channel: channel_id 0x%08X is reserved (sentinel value)",
                     channel_id);
        return false;
    }

    // Duplicate check: a channel_id must not be registered twice.
    if (find_channel_index(channel_id) != kChannelNotFound)
    {
        PIPELINE_LOG("register_channel: channel_id 0x%08X already registered",
                     channel_id);
        return false;
    }

    if (channel_count_ >= kMaxChannels)
    {
        PIPELINE_LOG("register_channel: channel table full (%u channels, max %u)",
                     static_cast<unsigned>(channel_count_),
                     static_cast<unsigned>(kMaxChannels));
        return false;
    }

    const uint32_t slot = channel_count_;

    // Initialise ChannelState with the provided parameters.
    init_channel_state(channel_states_[slot],
                       channel_id,
                       calibration_version,
                       nominal_delta_t_us,
                       roc_window_size);

    // Record the key in the parallel id array.
    channel_ids_[slot] = channel_id;

    // Persist registration parameters for reset_channel().
    channel_configs_[slot].nominal_delta_t_us  = nominal_delta_t_us;
    channel_configs_[slot].calibration_version = calibration_version;
    channel_configs_[slot].roc_window_size     = roc_window_size;

    ++channel_count_;

    PIPELINE_LOG("register_channel: channel_id 0x%08X at slot %u "
                 "(nominal_dt=%llu μs, roc_window=%u)",
                 channel_id,
                 static_cast<unsigned>(slot),
                 static_cast<unsigned long long>(nominal_delta_t_us),
                 static_cast<unsigned>(roc_window_size));
    return true;
}


// =============================================================================
// Hot path: process()
// =============================================================================
//
// This is the innermost loop of the signal ingestion pipeline. At 1 kHz per
// channel, process() is called once per millisecond per channel.
//
// Performance annotations:
//   - No heap allocation (all state is in pre-allocated arrays).
//   - No system calls (no logging in non-diagnostic builds).
//   - One linear scan over channel_ids_ (≤64 comparisons, L1-cache resident).
//   - One virtual dispatch per registered stage (≤9 total).
//   - One output_bus_fn_ call at the end.
//
// =============================================================================

PipelineResult IngestionPipeline::process(const RawSample& sample) noexcept
{
    // [FIX-6] Debug diagnostic: warn if pipeline has no registered stages.
    if (stage_count_ == 0u)
    {
        PIPELINE_LOG("process: WARNING — pipeline has zero registered stages; "
                     "samples will pass through unprocessed");
    }

    // ── 1. Channel lookup ────────────────────────────────────────────────────

    const uint32_t slot = find_channel_index(sample.channel_id);

    if (slot == kChannelNotFound)
    {
        PIPELINE_LOG("process: unknown channel_id 0x%08X — dropping sample",
                     sample.channel_id);

        PipelineResult result{};
        result.envelope   = make_nominal_envelope();  // safe empty baseline
        result.status     = PipelineStatus::CHANNEL_UNKNOWN;
        result.fault_stage_index = PipelineResult::kNoFault;
        return result;
    }

    ChannelState& cs = channel_states_[slot];

    // ── 2. Build initial envelope ────────────────────────────────────────────
    //
    // make_nominal_envelope() sets all float/double fields to NaN and all
    // status flags to NOMINAL. This is the safety baseline described in
    // SFX-M1-TDS-001 Rev 2.1: any field not written by its responsible stage
    // will carry NaN into the S7 gate and be rejected there rather than
    // silently propagating a zero.

    MeasurementEnvelope env = make_initial_envelope(sample);

    // [FIX-4] Timestamp safety: detect non-monotonic or zero-delta arrivals.
    // Only checked after first sample (watchdog_armed == true indicates at
    // least one sample has been processed for this channel).
    if (cs.watchdog_armed &&
        sample.arrival_time_us <= cs.last_arrival_us)
    {
        PIPELINE_LOG("process: non-monotonic timestamp detected "
                     "(channel=0x%08X, arrival=%llu, last=%llu) — TIMING_ANOMALY",
                     sample.channel_id,
                     static_cast<unsigned long long>(sample.arrival_time_us),
                     static_cast<unsigned long long>(cs.last_arrival_us));
        env.status |= SampleStatus::TIMING_ANOMALY;
    }

    // ── 3. Stage execution loop ───────────────────────────────────────────────
    //
    // Each stage receives the envelope by reference and may mutate it.
    // Stage execution terminates immediately on ABORT_DROP or ABORT_FAULT;
    // the envelope is returned in its partial state for diagnostics.

    for (uint8_t i = 0u; i < stage_count_; ++i)
    {
        // Null stage pointer check: this should never trigger if register_stage()
        // is the only way stages are added (it rejects null). The check is here
        // as a defence-in-depth invariant for the pipeline contract.
        if (stages_[i] == nullptr)
        {
            PIPELINE_LOG("process: null stage pointer at index %u — aborting (FAULT)",
                         static_cast<unsigned>(i));

            PipelineResult result{};
            result.envelope          = env;
            result.status            = PipelineStatus::FAULT;
            result.fault_stage_index = i;
            return result;
        }

        const StageResult sr = stages_[i]->process(env, cs);

        if (sr == StageResult::ABORT_DROP)
        {
            PIPELINE_LOG("process: stage '%s' (idx %u) requested DROP",
                         stages_[i]->stage_name(),
                         static_cast<unsigned>(i));

            PipelineResult result{};
            result.envelope          = env;
            result.status            = PipelineStatus::DROPPED;
            result.fault_stage_index = PipelineResult::kNoFault;
            return result;
        }

        if (sr == StageResult::ABORT_FAULT)
        {
            PIPELINE_LOG("process: stage '%s' (idx %u) reported FAULT",
                         stages_[i]->stage_name(),
                         static_cast<unsigned>(i));

            // Force HARD_INVALID if the faulting stage did not already set it.
            // This prevents a partially-populated envelope reaching the output bus
            // via any future code path that might bypass the fault check.
            if (!has_flag(env.status, SampleStatus::HARD_INVALID))
            {
                env.status |= SampleStatus::HARD_INVALID;
            }

            PipelineResult result{};
            result.envelope          = env;
            result.status            = PipelineStatus::FAULT;
            result.fault_stage_index = i;
            return result;
        }

        // StageResult::CONTINUE — advance to next stage.
    }

    // ── 4. S7 output bus gate ─────────────────────────────────────────────────
    //
    // Unconditional: runs regardless of how many stages are registered.
    // Validates status flag invariants (INV-1..6) and calls output_bus_fn_.

    const PipelineStatus bus_status = run_output_bus(env);

    PipelineResult result{};
    result.envelope          = env;
    result.status            = bus_status;
    result.fault_stage_index = PipelineResult::kNoFault;
    return result;
}


// =============================================================================
// Watchdog path: process_watchdog()
// =============================================================================
//
// Synthesises a STALE|MISSING envelope for a channel that has not produced
// a sample within 2 × nominal_delta_t_us. Bypasses S1–S6 entirely.
//
// The synthetic envelope carries:
//   - The next sequence_id (preserves monotonicity of the sequence stream).
//   - delta_t_us = 2 × nominal_delta_t_us (the watchdog timeout interval).
//   - status = STALE | MISSING.
//   - All value fields = NaN (no measurement data available).
//   - measurement_trust_tier = REJECTED (direct set; derive_trust_tier
//     would give the same result but we avoid the call in the hot path).
//
// Downstream SR-UKF performs a pure prediction step on REJECTED envelopes.
//
// The S7 NaN gate will reject this envelope (raw_value is NaN on a synthetic
// sample). In production, the watchdog path should write to a dedicated
// stale-event side-channel rather than the main ring buffer. The gate
// rejection is the correct architectural behaviour.
//
// =============================================================================

PipelineResult IngestionPipeline::process_watchdog(uint32_t channel_id,
                                                    uint64_t current_time_us) noexcept
{
    const uint32_t slot = find_channel_index(channel_id);

    if (slot == kChannelNotFound)
    {
        PIPELINE_LOG("process_watchdog: unknown channel_id 0x%08X", channel_id);

        PipelineResult result{};
        result.envelope          = make_nominal_envelope();
        result.status            = PipelineStatus::CHANNEL_UNKNOWN;
        result.fault_stage_index = PipelineResult::kNoFault;
        return result;
    }

    ChannelState& cs = channel_states_[slot];

    // Build the synthetic STALE|MISSING envelope.
    MeasurementEnvelope env = make_nominal_envelope();

    env.channel_id           = cs.channel_id;
    env.sequence_id          = cs.sequence_counter++;    // Increment for monotonicity.
    env.arrival_time_us      = current_time_us;
    env.timestamp_us         = current_time_us;          // No PLL correction possible.
    env.delta_t_us           = 2u * cs.nominal_delta_t_us;
    env.jitter_us            = 0;
    env.gap_event_id         = 0u;                       // Not a gap event; sensor absent.
    env.calibration_version  = cs.calibration_version;

    // nominal_streak_count: carries the streak count that was valid before this
    // timeout. The streak will be reset in ChannelState below.
    env.nominal_streak_count = cs.nominal_streak;
    env.roc_window_n         = 0u;  // No ROC derivable from a synthetic sample.

    // Value fields remain NaN (from make_nominal_envelope). This is correct:
    // there is no measurement data. The S7 gate enforces NaN for MISSING.

    // Status: STALE | MISSING is the canonical watchdog combination (SFX-M1-TDS-001
    // Rev 2.1 — STALE is always paired with MISSING per the SampleStatus contract).
    env.status               = SampleStatus::STALE | SampleStatus::MISSING;
    env.measurement_trust_tier = MeasurementTrustTier::REJECTED;
    env.pre_filter_applied   = false;
    env.filter_window_n      = 0u;

    // Update channel state to reflect the watchdog event.
    cs.watchdog_deadline_us  = current_time_us + 2u * cs.nominal_delta_t_us;
    cs.nominal_streak        = 0u;   // Watchdog break resets the nominal streak.

    PIPELINE_LOG("process_watchdog: channel 0x%08X seq=%llu — STALE|MISSING injected",
                 channel_id,
                 static_cast<unsigned long long>(env.sequence_id));

    // S7 gate — validate_status_flags + output bus.
    // Expected result: GATE_REJECT (raw_value is NaN on a synthetic sample).
    const PipelineStatus bus_status = run_output_bus(env);

    PipelineResult result{};
    result.envelope          = env;
    result.status            = bus_status;
    result.fault_stage_index = PipelineResult::kNoFault;
    return result;
}


// =============================================================================
// reset_channel()
// =============================================================================

bool IngestionPipeline::reset_channel(uint32_t channel_id) noexcept
{
    const uint32_t slot = find_channel_index(channel_id);
    if (slot == kChannelNotFound) { return false; }

    const ChannelConfig& cfg = channel_configs_[slot];
    init_channel_state(channel_states_[slot],
                       channel_id,
                       cfg.calibration_version,
                       cfg.nominal_delta_t_us,
                       cfg.roc_window_size);

    PIPELINE_LOG("reset_channel: channel_id 0x%08X state reset", channel_id);
    return true;
}


// =============================================================================
// get_channel_state() — diagnostic accessor
// =============================================================================

const ChannelState* IngestionPipeline::get_channel_state(uint32_t channel_id) const noexcept
{
    const uint32_t slot = find_channel_index(channel_id);
    if (slot == kChannelNotFound) { return nullptr; }
    return &channel_states_[slot];
}


// =============================================================================
// Private helpers
// =============================================================================

// ---------------------------------------------------------------------------
// find_channel_index — linear scan over channel_ids_ (L1-cache resident)
//
// Returns the slot index, or kChannelNotFound.
//
// Why linear scan instead of a hash map?
//   At kMaxChannels = 64, the channel_ids_ array is 256 bytes. It fits in
//   a single L1 cache line group and is accessed on every process() call.
//   A hash map adds indirection, memory overhead, and complexity with no
//   measurable benefit at this scale. Re-evaluate if kMaxChannels > 256.
// ---------------------------------------------------------------------------
uint32_t IngestionPipeline::find_channel_index(uint32_t channel_id) const noexcept
{
    for (uint32_t i = 0u; i < channel_count_; ++i)
    {
        if (channel_ids_[i] == channel_id)
        {
            return i;
        }
    }
    return kChannelNotFound;
}


// ---------------------------------------------------------------------------
// make_initial_envelope — baseline envelope for a new sample
//
// Starts from the Rev 2.1 safe defaults (all float/double = NaN, status =
// NOMINAL) and stamps in the two fields available at ingest time.
//
// Why not also stamp channel_id here?
//   S0 (Input Adapter) is contractually responsible for populating channel_id,
//   sequence_id, and calibration_version. Stamping them here would couple the
//   infrastructure to S0's responsibilities. S0 will overwrite channel_id on
//   its first execution; the NaN value fields serve as an audit trail that S0
//   ran and populated them correctly.
//
//   In practice, S0 runs first and overwrites channel_id immediately, so the
//   intermediate state is never visible outside the pipeline.
// ---------------------------------------------------------------------------
MeasurementEnvelope IngestionPipeline::make_initial_envelope(
    const RawSample& sample) noexcept
{
    MeasurementEnvelope env = make_nominal_envelope();

    // These two fields are the only ones the pipeline infrastructure populates
    // directly. Everything else is left to the stage implementations.
    env.arrival_time_us = sample.arrival_time_us;
    env.raw_value       = sample.raw_value;

    // Pre-stamp channel_id so that stages can safely read it even before S0
    // runs (e.g., a diagnostic logging wrapper). S0 will overwrite this with
    // the value from ChannelState, which should match. If they diverge, that
    // is a misconfiguration that S0 should detect and flag.
    env.channel_id = sample.channel_id;

    // [FIX-2] Raw input sanity: detect NaN / Inf before pipeline stages.
    // The raw_value is preserved (not overwritten) so that stages can inspect
    // the original value. The HARD_INVALID flag prevents silent propagation.
    if (!std::isfinite(sample.raw_value))
    {
        PIPELINE_LOG("make_initial_envelope: raw_value is NaN or Inf "
                     "(channel=0x%08X) — marking HARD_INVALID",
                     sample.channel_id);
        env.status |= SampleStatus::HARD_INVALID;
    }

    return env;
}


// ---------------------------------------------------------------------------
// run_output_bus — S7 invariant check and output bus write
//
// This function is the unconditional last step of every pipeline invocation.
// It enforces two independent gates in series:
//
//   Gate 1 — Status flag invariant check (validate_status_flags):
//     Detects architecturally invalid status combinations (INV-1..6).
//     A failure here indicates a bug in the packager (S6) or an earlier
//     stage that set conflicting flags. The envelope is rejected and the
//     caller is notified with INV_FAIL.
//
//   Gate 2 — Output bus write (output_bus_fn_):
//     The concrete output bus implementation (ring buffer write, etc.)
//     performs its own NaN/Inf check and returns false on rejection.
//     The pipeline maps false to GATE_REJECT.
//
// Both gates are always run for every envelope that reaches this point,
// regardless of how many stages are registered. This ensures that the
// contract with downstream modules (SR-UKF, fault detector, etc.) is
// enforced even during staged system bring-up where some stages are stubs.
// ---------------------------------------------------------------------------
PipelineStatus IngestionPipeline::run_output_bus(
    const MeasurementEnvelope& envelope) const noexcept
{
    // Gate 1: validate status flag invariants.
    if (!validate_status_flags(envelope.status))
    {
        PIPELINE_LOG("run_output_bus: validate_status_flags() failed "
                     "(status=0x%02X) — INV_FAIL, envelope dropped",
                     static_cast<unsigned>(envelope.status));
        return PipelineStatus::INV_FAIL;
    }

    // Gate 2: output bus write.
    // [FIX-1] Defensive null check — prevents crash if constructed with null.
    if (output_bus_fn_ == nullptr)
    {
        PIPELINE_LOG("run_output_bus: output_bus_fn_ is null — returning FAULT");
        return PipelineStatus::FAULT;
    }
    const bool accepted = output_bus_fn_(envelope);

    if (!accepted)
    {
        PIPELINE_LOG("run_output_bus: output_bus_fn_ rejected envelope "
                     "(channel=0x%08X seq=%llu status=0x%02X) — GATE_REJECT",
                     envelope.channel_id,
                     static_cast<unsigned long long>(envelope.sequence_id),
                     static_cast<unsigned>(envelope.status));
        return PipelineStatus::GATE_REJECT;
    }

    return PipelineStatus::OK;
}

} // namespace signalfix
