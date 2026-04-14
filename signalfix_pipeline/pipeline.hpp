// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : include/signalfix/module1/pipeline.hpp
// Spec   : SFX-M1-TDS-001  Revision 2.1
// =============================================================================
//
// IngestionPipeline — deterministic single-producer pipeline orchestrator.
//
// Responsibilities:
//   - Maintain an ordered registry of pipeline stages (S0–S7).
//   - Manage a fixed-capacity per-channel state table.
//   - Accept raw samples, construct initial MeasurementEnvelopes, and drive
//     the envelope through all registered stages.
//   - Enforce S7 invariants (validate_status_flags + NaN gate) unconditionally
//     before every output bus write, regardless of the registered stage set.
//   - Provide a separate watchdog path that injects STALE|MISSING envelopes
//     without running S1–S6.
//
// Thread model:
//   Single-producer per instance. process() and process_watchdog() must be
//   called from the same thread. No internal synchronisation primitives are
//   used; the caller is responsible for serialisation.
//
//   To process multiple sensor groups concurrently, instantiate one
//   IngestionPipeline per group and run each on its own thread.
//
// Error model:
//   -fno-exceptions is assumed. All errors are returned via PipelineResult.
//   The pipeline never calls abort() or triggers UB on invalid inputs —
//   it returns FAULT or DROPPED with the appropriate diagnostic fields set.
//
// Memory model:
//   No heap allocation after construction. All channel state and stage
//   pointers live in the pipeline's own storage. Stage objects are owned
//   by the caller; the pipeline holds non-owning pointers.
//
// =============================================================================

#pragma once

#include "signalfix/module1/types.hpp"
#include "signalfix/module1/stage_interface.hpp"
#include "signalfix/module1/channel_state.hpp"

#include <array>
#include <cstdint>

namespace signalfix {

// ---------------------------------------------------------------------------
// RawSample — minimal ingest token from a sensor driver
//
// This is the only external input the pipeline accepts. The Input Adapter
// stage (S0) consumes this together with the pre-populated initial envelope.
//
// raw_value carries the pre-calibration sensor measurement in protocol units
// (ADC counts, raw voltage, etc.). Units are channel-specific; S1 converts.
// ---------------------------------------------------------------------------
struct RawSample
{
    uint64_t arrival_time_us;  ///< Monotonic system clock at sample ingestion [μs].
    double   raw_value;        ///< Pre-calibration sensor value. Protocol units.
    uint32_t channel_id;       ///< Must match a registered channel.
    uint32_t _pad;             ///< Explicit padding; not transmitted to downstream.
};
static_assert(sizeof(RawSample) == 24u, "RawSample layout changed.");
static_assert(alignof(RawSample) == 8u, "RawSample alignment changed.");


// ---------------------------------------------------------------------------
// PipelineStatus — top-level outcome of one pipeline invocation
//
// OK:           All stages passed. Envelope written to output bus.
// DROPPED:      A stage returned ABORT_DROP. Envelope not emitted.
//               This is a policy outcome, not a fault (e.g., a gap engine
//               decision to discard rather than fill, a duplicate sequence).
// FAULT:        A stage returned ABORT_FAULT, or an unrecoverable internal
//               error was detected (e.g., unknown channel_id, null stage ptr).
//               fault_stage_index identifies the responsible stage.
// INV_FAIL:     validate_status_flags() detected an invalid status combination
//               at S7. Indicates a packager logic error (S6 bug). Not emitted.
// GATE_REJECT:  The S7 NaN/Inf gate rejected the envelope. Indicates a value
//               field was left uninitialised (NaN) by a prior stage without
//               the appropriate status flag being set.
// CHANNEL_UNKNOWN: process() called with a channel_id not registered.
// ---------------------------------------------------------------------------
enum class PipelineStatus : uint8_t
{
    OK              = 0u,
    DROPPED         = 1u,
    FAULT           = 2u,
    INV_FAIL        = 3u,
    GATE_REJECT     = 4u,
    CHANNEL_UNKNOWN = 5u,
};


// ---------------------------------------------------------------------------
// PipelineResult — returned by every process() and process_watchdog() call
//
// On OK:              envelope is valid and has been committed to the output bus.
// On DROPPED/FAULT:   envelope contains the partially-processed state at abort.
//                     fault_stage_index identifies the first failing stage.
// On INV_FAIL/GATE_REJECT: envelope is the fully-processed (but rejected) state.
// On CHANNEL_UNKNOWN: envelope is the make_nominal_envelope() baseline, unpopulated.
//
// fault_stage_index == kNoFault when status is OK or CHANNEL_UNKNOWN.
// ---------------------------------------------------------------------------
struct PipelineResult
{
    MeasurementEnvelope envelope;          ///< Final (or partial) envelope.
    PipelineStatus      status;            ///< Top-level outcome.
    uint8_t             fault_stage_index; ///< Stage index that aborted (or kNoFault).
    uint8_t             _pad[2];

    static constexpr uint8_t kNoFault = 0xFFu;
};
static_assert(std::is_trivially_copyable<PipelineResult>::value,
    "PipelineResult must be trivially copyable for safe pass-by-value.");


// ---------------------------------------------------------------------------
// Pipeline capacity constants
//
// kMaxPipelineStages: S0, S1, S2a, S2b, S3, S4, S5, S6, S7 = 9 stages.
// kMaxChannels: designed for robotics/aerospace multi-sensor deployments.
//               Increase if the application has more channels; the cost is
//               linear in the channel table size (~600 bytes per channel).
// ---------------------------------------------------------------------------
static constexpr uint8_t  kMaxPipelineStages = 9u;
static constexpr uint32_t kMaxChannels       = 64u;


// ---------------------------------------------------------------------------
// IngestionPipeline
//
// Usage pattern:
//
//   // 1. Construct with the output bus callback.
//   IngestionPipeline pipeline(my_output_bus_write);
//
//   // 2. Register stages in architecture order S0 → S7.
//   pipeline.register_stage(&my_s0_adapter);
//   pipeline.register_stage(&my_s1_calibration);
//   // ... S2a, S2b, S3, S4, S5, S6, S7
//
//   // 3. Register channels.
//   pipeline.register_channel(channel_id, cal_version, nominal_dt_us, roc_window);
//
//   // 4. Drive the pipeline from the sensor acquisition loop.
//   while (running) {
//       RawSample s = sensor.read();
//       PipelineResult r = pipeline.process(s);
//       if (r.status != PipelineStatus::OK) { handle_error(r); }
//   }
//
//   // 5. Drive watchdog from a timer callback.
//   if (now_us > deadline_us) {
//       pipeline.process_watchdog(channel_id, now_us);
//   }
// ---------------------------------------------------------------------------
class IngestionPipeline
{
public:
    // Output bus callback type.
    // Invoked with the final envelope after all stage passes and invariant checks.
    // Returns true  → envelope accepted and committed.
    // Returns false → envelope rejected (e.g., ring buffer full, NaN detected).
    // Must be noexcept and must not re-enter the pipeline.
    using OutputBusFn = bool (*)(const MeasurementEnvelope&) noexcept;

    // Construct with a mandatory output bus callback.
    // output_bus_fn must not be null; the pipeline asserts this in debug builds.
    explicit IngestionPipeline(OutputBusFn output_bus_fn) noexcept;

    // Not copyable or movable. The stage pointer array and channel state table
    // are intrinsically tied to this instance's address.
    IngestionPipeline(const IngestionPipeline&)            = delete;
    IngestionPipeline& operator=(const IngestionPipeline&) = delete;
    IngestionPipeline(IngestionPipeline&&)                 = delete;
    IngestionPipeline& operator=(IngestionPipeline&&)      = delete;

    ~IngestionPipeline() noexcept = default;

    // -------------------------------------------------------------------------
    // register_stage — append a stage to the pipeline in architecture order
    //
    // Stages are executed in registration order. The caller MUST register them
    // in the order S0, S1, S2a, S2b, S3, S4, S5, S6, S7.
    //
    // stage must not be null and must outlive this pipeline instance.
    // Returns false if the stage table is full (kMaxPipelineStages reached).
    // -------------------------------------------------------------------------
    [[nodiscard]] bool register_stage(IStage* stage) noexcept;

    // -------------------------------------------------------------------------
    // register_channel — add a channel to the processing table
    //
    // Must be called before any samples for this channel_id are submitted.
    // Calling register_channel for an already-registered channel_id is an error
    // and returns false without modifying the existing state.
    //
    // Parameters:
    //   channel_id          — Unique channel identifier.
    //   calibration_version — Initial calibration config version.
    //   nominal_delta_t_us  — Expected inter-sample interval [μs]. Must be > 0.
    //   roc_window_size     — ROC statistics window size. 0 → kRocWindowMax.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool register_channel(uint32_t channel_id,
                                        uint32_t calibration_version,
                                        uint64_t nominal_delta_t_us,
                                        uint16_t roc_window_size) noexcept;

    // -------------------------------------------------------------------------
    // process — hot-path: drive one raw sample through the full pipeline
    //
    // Execution sequence:
    //   1. Look up ChannelState by sample.channel_id.
    //   2. Construct initial envelope via make_nominal_envelope(); populate
    //      arrival_time_us and raw_value from sample.
    //   3. Run each registered stage in order. Stop on ABORT_DROP or ABORT_FAULT.
    //   4. Call run_output_bus() — validate_status_flags(), then output_bus_fn_.
    //   5. Return PipelineResult.
    //
    // This function must not be called concurrently with itself or with
    // process_watchdog() on the same pipeline instance.
    // -------------------------------------------------------------------------
    [[nodiscard]] PipelineResult process(const RawSample& sample) noexcept;

    // -------------------------------------------------------------------------
    // process_watchdog — inject a synthetic STALE|MISSING envelope
    //
    // Called when the watchdog timer expires for a channel (no sample received
    // within 2 × nominal_delta_t_us). Builds a synthetic envelope with:
    //   - status = STALE | MISSING
    //   - delta_t_us = 2 × nominal_delta_t_us
    //   - sequence_id from channel's counter (preserves monotonicity)
    //   - All value fields = NaN (correct: no measurement data)
    //
    // Stages S1–S6 are bypassed; S7 gate is still applied.
    //
    // Note: The S7 gate will reject this envelope because raw_value is NaN.
    // In production the watchdog path writes to a dedicated side-channel (see
    // the architecture note in signalfix_runtime.cpp). The gate rejection is
    // intentional and correct behaviour, not a fault.
    // -------------------------------------------------------------------------
    [[nodiscard]] PipelineResult process_watchdog(uint32_t channel_id,
                                                  uint64_t current_time_us) noexcept;

    // -------------------------------------------------------------------------
    // reset_channel — reinitialise per-channel state
    //
    // Resets the ChannelState for channel_id to clean defaults, preserving the
    // registration parameters (channel_id, calibration_version, nominal_delta_t_us,
    // roc_window_size). Useful after sensor re-calibration or detected channel fault.
    //
    // Does not affect stage-internal state. If a stage holds per-channel
    // data (e.g., a channel-keyed filter window), reset() must be called on
    // that stage separately.
    //
    // Returns false if channel_id is not registered.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool reset_channel(uint32_t channel_id) noexcept;

    // -------------------------------------------------------------------------
    // Accessors — diagnostic / test use only
    // -------------------------------------------------------------------------

    /// Number of stages currently registered.
    [[nodiscard]] uint8_t  stage_count()   const noexcept { return stage_count_;   }

    /// Number of channels currently registered.
    [[nodiscard]] uint32_t channel_count() const noexcept { return channel_count_; }

    /// Returns a const pointer to the channel state for diagnostics.
    /// Returns nullptr if channel_id is not registered.
    [[nodiscard]] const ChannelState* get_channel_state(uint32_t channel_id) const noexcept;

private:
    // -------------------------------------------------------------------------
    // find_channel_index — internal O(N) lookup by channel_id
    //
    // Returns the index into channel_states_/channel_ids_ arrays, or
    // kChannelNotFound if the channel_id is not registered.
    //
    // N = kMaxChannels (64). At 1 kHz this is ~64 integer comparisons per
    // sample — trivially cheap. The channel_ids_ array is compact in memory
    // and will typically reside in L1 cache.
    //
    // If channel counts grow significantly (>256), replace with an intrusive
    // hash table or sorted array with binary search.
    // -------------------------------------------------------------------------
    [[nodiscard]] uint32_t find_channel_index(uint32_t channel_id) const noexcept;

    static constexpr uint32_t kChannelNotFound = 0xFFFF'FFFFu;

    // -------------------------------------------------------------------------
    // make_initial_envelope — build the baseline envelope from a raw sample
    //
    // Starts from make_nominal_envelope() (all values NaN, status NOMINAL) and
    // stamps in the two fields that are available before S0 runs:
    //   - arrival_time_us from the sample
    //   - raw_value from the sample
    //
    // All other fields (channel_id, sequence_id, calibrated_value, etc.) are
    // populated by their responsible stages.
    // -------------------------------------------------------------------------
    [[nodiscard]] static MeasurementEnvelope make_initial_envelope(
        const RawSample& sample) noexcept;

    // -------------------------------------------------------------------------
    // run_output_bus — S7 gate: invariant check + output bus write
    //
    // Called after all pipeline stages complete with CONTINUE.
    // Also called on the watchdog path after synthetic envelope construction.
    //
    // Steps:
    //   1. validate_status_flags(envelope.status) — reject invalid combinations.
    //   2. Call output_bus_fn_(envelope).
    //   3. Return the appropriate PipelineStatus.
    //
    // This function is the unconditional last step in every pipeline invocation
    // that reaches the output bus. It cannot be bypassed by stage registration.
    // -------------------------------------------------------------------------
    [[nodiscard]] PipelineStatus run_output_bus(
        const MeasurementEnvelope& envelope) const noexcept;

    // ── Data members ──────────────────────────────────────────────────────────

    // Stage registry — non-owning pointers, registered in pipeline order.
    // Null entries indicate unfilled slots (valid for 0..stage_count_-1 only).
    std::array<IStage*, kMaxPipelineStages> stages_{};
    uint8_t stage_count_{0u};

    // Channel state table — one ChannelState per registered channel.
    // Parallel arrays: channel_ids_[i] is the key for channel_states_[i].
    // kChannelIdEmpty marks unused slots.
    std::array<ChannelState, kMaxChannels> channel_states_{};
    std::array<uint32_t,     kMaxChannels> channel_ids_{};
    uint32_t channel_count_{0u};

    // Snapshot of the registration parameters used for reset_channel().
    // Stored separately to avoid baking them redundantly into ChannelState.
    struct ChannelConfig {
        uint64_t nominal_delta_t_us;
        uint32_t calibration_version;
        uint16_t roc_window_size;
        uint8_t  _pad[2];
    };
    std::array<ChannelConfig, kMaxChannels> channel_configs_{};

    // Output bus callback — set at construction; never null during processing.
    OutputBusFn output_bus_fn_{nullptr};
};

} // namespace signalfix
