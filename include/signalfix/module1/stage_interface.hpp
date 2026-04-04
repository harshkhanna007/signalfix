// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : include/signalfix/module1/stage_interface.hpp
// Spec   : SFX-M1-TDS-001  Revision 2.1
// =============================================================================
//
// Abstract stage interface.
//
// Every pipeline stage (S0–S7) must derive from IStage and implement:
//   - process()    — hot-path sample transformation
//   - stage_name() — diagnostic identifier (string literal)
//   - reset()      — deterministic state reset
//
// Design constraints:
//   - Compiled with -fno-exceptions -fno-rtti.
//   - Errors propagate via StageResult; no throwing allowed.
//   - Stages are stateless by default; stateful stages document their state
//     requirements explicitly and consume ChannelState for cross-sample data.
//   - The IStage vtable overhead (one indirect call per stage per sample) is
//     acceptable at ≤1 kHz pipeline rates. At MHz rates, consider replacing
//     with std::function or a direct dispatch table.
//
// =============================================================================

#pragma once

#include "signalfix/module1/types.hpp"

namespace signalfix {

// Forward declaration — full definition in channel_state.hpp.
// Including channel_state.hpp here creates a circular dependency risk;
// the forward declaration is sufficient for the interface contract.
struct ChannelState;

// ---------------------------------------------------------------------------
// StageResult — control flow token returned by every stage
//
// The pipeline orchestrator inspects this after each stage invocation.
// Stages do NOT call each other; only the orchestrator drives sequencing.
//
// CONTINUE:     Stage completed normally. Envelope may have been mutated.
//               Orchestrator advances to the next stage.
//
// ABORT_DROP:   Stage determined the envelope should not proceed further.
//               This is a policy decision, not a fault (e.g., a duplicate
//               sequence ID, a channel marked disabled, a gap engine deciding
//               to discard rather than fill). The orchestrator records the
//               drop and does NOT call the output bus.
//
// ABORT_FAULT:  Stage detected an internal logic fault (e.g., null pointer
//               dereferenced in stage state, unexpected NaN in a field that
//               a prior stage was contractually required to populate).
//               The orchestrator records the fault stage index, sets status
//               to HARD_INVALID if not already set, and does NOT call the
//               output bus.
// ---------------------------------------------------------------------------
enum class StageResult : uint8_t
{
    CONTINUE    = 0u,
    ABORT_DROP  = 1u,
    ABORT_FAULT = 2u,
};


// ---------------------------------------------------------------------------
// IStage — abstract pipeline stage interface
//
// Lifetime contract:
//   The pipeline holds a non-owning raw pointer to each stage.
//   Stage objects MUST outlive the IngestionPipeline instance they are
//   registered with. The pipeline never deletes stage pointers.
//
// Thread safety:
//   The pipeline is single-producer per instance. Each IStage instance
//   must NOT be shared between pipeline instances unless the implementation
//   is explicitly thread-safe (documented per-stage).
// ---------------------------------------------------------------------------
class IStage
{
public:
    // Virtual destructor required even under -fno-rtti for correct destruction
    // through base pointers. The linker will emit the vtable regardless.
    virtual ~IStage() noexcept = default;

    // -------------------------------------------------------------------------
    // process() — hot-path stage execution
    //
    // Called once per sample, in pipeline order, on the sample's owning thread.
    //
    // Preconditions (enforced by orchestrator):
    //   - envelope has been initialised via make_nominal_envelope() before S0.
    //   - channel_state corresponds to envelope.channel_id (after S0 populates it).
    //
    // Contract:
    //   - Stage MAY mutate any field of envelope.
    //   - Stage MUST NOT clear status flags set by prior stages, except where
    //     the architecture specification explicitly permits it (e.g., S5 clearing
    //     FILTER_CLIPPED on the bypass path).
    //   - Stage MUST set the status flag(s) that correspond to any anomaly it
    //     detects. It must not silently discard anomaly information.
    //   - Stage MUST NOT allocate heap memory in this function.
    //   - Stage MUST NOT throw.
    //   - Stage MUST be deterministic given identical inputs.
    //
    // Returns StageResult per the definitions above.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual StageResult process(
        MeasurementEnvelope& envelope,
        ChannelState&        channel_state) noexcept = 0;

    // -------------------------------------------------------------------------
    // stage_name() — diagnostic label
    //
    // Returns a NUL-terminated string literal (static storage duration).
    // Used in fault reports and pipeline diagnostics. Must never return null.
    // Example: "S0-InputAdapter", "S3-PlausibilityValidator"
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual const char* stage_name() const noexcept = 0;

    // -------------------------------------------------------------------------
    // reset() — deterministic stage-local state reset
    //
    // Called by the pipeline when a channel is re-initialised or the pipeline
    // is restarted. Stages that carry internal state (e.g., a median filter
    // window for S5) must clear it here to prevent stale data from a previous
    // run contaminating a fresh start.
    //
    // Stateless stages implement this as a no-op.
    // -------------------------------------------------------------------------
    virtual void reset() noexcept = 0;

    // -------------------------------------------------------------------------
    // expected_registration_index() — declared pipeline slot (diagnostic)
    //
    // Returns the pipeline slot this stage expects to be registered at.
    // Used by the pipeline for debug-level ordering validation (FIX-5).
    // Default returns kNoExpectedIndex (no preference / no validation).
    // Override in concrete stages to enable order checking.
    // -------------------------------------------------------------------------
    static constexpr uint8_t kNoExpectedIndex = 0xFFu;

    [[nodiscard]] virtual uint8_t expected_registration_index() const noexcept
    { return kNoExpectedIndex; }

protected:
    // Default constructor is protected: IStage cannot be instantiated directly.
    IStage() noexcept = default;

    // Prevent slicing and accidental copies.
    IStage(const IStage&)            = delete;
    IStage& operator=(const IStage&) = delete;
    IStage(IStage&&)                 = delete;
    IStage& operator=(IStage&&)      = delete;
};

} // namespace signalfix
