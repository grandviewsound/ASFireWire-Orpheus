//
// SYTGenerator.hpp
// ASFWDriver
//
// Cycle-based SYT generation per IEC 61883-6
// Computes SYT from actual OHCI transmit cycle (Linux approach)
//

#pragma once

#include <cstdint>

namespace ASFW::Encoding {

/// Generates SYT timestamps from actual OHCI transmit cycles
///
/// Linux-style architecture (amdtp-stream.c):
/// 1. IsochTransmitContext reads hardware timestamp from consumed descriptors
/// 2. Tracks the expected transmit cycle for each new packet
/// 3. Passes transmit cycle and the packet's data-block count to computeDataSYT()
/// 4. SYT = (presentation_cycle & 0xF) << 12 | tick_offset
///
/// No HardwareInterface* dependency — cycle tracking is in IsochTransmitContext.
class SYTGenerator {
public:
    explicit SYTGenerator() noexcept = default;

    /// Initialize timing for given sample rate and frames per DATA packet.
    /// @param sampleRate Sample rate in Hz (e.g., 48000.0)
    /// @param framesPerPacket Data blocks per DATA packet (8 for blocking, 6 for non-blocking at 48kHz)
    void initialize(double sampleRate, uint32_t framesPerPacket = 6) noexcept;

    /// Reset running state (call on stream start)
    void reset() noexcept;

    /// Compute SYT for a DATA packet at the given OHCI transmit cycle
    /// @param transmitCycle 13-bit OHCI cycle count (0-7999)
    /// @param samplesInPacket Data blocks (events) carried in this DATA packet
    /// @return 16-bit SYT value
    [[nodiscard]] uint16_t computeDataSYT(uint32_t transmitCycle, uint32_t samplesInPacket) noexcept;

    /// Apply a small signed offset correction in 16-cycle tick domain.
    void nudgeOffsetTicks(int32_t deltaTicks) noexcept;

    /// SYT value meaning "no timestamp information"
    static constexpr uint16_t kNoInfo = 0xFFFF;

    /// Check if generator is initialized
    [[nodiscard]] bool isValid() const noexcept { return initialized_; }

    /// Get DATA packet counter for diagnostics
    [[nodiscard]] uint64_t dataPacketCount() const noexcept { return dataPacketCount_; }

private:
    // =========================================================================
    // Timing constants (from Linux amdtp-stream.c / OHCI spec)
    // =========================================================================

    /// 24.576 MHz ticks per 125 us bus cycle
    static constexpr uint32_t kTicksPerCycle = 3072;

    /// Transfer delay (presentation offset ahead of transmit).
    /// Was 0x2E00 (Linux TRANSFER_DELAY) → sub-cycle offset 0xA00. The Apple
    /// golden sniffer capture (research/captures/.../passive_sniffer_run.txt)
    /// shows Apple's SYT sub-cycle offset based at ~0x4CF, so 3 whole cycles +
    /// 0x4CF = 0x28CF matches Apple's base. The per-packet advance is EXACTLY
    /// nominal (cycle-locked) by design: Apple's host-rate SYT creep only works
    /// with their adaptive NuDCL cadence; on our rigid DMA-ring cadence any
    /// creep ramps SYT-vs-cycle phase unboundedly and wraps the 16-cycle SYT
    /// window (~2 ms presentation snap every ~26 s = periodic ticks, HW log
    /// 2026-06-11_22-26-05). See computeDataSYT.
    static constexpr uint32_t kTransferDelayTicks = 0x28CF;  // 3 cycles + 0x4CF (Apple golden base)

    /// Ticks per audio sample at 48 kHz: 24576000 / 48000 = 512
    static constexpr uint32_t kTicksPerSample48k = 512;

    // =========================================================================
    // Per-rate computed values (set in initialize())
    // =========================================================================

    /// Ticks per sample at the active sample rate. For 48 kHz: 512.
    uint32_t ticksPerSample_{kTicksPerSample48k};

    /// Wrap point for sytOffsetTicks_: 16 * kTicksPerCycle = 49152
    /// Matches the 4-bit cycle field in SYT format
    uint32_t sytOffsetWrap_{16 * kTicksPerCycle};  // 49152

    // =========================================================================
    // Running state
    // =========================================================================

    /// Sample-position offset accumulator (advances by samplesInPacket * ticksPerSample_ per DATA packet)
    uint32_t sytOffsetTicks_{0};

    /// Fixed presentation-cycle base, latched from the transmit cycle on the
    /// FIRST DATA packet after reset(). The presentation timestamp is a monotonic
    /// accumulator on top of this base; it must NOT re-anchor to the live transmit
    /// cycle each packet (that double-counts the bus-cycle progression → 2× SYT
    /// rate). Golden Orpheus SYT advances a constant 4096 ticks/DATA packet across
    /// the NO-DATA cadence gaps (golden_wire_2026-06-05), confirming a fixed base.
    uint32_t baseCycle_{0};
    bool baseCycleValid_{false};

    /// Diagnostic counter
    uint64_t dataPacketCount_{0};

    /// Whether initialize() has been called
    bool initialized_{false};
};

} // namespace ASFW::Encoding
