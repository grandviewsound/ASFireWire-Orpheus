// BlockingCadence48k.hpp
// ASFW - Phase 1.5 Encoding Layer
//
// Rate-parametrized blocking cadence per IEC 61883-6 (U6).
// (Legacy file/class name keeps the `48k` suffix; the generator is now general —
//  renaming is a tracked cosmetic follow-up.)
//
// Reference: docs/Isoch/PHASE_1_5_ENCODING.md
// Verified against: 000-48kORIG.txt FireBug capture (48 kHz pattern unchanged)
//

#pragma once

#include <cstdint>

namespace ASFW {
namespace Encoding {

/// Samples per DATA packet at 48 kHz (SYT interval). 48k-family default.
constexpr uint32_t kSamplesPerPacket48k = 8;

/// DATA packets per 8 cycles at 48 kHz
constexpr uint32_t kDataPacketsPer8Cycles = 6;

/// NO-DATA packets per 8 cycles at 48 kHz
constexpr uint32_t kNoDataPacketsPer8Cycles = 2;

/// Bus isochronous cycle rate (8 kHz).
constexpr uint32_t kIsochCyclesPerSecond = 8000;

/// Largest blocking SYT interval across all supported rates (192 kHz family).
constexpr uint32_t kMaxSamplesPerPacket = 32;

/// Blocking-mode SYT interval (frames per DATA packet) for a sample rate, per
/// Apple's SetupSendBufferForMacSync: 8 (≤48k) / 16 (88.2–96k) / 32 (176.4–192k).
/// Doubles per rate family so frames/cycle and frames/packet scale together,
/// keeping the cadence ratio (and on-wire packet shape) consistent across rates.
constexpr uint32_t blockingSamplesPerPacketForRate(uint32_t sampleRateHz) noexcept {
    if (sampleRateHz <= 48000) {
        return 8;
    }
    if (sampleRateHz <= 96000) {
        return 16;
    }
    return 32;
}

/// Manages the blocking cadence (DATA vs NO-DATA per bus cycle) for any rate.
///
/// A blocking DATA packet always carries a fixed SYT interval of frames
/// (`samplesPerPacket` = 8/16/32 for the 48k/96k/192k families); the cadence
/// modulates how many cycles carry DATA vs NO-DATA so the long-term frame rate
/// equals the sample rate. It uses a fixed-point Bresenham accumulator:
///
///   each cycle:  acc += sampleRateHz
///                if acc >= samplesPerPacket * 8000:  DATA, acc -= threshold
///                else:                               NO-DATA
///
/// This is exact (no drift) at integer rates AND fractional rates (e.g. 44.1k).
/// At 48 kHz / 8 spp it reproduces the original N-D-D-D pattern bit-for-bit:
///   threshold = 64000, +48000/cycle → NO-DATA at cycle%4==0, DATA otherwise
///   = 6 DATA + 2 NO-DATA per 8 cycles = 48,000 samples/sec.
/// The default-constructed state is exactly that 48 kHz behavior.
class BlockingCadence48k {
public:
    /// Construct a cadence generator defaulting to 48 kHz / 8 samples-per-packet.
    BlockingCadence48k() noexcept = default;

    /// Configure for a sample rate and SYT interval (samples per DATA packet).
    /// Resets the running state. Positional args: rate then samples/packet.
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    void configure(uint32_t sampleRateHz, uint32_t samplesPerPacket) noexcept {
        sampleRateHz_ = sampleRateHz;
        samplesPerPacket_ = samplesPerPacket;
        threshold_ = samplesPerPacket * kIsochCyclesPerSecond;
        reset();
    }

    /// Check if the current cycle should transmit a DATA packet.
    /// @return true if DATA packet, false if NO-DATA packet
    bool isDataPacket() const noexcept {
        // Peek: adding this cycle's frames reaches one full DATA packet's worth.
        return (acc_ + sampleRateHz_) >= threshold_;
    }

    /// Get the number of samples to transmit in the current cycle.
    /// @return samplesPerPacket for DATA packets, 0 for NO-DATA packets
    uint32_t samplesThisCycle() const noexcept {
        return isDataPacket() ? samplesPerPacket_ : 0;
    }

    /// Get the current cycle index within an 8-cycle window (diagnostic).
    uint32_t getCycleIndex() const noexcept {
        return static_cast<uint32_t>(cycleIndex_ % 8);
    }

    /// Get the total cycle count since reset.
    uint64_t getTotalCycles() const noexcept {
        return cycleIndex_;
    }

    /// Advance to the next cycle.
    void advance() noexcept {
        acc_ += sampleRateHz_;
        if (acc_ >= threshold_) {
            acc_ -= threshold_;
        }
        cycleIndex_++;
    }

    /// Advance by multiple cycles (steps the accumulator per cycle).
    void advanceBy(uint32_t cycles) noexcept {
        for (uint32_t i = 0; i < cycles; ++i) {
            advance();
        }
    }

    /// Reset the cadence to the starting position (cycle 0, empty accumulator).
    void reset() noexcept {
        acc_ = 0;
        cycleIndex_ = 0;
    }

private:
    uint32_t sampleRateHz_ = 48000;                                ///< Frames/sec
    uint32_t samplesPerPacket_ = kSamplesPerPacket48k;             ///< SYT interval
    uint32_t threshold_ = kSamplesPerPacket48k * kIsochCyclesPerSecond; ///< 64000
    uint64_t acc_ = 0;          ///< Bresenham accumulator (frames * 8000 units)
    uint64_t cycleIndex_ = 0;   ///< Running cycle counter
};

} // namespace Encoding
} // namespace ASFW
