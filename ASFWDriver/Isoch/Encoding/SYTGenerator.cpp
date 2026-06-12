//
// SYTGenerator.cpp
// ASFWDriver
//
// Cycle-based SYT generation (Linux-style, from amdtp-stream.c)
//

#include "SYTGenerator.hpp"
#include "../../Logging/Logging.hpp"

namespace ASFW::Encoding {

void SYTGenerator::initialize(double sampleRate, uint32_t framesPerPacket) noexcept {
    // SYT advance per DATA packet = framesPerPacket * ticksPerSample.
    // Blocking mode:     8 frames/packet → 8 * 512 = 4096 ticks
    // Non-blocking mode: 6 frames/packet → 6 * 512 = 3072 ticks
    if (sampleRate == 48000.0) {
        ticksPerSample_ = kTicksPerSample48k;
    } else {
        ASFW_LOG(Isoch, "SYTGenerator: Unsupported rate %.0f Hz, using 48kHz params", sampleRate);
        ticksPerSample_ = kTicksPerSample48k;
    }

    sytOffsetWrap_ = 16 * kTicksPerCycle;  // 49152

    reset();
    initialized_ = true;

    const uint32_t defaultIntervalTicks = 8u * ticksPerSample_;
    ASFW_LOG(Isoch, "SYTGenerator: Initialized cycle-based mode for %.0f Hz, "
             "ticksPerSample=%u defaultIntervalTicks(8)=%u wrapTicks=%u transferDelay=0x%x",
             sampleRate, ticksPerSample_, defaultIntervalTicks, sytOffsetWrap_, kTransferDelayTicks);
}

void SYTGenerator::reset() noexcept {
    sytOffsetTicks_ = 0;
    dataPacketCount_ = 0;
    baseCycle_ = 0;
    baseCycleValid_ = false;
    ASFW_LOG(Isoch, "SYTGenerator: Reset (cycle-based mode)");
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
uint16_t SYTGenerator::computeDataSYT(uint32_t transmitCycle, uint32_t samplesInPacket) noexcept {
    if (!initialized_) return kNoInfo;
    if (samplesInPacket == 0 || ticksPerSample_ == 0) return kNoInfo;

    // Latch the transmit cycle ONCE as a fixed base. The presentation timestamp
    // is a monotonic accumulator (sytOffsetTicks_ advancing one SYT interval per
    // DATA packet) on top of this base — it must NOT re-add the live transmit
    // cycle every call. Golden Orpheus SYT advances a constant 4096 ticks/DATA
    // packet (verified on the wire, golden_wire_2026-06-05) INCLUDING across the
    // NO-DATA cadence gaps, where the transmit cycle steps by 2. Anchoring to the
    // live transmit cycle double-counts the bus-cycle progression (≈8192 ticks/
    // packet, 2× too fast) and is what the device hears as garbled timing.
    if (!baseCycleValid_) {
        baseCycle_ = transmitCycle;
        baseCycleValid_ = true;
    }

    // Total presentation offset = sample position offset + transfer delay
    uint32_t totalTicks = sytOffsetTicks_ + kTransferDelayTicks;

    // Split into whole cycles and remaining ticks
    uint32_t extraCycles = totalTicks / kTicksPerCycle;
    uint32_t remainingTicks = totalTicks % kTicksPerCycle;

    // Presentation cycle = fixed base cycle + extra cycles from the accumulator
    uint32_t presentationCycle = baseCycle_ + extraCycles;

    // Encode SYT: 4-bit cycle | 12-bit tick offset
    uint16_t syt = static_cast<uint16_t>(
        ((presentationCycle & 0xF) << 12) | (remainingTicks & 0xFFF));

    // Advance offset for next DATA packet: EXACTLY one SYT interval, cycle-locked.
    //
    // No measured-rate drift here. The IT DMA ring has a RIGID N-D-D-D cadence
    // (a supply shortfall is silence-filled, never an extra NO-DATA packet), so
    // the data-packet cycle grid advances exactly 4096 ticks per DATA packet.
    // Any per-packet excess ε accumulates SYT-vs-cycle phase without bound and
    // wraps the 16-cycle SYT ambiguity window (49152 ticks) every 49152/(ε·6000)
    // seconds — at the ~70 ppm host-vs-bus offset that is a ~2 ms presentation
    // snap every ~26 s, heard as periodic ticks (HW log 2026-06-11_22-26-05).
    // Apple's golden SYT does creep at the host rate, but Apple's NuDCL talker
    // bounds the phase by adapting cadence (slipping an extra empty packet);
    // we instead slave the HAL sample clock to the bus (zts hw-pll), making the
    // sample axis identical to the cycle axis — the correct advance is nominal.
    const uint32_t intervalTicks = samplesInPacket * ticksPerSample_;

    sytOffsetTicks_ += intervalTicks;
    if (sytOffsetTicks_ >= sytOffsetWrap_) {
        sytOffsetTicks_ -= sytOffsetWrap_;
    }

    dataPacketCount_++;

    return syt;
}

void SYTGenerator::nudgeOffsetTicks(int32_t deltaTicks) noexcept {
    if (!initialized_ || deltaTicks == 0 || sytOffsetWrap_ == 0) {
        return;
    }

    int64_t adjusted = static_cast<int64_t>(sytOffsetTicks_) + static_cast<int64_t>(deltaTicks);
    const int64_t wrap = static_cast<int64_t>(sytOffsetWrap_);
    adjusted %= wrap;
    if (adjusted < 0) {
        adjusted += wrap;
    }

    sytOffsetTicks_ = static_cast<uint32_t>(adjusted);
}

} // namespace ASFW::Encoding
