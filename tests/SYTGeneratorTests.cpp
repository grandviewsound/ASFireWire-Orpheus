#include <gtest/gtest.h>

#include "../ASFWDriver/Isoch/Encoding/SYTGenerator.hpp"

namespace {
constexpr int32_t kTickDomain = 16 * 3072;

int32_t TickIndex(uint16_t syt) {
    constexpr int32_t kTicksPerCycle = 3072;
    return (static_cast<int32_t>((syt >> 12) & 0x0F) * kTicksPerCycle) +
           static_cast<int32_t>(syt & 0x0FFF);
}

int32_t WrapSigned(int32_t ticks) {
    constexpr int32_t half = kTickDomain / 2;
    int32_t wrapped = ticks % kTickDomain;
    if (wrapped >= half) {
        wrapped -= kTickDomain;
    } else if (wrapped < -half) {
        wrapped += kTickDomain;
    }
    return wrapped;
}
} // namespace

TEST(SYTGenerator, NudgePositiveAndNegativeTicks) {
    ASFW::Encoding::SYTGenerator gen;
    gen.initialize(48000.0);

    gen.reset();
    const int32_t base = TickIndex(gen.computeDataSYT(0, 8));

    gen.reset();
    gen.nudgeOffsetTicks(+1);
    const int32_t plusOne = TickIndex(gen.computeDataSYT(0, 8));
    EXPECT_EQ(WrapSigned(plusOne - base), +1);

    gen.reset();
    gen.nudgeOffsetTicks(-1);
    const int32_t minusOne = TickIndex(gen.computeDataSYT(0, 8));
    EXPECT_EQ(WrapSigned(minusOne - base), -1);
}

TEST(SYTGenerator, CycleLockedNoPhaseCreepOverLongRun) {
    // Regression guard for the jun11 periodic-tick bug: a measured-rate drift
    // correction in the SYT advance ramps SYT-vs-cycle phase without bound on
    // our rigid N-D-D-D cadence (+~0.3 ticks/packet at ~70 ppm), wrapping the
    // 16-cycle SYT window every ~26 s = a ~2 ms presentation snap heard as a
    // tick (HW log 2026-06-11_22-26-05). The advance must be EXACTLY one SYT
    // interval (8 * 512 = 4096 ticks) per DATA packet, forever: after every
    // 12 DATA packets (12 * 4096 = 49152 = one full wrap) the tick index must
    // return to its base bit-for-bit. 144,000 packets ≈ 24 s of wire time.
    using ASFW::Encoding::SYTGenerator;
    SYTGenerator gen;
    gen.initialize(48000.0);
    gen.reset();

    const int32_t base = TickIndex(gen.computeDataSYT(0, 8));
    constexpr int kWraps = 12000;
    for (int wrap = 0; wrap < kWraps; ++wrap) {
        for (int pkt = 0; pkt < 11; ++pkt) {
            (void)gen.computeDataSYT(0, 8);
        }
        const int32_t idx = TickIndex(gen.computeDataSYT(0, 8));
        ASSERT_EQ(idx, base) << "SYT-vs-cycle phase crept after "
                             << ((wrap + 1) * 12) << " DATA packets";
    }
}

TEST(SYTGenerator, AdvancesConstant4096PerPacketRegardlessOfTransmitCycle) {
    // Golden Orpheus SYT advances a constant 4096 ticks/DATA packet — including
    // across the N-D-D-D NO-DATA cadence gaps, where the transmit cycle steps by
    // 2 instead of 1 (golden_wire_2026-06-05). The presentation timestamp must be
    // a monotonic accumulator from a FIXED base, NOT re-anchored to the live
    // transmit cycle each call (that double-counts to ~8192 ticks/packet, 2×).
    using ASFW::Encoding::SYTGenerator;
    SYTGenerator gen;
    gen.initialize(48000.0);
    gen.reset();

    // Simulate the real cadence: transmit cycle steps 1,1,2,1,1,2,... (the gap of
    // 2 spans the skipped NO-DATA cycle). DATA packets only — NO-DATA carries
    // 0xFFFF and does not call computeDataSYT.
    constexpr int kSteps[] = {1, 1, 2, 1, 1, 2, 1, 1, 2, 1, 1, 2};
    uint32_t cycle = 5;  // arbitrary starting transmit cycle

    int32_t prev = TickIndex(gen.computeDataSYT(cycle, 8));
    for (int step : kSteps) {
        cycle = (cycle + static_cast<uint32_t>(step)) % 8000;
        const int32_t cur = TickIndex(gen.computeDataSYT(cycle, 8));
        EXPECT_EQ(WrapSigned(cur - prev), 4096)
            << "SYT must advance one SYT interval (4096 ticks) per DATA packet "
               "regardless of the transmit-cycle step";
        prev = cur;
    }
}

TEST(SYTGenerator, NudgeWrapBehaviorAcrossDomain) {
    ASFW::Encoding::SYTGenerator gen;
    gen.initialize(48000.0);

    gen.reset();
    const int32_t base = TickIndex(gen.computeDataSYT(0, 8));

    gen.reset();
    gen.nudgeOffsetTicks(kTickDomain + 3);
    const int32_t plusWrapped = TickIndex(gen.computeDataSYT(0, 8));
    EXPECT_EQ(WrapSigned(plusWrapped - base), +3);

    gen.reset();
    gen.nudgeOffsetTicks(-(kTickDomain + 5));
    const int32_t minusWrapped = TickIndex(gen.computeDataSYT(0, 8));
    EXPECT_EQ(WrapSigned(minusWrapped - base), -5);
}
