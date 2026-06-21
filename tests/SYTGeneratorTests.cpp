#include <gtest/gtest.h>

#include <cstdlib>

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

TEST(SYTGenerator, FortyEightKFamilyAdvancesOneSytIntervalPerPacket) {
    // U6 — at the 48k family the SYT advance per DATA packet is exactly
    // samplesPerPacket * (24576000/rate), which is constant 4096 ticks because
    // samplesPerPacket and ticksPerSample scale inversely:
    //   48k:  8 * 512 = 4096   96k: 16 * 256 = 4096   192k: 32 * 128 = 4096
    // Each is an exact divisor of the clock, so there is never a remainder.
    using ASFW::Encoding::SYTGenerator;
    struct Case { double rate; uint32_t spp; };
    for (const Case c : {Case{48000.0, 8}, Case{96000.0, 16}, Case{192000.0, 32}}) {
        SYTGenerator gen;
        gen.initialize(c.rate, c.spp);
        gen.reset();
        int32_t prev = TickIndex(gen.computeDataSYT(0, c.spp));
        for (int pkt = 0; pkt < 64; ++pkt) {
            const int32_t cur = TickIndex(gen.computeDataSYT(0, c.spp));
            EXPECT_EQ(WrapSigned(cur - prev), 4096)
                << "rate=" << c.rate << " spp=" << c.spp;
            prev = cur;
        }
    }
}

TEST(SYTGenerator, FractionalRateAdvanceAveragesExactTicksNoDrift) {
    // U6 — at 44.1 kHz blocking (8 samples/packet) the true advance is
    // 8 * 24576000 / 44100 = 4458.2313… ticks/packet — non-integer. The
    // remainder-carrying accumulator must make the SUMMED advance over many
    // packets match the exact rational total to within one tick (no unbounded
    // drift). Sum over N packets must equal floor/round of N * 196608000/44100.
    using ASFW::Encoding::SYTGenerator;
    SYTGenerator gen;
    gen.initialize(44100.0, 8);
    gen.reset();

    constexpr uint64_t kNum = 8ULL * 24576000ULL;  // ticks numerator per packet
    constexpr uint32_t kRate = 44100;
    constexpr int kPackets = 44100;  // ~5.5 s of DATA packets

    int32_t prev = TickIndex(gen.computeDataSYT(0, 8));
    int64_t summed = 0;
    for (int n = 1; n <= kPackets; ++n) {
        const int32_t cur = TickIndex(gen.computeDataSYT(0, 8));
        summed += WrapSigned(cur - prev);
        prev = cur;

        const int64_t exactTotal =
            static_cast<int64_t>((kNum * static_cast<uint64_t>(n)) / kRate);
        // Accumulated SYT advance tracks the exact rational total within 1 tick.
        ASSERT_LE(std::abs(summed - exactTotal), 1)
            << "fractional SYT drifted at packet " << n;
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
