// ZeroTimestampAnchorTests.cpp
// ASFW - Host-safe unit tests for the hardware-driven zero-timestamp anchor.
//
// Validates, off-hardware, the pieces of the Apple-faithful clock path that
// would otherwise only be exercisable on the device:
//   1. FwSampleClock — FireWire cycle-timer -> monotonic device sample-time.
//   2. TxSharedQueueSPSC hardware anchor — seqlock-protected (sampleTime,
//      hostTicks) publish/read across the shared-memory boundary.
//   3. ZtsAnchorPll — smooth published zero-timestamp timeline phase-slewed
//      toward the jittery raw RX-poll anchor (HW log 2026-06-10_21-22-25:
//      raw republish made HAL IO wakes run 161/s instead of 250/s).

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "../ASFWDriver/Isoch/Encoding/TimingUtils.hpp"
#include "../ASFWDriver/Shared/TxSharedQueue.hpp"

using ASFW::Timing::FwSampleClock;
using ASFW::Timing::kCyclesPerSecond;
using ASFW::Timing::kTicksPerCycle;

namespace {

// Build a 32-bit OHCI cycle-timer value from seconds/cycles/offset fields.
uint32_t MakeCycleTimer(uint32_t seconds, uint32_t cycles, uint32_t offset) {
    return ((seconds & 0x7F) << ASFW::Timing::kCycleTimerSecondsShift)
         | ((cycles & 0x1FFF) << ASFW::Timing::kCycleTimerCyclesShift)
         | (offset & 0xFFF);
}

} // namespace

TEST(FwSampleClock, FirstAdvanceEstablishesBaselineAtZero) {
    FwSampleClock clk;
    // First reading has no previous, so the clock stays at 0 frames.
    EXPECT_EQ(clk.Advance(MakeCycleTimer(5, 1234, 100), 48000.0), 0u);
}

TEST(FwSampleClock, OneSecondOfCycleTimeIsOneSampleRateOfFrames) {
    FwSampleClock clk;
    (void)clk.Advance(MakeCycleTimer(0, 0, 0), 48000.0);    // baseline
    // Accumulate exactly one FireWire second (8000 cycles) in realistic ~10ms
    // steps (80 cycles each) — a single 1s jump would (correctly) trip the
    // anomaly guard, so the device clock must be built from small poll-sized deltas.
    uint64_t frames = 0;
    for (uint32_t step = 1; step <= 100; ++step) {
        const uint32_t totalCycles = step * 80u;            // 100 * 80 = 8000 = 1s
        frames = clk.Advance(
            MakeCycleTimer(totalCycles / kCyclesPerSecond, totalCycles % kCyclesPerSecond, 0),
            48000.0);
    }
    EXPECT_EQ(frames, 48000u);
}

TEST(FwSampleClock, IsMonotonicAcrossManyAdvances) {
    FwSampleClock clk;
    (void)clk.Advance(MakeCycleTimer(0, 0, 0), 48000.0);
    uint64_t prev = 0;
    // Step ~one audio buffer (512 frames ~= 85.3 cycles) repeatedly.
    for (uint32_t i = 1; i <= 1000; ++i) {
        const uint32_t totalCycles = (i * 512u * kCyclesPerSecond) / 48000u;
        const uint32_t sec = totalCycles / kCyclesPerSecond;
        const uint32_t cyc = totalCycles % kCyclesPerSecond;
        const uint64_t frames = clk.Advance(MakeCycleTimer(sec, cyc, 0), 48000.0);
        EXPECT_GE(frames, prev);
        prev = frames;
    }
    EXPECT_GT(prev, 0u);
}

TEST(FwSampleClock, HandlesCycleTimerWrapWithoutGoingBackwards) {
    FwSampleClock clk;
    // Just before the 128s wrap (seconds field is 7 bits -> wraps at 128).
    (void)clk.Advance(MakeCycleTimer(127, 7999, 0), 48000.0);
    const uint64_t before = clk.Advance(MakeCycleTimer(127, 7999, kTicksPerCycle / 2), 48000.0);
    // Cross the wrap: seconds 127 -> 0. deltaFWTimeNanos must treat this as a
    // small forward step, not a ~128s backward jump.
    const uint64_t after = clk.Advance(MakeCycleTimer(0, 5, 0), 48000.0);
    EXPECT_GE(after, before);
    // The forward step across the wrap is tiny (~5 cycles), nowhere near 128s.
    EXPECT_LT(after - before, 48000u);
}

TEST(FwSampleClock, RejectsAbsurdForwardGap) {
    FwSampleClock clk;
    (void)clk.Advance(MakeCycleTimer(0, 0, 0), 48000.0);
    // A >100ms apparent gap (e.g. anomalous read) must be ignored so the
    // monotonic clock is not corrupted.
    const uint64_t frames = clk.Advance(MakeCycleTimer(1, 0, 0), 48000.0); // 1s gap > 100ms cap
    EXPECT_EQ(frames, 0u);
}

TEST(FwSampleClock, ResetReturnsToZero) {
    FwSampleClock clk;
    (void)clk.Advance(MakeCycleTimer(0, 0, 0), 48000.0);
    (void)clk.Advance(MakeCycleTimer(0, 400, 0), 48000.0); // 400 cyc = 50ms < 100ms cap
    EXPECT_GT(clk.deviceNanos, 0u);
    clk.Reset();
    EXPECT_EQ(clk.deviceNanos, 0u);
    EXPECT_FALSE(clk.hasPrev);
    EXPECT_EQ(clk.Advance(MakeCycleTimer(50, 0, 0), 48000.0), 0u); // new baseline
}

// --- Shared-queue hardware anchor (seqlock) -------------------------------

class HwAnchorQueueTest : public ::testing::Test {
protected:
    static constexpr uint32_t kCapacityFrames = 256;
    static constexpr uint32_t kChannels = 2;

    void SetUp() override {
        bytes_ = ASFW::Shared::TxSharedQueueSPSC::RequiredBytes(kCapacityFrames, kChannels);
        storage_.assign(static_cast<size_t>(bytes_), 0);
        ASSERT_TRUE(ASFW::Shared::TxSharedQueueSPSC::InitializeInPlace(
            storage_.data(), bytes_, kCapacityFrames, kChannels));
        ASSERT_TRUE(producer_.Attach(storage_.data(), bytes_));
        ASSERT_TRUE(consumer_.Attach(storage_.data(), bytes_));
    }

    uint64_t bytes_{0};
    std::vector<uint8_t> storage_;
    ASFW::Shared::TxSharedQueueSPSC producer_;
    ASFW::Shared::TxSharedQueueSPSC consumer_;
};

TEST_F(HwAnchorQueueTest, NoAnchorBeforeFirstPublish) {
    uint64_t s = 123, h = 456;
    EXPECT_FALSE(consumer_.ReadHwZeroTimestampAnchor(s, h));
}

TEST_F(HwAnchorQueueTest, PublishThenReadRoundTrips) {
    producer_.PublishHwZeroTimestampAnchor(/*sampleTime=*/100000, /*hostTicks=*/0xDEADBEEF12ULL);
    uint64_t s = 0, h = 0;
    ASSERT_TRUE(consumer_.ReadHwZeroTimestampAnchor(s, h));
    EXPECT_EQ(s, 100000u);
    EXPECT_EQ(h, 0xDEADBEEF12ULL);
}

TEST_F(HwAnchorQueueTest, LatestPublishWins) {
    producer_.PublishHwZeroTimestampAnchor(1, 1000);
    producer_.PublishHwZeroTimestampAnchor(2, 2000);
    producer_.PublishHwZeroTimestampAnchor(3, 3000);
    uint64_t s = 0, h = 0;
    ASSERT_TRUE(consumer_.ReadHwZeroTimestampAnchor(s, h));
    EXPECT_EQ(s, 3u);
    EXPECT_EQ(h, 3000u);
}

TEST_F(HwAnchorQueueTest, ZeroHostTicksReportsNoAnchor) {
    // hostTicks == 0 is the sentinel for "not yet valid".
    producer_.PublishHwZeroTimestampAnchor(/*sampleTime=*/999, /*hostTicks=*/0);
    uint64_t s = 0, h = 0;
    EXPECT_FALSE(consumer_.ReadHwZeroTimestampAnchor(s, h));
}

// --- Zero-timestamp anchor PLL ---------------------------------------------

namespace {

using ASFW::Timing::ZtsAnchorPll;

// 512-frame zts buffer at 48 kHz against a 24 MHz host timebase.
constexpr uint32_t kPeriodFrames = 512;
constexpr double kTicksPerBuffer = 256000.0;  // 24e6 * 512 / 48000
constexpr double kTicksPerFrame = kTicksPerBuffer / kPeriodFrames;

// Deterministic pseudo-noise in [-amplitude, +amplitude].
int64_t PairNoise(uint32_t k, int64_t amplitude) {
    const uint32_t h = (k * 2654435761u) ^ (k << 7);
    return static_cast<int64_t>(h % (2 * static_cast<uint64_t>(amplitude) + 1)) - amplitude;
}

} // namespace

TEST(ZtsAnchorPll, FallsBackUntilFirstAnchorThenSeedsRaw) {
    ZtsAnchorPll pll;
    EXPECT_FALSE(pll.Tick(kPeriodFrames, kTicksPerBuffer, false, 0, 0));
    EXPECT_FALSE(pll.valid);
    ASSERT_TRUE(pll.Tick(kPeriodFrames, kTicksPerBuffer, true, 96000, 5'000'000));
    EXPECT_EQ(pll.sampleTime, 96000u);
    EXPECT_EQ(pll.hostTicks, 5'000'000u);
}

TEST(ZtsAnchorPll, CoastsOnExactGridWhenAnchorFrozen) {
    ZtsAnchorPll pll;
    ASSERT_TRUE(pll.Tick(kPeriodFrames, kTicksPerBuffer, true, 0, 1'000'000));
    // Same anchor pair every tick (RX stopped publishing): pure accumulator.
    for (uint32_t k = 1; k <= 100; ++k) {
        ASSERT_TRUE(pll.Tick(kPeriodFrames, kTicksPerBuffer, true, 0, 1'000'000));
        EXPECT_EQ(pll.sampleTime, static_cast<uint64_t>(k) * kPeriodFrames);
        EXPECT_EQ(pll.hostTicks, 1'000'000u + static_cast<uint64_t>(k * kTicksPerBuffer));
    }
}

TEST(ZtsAnchorPll, SmoothsJitteryAnchorHostDeltas) {
    // True device timeline: sample k*period at host k*ticksPerBuffer. The raw
    // anchor pair carries ±65k ticks (~2.7 ms) of capture noise — the noise
    // observed in HW log 21-22-25's `lead` field. The published host deltas
    // must stay within the 1% slew bound of the nominal period (raw republish
    // would carry the full ±2.7 ms into consecutive deltas).
    ZtsAnchorPll pll;
    constexpr int64_t kNoise = 65000;
    ASSERT_TRUE(pll.Tick(kPeriodFrames, kTicksPerBuffer, true, 0, 10'000'000));
    uint64_t prevHost = pll.hostTicks;
    for (uint32_t k = 1; k <= 2000; ++k) {
        const uint64_t hwSample = static_cast<uint64_t>(k) * kPeriodFrames;
        const uint64_t hwHost = 10'000'000u
            + static_cast<uint64_t>(static_cast<int64_t>(k * kTicksPerBuffer)
                                    + PairNoise(k, kNoise));
        ASSERT_TRUE(pll.Tick(kPeriodFrames, kTicksPerBuffer, true, hwSample, hwHost));
        const int64_t delta = static_cast<int64_t>(pll.hostTicks - prevHost);
        EXPECT_GT(delta, 0) << "host must be strictly monotonic (tick " << k << ")";
        EXPECT_NEAR(static_cast<double>(delta), kTicksPerBuffer,
                    kTicksPerBuffer * ZtsAnchorPll::kMaxSlewFractionOfPeriod + 1.0)
            << "published delta exceeded the slew bound at tick " << k;
        prevHost = pll.hostTicks;
    }
    // Long-run phase must track the true line (not wander off): the published
    // host at the published sample should sit well inside the raw noise band.
    const double trueHostAtSample = 10'000'000.0
        + static_cast<double>(pll.sampleTime) * kTicksPerFrame;
    EXPECT_NEAR(static_cast<double>(pll.hostTicks), trueHostAtSample,
                static_cast<double>(kNoise));
}

TEST(ZtsAnchorPll, TracksRealRateOffsetThroughPhaseSlew) {
    // Device runs +500 ppm fast relative to the rate fed to the PLL; the
    // bounded slew (1% per tick >> 500 ppm) must absorb it so the published
    // timeline follows the device line instead of drifting away linearly.
    ZtsAnchorPll pll;
    const double deviceTicksPerBuffer = kTicksPerBuffer * (1.0 - 500e-6);
    ASSERT_TRUE(pll.Tick(kPeriodFrames, kTicksPerBuffer, true, 0, 10'000'000));
    for (uint32_t k = 1; k <= 4000; ++k) {
        const uint64_t hwSample = static_cast<uint64_t>(k) * kPeriodFrames;
        const uint64_t hwHost = 10'000'000u
            + static_cast<uint64_t>(k * deviceTicksPerBuffer);
        ASSERT_TRUE(pll.Tick(kPeriodFrames, kTicksPerBuffer, true, hwSample, hwHost));
    }
    const double trueHostAtSample = 10'000'000.0
        + static_cast<double>(pll.sampleTime) * (deviceTicksPerBuffer / kPeriodFrames);
    // Steady-state lag of a first-order loop = rate-error/gain ≈ 128/0.03125
    // = ~4096 ticks; allow a few times that.
    EXPECT_NEAR(static_cast<double>(pll.hostTicks), trueHostAtSample, 16384.0);
}

TEST(ZtsAnchorPll, ReseedsWhenAnchorRestarts) {
    ZtsAnchorPll pll;
    ASSERT_TRUE(pll.Tick(kPeriodFrames, kTicksPerBuffer, true, 0, 10'000'000));
    for (uint32_t k = 1; k <= 100; ++k) {
        const uint64_t hwSample = static_cast<uint64_t>(k) * kPeriodFrames;
        const uint64_t hwHost = 10'000'000u + static_cast<uint64_t>(k * kTicksPerBuffer);
        ASSERT_TRUE(pll.Tick(kPeriodFrames, kTicksPerBuffer, true, hwSample, hwHost));
    }
    // Bus reset: FwSampleClock restarts near 0 while host time keeps going —
    // the projected error is enormous, so the PLL must snap, not slew.
    const uint64_t restartHost = pll.hostTicks + 50'000'000;
    ASSERT_TRUE(pll.Tick(kPeriodFrames, kTicksPerBuffer, true, 8, restartHost));
    EXPECT_EQ(pll.sampleTime, 8u);
    EXPECT_EQ(pll.hostTicks, restartHost);
}

TEST(ZtsAnchorPll, ReseedsWhenGridRecedesOnSampleAxis) {
    // HW log 2026-06-10_22-22-09: timer beats arrived ~18% slower than the grid
    // period, so the published pair fell behind real time while staying exactly
    // ON the device line — the projected host error never saw it (lead reached
    // -132M ticks with zero re-seeds). Model that cadence deficit: the raw
    // anchor advances 604 frames of device time per tick while the grid only
    // credits 512. The sample-axis guard must re-seed once the gap exceeds the
    // threshold, keeping the published pair near the present.
    ZtsAnchorPll pll;
    constexpr uint64_t kDeviceFramesPerTick = 604;
    ASSERT_TRUE(pll.Tick(kPeriodFrames, kTicksPerBuffer, true, 0, 10'000'000));
    for (uint32_t k = 1; k <= 600; ++k) {
        const uint64_t hwSample = k * kDeviceFramesPerTick;
        const uint64_t hwHost = 10'000'000u
            + static_cast<uint64_t>(static_cast<double>(hwSample) * kTicksPerFrame);
        ASSERT_TRUE(pll.Tick(kPeriodFrames, kTicksPerBuffer, true, hwSample, hwHost));
        const double gap = static_cast<double>(pll.sampleTime)
            - static_cast<double>(hwSample);
        EXPECT_LE(std::abs(gap),
                  ZtsAnchorPll::kReseedThresholdPeriods * kPeriodFrames
                      + kPeriodFrames)
            << "published sample receded past the staleness threshold (tick "
            << k << ")";
    }
}

TEST(ZtsAnchorPll, ResetReturnsToFallback) {
    ZtsAnchorPll pll;
    ASSERT_TRUE(pll.Tick(kPeriodFrames, kTicksPerBuffer, true, 100, 200));
    pll.Reset();
    EXPECT_FALSE(pll.valid);
    EXPECT_FALSE(pll.Tick(kPeriodFrames, kTicksPerBuffer, false, 0, 0));
}

// --- Zero-timestamp anchor history-fit (AppleUSBAudio-faithful) -------------

namespace {

using ASFW::Timing::ZtsAnchorFit;

// ~10us floor in the tests' 24 MHz-equivalent tick scale (kTicksPerFrame = 500
// ticks/frame -> 24e6 ticks/s -> 10us = 240 ticks). Set explicitly because the
// real value is timebase-derived (the engine sets it; the struct defaults to 0).
constexpr double kFitFloorTicks = 240.0;

} // namespace

TEST(ZtsAnchorFit, FallsBackUntilFirstAnchorThenSeedsRaw) {
    ZtsAnchorFit fit;
    fit.jitterFloorTicks = kFitFloorTicks;
    EXPECT_FALSE(fit.Tick(kPeriodFrames, kTicksPerBuffer, false, 0, 0));
    ASSERT_TRUE(fit.Tick(kPeriodFrames, kTicksPerBuffer, true, 96000, 5'000'000));
    EXPECT_TRUE(fit.valid);
    EXPECT_EQ(fit.sampleTime, 96000u);
    EXPECT_EQ(fit.hostTicks, 5'000'000u);
}

TEST(ZtsAnchorFit, WarmupAdvancesOnGridThenFits) {
    ZtsAnchorFit fit;
    fit.jitterFloorTicks = kFitFloorTicks;
    ASSERT_TRUE(fit.Tick(kPeriodFrames, kTicksPerBuffer, true, 0, 1'000'000));
    // Before warmup completes the publish rides the q8 grid (no fit yet).
    for (uint32_t k = 1; k < ZtsAnchorFit::kWarmup - 1; ++k) {
        const uint64_t hwSample = static_cast<uint64_t>(k) * kPeriodFrames;
        const uint64_t hwHost = 1'000'000u + static_cast<uint64_t>(k * kTicksPerBuffer);
        ASSERT_TRUE(fit.Tick(kPeriodFrames, kTicksPerBuffer, true, hwSample, hwHost));
        EXPECT_EQ(fit.hostTicks, 1'000'000u + static_cast<uint64_t>(k * kTicksPerBuffer));
    }
    // Once enough samples land, the fit takes over and recovers the rate.
    for (uint32_t k = ZtsAnchorFit::kWarmup - 1; k <= 64; ++k) {
        const uint64_t hwSample = static_cast<uint64_t>(k) * kPeriodFrames;
        const uint64_t hwHost = 1'000'000u + static_cast<uint64_t>(k * kTicksPerBuffer);
        ASSERT_TRUE(fit.Tick(kPeriodFrames, kTicksPerBuffer, true, hwSample, hwHost));
    }
    EXPECT_NEAR(fit.slope, kTicksPerFrame, 1e-6);
}

// The whole point vs ZtsAnchorPll: publishing from the FITTED line makes the
// per-tick host delta smooth even when the raw anchor host carries large white
// jitter (raw deltas would swing by +/- 2*amplitude).
TEST(ZtsAnchorFit, SmoothsJitteryAnchorIntoConstantDeltas) {
    ZtsAnchorFit fit;
    fit.jitterFloorTicks = kFitFloorTicks;
    constexpr int64_t kJitter = 2000;  // ~83us p-p on the capture host
    ASSERT_TRUE(fit.Tick(kPeriodFrames, kTicksPerBuffer, true, 0, 10'000'000));
    uint64_t prevHost = fit.hostTicks;
    for (uint32_t k = 1; k <= 400; ++k) {
        const uint64_t hwSample = static_cast<uint64_t>(k) * kPeriodFrames;
        const uint64_t hwHost = 10'000'000u
            + static_cast<uint64_t>(static_cast<int64_t>(k * kTicksPerBuffer)
                                    + PairNoise(k, kJitter));
        ASSERT_TRUE(fit.Tick(kPeriodFrames, kTicksPerBuffer, true, hwSample, hwHost));
        if (k > ZtsAnchorFit::kCapacity) {  // fully warmed, ring saturated
            const double delta = static_cast<double>(fit.hostTicks)
                - static_cast<double>(prevHost);
            // Published deltas hug the true period to a tiny fraction of the raw
            // jitter — they lie on the slowly-moving fitted line, not the samples.
            EXPECT_NEAR(delta, kTicksPerBuffer, static_cast<double>(kJitter) / 4.0)
                << "published host delta tracked raw jitter at tick " << k;
        }
        prevHost = fit.hostTicks;
    }
}

TEST(ZtsAnchorFit, TracksRealRateOffset) {
    ZtsAnchorFit fit;
    fit.jitterFloorTicks = kFitFloorTicks;
    const double deviceTicksPerBuffer = kTicksPerBuffer * (1.0 - 500e-6);  // -500 ppm
    const double deviceTicksPerFrame = deviceTicksPerBuffer / kPeriodFrames;
    ASSERT_TRUE(fit.Tick(kPeriodFrames, kTicksPerBuffer, true, 0, 10'000'000));
    for (uint32_t k = 1; k <= 300; ++k) {
        const uint64_t hwSample = static_cast<uint64_t>(k) * kPeriodFrames;
        const uint64_t hwHost = 10'000'000u
            + static_cast<uint64_t>(static_cast<double>(k) * deviceTicksPerBuffer);
        ASSERT_TRUE(fit.Tick(kPeriodFrames, kTicksPerBuffer, true, hwSample, hwHost));
    }
    EXPECT_NEAR(fit.slope, deviceTicksPerFrame, deviceTicksPerFrame * 1e-4);
    // Published anchor sits on the real device line, not the nominal grid.
    const double ideal = 10'000'000.0
        + static_cast<double>(fit.sampleTime) * deviceTicksPerFrame;
    EXPECT_NEAR(static_cast<double>(fit.hostTicks), ideal, kTicksPerBuffer);
}

TEST(ZtsAnchorFit, RejectsLoneOutlierWithoutChasingIt) {
    ZtsAnchorFit fit;
    fit.jitterFloorTicks = kFitFloorTicks;
    ASSERT_TRUE(fit.Tick(kPeriodFrames, kTicksPerBuffer, true, 0, 10'000'000));
    for (uint32_t k = 1; k <= 200; ++k) {
        const uint64_t hwSample = static_cast<uint64_t>(k) * kPeriodFrames;
        const uint64_t hwHost = 10'000'000u + static_cast<uint64_t>(k * kTicksPerBuffer);
        ASSERT_TRUE(fit.Tick(kPeriodFrames, kTicksPerBuffer, true, hwSample, hwHost));
    }
    const int countBefore = fit.count;
    const double slopeBefore = fit.slope;
    // A single anchor with a host time ~1ms (>> kReseedK*bound) off the line.
    const uint32_t kOut = 201;
    const uint64_t outSample = static_cast<uint64_t>(kOut) * kPeriodFrames;
    const uint64_t outHost = 10'000'000u + static_cast<uint64_t>(kOut * kTicksPerBuffer)
                             + 1'000'000u;
    ASSERT_TRUE(fit.Tick(kPeriodFrames, kTicksPerBuffer, true, outSample, outHost));
    EXPECT_EQ(fit.count, countBefore) << "outlier polluted the fit history";
    EXPECT_NEAR(fit.slope, slopeBefore, slopeBefore * 1e-3) << "outlier bent the fit";
    EXPECT_TRUE(fit.valid);
}

TEST(ZtsAnchorFit, ReseedsOnSustainedDiscontinuity) {
    ZtsAnchorFit fit;
    fit.jitterFloorTicks = kFitFloorTicks;
    ASSERT_TRUE(fit.Tick(kPeriodFrames, kTicksPerBuffer, true, 0, 10'000'000));
    for (uint32_t k = 1; k <= 200; ++k) {
        const uint64_t hwSample = static_cast<uint64_t>(k) * kPeriodFrames;
        const uint64_t hwHost = 10'000'000u + static_cast<uint64_t>(k * kTicksPerBuffer);
        ASSERT_TRUE(fit.Tick(kPeriodFrames, kTicksPerBuffer, true, hwSample, hwHost));
    }
    // Device clock restarts: host jumps by a large constant offset and stays there.
    constexpr uint64_t kStep = 5'000'000u;
    bool reseeded = false;
    for (uint32_t k = 201; k <= 201 + ZtsAnchorFit::kReseedRun + 2; ++k) {
        const uint64_t hwSample = static_cast<uint64_t>(k) * kPeriodFrames;
        const uint64_t hwHost = 10'000'000u + static_cast<uint64_t>(k * kTicksPerBuffer) + kStep;
        ASSERT_TRUE(fit.Tick(kPeriodFrames, kTicksPerBuffer, true, hwSample, hwHost));
        if (fit.count == 1) reseeded = true;  // ring dropped to the lone reseed sample
    }
    EXPECT_TRUE(reseeded) << "sustained discontinuity never triggered a reseed";
    EXPECT_TRUE(fit.valid);
}

TEST(ZtsAnchorFit, ResetReturnsToFallback) {
    ZtsAnchorFit fit;
    fit.jitterFloorTicks = kFitFloorTicks;
    ASSERT_TRUE(fit.Tick(kPeriodFrames, kTicksPerBuffer, true, 100, 200));
    fit.Reset();
    EXPECT_FALSE(fit.valid);
    EXPECT_EQ(fit.count, 0);
    EXPECT_FALSE(fit.Tick(kPeriodFrames, kTicksPerBuffer, false, 0, 0));
}
