// ZeroTimestampAnchorTests.cpp
// ASFW - Host-safe unit tests for the hardware-driven zero-timestamp anchor.
//
// Validates, off-hardware, the two pieces of the Apple-faithful clock path that
// would otherwise only be exercisable on the device:
//   1. FwSampleClock — FireWire cycle-timer -> monotonic device sample-time.
//   2. TxSharedQueueSPSC hardware anchor — seqlock-protected (sampleTime,
//      hostTicks) publish/read across the shared-memory boundary.

#include <gtest/gtest.h>

#include <array>
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
