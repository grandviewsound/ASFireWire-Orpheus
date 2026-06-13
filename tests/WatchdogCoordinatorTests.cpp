// WatchdogCoordinatorTests.cpp
// ASFW - Host-safe unit tests for the watchdog timer's deadline-anchored re-arm.
//
// The watchdog tick is nominally 1 ms, but the old `WakeAtTime(now + period)`
// sampled `now` after HandleTick ran, folding ~0.65 ms of handler work +
// delivery lag into every period (the tick effectively ran ~1.65 ms). The fix
// anchors the next deadline on the previous one. These tests pin that math:
// constant-lag delivery must NOT drift the cadence, slow handlers skip beats
// instead of double-polling, and long stalls re-anchor.

#include <gtest/gtest.h>

#include <cstdint>

#include "ASFWDriver/Scheduling/WatchdogCoordinator.hpp"

using ASFW::Driver::WatchdogCoordinator;

namespace {

constexpr uint64_t kPeriod = 1000;  // arbitrary mach-tick period for the math

TEST(WatchdogCoordinatorDeadline, FirstArmAnchorsAtNow) {
    // prevDeadline == 0 means "never armed": anchor one period past now.
    EXPECT_EQ(WatchdogCoordinator::ComputeNextDeadline(0, 5000, kPeriod), 6000u);
}

TEST(WatchdogCoordinatorDeadline, LagDoesNotCompound) {
    // Delivered 300 ticks late (< period). Next deadline is exactly one period
    // past the PREVIOUS deadline, not past `now` — lag is absorbed as phase.
    EXPECT_EQ(WatchdogCoordinator::ComputeNextDeadline(1000, 1300, kPeriod), 2000u);
}

TEST(WatchdogCoordinatorDeadline, SteadyCadenceHoldsOverManyTicks) {
    // Simulate the rearm loop: each tick is delivered a constant fraction of a
    // period late. The armed deadlines must advance by exactly one period each
    // time, with zero accumulated drift (the bug the fix removes).
    const uint64_t lag = kPeriod / 2;  // constant delivery+handler lag
    uint64_t prevDeadline = 0;
    uint64_t now = 10000;

    // First arm.
    uint64_t d0 = WatchdogCoordinator::ComputeNextDeadline(prevDeadline, now, kPeriod);
    prevDeadline = d0;

    for (uint64_t i = 1; i <= 1000; ++i) {
        now = prevDeadline + lag;  // delivered late by `lag`
        const uint64_t next = WatchdogCoordinator::ComputeNextDeadline(prevDeadline, now, kPeriod);
        EXPECT_EQ(next, d0 + i * kPeriod) << "drift at tick " << i;
        prevDeadline = next;
    }
}

TEST(WatchdogCoordinatorDeadline, SlowHandlerSkipsBeatsNoDoublePoll) {
    // Handler ran 2.5 periods. Catch up to the next FUTURE period boundary
    // (skipping the missed beats) rather than scheduling in the past.
    EXPECT_EQ(WatchdogCoordinator::ComputeNextDeadline(1000, 3500, kPeriod), 4000u);
}

TEST(WatchdogCoordinatorDeadline, ExactlyOnDeadlineStepsForward) {
    // now == prevDeadline + period: the `<= now` guard must still step past it.
    EXPECT_EQ(WatchdogCoordinator::ComputeNextDeadline(1000, 2000, kPeriod), 3000u);
}

TEST(WatchdogCoordinatorDeadline, LongStallReAnchorsAtNow) {
    // Stalled far longer than 64 periods (e.g. system sleep): re-anchor at now
    // instead of stepping the loop across the whole gap.
    const uint64_t now = 1000 + kPeriod * 100;
    EXPECT_EQ(WatchdogCoordinator::ComputeNextDeadline(1000, now, kPeriod), now + kPeriod);
}

TEST(WatchdogCoordinatorDeadline, JustUnderStallThresholdStepsNotReanchor) {
    // 60 periods behind (< 64): still step from the prior deadline, no re-anchor.
    const uint64_t now = 1000 + kPeriod * 60;
    EXPECT_EQ(WatchdogCoordinator::ComputeNextDeadline(1000, now, kPeriod), now + kPeriod);
}

TEST(WatchdogCoordinatorDeadline, ZeroPeriodIsDefensiveNoInfiniteLoop) {
    // Degenerate period must not spin the catch-up loop; return now.
    EXPECT_EQ(WatchdogCoordinator::ComputeNextDeadline(1000, 5000, 0), 5000u);
}

} // namespace
