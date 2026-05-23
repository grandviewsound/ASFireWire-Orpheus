#include <gtest/gtest.h>

#include "../ASFWDriver/Isoch/Core/ExternalSyncBridge.hpp"

using ASFW::Isoch::Core::ExternalSyncBridge;
using ASFW::Isoch::Core::ExternalSyncClockState;

TEST(ExternalSyncBridge, EstablishesAfterSixteenValidUpdates) {
    ExternalSyncBridge bridge;
    ExternalSyncClockState state;
    bridge.active.store(true, std::memory_order_release);

    for (uint32_t i = 0; i < 15; ++i) {
        uint32_t seq = 0;
        EXPECT_FALSE(state.ObserveSample(bridge,
                                         /*nowHostTicks=*/1000 + i,
                                         /*syt=*/0x1234,
                                         /*fdf=*/ExternalSyncBridge::kFdf48k,
                                         /*dbs=*/6,
                                         &seq));
        EXPECT_EQ(seq, i + 1);
        EXPECT_FALSE(bridge.clockEstablished.load(std::memory_order_acquire));
    }

    uint32_t transitionSeq = 0;
    EXPECT_TRUE(state.ObserveSample(bridge,
                                    /*nowHostTicks=*/2000,
                                    /*syt=*/0x1234,
                                    /*fdf=*/ExternalSyncBridge::kFdf48k,
                                    /*dbs=*/6,
                                    &transitionSeq));
    EXPECT_EQ(transitionSeq, 16u);
    EXPECT_FALSE(bridge.clockEstablished.load(std::memory_order_acquire));
}

TEST(ExternalSyncBridge, ClearsEstablishedOnStaleUpdate) {
    ExternalSyncBridge bridge;
    ExternalSyncClockState state;

    bridge.active.store(true, std::memory_order_release);
    bridge.clockEstablished.store(true, std::memory_order_release);
    bridge.lastUpdateHostTicks.store(100, std::memory_order_release);

    EXPECT_TRUE(state.HandleStale(bridge, /*nowHostTicks=*/250, /*staleThresholdHostTicks=*/100));
    EXPECT_FALSE(bridge.clockEstablished.load(std::memory_order_acquire));
}

TEST(ExternalSyncBridge, ExternalClockSourceDefaultsFalseAndSurvivesReset) {
    ExternalSyncBridge bridge;

    // Defaults to internal clock (Apple externalSync=0 → transmit SYT free-runs).
    EXPECT_FALSE(bridge.externalClockSource.load(std::memory_order_acquire));

    // Orchestration marks an external clock source (Wordclock/SPDIF/ADAT).
    bridge.externalClockSource.store(true, std::memory_order_release);
    EXPECT_TRUE(bridge.externalClockSource.load(std::memory_order_acquire));

    // Runtime sync state is wiped on (re)start, but the clock-source CONFIG must
    // persist so SetExternalClockSource is order-independent vs StartReceive's
    // internal Reset(). Verify Reset() clears runtime fields but not config.
    bridge.active.store(true, std::memory_order_release);
    bridge.clockEstablished.store(true, std::memory_order_release);
    bridge.lastUpdateHostTicks.store(42, std::memory_order_release);
    bridge.Reset();

    EXPECT_FALSE(bridge.active.load(std::memory_order_acquire));
    EXPECT_FALSE(bridge.clockEstablished.load(std::memory_order_acquire));
    EXPECT_EQ(bridge.lastUpdateHostTicks.load(std::memory_order_acquire), 0u);
    EXPECT_TRUE(bridge.externalClockSource.load(std::memory_order_acquire))
        << "externalClockSource is configuration, not runtime state; Reset() must not clear it";
}

TEST(ExternalSyncBridge, TransitionRequiresCallerToFlipEstablishedFlag) {
    ExternalSyncBridge bridge;
    ExternalSyncClockState state;
    bridge.active.store(true, std::memory_order_release);

    for (uint32_t i = 0; i < ExternalSyncClockState::kEstablishValidUpdates; ++i) {
        (void)state.ObserveSample(bridge,
                                  100 + i,
                                  0x2000,
                                  ExternalSyncBridge::kFdf48k,
                                  2,
                                  nullptr);
    }

    EXPECT_FALSE(bridge.clockEstablished.load(std::memory_order_acquire));
    bridge.clockEstablished.store(true, std::memory_order_release);

    EXPECT_FALSE(state.ObserveSample(bridge,
                                     /*nowHostTicks=*/500,
                                     /*syt=*/0x2000,
                                     /*fdf=*/ExternalSyncBridge::kFdf48k,
                                     /*dbs=*/2,
                                     nullptr));
}
