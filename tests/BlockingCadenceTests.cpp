// BlockingCadenceTests.cpp
// ASFW - Phase 1.5 Encoding Tests
//
// Tests for 48 kHz blocking cadence pattern.
// Reference: 000-48kORIG.txt
//

#include <gtest/gtest.h>

#include <cstdlib>

#include "Isoch/Encoding/BlockingCadence48k.hpp"

using namespace ASFW::Encoding;

//==============================================================================
// Constants Tests
//==============================================================================

TEST(BlockingCadenceTests, CorrectSamplesPerPacket) {
    EXPECT_EQ(kSamplesPerPacket48k, 8);
}

TEST(BlockingCadenceTests, CorrectDataPacketsPerPeriod) {
    EXPECT_EQ(kDataPacketsPer8Cycles, 6);
}

TEST(BlockingCadenceTests, CorrectNoDataPacketsPerPeriod) {
    EXPECT_EQ(kNoDataPacketsPer8Cycles, 2);
}

//==============================================================================
// Initial State Tests
//==============================================================================

TEST(BlockingCadenceTests, StartsAtCycleZero) {
    BlockingCadence48k cadence;
    EXPECT_EQ(cadence.getCycleIndex(), 0);
    EXPECT_EQ(cadence.getTotalCycles(), 0);
}

TEST(BlockingCadenceTests, FirstCycleIsNoData) {
    BlockingCadence48k cadence;
    EXPECT_FALSE(cadence.isDataPacket());
    EXPECT_EQ(cadence.samplesThisCycle(), 0);
}

//==============================================================================
// Pattern Tests - N-D-D-D Repeating
//==============================================================================

TEST(BlockingCadenceTests, FullPatternOver8Cycles) {
    BlockingCadence48k cadence;
    
    // Expected pattern: N-D-D-D-N-D-D-D
    bool expected[] = {false, true, true, true, false, true, true, true};
    
    for (int i = 0; i < 8; i++) {
        SCOPED_TRACE("Cycle " + std::to_string(i));
        EXPECT_EQ(cadence.isDataPacket(), expected[i]);
        cadence.advance();
    }
}

TEST(BlockingCadenceTests, PatternRepeatsAfter8Cycles) {
    BlockingCadence48k cadence;
    
    // Get first 8 cycles
    bool first8[8];
    for (int i = 0; i < 8; i++) {
        first8[i] = cadence.isDataPacket();
        cadence.advance();
    }
    
    // Next 8 should match
    for (int i = 0; i < 8; i++) {
        SCOPED_TRACE("Cycle " + std::to_string(i + 8));
        EXPECT_EQ(cadence.isDataPacket(), first8[i]);
        cadence.advance();
    }
}

TEST(BlockingCadenceTests, SamplesMatchPattern) {
    BlockingCadence48k cadence;
    
    // Expected: 0, 8, 8, 8, 0, 8, 8, 8
    uint32_t expected[] = {0, 8, 8, 8, 0, 8, 8, 8};
    
    for (int i = 0; i < 8; i++) {
        SCOPED_TRACE("Cycle " + std::to_string(i));
        EXPECT_EQ(cadence.samplesThisCycle(), expected[i]);
        cadence.advance();
    }
}

//==============================================================================
// Sample Count Verification
//==============================================================================

TEST(BlockingCadenceTests, Total48SamplesPer8Cycles) {
    BlockingCadence48k cadence;
    uint32_t totalSamples = 0;
    
    for (int i = 0; i < 8; i++) {
        totalSamples += cadence.samplesThisCycle();
        cadence.advance();
    }
    
    // 6 DATA × 8 samples = 48 samples
    EXPECT_EQ(totalSamples, 48);
}

TEST(BlockingCadenceTests, Correct48kSamplesPerSecond) {
    BlockingCadence48k cadence;
    uint32_t totalSamples = 0;
    
    // 8000 cycles = 1 second at FireWire rate
    for (int i = 0; i < 8000; i++) {
        totalSamples += cadence.samplesThisCycle();
        cadence.advance();
    }
    
    // Should be exactly 48000 samples
    EXPECT_EQ(totalSamples, 48000);
}

//==============================================================================
// Advance and Reset Tests
//==============================================================================

TEST(BlockingCadenceTests, AdvanceIncrementsCycle) {
    BlockingCadence48k cadence;
    
    EXPECT_EQ(cadence.getTotalCycles(), 0);
    cadence.advance();
    EXPECT_EQ(cadence.getTotalCycles(), 1);
    cadence.advance();
    EXPECT_EQ(cadence.getTotalCycles(), 2);
}

TEST(BlockingCadenceTests, AdvanceByMultiple) {
    BlockingCadence48k cadence;
    
    cadence.advanceBy(5);
    EXPECT_EQ(cadence.getTotalCycles(), 5);
    EXPECT_EQ(cadence.getCycleIndex(), 5);
}

TEST(BlockingCadenceTests, ResetClearsState) {
    BlockingCadence48k cadence;
    
    cadence.advanceBy(100);
    EXPECT_GT(cadence.getTotalCycles(), 0);
    
    cadence.reset();
    EXPECT_EQ(cadence.getTotalCycles(), 0);
    EXPECT_EQ(cadence.getCycleIndex(), 0);
    EXPECT_FALSE(cadence.isDataPacket());  // First cycle is NO-DATA
}

//==============================================================================
// FireBug Capture Pattern Validation
// Reference: 000-48kORIG.txt cycles 977-984
//==============================================================================

TEST(BlockingCadenceTests, MatchesFireBugPattern) {
    BlockingCadence48k cadence;
    
    // From capture (starting at an arbitrary point in the pattern):
    // 977: NO-DATA (8 bytes)
    // 978: DATA (72 bytes)
    // 979: DATA (72 bytes)
    // 980: DATA (72 bytes)
    // 981: NO-DATA (8 bytes)
    // 982: DATA (72 bytes)
    // 983: DATA (72 bytes)
    // 984: DATA (72 bytes)
    
    // This matches: N-D-D-D-N-D-D-D
    // Which is our pattern starting at cycle 0
    
    EXPECT_FALSE(cadence.isDataPacket()); cadence.advance();  // N
    EXPECT_TRUE(cadence.isDataPacket());  cadence.advance();  // D
    EXPECT_TRUE(cadence.isDataPacket());  cadence.advance();  // D
    EXPECT_TRUE(cadence.isDataPacket());  cadence.advance();  // D
    EXPECT_FALSE(cadence.isDataPacket()); cadence.advance();  // N
    EXPECT_TRUE(cadence.isDataPacket());  cadence.advance();  // D
    EXPECT_TRUE(cadence.isDataPacket());  cadence.advance();  // D
    EXPECT_TRUE(cadence.isDataPacket());  cadence.advance();  // D
}

//==============================================================================
// U6 — multi-rate cadence (Bresenham). One bus second (8000 cycles) must carry
// EXACTLY `rate` frames at every supported rate, including the fractional 44.1
// family where the per-cycle frame count never settles to an integer pattern.
//==============================================================================

TEST(BlockingCadenceTests, ExactFramesPerSecondAtAllRates) {
    struct Case { uint32_t rate; uint32_t spp; };
    // samplesPerPacket = 8 (<=48k) / 16 (88.2-96k) / 32 (176.4-192k) per Apple.
    const Case cases[] = {
        {44100, 8}, {48000, 8},
        {88200, 16}, {96000, 16},
        {176400, 32}, {192000, 32},
    };

    for (const Case c : cases) {
        BlockingCadence48k cadence;
        cadence.configure(c.rate, c.spp);

        // One bus second: within one packet of the exact rate. The 44.1 family
        // can't hit an exact integer in a single second (44100 isn't a multiple
        // of 8 frames/packet) — the few-frame remainder carries into next second.
        uint64_t oneSecond = 0;
        for (int i = 0; i < 8000; ++i) {
            oneSecond += cadence.samplesThisCycle();
            cadence.advance();
        }
        EXPECT_LE(std::llabs(static_cast<long long>(oneSecond) - c.rate), c.spp)
            << "rate=" << c.rate << " spp=" << c.spp << " oneSecond=" << oneSecond;

        // Two bus seconds: EXACT for every supported rate (the remainder closes).
        cadence.reset();
        uint64_t totalSamples = 0;
        for (int i = 0; i < 16000; ++i) {
            totalSamples += cadence.samplesThisCycle();
            cadence.advance();
        }
        EXPECT_EQ(totalSamples, 2ULL * c.rate) << "rate=" << c.rate << " spp=" << c.spp;

        // Every DATA packet carries exactly samplesPerPacket frames (or 0).
        cadence.reset();
        for (int i = 0; i < 64; ++i) {
            const uint32_t s = cadence.samplesThisCycle();
            EXPECT_TRUE(s == 0 || s == c.spp) << "rate=" << c.rate << " s=" << s;
            cadence.advance();
        }
    }
}

TEST(BlockingCadenceTests, ConfigureFortyEightKMatchesDefaultPattern) {
    // configure(48000, 8) must be identical to the default-constructed 48k state.
    BlockingCadence48k def;
    BlockingCadence48k cfg;
    cfg.configure(48000, 8);
    for (int i = 0; i < 256; ++i) {
        ASSERT_EQ(def.isDataPacket(), cfg.isDataPacket()) << "cycle " << i;
        ASSERT_EQ(def.samplesThisCycle(), cfg.samplesThisCycle()) << "cycle " << i;
        def.advance();
        cfg.advance();
    }
}
