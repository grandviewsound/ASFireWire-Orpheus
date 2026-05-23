#include <gtest/gtest.h>

#include "ASFWDriver/Protocols/AVC/CMP/LocalPcrRegisterFile.hpp"

using ASFW::CMP::LocalPcrRegisterFile;
using Reg = ASFW::CMP::LocalPcrRegisterFile::Reg;

// A fresh register file reads as zero everywhere.
TEST(LocalPcrRegisterFile, DefaultsZero) {
    LocalPcrRegisterFile rf;
    EXPECT_EQ(rf.Read(Reg::kOutputMaster, 0).value(), 0u);
    EXPECT_EQ(rf.Read(Reg::kInputMaster, 0).value(), 0u);
    EXPECT_EQ(rf.Read(Reg::kOutputPlug, 0).value(), 0u);
    EXPECT_EQ(rf.Read(Reg::kInputPlug, 30).value(), 0u);
}

// Set then Read round-trips the exact quadlet (raw store, no interpretation).
TEST(LocalPcrRegisterFile, SetReadRoundTrip) {
    LocalPcrRegisterFile rf;
    ASSERT_TRUE(rf.Set(Reg::kOutputPlug, 0, 0x8001805Bu));
    EXPECT_EQ(rf.Read(Reg::kOutputPlug, 0).value(), 0x8001805Bu);
    // Independent slots don't alias.
    EXPECT_EQ(rf.Read(Reg::kInputPlug, 0).value(), 0u);
    EXPECT_EQ(rf.Read(Reg::kOutputPlug, 1).value(), 0u);
}

// Out-of-range plug index is rejected (nullopt / false), masters need no index.
TEST(LocalPcrRegisterFile, OutOfRangeRejected) {
    LocalPcrRegisterFile rf;
    EXPECT_FALSE(rf.Read(Reg::kOutputPlug, 31).has_value());
    EXPECT_FALSE(rf.Read(Reg::kInputPlug, 200).has_value());
    EXPECT_FALSE(rf.Set(Reg::kOutputPlug, 31, 0x1234u));
    bool swapped = true;
    EXPECT_FALSE(rf.CompareSwap(Reg::kInputPlug, 99, 0, 1, &swapped).has_value());
    EXPECT_FALSE(swapped);
}

// compare_swap on a match: stores desired, returns the prior value.
TEST(LocalPcrRegisterFile, CompareSwapMatchStores) {
    LocalPcrRegisterFile rf;
    ASSERT_TRUE(rf.Set(Reg::kInputPlug, 0, 0x80000000u)); // online, p2p=0

    bool swapped = false;
    const auto prior = rf.CompareSwap(Reg::kInputPlug, 0,
                                      /*expected=*/0x80000000u,
                                      /*desired=*/0x81010000u, // p2p 0→1, channel 1
                                      &swapped);
    ASSERT_TRUE(prior.has_value());
    EXPECT_EQ(prior.value(), 0x80000000u); // prior returned
    EXPECT_TRUE(swapped);
    EXPECT_EQ(rf.Read(Reg::kInputPlug, 0).value(), 0x81010000u); // stored
}

// compare_swap on a mismatch: does NOT store, still returns the (unchanged) prior.
TEST(LocalPcrRegisterFile, CompareSwapMismatchNoStore) {
    LocalPcrRegisterFile rf;
    ASSERT_TRUE(rf.Set(Reg::kOutputPlug, 2, 0x8000805Bu));

    bool swapped = true;
    const auto prior = rf.CompareSwap(Reg::kOutputPlug, 2,
                                      /*expected=*/0xDEADBEEFu, // wrong
                                      /*desired=*/0x8101805Bu,
                                      &swapped);
    ASSERT_TRUE(prior.has_value());
    EXPECT_EQ(prior.value(), 0x8000805Bu); // current returned
    EXPECT_FALSE(swapped);
    EXPECT_EQ(rf.Read(Reg::kOutputPlug, 2).value(), 0x8000805Bu); // unchanged
}

// A second matching compare_swap (the p2p++ idempotency the early-accept path
// relies on): re-running with the now-current expected swaps again cleanly.
TEST(LocalPcrRegisterFile, CompareSwapSequentialP2p) {
    LocalPcrRegisterFile rf;
    rf.Set(Reg::kInputPlug, 0, 0x80000000u);
    // first connection p2p 0→1
    EXPECT_EQ(rf.CompareSwap(Reg::kInputPlug, 0, 0x80000000u, 0x81000000u).value(), 0x80000000u);
    // second connection p2p 1→2
    EXPECT_EQ(rf.CompareSwap(Reg::kInputPlug, 0, 0x81000000u, 0x82000000u).value(), 0x81000000u);
    EXPECT_EQ(rf.Read(Reg::kInputPlug, 0).value(), 0x82000000u);
}

// Master plug registers are independently addressable.
TEST(LocalPcrRegisterFile, MasterPlugsIndependent) {
    LocalPcrRegisterFile rf;
    rf.Set(Reg::kOutputMaster, 0, 0x80000010u);
    rf.Set(Reg::kInputMaster, 0, 0x8000000Au);
    EXPECT_EQ(rf.Read(Reg::kOutputMaster, 0).value(), 0x80000010u);
    EXPECT_EQ(rf.Read(Reg::kInputMaster, 0).value(), 0x8000000Au);
}
