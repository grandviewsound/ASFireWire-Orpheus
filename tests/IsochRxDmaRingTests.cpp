#include <gtest/gtest.h>

#include "Isoch/Receive/IsochRxDmaRing.hpp"
#include "Isoch/Memory/IsochDMAMemoryManager.hpp"
#include "Hardware/HardwareInterface.hpp"

#include "Hardware/OHCIDescriptors.hpp"

using namespace ASFW::Isoch;
using namespace ASFW::Isoch::Rx;
using namespace ASFW::Isoch::Memory;

namespace {

std::shared_ptr<IIsochDMAMemory> MakeTestIsochMemory(::ASFW::Driver::HardwareInterface& hw,
                                                     size_t numDescriptors,
                                                     size_t packetSizeBytes) {
    IsochMemoryConfig config;
    config.numDescriptors = numDescriptors;
    config.packetSizeBytes = packetSizeBytes;
    config.descriptorAlignment = 16;
    config.payloadPageAlignment = 4096;

    auto concreteMgr = IsochDMAMemoryManager::Create(config);
    EXPECT_TRUE(concreteMgr);
    EXPECT_TRUE(concreteMgr->Initialize(hw));
    return concreteMgr;
}

} // namespace

TEST(IsochRxDmaRingTests, InitialCommandPtrWord_SetsZBitAndPointsToDesc0) {
    ::ASFW::Driver::HardwareInterface hw;
    auto mem = MakeTestIsochMemory(hw, 8, 64);
    ASSERT_TRUE(mem);

    IsochRxDmaRing ring;
    ASSERT_EQ(ring.SetupRings(*mem, 8, 64), kIOReturnSuccess);

    const uint32_t cmdPtr = ring.InitialCommandPtrWord();
    EXPECT_NE(cmdPtr, 0u);
    EXPECT_EQ(cmdPtr & 0x1u, 0x1u);
    EXPECT_EQ(cmdPtr & ~0x1u, ring.Descriptor0IOVA());
}

TEST(IsochRxDmaRingTests, ChainTopology_LastDescriptorIsTerminator) {
    // Apple's MultiIsochReceiver builds a chain-with-terminator IR program:
    // descriptor i (i < cap-1) branchWord = next.phys|1; descriptor (cap-1)
    // branchWord = 0. Hardware halts at the terminator until Recycle() relinks.
    ::ASFW::Driver::HardwareInterface hw;
    constexpr size_t kCap = 8;
    constexpr size_t kPkt = 64;
    auto mem = MakeTestIsochMemory(hw, kCap, kPkt);
    ASSERT_TRUE(mem);

    IsochRxDmaRing ring;
    ASSERT_EQ(ring.SetupRings(*mem, kCap, kPkt), kIOReturnSuccess);

    for (size_t i = 0; i + 1 < kCap; ++i) {
        auto* d = ring.DescriptorAt(i);
        ASSERT_NE(d, nullptr);
        EXPECT_NE(d->branchWord & ~0xFu, 0u) << "desc " << i << " missing next link";
        EXPECT_EQ(d->branchWord & 0xFu, 1u) << "desc " << i << " Z must be 1";
    }
    auto* terminator = ring.DescriptorAt(kCap - 1);
    ASSERT_NE(terminator, nullptr);
    EXPECT_EQ(terminator->branchWord, 0u) << "last descriptor must be terminator";
    EXPECT_EQ(ring.HeadIndex(), 0u);
    EXPECT_EQ(ring.TailIndex(), kCap - 1);
}

TEST(IsochRxDmaRingTests, ControlWord_MatchesAppleIR) {
    // Per AppleFWOHCI analysis of MultiIsochReceiver::newCommandElement
    // (sym 0x197dc), the IR descriptor control word is 0x280C1000 for a
    // 4096-byte buffer: cmd=INPUT_MORE, s=1, key=0, i=Never, b=Always,
    // w=Never, reqCount=4096. Our 64-byte test buffer differs only in the
    // reqCount low half; the upper half (control flags) must match.
    ::ASFW::Driver::HardwareInterface hw;
    constexpr size_t kCap = 8;
    constexpr size_t kPkt = 4096;
    auto mem = MakeTestIsochMemory(hw, kCap, kPkt);
    ASSERT_TRUE(mem);

    IsochRxDmaRing ring;
    ASSERT_EQ(ring.SetupRings(*mem, kCap, kPkt), kIOReturnSuccess);

    auto* d0 = ring.DescriptorAt(0);
    ASSERT_NE(d0, nullptr);
    EXPECT_EQ(d0->control, 0x280C1000u) << "must match Apple IR control word exactly";
}

TEST(IsochRxDmaRingTests, DequeueDescriptorBytes_ReturnsFreshBytes) {
    ::ASFW::Driver::HardwareInterface hw;
    constexpr size_t kCap = 4;
    constexpr size_t kPkt = 64;
    auto mem = MakeTestIsochMemory(hw, kCap, kPkt);
    ASSERT_TRUE(mem);

    IsochRxDmaRing ring;
    ASSERT_EQ(ring.SetupRings(*mem, kCap, kPkt), kIOReturnSuccess);

    auto* payload0 = static_cast<uint8_t*>(ring.PayloadVA(0));
    ASSERT_NE(payload0, nullptr);
    payload0[0] = 0xAA;
    payload0[1] = 0xBB;

    auto* d0 = ring.DescriptorAt(0);
    ASSERT_NE(d0, nullptr);
    constexpr uint16_t kFilled = 16;
    d0->statusWord = (0u << 16) | static_cast<uint16_t>(kPkt - kFilled);

    auto span = ring.DequeueDescriptorBytes(*mem);
    ASSERT_TRUE(span.has_value());
    EXPECT_EQ(span->descriptorIndex, 0u);
    EXPECT_EQ(span->length, kFilled);
    EXPECT_FALSE(span->exhausted);
    EXPECT_EQ(span->bytes[0], 0xAA);
    EXPECT_EQ(span->bytes[1], 0xBB);

    // No new bytes since last call → no span.
    EXPECT_FALSE(ring.DequeueDescriptorBytes(*mem).has_value());

    // Mark fully exhausted (resCount=0).
    d0->statusWord = 0u;
    auto spanFinal = ring.DequeueDescriptorBytes(*mem);
    ASSERT_TRUE(spanFinal.has_value());
    EXPECT_TRUE(spanFinal->exhausted);
    EXPECT_EQ(spanFinal->length, kPkt - kFilled);
    EXPECT_EQ(ring.HeadIndex(), 1u) << "head advances when descriptor exhausted";
}

TEST(IsochRxDmaRingTests, Recycle_SplicesOntoTailAndUpdatesBranch) {
    ::ASFW::Driver::HardwareInterface hw;
    constexpr size_t kCap = 4;
    constexpr size_t kPkt = 64;
    auto mem = MakeTestIsochMemory(hw, kCap, kPkt);
    ASSERT_TRUE(mem);

    IsochRxDmaRing ring;
    ASSERT_EQ(ring.SetupRings(*mem, kCap, kPkt), kIOReturnSuccess);

    // Initial: head=0, tail=3 (capacity-1). Recycle desc 0 → tail=0.
    EXPECT_EQ(ring.HeadIndex(), 0u);
    EXPECT_EQ(ring.TailIndex(), 3u);

    auto* tailBefore = ring.DescriptorAt(3);
    ASSERT_NE(tailBefore, nullptr);
    EXPECT_EQ(tailBefore->branchWord, 0u) << "initial terminator at index cap-1";

    EXPECT_EQ(ring.Recycle(0, *mem), kIOReturnSuccess);
    EXPECT_EQ(ring.TailIndex(), 0u);

    auto* d0 = ring.DescriptorAt(0);
    ASSERT_NE(d0, nullptr);
    EXPECT_EQ(d0->branchWord, 0u) << "recycled descriptor becomes new terminator";
    EXPECT_EQ(ASFW::Async::HW::AR_resCount(*d0), kPkt);

    auto* d3 = ring.DescriptorAt(3);
    ASSERT_NE(d3, nullptr);
    EXPECT_NE(d3->branchWord & ~0xFu, 0u) << "previous tail now points at recycled desc";
    EXPECT_EQ(d3->branchWord & 0xFu, 1u);
}

TEST(IsochRxDmaRingTests, ResetForStart_RearmsAllDescriptorStatusWords) {
    // Apple's prepareElementListForStart walks the element list and writes
    // (0 << 16 | reqCount) into every descriptor's xferStatus|resCount before
    // the IR context starts. Our ResetForStart must do the same so a Stop→
    // Start cycle does not replay the previous run's residual data.
    ::ASFW::Driver::HardwareInterface hw;
    constexpr size_t kCap = 8;
    constexpr size_t kPkt = 64;
    auto mem = MakeTestIsochMemory(hw, kCap, kPkt);
    ASSERT_TRUE(mem);

    IsochRxDmaRing ring;
    ASSERT_EQ(ring.SetupRings(*mem, kCap, kPkt), kIOReturnSuccess);

    // Simulate a previous run leaving non-fresh state on every descriptor.
    for (size_t i = 0; i < kCap; ++i) {
        auto* d = ring.DescriptorAt(i);
        ASSERT_NE(d, nullptr);
        d->statusWord = (0x00ABu << 16) | static_cast<uint16_t>(kPkt - 17);
    }

    ring.ResetForStart();

    // Every descriptor must now read xferStatus=0, resCount=reqCount.
    for (size_t i = 0; i < kCap; ++i) {
        auto* d = ring.DescriptorAt(i);
        ASSERT_NE(d, nullptr);
        mem->FetchFromDevice(d, sizeof(*d));
        EXPECT_EQ(ASFW::Async::HW::AR_xferStatus(*d), 0u) << "desc " << i;
        EXPECT_EQ(ASFW::Async::HW::AR_resCount(*d), kPkt) << "desc " << i;
    }
}

