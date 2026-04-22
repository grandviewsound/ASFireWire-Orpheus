#include <gtest/gtest.h>

#include <array>
#include <cstring>

#include "ASFWDriver/Hardware/OHCIDescriptors.hpp"
#include "ASFWDriver/Shared/Rings/BufferRing.hpp"
#include "ASFWDriver/Testing/FakeDMAMemory.hpp"

namespace ASFW::Testing {

class BufferRingDMATest : public ::testing::Test {
protected:
    FakeDMAMemory dma_{512 * 1024};
    Shared::BufferRing ring_{};
    uint64_t descBaseIOVA_{0};
    uint64_t bufBaseIOVA_{0};

    void SetUp() override {
        constexpr size_t kNum = 32;
        constexpr size_t kBufSize = 256;

        auto descRegion = dma_.AllocateRegion(kNum * sizeof(Async::HW::OHCIDescriptor));
        ASSERT_TRUE(descRegion.has_value());
        descBaseIOVA_ = descRegion->deviceBase;

        auto bufRegion = dma_.AllocateRegion(kNum * kBufSize);
        ASSERT_TRUE(bufRegion.has_value());
        bufBaseIOVA_ = bufRegion->deviceBase;

        auto* descs = reinterpret_cast<Async::HW::OHCIDescriptor*>(descRegion->virtualBase);
        std::span<Async::HW::OHCIDescriptor> descSpan{descs, kNum};
        std::span<uint8_t> bufSpan{bufRegion->virtualBase, kNum * kBufSize};

        ASSERT_TRUE(ring_.Initialize(descSpan, bufSpan, kNum, kBufSize));
        ring_.BindDma(&dma_);
        ASSERT_TRUE(ring_.Finalize(descBaseIOVA_, bufBaseIOVA_));
    }
};

TEST_F(BufferRingDMATest, DequeueFlagsAutoRecycleWhenHardwareAdvances) {
    constexpr size_t kBufSize = 256;

    // Simulate hardware: buffer 0 fully consumed (resCount=0), buffer 1 has 32 bytes written.
    auto* desc0 = ring_.GetDescriptor(0);
    auto* desc1 = ring_.GetDescriptor(1);
    ASSERT_NE(desc0, nullptr);
    ASSERT_NE(desc1, nullptr);

    // reqCount stays in low 16 of control; resCount lives in low 16 of statusWord.
    // Setting statusWord: xferStatus=0, resCount=0 → buffer 0 full.
    desc0->statusWord = 0u;  // full
    // Setting buffer 1: xferStatus=0, resCount=kBufSize-32 → 32 bytes new.
    desc1->statusWord = static_cast<uint32_t>(kBufSize - 32);

    auto info = ring_.Dequeue();
    ASSERT_TRUE(info.has_value());
    EXPECT_TRUE(info->autoRecycledPrev)
        << "Dequeue must flag auto-recycle when next descriptor shows hardware advanced; "
           "ARContextBase relies on this flag to write the OHCI WAKE bit.";
    EXPECT_EQ(info->descriptorIndex, 1u);
    EXPECT_EQ(info->bytesFilled, 32u);
    EXPECT_EQ(info->startOffset, 0u);

    // Buffer 0 must have been recycled in place (resCount restored to reqCount=kBufSize).
    EXPECT_EQ(Async::HW::AR_resCount(*desc0), static_cast<uint16_t>(kBufSize));
}

TEST_F(BufferRingDMATest, DequeueDoesNotFlagAutoRecycleOnNormalReturn) {
    constexpr size_t kBufSize = 256;

    // Only buffer 0 has data; buffer 1 is still pristine (resCount==reqCount).
    auto* desc0 = ring_.GetDescriptor(0);
    ASSERT_NE(desc0, nullptr);
    desc0->statusWord = static_cast<uint32_t>(kBufSize - 16);  // 16 bytes written

    auto info = ring_.Dequeue();
    ASSERT_TRUE(info.has_value());
    EXPECT_FALSE(info->autoRecycledPrev)
        << "Auto-recycle must only flag when hardware has advanced past the head buffer.";
    EXPECT_EQ(info->descriptorIndex, 0u);
    EXPECT_EQ(info->bytesFilled, 16u);
}

// --------------------------------------------------------------------------
// Fix67 straddle-scratch: when hardware advances from old buffer to new, the
// old buffer's tail (containing the HEAD of a straddling packet) must be
// preserved before the old descriptor is reset. AppleFWOHCI does this via
// IOMalloc(0x2000u) scratch; we mirror with a per-ring scratch span.
// --------------------------------------------------------------------------
TEST_F(BufferRingDMATest, StraddleScratchCopiesOldTailAndNewHead) {
    constexpr size_t kBufSize = 256;
    static std::array<uint8_t, 8192> scratch{};
    scratch.fill(0xAA);
    ring_.AttachScratch(std::span<uint8_t>(scratch));

    // Fill old buffer[0] with a known pattern in the TAIL (last 12 bytes) —
    // simulates the head of a straddling FCP response packet. Head was already
    // returned earlier (last_dequeued_bytes_ = kBufSize-12 conceptually) but
    // for this test we dequeue once first with just the tail being "new".
    auto* buf0_va = static_cast<uint8_t*>(ring_.GetBufferAddress(0));
    auto* buf1_va = static_cast<uint8_t*>(ring_.GetBufferAddress(1));
    ASSERT_NE(buf0_va, nullptr);
    ASSERT_NE(buf1_va, nullptr);

    for (size_t i = 0; i < kBufSize; ++i) buf0_va[i] = 0x11;  // sentinel
    // Straddle HEAD bytes at end of buffer 0:
    const std::array<uint8_t, 12> straddleHead{0x01, 0x60, 0xBF, 0xC0,
                                               0x01, 0x01, 0x03, 0xFF,
                                               0xDE, 0xAD, 0xBE, 0xEF};
    std::memcpy(buf0_va + (kBufSize - 12), straddleHead.data(), straddleHead.size());

    // Buffer 1 holds the TAIL bytes of the straddling packet + next packet.
    for (size_t i = 0; i < kBufSize; ++i) buf1_va[i] = 0x22;  // sentinel
    const std::array<uint8_t, 8> straddleTail{0xCA, 0xFE, 0xBA, 0xBE,
                                              0x12, 0x34, 0x56, 0x78};
    std::memcpy(buf1_va, straddleTail.data(), straddleTail.size());

    // Mark buffer 0 fully consumed; buffer 1 has 8 bytes of new data.
    auto* desc0 = ring_.GetDescriptor(0);
    auto* desc1 = ring_.GetDescriptor(1);
    desc0->statusWord = 0u;                                       // full
    desc1->statusWord = static_cast<uint32_t>(kBufSize - 8);      // 8 bytes in

    auto info = ring_.Dequeue();
    ASSERT_TRUE(info.has_value());
    EXPECT_TRUE(info->isStraddleScratch)
        << "Auto-recycle with scratch attached must return a straddle-scratch view.";
    EXPECT_TRUE(info->autoRecycledPrev);
    EXPECT_EQ(info->descriptorIndex, 0u)
        << "descriptorIndex is the OLD index so WAKE accounting is correct.";
    EXPECT_EQ(info->startOffset, 0u);
    // With last_dequeued_bytes_ == 0 on first call, the old-tail span covers
    // the whole buffer (256 bytes). Scratch holds old[0..256) + new[0..8).
    EXPECT_EQ(info->bytesFilled, kBufSize + 8u);
    EXPECT_EQ(info->virtualAddress, scratch.data());

    // The straddle HEAD bytes must appear at scratch offset (kBufSize-12).
    EXPECT_EQ(0, std::memcmp(scratch.data() + (kBufSize - 12),
                             straddleHead.data(), straddleHead.size()))
        << "Old buffer's tail (straddle head) must be preserved at the end of "
           "the old-portion in scratch.";
    // The straddle TAIL bytes must appear immediately after (at offset kBufSize).
    EXPECT_EQ(0, std::memcmp(scratch.data() + kBufSize,
                             straddleTail.data(), straddleTail.size()))
        << "New buffer's head must follow directly after old-portion in scratch, "
           "making the straddling packet readable as one contiguous span.";
}

TEST_F(BufferRingDMATest, StraddleNextDequeueSkipsBytesAlreadyInScratch) {
    constexpr size_t kBufSize = 256;
    static std::array<uint8_t, 8192> scratch{};
    ring_.AttachScratch(std::span<uint8_t>(scratch));

    auto* buf0_va = static_cast<uint8_t*>(ring_.GetBufferAddress(0));
    auto* buf1_va = static_cast<uint8_t*>(ring_.GetBufferAddress(1));
    ASSERT_NE(buf0_va, nullptr);
    ASSERT_NE(buf1_va, nullptr);
    for (size_t i = 0; i < kBufSize; ++i) buf0_va[i] = 0x00;
    for (size_t i = 0; i < kBufSize; ++i) buf1_va[i] = 0x55;

    auto* desc0 = ring_.GetDescriptor(0);
    auto* desc1 = ring_.GetDescriptor(1);
    desc0->statusWord = 0u;                                     // full
    desc1->statusWord = static_cast<uint32_t>(kBufSize - 16);   // 16 new bytes

    auto first = ring_.Dequeue();
    ASSERT_TRUE(first.has_value());
    EXPECT_TRUE(first->isStraddleScratch);
    EXPECT_EQ(first->bytesFilled, kBufSize + 16u);

    // Hardware now writes 32 more bytes into buffer 1 (total 48).
    desc1->statusWord = static_cast<uint32_t>(kBufSize - 48);

    auto second = ring_.Dequeue();
    ASSERT_TRUE(second.has_value());
    EXPECT_FALSE(second->isStraddleScratch)
        << "Second Dequeue must return a normal buffer view, not scratch.";
    EXPECT_EQ(second->descriptorIndex, 1u);
    EXPECT_EQ(second->startOffset, 16u)
        << "First 16 bytes of new buffer were delivered via scratch; second "
           "Dequeue must resume at offset 16 to avoid double-delivery.";
    EXPECT_EQ(second->bytesFilled, 48u);
}

TEST_F(BufferRingDMATest, StraddleFallsBackWhenNoScratchAttached) {
    constexpr size_t kBufSize = 256;
    // Do NOT attach scratch — legacy path must still function.
    auto* desc0 = ring_.GetDescriptor(0);
    auto* desc1 = ring_.GetDescriptor(1);
    desc0->statusWord = 0u;
    desc1->statusWord = static_cast<uint32_t>(kBufSize - 24);

    auto info = ring_.Dequeue();
    ASSERT_TRUE(info.has_value());
    EXPECT_FALSE(info->isStraddleScratch)
        << "Without scratch, auto-recycle falls back to legacy (lossy) path.";
    EXPECT_TRUE(info->autoRecycledPrev);
    EXPECT_EQ(info->descriptorIndex, 1u);
    EXPECT_EQ(info->bytesFilled, 24u);
}

TEST_F(BufferRingDMATest, FinalizeProgramsDataAddressAndBranchWords) {
    constexpr size_t kNum = 32;
    constexpr size_t kBufSize = 256;

    for (size_t i = 0; i < kNum; ++i) {
        auto* desc = ring_.GetDescriptor(i);
        ASSERT_NE(desc, nullptr);

        const uint32_t expectedData =
            static_cast<uint32_t>((bufBaseIOVA_ + i * kBufSize) & 0xFFFFFFFFu);
        EXPECT_EQ(desc->dataAddress, expectedData);

        const size_t nextIndex = (i + 1) % kNum;
        const uint32_t expectedNextDescAddr =
            static_cast<uint32_t>((descBaseIOVA_ + nextIndex * sizeof(Async::HW::OHCIDescriptor)) & 0xFFFFFFF0u);
        const uint32_t branchAddr = Async::HW::DecodeBranchPhys32_AR(desc->branchWord);
        EXPECT_EQ(branchAddr, expectedNextDescAddr);
        EXPECT_NE(desc->branchWord, 0u);
    }
}

} // namespace ASFW::Testing

