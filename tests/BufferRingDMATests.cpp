#include <gtest/gtest.h>

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

