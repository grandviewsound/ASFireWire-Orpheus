// IsochTransmitContextTests.cpp
// ASFW - Host-safe unit tests for transmit packetization behavior
//
// NOTE:
// Full IsochTransmitContext runtime tests require DriverKit DMA/hardware wiring.
// In ASFW_HOST_TEST, we validate the same cadence/DBC/underrun behavior through
// PacketAssembler, plus a lightweight API state smoke test.

#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <vector>

#include "../ASFWDriver/Isoch/Transmit/IsochTransmitContext.hpp"
#include "../ASFWDriver/Isoch/Config/AudioConstants.hpp"
#include "../ASFWDriver/Isoch/Config/AudioTxProfiles.hpp"
#include "../ASFWDriver/Isoch/Memory/IsochDMAMemoryManager.hpp"
#include "../ASFWDriver/Shared/TxSharedQueue.hpp"
#include "../ASFWDriver/Hardware/HardwareInterface.hpp"

using namespace ASFW::Isoch;
using namespace ASFW::Encoding;

TEST(IsochTransmitContext, InitialStateIsUnconfigured) {
    IsochTransmitContext ctx;
    EXPECT_EQ(ctx.GetState(), ITState::Unconfigured);
}

TEST(IsochTransmitContext, ConfigureSucceedsWithQueueChannelMetadata) {
    constexpr uint32_t kQueueChannels = 6;
    constexpr uint32_t kCapacityFrames = 256;
    const uint64_t bytes = ASFW::Shared::TxSharedQueueSPSC::RequiredBytes(kCapacityFrames, kQueueChannels);
    std::vector<uint8_t> storage(bytes);

    ASSERT_TRUE(ASFW::Shared::TxSharedQueueSPSC::InitializeInPlace(storage.data(),
                                                                    bytes,
                                                                    kCapacityFrames,
                                                                    kQueueChannels));

    IsochTransmitContext ctx;
    ctx.SetSharedTxQueue(storage.data(), bytes);

    EXPECT_EQ(ctx.Configure(/*channel=*/0, /*sid=*/0x3F, /*streamModeRaw=*/0, /*requestedChannels=*/kQueueChannels),
              kIOReturnSuccess);
    EXPECT_EQ(ctx.GetState(), ITState::Configured);
}

TEST(TxSharedQueueSPSC, SupportsMaxPcmChannels) {
    constexpr uint32_t kQueueChannels = ASFW::Isoch::Config::kMaxPcmChannels;
    constexpr uint32_t kCapacityFrames = 256;
    const uint64_t bytes = ASFW::Shared::TxSharedQueueSPSC::RequiredBytes(kCapacityFrames, kQueueChannels);
    std::vector<uint8_t> storage(bytes);

    EXPECT_TRUE(ASFW::Shared::TxSharedQueueSPSC::InitializeInPlace(storage.data(),
                                                                    bytes,
                                                                    kCapacityFrames,
                                                                    kQueueChannels));
}

TEST(IsochTransmitContext, ConfigureFailsOnRequestedChannelMismatch) {
    constexpr uint32_t kQueueChannels = 4;
    constexpr uint32_t kRequestedChannels = 6;
    constexpr uint32_t kCapacityFrames = 256;
    const uint64_t bytes = ASFW::Shared::TxSharedQueueSPSC::RequiredBytes(kCapacityFrames, kQueueChannels);
    std::vector<uint8_t> storage(bytes);

    ASSERT_TRUE(ASFW::Shared::TxSharedQueueSPSC::InitializeInPlace(storage.data(),
                                                                    bytes,
                                                                    kCapacityFrames,
                                                                    kQueueChannels));

    IsochTransmitContext ctx;
    ctx.SetSharedTxQueue(storage.data(), bytes);

    EXPECT_EQ(ctx.Configure(/*channel=*/0, /*sid=*/0x3F, /*streamModeRaw=*/0, /*requestedChannels=*/kRequestedChannels),
              kIOReturnBadArgument);
}

TEST(IsochTransmitContext, ConfigureFailsOnInvalidQueueChannelValue) {
    constexpr uint32_t kQueueChannels = 2;
    constexpr uint32_t kCapacityFrames = 256;
    const uint64_t bytes = ASFW::Shared::TxSharedQueueSPSC::RequiredBytes(kCapacityFrames, kQueueChannels);
    std::vector<uint8_t> storage(bytes);

    ASSERT_TRUE(ASFW::Shared::TxSharedQueueSPSC::InitializeInPlace(storage.data(),
                                                                    bytes,
                                                                    kCapacityFrames,
                                                                    kQueueChannels));

    IsochTransmitContext ctx;
    ctx.SetSharedTxQueue(storage.data(), bytes);

    auto* hdr = reinterpret_cast<ASFW::Shared::TxQueueHeader*>(storage.data());
    hdr->channels = 0;

    EXPECT_EQ(ctx.Configure(/*channel=*/0, /*sid=*/0x3F, /*streamModeRaw=*/0, /*requestedChannels=*/kQueueChannels),
              kIOReturnBadArgument);
}

TEST(IsochAudioTxPipeline, ActivatesAppleOutputChannelPositionMapWhenValid) {
    constexpr uint32_t kQueueChannels = 4;
    constexpr uint32_t kCapacityFrames = 256;
    const uint64_t bytes = ASFW::Shared::TxSharedQueueSPSC::RequiredBytes(kCapacityFrames, kQueueChannels);
    std::vector<uint8_t> storage(bytes);

    ASSERT_TRUE(ASFW::Shared::TxSharedQueueSPSC::InitializeInPlace(storage.data(),
                                                                    bytes,
                                                                    kCapacityFrames,
                                                                    kQueueChannels));

    IsochAudioTxPipeline pipeline;
    pipeline.SetSharedTxQueue(storage.data(), bytes);
    const std::array<uint8_t, kQueueChannels> map = {1, 0, 3, 2};
    pipeline.SetOutputChannelMap(map.data(), static_cast<uint32_t>(map.size()));

    ASSERT_EQ(pipeline.Configure(/*sid=*/0x3F,
                                 /*streamModeRaw=*/0,
                                 /*requestedChannels=*/kQueueChannels,
                                 /*requestedAm824Slots=*/kQueueChannels + 1),
              kIOReturnSuccess);
    EXPECT_TRUE(pipeline.IsOutputChannelMapActive());
    EXPECT_EQ(pipeline.OutputChannelSlotForChannel(0), 1);
    EXPECT_EQ(pipeline.OutputChannelSlotForChannel(1), 0);
    EXPECT_EQ(pipeline.OutputChannelSlotForChannel(2), 3);
    EXPECT_EQ(pipeline.OutputChannelSlotForChannel(3), 2);
}

TEST(IsochAudioTxPipeline, IgnoresInvalidAppleOutputChannelPositionMap) {
    constexpr uint32_t kQueueChannels = 4;
    constexpr uint32_t kCapacityFrames = 256;
    const uint64_t bytes = ASFW::Shared::TxSharedQueueSPSC::RequiredBytes(kCapacityFrames, kQueueChannels);
    std::vector<uint8_t> storage(bytes);

    ASSERT_TRUE(ASFW::Shared::TxSharedQueueSPSC::InitializeInPlace(storage.data(),
                                                                    bytes,
                                                                    kCapacityFrames,
                                                                    kQueueChannels));

    IsochAudioTxPipeline pipeline;
    pipeline.SetSharedTxQueue(storage.data(), bytes);
    const std::array<uint8_t, kQueueChannels> duplicateSlotMap = {0, 1, 1, 3};
    pipeline.SetOutputChannelMap(duplicateSlotMap.data(),
                                 static_cast<uint32_t>(duplicateSlotMap.size()));

    ASSERT_EQ(pipeline.Configure(/*sid=*/0x3F,
                                 /*streamModeRaw=*/0,
                                 /*requestedChannels=*/kQueueChannels,
                                 /*requestedAm824Slots=*/kQueueChannels + 1),
              kIOReturnSuccess);
    EXPECT_FALSE(pipeline.IsOutputChannelMapActive());
    EXPECT_EQ(pipeline.OutputChannelSlotForChannel(2), 2);
}

TEST(IsochTransmitContext, BlockingCadenceCountsMatchOneSecond) {
    PacketAssembler assembler(2, 0x3F);

    uint64_t dataPackets = 0;
    uint64_t noDataPackets = 0;

    // 1 second on FireWire bus cadence = 8000 cycles.
    for (int i = 0; i < 8000; ++i) {
        auto pkt = assembler.assembleNext(0x1234);
        if (pkt.isData) {
            ++dataPackets;
        } else {
            ++noDataPackets;
        }
    }

    EXPECT_EQ(dataPackets, 6000);
    EXPECT_EQ(noDataPackets, 2000);
}

TEST(IsochTransmitContext, CadenceOrderingTrace32Packets) {
    // Verify exact sequence: N-D-D-D-N-D-D-D repeated 4 times.
    std::array<bool, 32> expectedIsData = {
        false, true, true, true, false, true, true, true,
        false, true, true, true, false, true, true, true,
        false, true, true, true, false, true, true, true,
        false, true, true, true, false, true, true, true
    };

    PacketAssembler assembler(2, 0x3F);

    for (int i = 0; i < 32; ++i) {
        auto pkt = assembler.assembleNext(0xFFFF);
        EXPECT_EQ(pkt.isData, expectedIsData[i])
            << "Packet " << i << " expected "
            << (expectedIsData[i] ? "DATA" : "NO-DATA")
            << " but got " << (pkt.isData ? "DATA" : "NO-DATA");
    }
}

TEST(IsochTransmitContext, DBCNoDataBoundary) {
    // Per IEC 61883-1 blocking mode:
    // - NO-DATA carries the DBC of the next DATA packet
    // - Next DATA packet uses the same DBC value
    // - DATA increments DBC by sample count (8)
    PacketAssembler assembler(2, 0x3F);

    auto pkt0 = assembler.assembleNext(0xFFFF); // NO-DATA
    EXPECT_FALSE(pkt0.isData);

    auto pkt1 = assembler.assembleNext(0x1234); // DATA
    EXPECT_TRUE(pkt1.isData);
    EXPECT_EQ(pkt0.dbc, pkt1.dbc);

    auto pkt2 = assembler.assembleNext(0x1234); // DATA
    EXPECT_TRUE(pkt2.isData);
    EXPECT_EQ(pkt2.dbc, static_cast<uint8_t>((pkt1.dbc + 8) & 0xFF));

    auto pkt3 = assembler.assembleNext(0x1234); // DATA
    EXPECT_TRUE(pkt3.isData);
    EXPECT_EQ(pkt3.dbc, static_cast<uint8_t>((pkt2.dbc + 8) & 0xFF));

    auto pkt4 = assembler.assembleNext(0xFFFF); // NO-DATA
    EXPECT_FALSE(pkt4.isData);

    auto pkt5 = assembler.assembleNext(0x1234); // DATA
    EXPECT_TRUE(pkt5.isData);
    EXPECT_EQ(pkt4.dbc, pkt5.dbc);
    EXPECT_EQ(pkt5.dbc, static_cast<uint8_t>((pkt3.dbc + 8) & 0xFF));
}

TEST(IsochTransmitContext, UnderrunCountsOnEmptyBuffer) {
    PacketAssembler assembler(2, 0x3F);

    // One cadence group: N-D-D-D-N-D-D-D (6 DATA reads, all underrun on empty ring).
    for (int i = 0; i < 8; ++i) {
        assembler.assembleNext(0x1234);
    }

    EXPECT_EQ(assembler.underrunCount(), 6);
}

TEST(IsochTransmitContext, NoUnderrunsWithPrefilledBuffer) {
    PacketAssembler assembler(2, 0x3F);

    std::array<int32_t, 512 * 2> audioData{};
    for (size_t i = 0; i < audioData.size(); ++i) {
        audioData[i] = static_cast<int32_t>(i);
    }
    assembler.ringBuffer().write(audioData.data(), 512);

    for (int i = 0; i < 8; ++i) {
        assembler.assembleNext(0x1234);
    }

    // 8 packets in blocking mode => 6 DATA packets => 6 * 8 = 48 frames consumed.
    EXPECT_EQ(assembler.underrunCount(), 0);
    EXPECT_EQ(assembler.bufferFillLevel(), 512 - 48);
}

// ============================================================================
// Inject priming gate (standing cushion, 2026-06-10)
// ============================================================================
//
// InjectNearHw must not drain the assembler ring until it holds the adaptive
// fill target of REAL frames; until then the DMA ring keeps its Phase-2 silent
// CIP packets. When the ring later runs dry, the gate re-arms so the cushion is
// rebuilt instead of the level settling at one CoreAudio burst (the 22-17-26
// underrun garble).

namespace {

constexpr uint32_t kGateTestChannels = 2;
constexpr uint32_t kGateTestQueueCapacity = 2048;  // power of two, > target 768

struct TxInjectHarness {
    ::ASFW::Driver::HardwareInterface hw;
    std::shared_ptr<Memory::IsochDMAMemoryManager> mem;
    Tx::IsochTxDmaRing ring;
    IsochAudioTxPipeline pipeline;
    std::vector<uint8_t> queueStorage;
    ASFW::Shared::TxSharedQueueSPSC producer;

    bool Setup() {
        Memory::IsochMemoryConfig config;
        config.numDescriptors = 1024;
        config.packetSizeBytes = 1024;
        config.descriptorAlignment = 4096;   // TX slab requires 4K-aligned descriptor base
        config.payloadPageAlignment = 4096;

        mem = Memory::IsochDMAMemoryManager::Create(config);
        if (!mem || !mem->Initialize(hw)) return false;
        if (ring.SetupRings(*mem) != kIOReturnSuccess) return false;

        const uint64_t bytes = ASFW::Shared::TxSharedQueueSPSC::RequiredBytes(
            kGateTestQueueCapacity, kGateTestChannels);
        queueStorage.resize(bytes);
        if (!ASFW::Shared::TxSharedQueueSPSC::InitializeInPlace(
                queueStorage.data(), bytes, kGateTestQueueCapacity, kGateTestChannels)) {
            return false;
        }

        pipeline.SetSharedTxQueue(queueStorage.data(), bytes);
        if (pipeline.Configure(/*sid=*/0x3F, /*streamModeRaw=*/1,
                               kGateTestChannels, /*requestedAm824Slots=*/0) != kIOReturnSuccess) {
            return false;
        }
        if (!producer.Attach(queueStorage.data(), bytes)) return false;

        ring.ResetForStart();
        pipeline.ResetForStart();
        return ring.Prime(pipeline).packetsAssembled == Tx::Layout::kNumPackets;
    }

    // CoreAudio-producer stand-in: nonzero interleaved frames into the shared queue.
    uint32_t Produce(uint32_t frames) {
        std::vector<int32_t> buf(static_cast<size_t>(frames) * kGateTestChannels, 0x00123400);
        return producer.Write(buf.data(), frames);
    }
};

} // namespace

TEST(IsochInjectPrimingGate, PrePrimeDoesNotSilencePad) {
    TxInjectHarness h;
    ASSERT_TRUE(h.Setup());

    // Empty queue at Start: pre-prime must leave the ring EMPTY (the old one-shot
    // silence pad would have filled it to 768 and defeated the gate).
    h.pipeline.PrePrimeFromSharedQueue();
    EXPECT_EQ(h.pipeline.BufferFillLevel(), 0u);
    EXPECT_TRUE(h.pipeline.IsInjectPriming());

    // Frames already queued at Start transfer as REAL frames (count toward gate).
    ASSERT_EQ(h.Produce(100), 100u);
    h.pipeline.PrePrimeFromSharedQueue();
    EXPECT_EQ(h.pipeline.BufferFillLevel(), 100u);
}

TEST(IsochInjectPrimingGate, HoldsBelowTargetThenOpensAtTarget) {
    TxInjectHarness h;
    ASSERT_TRUE(h.Setup());
    const uint32_t target = Config::kTxBufferProfile.legacyRbTargetFrames;

    // Below target: inject must HOLD — no ring drain, no frames read.
    ASSERT_EQ(h.Produce(256), 256u);
    h.pipeline.OnRefillTickPreHW();
    ASSERT_EQ(h.pipeline.BufferFillLevel(), 256u);

    h.pipeline.InjectNearHw(0, h.ring.Slab());
    EXPECT_TRUE(h.pipeline.IsInjectPriming());
    EXPECT_EQ(h.pipeline.BufferFillLevel(), 256u);
    EXPECT_EQ(h.pipeline.RTCounters().injectFramesRead.load(), 0u);

    // At target: the gate opens and injection drains real frames. (Advance the
    // HW index past the held call's cursor so the window exposes new slots.)
    ASSERT_EQ(h.Produce(target - 256), target - 256);
    h.pipeline.OnRefillTickPreHW();
    ASSERT_EQ(h.pipeline.BufferFillLevel(), target);

    h.pipeline.InjectNearHw(8, h.ring.Slab());
    EXPECT_FALSE(h.pipeline.IsInjectPriming());
    EXPECT_LT(h.pipeline.BufferFillLevel(), target);
    EXPECT_GT(h.pipeline.RTCounters().injectFramesRead.load(), 0u);
}

TEST(IsochInjectPrimingGate, RearmsWhenRingRunsDryAndRecompletes) {
    TxInjectHarness h;
    ASSERT_TRUE(h.Setup());
    const uint32_t target = Config::kTxBufferProfile.legacyRbTargetFrames;

    ASSERT_EQ(h.Produce(target), target);
    h.pipeline.OnRefillTickPreHW();
    h.pipeline.InjectNearHw(0, h.ring.Slab());
    ASSERT_FALSE(h.pipeline.IsInjectPriming());

    // Producer stalls; HW keeps consuming. Walk hwPacketIndex forward so each
    // call drains the next window slice until the ring runs dry -> re-arm.
    uint32_t hwIdx = 0;
    bool rearmed = false;
    for (int i = 0; i < 64 && !rearmed; ++i) {
        hwIdx = (hwIdx + 8) % Tx::Layout::kNumPackets;
        h.pipeline.InjectNearHw(hwIdx, h.ring.Slab());
        rearmed = h.pipeline.IsInjectPriming();
    }
    EXPECT_TRUE(rearmed);
    EXPECT_GE(h.pipeline.RTCounters().injectPrimingRearms.load(), 1u);

    // While re-armed and below target, further calls must not drain.
    const uint32_t fillAfterRearm = h.pipeline.BufferFillLevel();
    hwIdx = (hwIdx + 8) % Tx::Layout::kNumPackets;
    h.pipeline.InjectNearHw(hwIdx, h.ring.Slab());
    EXPECT_TRUE(h.pipeline.IsInjectPriming());
    EXPECT_EQ(h.pipeline.BufferFillLevel(), fillAfterRearm);

    // Producer resumes: once the ring re-banks the target, the gate re-opens.
    ASSERT_EQ(h.Produce(target), target);
    h.pipeline.OnRefillTickPreHW();
    while (h.pipeline.BufferFillLevel() < target) {
        ASSERT_GT(h.Produce(target), 0u);
        h.pipeline.OnRefillTickPreHW();
    }
    hwIdx = (hwIdx + 8) % Tx::Layout::kNumPackets;
    h.pipeline.InjectNearHw(hwIdx, h.ring.Slab());
    EXPECT_FALSE(h.pipeline.IsInjectPriming());
}
