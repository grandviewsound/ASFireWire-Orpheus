// IsochRxDmaRing.hpp
// ASFW - Low-level OHCI IR DMA ring engine (generic, no audio semantics).
//
// Apple-fidelity layout (post-Path B, fix 78):
//   - 24 INPUT_MORE descriptors, 4 KB buffer each (mirrors AppleFWOHCI
//     MultiIsochReceiver::createInitialCommandElements: 24 elements, 0x1000-byte
//     IOMemoryBlock32 each, descriptor control word 0x280C1000).
//   - Chain-with-terminator topology: descriptor i->branchWord = (next_phys|1)
//     for i in [0, capacity-2], descriptor (capacity-1)->branchWord = 0.
//     Hardware halts at the terminator; software relinks consumed elements
//     onto the tail and pulses the ContextControl WAKE bit.
//   - Used in OHCI bufferFill mode: packets land contiguously across
//     descriptor boundaries and are extracted by PacketStreamParser.

#pragma once

#include "../Memory/IIsochDMAMemory.hpp"
#include "../../Shared/Rings/BufferRing.hpp"
#include "../../Hardware/OHCIDescriptors.hpp"
#include "../../Common/BarrierUtils.hpp"

#include <cstddef>
#include <cstdint>

namespace ASFW::Isoch::Rx {

class IsochRxDmaRing final {
public:
    using OHCIDescriptor = Async::HW::OHCIDescriptor;

    // A view of the bytes hardware has written into one descriptor since the
    // parser last consumed from it. After processing these bytes, the caller
    // must Recycle(descriptorIndex) to return the buffer to hardware (which
    // also pulses WAKE through the ContextControl in IsochReceiveContext).
    struct ByteSpan final {
        const uint8_t* bytes{nullptr};
        size_t length{0};
        uint32_t descriptorIndex{0};
        bool exhausted{false};   // true if buffer has reached resCount=0 (full)
    };

    [[nodiscard]] kern_return_t SetupRings(Memory::IIsochDMAMemory& dma,
                                           size_t numDescriptors,
                                           size_t maxPacketSizeBytes) noexcept;

    // Re-arm every descriptor's status word and reset the head/tail cursors.
    // Mirrors AppleFWOHCI MultiIsochReceiver::prepareElementListForStart.
    void ResetForStart() noexcept;

    [[nodiscard]] uint32_t InitialCommandPtrWord() const noexcept;

    // Dequeue the next chunk of fresh bytes from the head descriptor. Returns
    // nullopt when the head descriptor has produced nothing new since the last
    // call. Tracks an internal cursor so partial fills are returned
    // incrementally as hardware fills the buffer.
    [[nodiscard]] std::optional<ByteSpan> DequeueDescriptorBytes(Memory::IIsochDMAMemory& dma) noexcept;

    // Splice descriptor at `index` onto the tail of the chain:
    //   - re-arm its statusWord to (0 | reqCount)
    //   - set its branchWord = 0 (it becomes the new terminator)
    //   - patch the previous tail's branchWord = (this_desc.phys | 1)
    //   - update tail_ = index
    // Caller (IsochReceiveContext::Poll) must pulse the ContextControl WAKE
    // bit afterwards so the hardware notices the chain extension.
    [[nodiscard]] kern_return_t Recycle(uint32_t descriptorIndex,
                                        Memory::IIsochDMAMemory& dma) noexcept;

    // Debug/test helpers.
    [[nodiscard]] size_t Capacity() const noexcept { return bufferRing_.Capacity(); }
    [[nodiscard]] OHCIDescriptor* DescriptorAt(size_t index) noexcept { return bufferRing_.GetDescriptor(index); }
    [[nodiscard]] void* PayloadVA(size_t index) const noexcept { return bufferRing_.GetElementVA(index); }
    [[nodiscard]] uint32_t Descriptor0IOVA() const noexcept;
    [[nodiscard]] uint32_t HeadIndex() const noexcept { return head_; }
    [[nodiscard]] uint32_t TailIndex() const noexcept { return tail_; }

private:
    // Program every descriptor in INPUT_MORE chain-with-terminator form:
    //   control = INPUT_MORE | s=1 | key=0 | i=Never | b=Always | reqCount
    //   dataAddress = element[i] IOVA
    //   branchWord = (i < cap-1) ? next_phys|1 : 0
    //   statusWord = 0 << 16 | reqCount
    [[nodiscard]] kern_return_t ProgramDescriptors(uint16_t reqCount) noexcept;

    Shared::BufferRing bufferRing_{};
    size_t maxPacketSizeBytes_{0};
    uint32_t head_{0};                  // descriptor we are currently consuming bytes from
    uint32_t tail_{0};                  // last descriptor in the active chain (terminator)
    size_t headBytesConsumed_{0};       // bytes already returned from head descriptor
};

} // namespace ASFW::Isoch::Rx
