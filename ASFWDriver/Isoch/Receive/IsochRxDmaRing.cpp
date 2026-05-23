// IsochRxDmaRing.cpp

#include "IsochRxDmaRing.hpp"

#include "../../Hardware/OHCIConstants.hpp"
#include "../../Hardware/OHCIDescriptors.hpp"
#include "../../Logging/Logging.hpp"

#include <span>

namespace ASFW::Isoch::Rx {

kern_return_t IsochRxDmaRing::ProgramDescriptors(uint16_t reqCount) noexcept {
    const uint32_t cap = static_cast<uint32_t>(bufferRing_.Capacity());
    if (cap == 0) {
        return kIOReturnInternalError;
    }

    for (uint32_t i = 0; i < cap; ++i) {
        auto* desc = bufferRing_.GetDescriptor(i);
        if (desc == nullptr) {
            return kIOReturnInternalError;
        }

        // Match Apple's IR descriptor control word 0x280C1000 exactly:
        //   cmd=INPUT_MORE, s=1, key=0, i=Never, b=Always, w=Never, reqCount=4096.
        // Apple sets `i` to Never on every descriptor and relies on the cycle
        // timer callback for completion polling. We do the same — interrupts
        // are wired separately at the IR-context level via the IsoRecvIntMask.
        uint32_t control = Async::HW::OHCIDescriptor::BuildControl({
            .reqCount = reqCount,
            .command = Async::HW::OHCIDescriptor::kCmdInputMore,
            .key = Async::HW::OHCIDescriptor::kKeyStandard,
            .interruptBits = Async::HW::OHCIDescriptor::kIntNever,
            .branchBits = Async::HW::OHCIDescriptor::kBranchAlways,
        });
        control |= (1u << (Async::HW::OHCIDescriptor::kStatusShift +
                           Async::HW::OHCIDescriptor::kControlHighShift));
        desc->control = control;

        const uint64_t dataIOVA = bufferRing_.GetElementIOVA(i);
        if (dataIOVA == 0 || dataIOVA > 0xFFFFFFFFULL) {
            return kIOReturnInternalError;
        }
        desc->dataAddress = static_cast<uint32_t>(dataIOVA);

        if (i + 1 < cap) {
            const uint64_t nextIOVA = bufferRing_.GetDescriptorIOVA(i + 1);
            if (nextIOVA == 0 || nextIOVA > 0xFFFFFFFFULL || (nextIOVA & 0xF) != 0) {
                return kIOReturnInternalError;
            }
            desc->branchWord = Async::HW::MakeBranchWordAR(static_cast<uint32_t>(nextIOVA), 1);
        } else {
            // Terminator. Hardware halts here; Recycle() will splice fresh
            // descriptors onto the tail and pulse WAKE.
            desc->branchWord = 0;
        }

        Async::HW::AR_init_status(*desc, reqCount);
    }

    head_ = 0;
    tail_ = cap - 1;
    headBytesConsumed_ = 0;

    return kIOReturnSuccess;
}

kern_return_t IsochRxDmaRing::SetupRings(Memory::IIsochDMAMemory& dma,
                                         size_t numDescriptors,
                                         size_t maxPacketSizeBytes) noexcept {
    if (numDescriptors == 0 || maxPacketSizeBytes == 0) {
        return kIOReturnBadArgument;
    }
    if (maxPacketSizeBytes > 0xFFFFu) {
        return kIOReturnBadArgument;
    }

    const uint16_t reqCount = static_cast<uint16_t>(maxPacketSizeBytes);

    // Allocate-once policy: IsochService keeps the IR context (and its
    // dedicated DMA slabs) alive across start/stop. Re-allocating on every
    // Configure() will exhaust the bump-pointer allocator and fail on the
    // second StartDevice. If we already have a ring, just reinitialize the
    // descriptor program and status words.
    if (bufferRing_.Capacity() != 0) {
        if (bufferRing_.Capacity() != numDescriptors || bufferRing_.BufferSize() != maxPacketSizeBytes) {
            ASFW_LOG(Isoch,
                     "IR: SetupRings reconfigure unsupported (have cap=%zu maxPkt=%zu, want cap=%zu maxPkt=%zu)",
                     bufferRing_.Capacity(),
                     bufferRing_.BufferSize(),
                     numDescriptors,
                     maxPacketSizeBytes);
            return kIOReturnUnsupported;
        }

        bufferRing_.BindDma(&dma);
        const auto status = ProgramDescriptors(reqCount);
        if (status != kIOReturnSuccess) {
            return status;
        }
        bufferRing_.PublishAllDescriptorsOnce();
        maxPacketSizeBytes_ = maxPacketSizeBytes;
        return kIOReturnSuccess;
    }

    const size_t descriptorsSize = numDescriptors * sizeof(Async::HW::OHCIDescriptor);
    const size_t buffersSize = numDescriptors * maxPacketSizeBytes;

    auto descRegion = dma.AllocateDescriptor(descriptorsSize);
    if (!descRegion) {
        return kIOReturnNoMemory;
    }

    auto bufRegion = dma.AllocatePayloadBuffer(buffersSize);
    if (!bufRegion) {
        return kIOReturnNoMemory;
    }

    auto descSpan = std::span<Async::HW::OHCIDescriptor>(
        reinterpret_cast<Async::HW::OHCIDescriptor*>(descRegion->virtualBase),
        numDescriptors);
    auto bufSpan = std::span<uint8_t>(bufRegion->virtualBase, buffersSize);

    if (!bufferRing_.Initialize(descSpan, bufSpan, numDescriptors, maxPacketSizeBytes)) {
        return kIOReturnInternalError;
    }

    bufferRing_.BindDma(&dma);
    if (!bufferRing_.Finalize(descRegion->deviceBase, bufRegion->deviceBase)) {
        return kIOReturnInternalError;
    }

    const auto status = ProgramDescriptors(reqCount);
    if (status != kIOReturnSuccess) {
        return status;
    }
    bufferRing_.PublishAllDescriptorsOnce();
    maxPacketSizeBytes_ = maxPacketSizeBytes;
    return kIOReturnSuccess;
}

void IsochRxDmaRing::ResetForStart() noexcept {
    // Re-arm every descriptor's status word + rebuild chain-with-terminator.
    // After Stop, the chain may have been mutated by Recycle() so we restore
    // the canonical i->next|1 chain and a single terminator at capacity-1.
    if (maxPacketSizeBytes_ > 0) {
        (void)ProgramDescriptors(static_cast<uint16_t>(maxPacketSizeBytes_));
        bufferRing_.PublishAllDescriptorsOnce();
    }
}

uint32_t IsochRxDmaRing::Descriptor0IOVA() const noexcept {
    const uint64_t iova = bufferRing_.GetDescriptorIOVA(0);
    if (iova == 0 || iova > 0xFFFFFFFFULL) {
        return 0;
    }
    return static_cast<uint32_t>(iova);
}

uint32_t IsochRxDmaRing::InitialCommandPtrWord() const noexcept {
    const uint32_t base = Descriptor0IOVA();
    if (base == 0 || (base & 0xF) != 0) {
        return 0;
    }
    return base | 1u; // Z=1 (fetch 1 descriptor)
}

std::optional<IsochRxDmaRing::ByteSpan>
IsochRxDmaRing::DequeueDescriptorBytes(Memory::IIsochDMAMemory& dma) noexcept {
    const uint32_t cap = static_cast<uint32_t>(bufferRing_.Capacity());
    if (cap == 0 || maxPacketSizeBytes_ == 0) {
        return std::nullopt;
    }

    auto* desc = bufferRing_.GetDescriptor(head_);
    if (desc == nullptr) {
        return std::nullopt;
    }

    dma.FetchFromDevice(desc, sizeof(*desc));
    const uint16_t reqCount = static_cast<uint16_t>(maxPacketSizeBytes_);
    const uint16_t resCount = Async::HW::AR_resCount(*desc);
    if (resCount > reqCount) {
        return std::nullopt;
    }

    const size_t bytesWritten = static_cast<size_t>(reqCount - resCount);
    if (bytesWritten <= headBytesConsumed_) {
        return std::nullopt;
    }

    const size_t startOffset = headBytesConsumed_;
    const size_t newBytes = bytesWritten - startOffset;

    auto* baseVA = static_cast<uint8_t*>(bufferRing_.GetElementVA(head_));
    if (baseVA == nullptr) {
        return std::nullopt;
    }
    dma.FetchFromDevice(baseVA + startOffset, newBytes);

    headBytesConsumed_ = bytesWritten;

    ByteSpan span;
    span.bytes = baseVA + startOffset;
    span.length = newBytes;
    span.descriptorIndex = head_;
    span.exhausted = (resCount == 0);
    if (span.exhausted) {
        head_ = (head_ + 1) % cap;
        headBytesConsumed_ = 0;
    }
    return span;
}

kern_return_t IsochRxDmaRing::Recycle(uint32_t descriptorIndex,
                                      Memory::IIsochDMAMemory& dma) noexcept {
    const uint32_t cap = static_cast<uint32_t>(bufferRing_.Capacity());
    if (cap == 0 || descriptorIndex >= cap) {
        return kIOReturnBadArgument;
    }

    auto* desc = bufferRing_.GetDescriptor(descriptorIndex);
    auto* tailDesc = bufferRing_.GetDescriptor(tail_);
    if (desc == nullptr || tailDesc == nullptr) {
        return kIOReturnInternalError;
    }
    if (descriptorIndex == tail_) {
        // Already the terminator; nothing to splice.
        return kIOReturnSuccess;
    }

    const uint64_t descIOVA = bufferRing_.GetDescriptorIOVA(descriptorIndex);
    if (descIOVA == 0 || descIOVA > 0xFFFFFFFFULL || (descIOVA & 0xF) != 0) {
        return kIOReturnInternalError;
    }

    const uint16_t reqCount = static_cast<uint16_t>(maxPacketSizeBytes_);

    Async::HW::AR_init_status(*desc, reqCount);
    desc->branchWord = 0; // becomes new terminator
    dma.PublishToDevice(desc, sizeof(*desc));

    tailDesc->branchWord = Async::HW::MakeBranchWordAR(static_cast<uint32_t>(descIOVA), 1);
    dma.PublishToDevice(tailDesc, sizeof(*tailDesc));

    ::ASFW::Driver::WriteBarrier();

    tail_ = descriptorIndex;
    return kIOReturnSuccess;
}

} // namespace ASFW::Isoch::Rx
