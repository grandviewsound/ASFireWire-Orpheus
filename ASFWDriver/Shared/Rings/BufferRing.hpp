#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <memory>

#include <DriverKit/IOLib.h>

#include "../../Hardware/HWNamespaceAlias.hpp"

namespace ASFW::Shared {

class IDMAMemory;

struct FilledBufferInfo {
    void* virtualAddress;
    size_t startOffset;
    size_t bytesFilled;
    size_t descriptorIndex;
    // True iff Dequeue's auto-recycle path fired this call (hardware advanced
    // past the previous buffer; descriptor was reset and republished but WAKE
    // has NOT been written). The caller (ARContextBase::Dequeue) must write
    // the OHCI ContextControl WAKE bit while still holding the dequeue lock.
    // Without this, the controller can stay paused at a buffer boundary and
    // silently drop subsequent packets. See fix64b root-cause memo.
    bool autoRecycledPrev{false};
    // True iff virtualAddress points into the per-ring scratch buffer, not
    // the DMA region. Set when Dequeue's auto-recycle path copied a straddle
    // window (old buffer tail + new buffer head) into scratch to preserve the
    // straddling packet's bytes before resetting the old descriptor. Parser
    // walks this as a normal byte stream; no special handling needed. See
    // fix66-test-results memo for the straddling-packet root cause.
    bool isStraddleScratch{false};
};

class BufferRing {
public:
    BufferRing() = default;
    ~BufferRing() = default;
    [[nodiscard]] bool Initialize(std::span<HW::OHCIDescriptor> descriptors, std::span<uint8_t> buffers, size_t bufferCount, size_t bufferSize) noexcept;
    // Optional: attach a per-ring scratch buffer used by Dequeue's
    // auto-recycle path to preserve straddling packets (old buffer tail +
    // new buffer head). Recommended size: 8 KB (matches AppleFWOHCI's
    // IOMalloc(0x2000u) per-context scratch). If not attached, straddling
    // packets at buffer boundaries will be lost.
    void AttachScratch(std::span<uint8_t> scratch) noexcept { scratch_ = scratch; }
    [[nodiscard]] bool Finalize(uint64_t descriptorsPhysBase, uint64_t buffersPhysBase) noexcept;
    [[nodiscard]] std::optional<FilledBufferInfo> Dequeue() noexcept;
    [[nodiscard]] kern_return_t Recycle(size_t index) noexcept;
    [[nodiscard]] void* GetBufferAddress(size_t index) const noexcept;
    [[nodiscard]] size_t Head() const noexcept { return head_; }
    [[nodiscard]] size_t BufferCount() const noexcept { return bufferCount_; }
    [[nodiscard]] size_t BufferSize() const noexcept { return bufferSize_; }
    [[nodiscard]] uint32_t CommandPtrWord() const noexcept;
    void BindDma(IDMAMemory* dma) noexcept;
    void PublishAllDescriptorsOnce() noexcept;
    [[nodiscard]] void* DescriptorBaseVA() noexcept { return descriptors_.data(); }
    [[nodiscard]] const void* DescriptorBaseVA() const noexcept { return descriptors_.data(); }
    
    // Low-level access for custom programming (Isoch, etc.)
    [[nodiscard]] HW::OHCIDescriptor* GetDescriptor(size_t index) noexcept {
        if (index >= bufferCount_) return nullptr;
        return &descriptors_[index];
    }
    
    [[nodiscard]] uint64_t GetElementIOVA(size_t index) const noexcept {
        if (index >= bufferCount_) return 0;
        return bufIOVABase_ + (index * bufferSize_);
    }

    [[nodiscard]] uint64_t GetDescriptorIOVA(size_t index) const noexcept {
        if (index >= bufferCount_) return 0;
        return descIOVABase_ + (index * sizeof(HW::OHCIDescriptor));
    }

    [[nodiscard]] void* GetElementVA(size_t index) const noexcept {
        return GetBufferAddress(index);
    }
    
    [[nodiscard]] size_t Capacity() const noexcept { return bufferCount_; }
    BufferRing(const BufferRing&) = delete;
    BufferRing& operator=(const BufferRing&) = delete;
private:
    std::span<HW::OHCIDescriptor> descriptors_;
    std::span<uint8_t> buffers_;
    size_t bufferCount_{0};
    size_t bufferSize_{0};
    size_t head_{0};
    size_t last_dequeued_bytes_{0};
    uint32_t descIOVABase_{0};
    uint32_t bufIOVABase_{0};
    IDMAMemory* dma_{nullptr};
    std::span<uint8_t> scratch_{};  // optional per-ring straddle scratch
};

} // namespace ASFW::Shared
