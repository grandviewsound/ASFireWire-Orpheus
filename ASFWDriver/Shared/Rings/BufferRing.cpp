#include "BufferRing.hpp"

#include <atomic>
#include <cstring>

#include "../../Common/BarrierUtils.hpp"
#include "../Memory/DMAMemoryManager.hpp"
#include "../Memory/IDMAMemory.hpp"
#include "../../Logging/Logging.hpp"
#include "../../Logging/LogConfig.hpp"

namespace ASFW::Shared {

bool BufferRing::Initialize(std::span<HW::OHCIDescriptor> descriptors, std::span<uint8_t> buffers, size_t bufferCount, size_t bufferSize) noexcept {
    if (descriptors.empty() || buffers.empty()) {
        ASFW_LOG(Async, "BufferRing::Initialize: empty storage");
        return false;
    }
    if (descriptors.size() != bufferCount) {
        ASFW_LOG(Async, "BufferRing::Initialize: descriptor count %zu != buffer count %zu", descriptors.size(), bufferCount);
        return false;
    }
    if (buffers.size() < bufferCount * bufferSize) {
        ASFW_LOG(Async, "BufferRing::Initialize: buffer storage too small (%zu < %zu)", buffers.size(), bufferCount * bufferSize);
        return false;
    }
    if (reinterpret_cast<uintptr_t>(descriptors.data()) % 16 != 0) {
        ASFW_LOG(Async, "BufferRing::Initialize: descriptors not 16-byte aligned");
        return false;
    }
    descriptors_ = descriptors;
    buffers_ = buffers;
    bufferCount_ = bufferCount;
    bufferSize_ = bufferSize;
    head_ = 0;
    for (size_t i = 0; i < bufferCount; ++i) {
        auto& desc = descriptors_[i];
        desc = HW::OHCIDescriptor{};
        constexpr uint32_t kCmdInputMore = HW::OHCIDescriptor::kCmdInputMore;
        constexpr uint32_t kKeyStandard = HW::OHCIDescriptor::kKeyStandard;
        constexpr uint32_t kS = 1u;  
        constexpr uint32_t kIntAlways = HW::OHCIDescriptor::kIntAlways;
        constexpr uint32_t kBranchAlways = HW::OHCIDescriptor::kBranchAlways;
        desc.control = (kCmdInputMore << 28) | (kKeyStandard << 25) | (kS << 24) | (kIntAlways << 22) | (kBranchAlways << 20) | static_cast<uint32_t>(bufferSize);
        desc.dataAddress = static_cast<uint32_t>(i * bufferSize);
        size_t nextIndex = (i + 1) % bufferCount;
        desc.branchWord = (1u << 28) | (static_cast<uint32_t>(nextIndex) << 4);
        HW::AR_init_status(desc, static_cast<uint16_t>(bufferSize));
    }
    ASFW_LOG(Async, "BufferRing initialized: %zu buffers x %zu bytes", bufferCount, bufferSize);
    return true;
}

bool BufferRing::Finalize(uint64_t descriptorsIOVABase, uint64_t buffersIOVABase) noexcept {
    if (descriptors_.empty() || buffers_.empty() || bufferCount_ == 0 || bufferSize_ == 0) {
        ASFW_LOG(Async, "BufferRing::Finalize: ring not initialized");
        return false;
    }
    if ((descriptorsIOVABase & 0xFULL) != 0 || (buffersIOVABase & 0xFULL) != 0) {
        ASFW_LOG(Async, "BufferRing::Finalize: device bases not 16-byte aligned (desc=0x%llx buf=0x%llx)", descriptorsIOVABase, buffersIOVABase);
        return false;
    }
    for (size_t i = 0; i < bufferCount_; ++i) {
        auto& desc = descriptors_[i];
        const uint64_t dataIOVA = buffersIOVABase + static_cast<uint64_t>(i) * bufferSize_;
        if (dataIOVA > 0xFFFFFFFFu) {
            ASFW_LOG(Async, "BufferRing::Finalize: buffer IOVA out of range (index=%zu iova=0x%llx)", i, dataIOVA);
            return false;
        }
        desc.dataAddress = static_cast<uint32_t>(dataIOVA);
        const size_t nextIndex = (i + 1) % bufferCount_;
        const uint64_t nextDescIOVA = descriptorsIOVABase + static_cast<uint64_t>(nextIndex) * sizeof(HW::OHCIDescriptor);
        const uint32_t branchWord = HW::MakeBranchWordAR(nextDescIOVA, /*continueFlag=*/true);
        if (branchWord == 0) {
            ASFW_LOG(Async, "BufferRing::Finalize: invalid branchWord for index %zu (nextIOVA=0x%llx)", i, nextDescIOVA);
            return false;
        }
        desc.branchWord = branchWord;
    }
    ASFW_LOG(Async, "BufferRing finalized: descIOVA=0x%llx bufIOVA=0x%llx buffers=%zu", descriptorsIOVABase, buffersIOVABase, bufferCount_);
    descIOVABase_ = static_cast<uint32_t>(descriptorsIOVABase & 0xFFFFFFFFu);
    bufIOVABase_ = static_cast<uint32_t>(buffersIOVABase & 0xFFFFFFFFu);
    return true;
}

std::optional<FilledBufferInfo> BufferRing::Dequeue() noexcept {
    if (descriptors_.empty()) {
        return std::nullopt;
    }

    size_t index = head_;
    bool autoRecycledPrev = false;

    // CRITICAL: Auto-recycling logic for AR DMA stream semantics
    // Per OHCI §3.3, §8.4.2 bufferFill mode: hardware fills current buffer until exhausted,
    // then advances to next. We detect this and recycle automatically.

    // Check if next descriptor has data (hardware advanced)
    const size_t next_index = (index + 1) % bufferCount_;
    auto& next_desc = descriptors_[next_index];

    if (dma_) {
        dma_->FetchFromDevice(&next_desc, sizeof(next_desc));
    }

    const uint16_t next_resCount = HW::AR_resCount(next_desc);
    const uint16_t next_reqCount = static_cast<uint16_t>(next_desc.control & 0xFFFF);

    // If next buffer has data, hardware has moved to it.
    // STRADDLE HANDLING (fix67, per AppleFWOHCI): before the old descriptor is
    // reset, the old buffer's tail may contain the HEAD of a packet whose
    // payload lies in the new buffer. If we reset + republish the old
    // descriptor without preserving those bytes, hardware reuses the old
    // buffer for fresh packets and the straddling packet's head is lost.
    // Fix66 HW test (console log fix 66.txt) captured this exact event: a
    // 24-byte straddle at buffer[1]+0 whose FCP response was in the discarded
    // old-buffer tail, producing tCode=0xF at the parser.
    //
    // Apple's getPacket/getQuadlet copy [old_tail + new_head] into an 8 KB
    // scratch (IOMalloc(0x2000u)) and return scratch to the parser. We mirror
    // that shape here. Buffers in our arch are VA-contiguous so direct reads
    // would also work, but the scratch copy is race-safe against hardware
    // reusing the old buffer the instant we publish the reset.
    if (next_resCount != next_reqCount) {
        auto& desc_to_recycle = descriptors_[index];
        const uint16_t reqCount_recycle = static_cast<uint16_t>(desc_to_recycle.control & 0xFFFF);

        // Old buffer's new tail bytes (those the parser hasn't returned yet).
        // Auto-recycle fires when HW has moved on, so total_in_old ≈ reqCount.
        const size_t total_in_old = reqCount_recycle;
        const size_t old_tail_len =
            (total_in_old > last_dequeued_bytes_) ? (total_in_old - last_dequeued_bytes_) : 0;

        // New buffer bytes already written (may grow; this is a snapshot).
        const size_t new_head_len = (next_reqCount > next_resCount)
            ? static_cast<size_t>(next_reqCount - next_resCount) : 0;

        ASFW_LOG_V0(Async,
                    "🔄 BufferRing::Dequeue: HW advanced to buffer[%zu] (resCount=%u/%u). "
                    "Auto-recycling buffer[%zu] (prev last_dequeued=%zu old_tail=%zu new_head=%zu)",
                    next_index, next_resCount, next_reqCount, index,
                    last_dequeued_bytes_, old_tail_len, new_head_len);

        if (!scratch_.empty() && (old_tail_len > 0 || new_head_len > 0)) {
            const size_t scratchCap = scratch_.size();
            const size_t copyOld = (old_tail_len < scratchCap) ? old_tail_len : scratchCap;
            const size_t remainAfterOld = scratchCap - copyOld;
            const size_t copyNew = (new_head_len < remainAfterOld) ? new_head_len : remainAfterOld;

            // Invalidate CPU cache for the source ranges before memcpy — these
            // were written by DMA and may not be visible to the CPU yet.
            auto* old_vaddr = static_cast<uint8_t*>(GetBufferAddress(index));
            auto* new_vaddr = static_cast<uint8_t*>(GetBufferAddress(next_index));
            if (dma_) {
                if (copyOld > 0 && old_vaddr) {
                    dma_->FetchFromDevice(old_vaddr + last_dequeued_bytes_, copyOld);
                }
                if (copyNew > 0 && new_vaddr) {
                    dma_->FetchFromDevice(new_vaddr, copyNew);
                }
            }
            // Source is a cache-inhibited DMA mapping (Device memory on arm64),
            // which rejects unaligned NEON vector loads — std::memcpy compiles
            // to _platform_memmove, which uses LDP Q0,Q1. That raises
            // EXC_ARM_DA_ALIGN on any copy ≥16B at a non-16-aligned offset.
            // Mirror the scalar volatile-store loop used in
            // DMAMemoryManager.cpp:305 for the same reason, on the read side.
            if (copyOld > 0 && old_vaddr) {
                auto* src = reinterpret_cast<volatile const uint8_t*>(old_vaddr + last_dequeued_bytes_);
                auto* dst = scratch_.data();
                for (size_t i = 0; i < copyOld; ++i) {
                    dst[i] = src[i];
                }
            }
            if (copyNew > 0 && new_vaddr) {
                auto* src = reinterpret_cast<volatile const uint8_t*>(new_vaddr);
                auto* dst = scratch_.data() + copyOld;
                for (size_t i = 0; i < copyNew; ++i) {
                    dst[i] = src[i];
                }
            }

            // Reset old descriptor and publish AFTER the copy — hardware may
            // now reuse the old buffer without losing straddle data.
            HW::AR_init_status(desc_to_recycle, reqCount_recycle);
            if (dma_) {
                dma_->PublishToDevice(&desc_to_recycle, sizeof(desc_to_recycle));
            }
            Driver::WriteBarrier();

            // Advance head. The new buffer's first `copyNew` bytes were just
            // delivered via scratch — next Dequeue resumes parsing from there.
            head_ = next_index;
            last_dequeued_bytes_ = copyNew;

            ASFW_LOG_V0(Async,
                        "♻️  BufferRing: Straddle-scratch: %zu old_tail + %zu new_head = %zu bytes. "
                        "head_→%zu, last_dequeued_bytes_→%zu",
                        copyOld, copyNew, copyOld + copyNew, next_index, last_dequeued_bytes_);

            return FilledBufferInfo{
                .virtualAddress    = scratch_.data(),
                .startOffset       = 0,
                .bytesFilled       = copyOld + copyNew,
                .descriptorIndex   = index,  // old index; caller may use for WAKE accounting
                .autoRecycledPrev  = true,
                .isStraddleScratch = true,
            };
        }

        // No scratch attached (legacy path) OR nothing to copy: fall back to
        // the original behaviour — reset old descriptor, advance head, lose
        // any straddle bytes. Rings without scratch (e.g. isoch RX) take this
        // branch and are unaffected by the async straddle bug.
        HW::AR_init_status(desc_to_recycle, reqCount_recycle);
        if (dma_) {
            dma_->PublishToDevice(&desc_to_recycle, sizeof(desc_to_recycle));
        }
        Driver::WriteBarrier();

        head_ = next_index;
        last_dequeued_bytes_ = 0;
        index = next_index;
        autoRecycledPrev = true;

        ASFW_LOG_V4(Async,
                    "✅ BufferRing: Auto-recycled buffer (no scratch), advanced head_→%zu",
                    index);
    }

    // Now process current buffer (either same as before, or newly advanced)
    auto& desc = descriptors_[index];

    // CRITICAL FIX: Invalidate CPU cache before reading descriptor status
    // Hardware wrote statusWord via DMA, must fetch fresh data to avoid reading stale cache
    if (dma_) {
        dma_->FetchFromDevice(&desc, sizeof(desc));
    }

    // CRITICAL: Do NOT add ReadBarrier() after FetchRange for uncached device memory!
    // For uncached memory, IoBarrier (DSB) is sufficient. Adding DMB may cause issues.

    #ifndef ASFW_HOST_TEST
    if (DMAMemoryManager::IsTracingEnabled()) {
        ASFW_LOG_V4(Async,
                    "  🔍 BufferRing::Dequeue: ReadBarrier NOT used (uncached device memory, DSB sufficient)");
    }
    #endif

    // Extract resCount and xferStatus using AR-specific accessors
    // CRITICAL: statusWord is in BIG-ENDIAN per OHCI §8.4.2, Table 8-1
    const uint16_t resCount = HW::AR_resCount(desc);
    const uint16_t reqCount = static_cast<uint16_t>(desc.control & 0xFFFF);

    // Calculate total bytes filled by hardware
    const size_t total_bytes_in_buffer = (resCount <= reqCount) ? (reqCount - resCount) : 0;

    // CRITICAL: AR DMA stream semantics (OHCI §3.3, §8.4.2 bufferFill mode)
    // Hardware ACCUMULATES multiple packets in the SAME buffer, raising an interrupt
    // after EACH packet. We must return only the NEW bytes since last Dequeue().
    
    // Check if there are NEW bytes beyond what we've already returned
    if (total_bytes_in_buffer <= last_dequeued_bytes_) {
        // No new data since last call
        return std::nullopt;
    }

    // Calculate NEW bytes that appeared since last Dequeue()
    const size_t start_offset = last_dequeued_bytes_;
    const size_t new_bytes = total_bytes_in_buffer - last_dequeued_bytes_;

    // Validate resCount sanity
    if (resCount > reqCount) {
        ASFW_LOG(Async, "BufferRing::Dequeue: invalid resCount %u > reqCount %u at index %zu",
                 resCount, reqCount, index);
        return std::nullopt;
    }

    #ifndef ASFW_HOST_TEST
    if (DMAMemoryManager::IsTracingEnabled()) {
        ASFW_LOG_V4(Async,
                    "🧭 BufferRing::Dequeue idx=%zu desc=%p reqCount=%u resCount=%u "
                    "total=%zu last_dequeued=%zu startOffset=%zu newBytes=%zu",
                    index, &desc, reqCount, resCount,
                    total_bytes_in_buffer, last_dequeued_bytes_, start_offset, new_bytes);
    }
    #endif

    // Get virtual address of buffer START (caller will add startOffset)
    void* bufferAddr = GetBufferAddress(index);
    if (!bufferAddr) {
        ASFW_LOG(Async, "BufferRing::Dequeue: invalid buffer address at index %zu", index);
        return std::nullopt;
    }

    // CRITICAL FIX: Invalidate buffer cache ONLY for the NEW bytes
    if (dma_) {
        auto* byte_ptr = static_cast<uint8_t*>(bufferAddr);
        dma_->FetchFromDevice(byte_ptr + start_offset, new_bytes);
    }

    // Update tracking: remember how many total bytes we've now returned
    last_dequeued_bytes_ = total_bytes_in_buffer;

    // Fix-65 diagnostic: unconditional ring state on every successful
    // Dequeue, so a tCode=0xF drop in ARPacketParser can be correlated
    // against resCount/reqCount and the next descriptor's fill state.
    // One line per RX packet; noise acceptable for the duration of the
    // diagnostic pass, removed once root cause is classified.
    ASFW_LOG_V0(Async,
        "📥 BufferRing::Dequeue: idx=%zu reqCount=%u resCount=%u "
        "total=%zu startOffset=%zu newBytes=%zu "
        "nextIdx=%zu next_reqCount=%u next_resCount=%u",
        index, reqCount, resCount,
        total_bytes_in_buffer, start_offset, new_bytes,
        next_index, next_reqCount, next_resCount);

    return FilledBufferInfo{
        .virtualAddress = bufferAddr,
        .startOffset = start_offset,
        .bytesFilled = total_bytes_in_buffer,  // Total bytes (caller parses [startOffset, bytesFilled))
        .descriptorIndex = index,
        .autoRecycledPrev = autoRecycledPrev
    };
}

kern_return_t BufferRing::Recycle(size_t index) noexcept {
    // Validate index is current head
    if (index != head_) {
        ASFW_LOG(Async, "BufferRing::Recycle: index %zu != head %zu (out-of-order recycle)",
                 index, head_);
        return kIOReturnBadArgument;
    }

    if (index >= bufferCount_) {
        ASFW_LOG(Async, "BufferRing::Recycle: index %zu out of bounds", index);
        return kIOReturnBadArgument;
    }

    auto& desc = descriptors_[index];
    const uint16_t reqCount = static_cast<uint16_t>(desc.control & 0xFFFF);

    // DIAGNOSTIC: Read descriptor state BEFORE reset
    const uint16_t resCountBefore = HW::AR_resCount(desc);
    const uint16_t xferStatusBefore = HW::AR_xferStatus(desc);
    const uint32_t statusWordBefore = desc.statusWord;

    // Reset statusWord to indicate buffer is empty
    // CRITICAL: Use AR_init_status() to handle native byte order correctly
    HW::AR_init_status(desc, reqCount);

    // DIAGNOSTIC: Read descriptor state AFTER reset (but before cache flush)
    const uint16_t resCountAfter = HW::AR_resCount(desc);
    const uint16_t xferStatusAfter = HW::AR_xferStatus(desc);
    const uint32_t statusWordAfter = desc.statusWord;

    // Sync descriptor to device after AR_init_status (publish to HC)
    if (dma_) {
        dma_->PublishToDevice(&desc, sizeof(desc));
    }
    Driver::WriteBarrier();

    // CRITICAL DIAGNOSTIC: Always log recycle operation to trace buffer lifecycle
    ASFW_LOG_V4(Async,
                "♻️  BufferRing::Recycle[%zu]: BEFORE statusWord=0x%08X (resCount=%u xferStatus=0x%04X)",
                index, statusWordBefore, resCountBefore, xferStatusBefore);
    ASFW_LOG_V4(Async,
                "♻️  BufferRing::Recycle[%zu]: AFTER  statusWord=0x%08X (resCount=%u xferStatus=0x%04X) reqCount=%u",
                index, statusWordAfter, resCountAfter, xferStatusAfter, reqCount);
    ASFW_LOG_V4(Async,
                "♻️  BufferRing::Recycle[%zu]: head_ %zu → %zu (next buffer)",
                index, head_, (head_ + 1) % bufferCount_);

    if (resCountAfter != reqCount) {
        ASFW_LOG(Async,
                 "⚠️  BufferRing::Recycle[%zu]: UNEXPECTED! resCount=%u after reset, expected %u",
                 index, resCountAfter, reqCount);
    }

    #ifndef ASFW_HOST_TEST
    if (DMAMemoryManager::IsTracingEnabled()) {
        ASFW_LOG_V4(Async,
                    "🧭BufferRing::Recycle idx=%zu desc=%p reqCount=%u",
                    index,
                    &desc,
                    reqCount);
    }
    #endif

    // Advance head to next buffer (circular)
    head_ = (head_ + 1) % bufferCount_;

    // CRITICAL: Reset stream tracking for new buffer
    last_dequeued_bytes_ = 0;

    ASFW_LOG_V4(Async,
                "♻️  BufferRing::Recycle[%zu]: Advanced to next buffer, reset last_dequeued_bytes_=0",
                index);

    return kIOReturnSuccess;
}

void* BufferRing::GetBufferAddress(size_t index) const noexcept {
    if (index >= bufferCount_) return nullptr;
    const size_t offset = index * bufferSize_;
    if (offset + bufferSize_ > buffers_.size()) return nullptr;
    return buffers_.data() + offset;
}

uint32_t BufferRing::CommandPtrWord() const noexcept {
    if (descIOVABase_ == 0) return 0;
    return HW::MakeBranchWordAR(static_cast<uint64_t>(descIOVABase_), 1);
}

void BufferRing::BindDma(IDMAMemory* dma) noexcept { dma_ = dma; }

void BufferRing::PublishAllDescriptorsOnce() noexcept {
    if (!dma_ || descriptors_.empty()) return;
    dma_->PublishToDevice(descriptors_.data(), descriptors_.size_bytes());
    ::ASFW::Driver::IoBarrier();
}

} // namespace ASFW::Shared
