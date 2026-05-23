// PacketStreamParser.hpp
// ASFW - Byte-stream parser for OHCI IR bufferFill mode (matches Apple's
// MultiIsochReceiver::processReceivedPackets, sym 0x19ccc in AppleFWOHCI).
//
// In bufferFill mode each isoch packet lands in the buffer as:
//   [+0..3] cycleTimestamp (hardware prepend)
//   [+4..7] isoch header (data_length:16 | tag:2 | chan:6 | tcode:4 | sy:4)
//   [+8..]  data_length bytes of payload, padded up to a quadlet boundary
// Packets are written contiguously and may straddle descriptor boundaries.
//
// The parser accumulates bytes across calls, identifies packet boundaries by
// reading the isoch header's data_length field, and emits whole packets via a
// callback. Straddling packets are reassembled into an internal scratch buffer
// so the caller can recycle a descriptor immediately after handing its bytes
// to OnBytes(...) — the parser does not retain pointers into caller storage
// across calls.

#pragma once

#include <array>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <functional>

namespace ASFW::Isoch::Rx {

class PacketStreamParser final {
public:
    static constexpr size_t kPrefixBytes = 8;     // cycleTimestamp(4) + isoch header(4)
    static constexpr size_t kMaxPacketBytes = 4096; // a single FW isoch packet cannot exceed one IR buffer

    using PacketCallback = std::function<void(const uint8_t* packet, size_t length)>;

    PacketStreamParser() = default;

    // Reset all state — call on Start/Stop transitions.
    void Reset() noexcept {
        scratchLen_ = 0;
        scratchPktSize_ = 0;
        droppedPackets_ = 0;
        emittedPackets_ = 0;
    }

    // Feed a chunk of bytes from one descriptor. The parser may emit zero or
    // more packets via `cb`. Bytes that complete a straddling packet are
    // copied into the parser's internal scratch; bytes belonging to a packet
    // contained entirely in this chunk are passed by pointer back into
    // `bytes` (no copy). Caller must process those packets synchronously
    // before recycling the descriptor.
    void OnBytes(const uint8_t* bytes, size_t length, const PacketCallback& cb) noexcept {
        if (bytes == nullptr || length == 0) {
            return;
        }
        const uint8_t* cur = bytes;
        const uint8_t* end = bytes + length;

        // 1) If a straddle is in progress, finish it first.
        if (scratchLen_ > 0) {
            // Make sure we have at least the 8-byte prefix in scratch so we
            // can compute the packet size.
            if (scratchLen_ < kPrefixBytes) {
                const size_t need = kPrefixBytes - scratchLen_;
                const size_t take = (static_cast<size_t>(end - cur) < need) ? static_cast<size_t>(end - cur) : need;
                CopyFromDma(scratch_.data() + scratchLen_, cur, take);
                scratchLen_ += take;
                cur += take;
                if (scratchLen_ < kPrefixBytes) {
                    return; // still need more for header
                }
                scratchPktSize_ = ComputePacketSize(scratch_.data());
                if (scratchPktSize_ == 0 || scratchPktSize_ > kMaxPacketBytes) {
                    // Bogus header; drop the straddle
                    ++droppedPackets_;
                    scratchLen_ = 0;
                    scratchPktSize_ = 0;
                    return;
                }
            }

            // We have the header in scratch. Keep filling until pkt_size.
            const size_t need = scratchPktSize_ - scratchLen_;
            const size_t avail = static_cast<size_t>(end - cur);
            if (avail < need) {
                CopyFromDma(scratch_.data() + scratchLen_, cur, avail);
                scratchLen_ += avail;
                return;
            }
            CopyFromDma(scratch_.data() + scratchLen_, cur, need);
            cur += need;
            cb(scratch_.data(), scratchPktSize_);
            ++emittedPackets_;
            scratchLen_ = 0;
            scratchPktSize_ = 0;
        }

        // 2) Parse contiguous packets directly from the caller's buffer.
        while (cur < end) {
            const size_t avail = static_cast<size_t>(end - cur);
            if (avail < kPrefixBytes) {
                // Partial header — save for next call.
                CopyFromDma(scratch_.data(), cur, avail);
                scratchLen_ = avail;
                scratchPktSize_ = 0;
                return;
            }
            const size_t pktSize = ComputePacketSize(cur);
            if (pktSize == 0 || pktSize > kMaxPacketBytes) {
                // Bogus header — skip 4 bytes and try to resync.
                ++droppedPackets_;
                cur += 4;
                continue;
            }
            if (avail < pktSize) {
                // Partial packet — save for next call.
                CopyFromDma(scratch_.data(), cur, avail);
                scratchLen_ = avail;
                scratchPktSize_ = pktSize;
                return;
            }
            cb(cur, pktSize);
            ++emittedPackets_;
            cur += pktSize;
        }
    }

    // Per OHCI §10.2.2 + AppleFWOHCI processReceivedPackets analysis:
    // pktSize = 8 (prefix) + roundUp4(data_length).
    // data_length is bits 31-16 of the isoch header quadlet at offset +4
    // when read as a host-endian uint32_t (OHCI byte-swaps wire→host).
    [[nodiscard]] static constexpr size_t ComputePacketSize(const uint8_t* hdr) noexcept {
        const uint32_t isochHdr =
            static_cast<uint32_t>(hdr[4]) |
            (static_cast<uint32_t>(hdr[5]) << 8) |
            (static_cast<uint32_t>(hdr[6]) << 16) |
            (static_cast<uint32_t>(hdr[7]) << 24);
        const uint32_t dataLength = (isochHdr >> 16) & 0xFFFFU;
        const size_t padded = (static_cast<size_t>(dataLength) + 3U) & ~size_t{3U};
        return kPrefixBytes + padded;
    }

    [[nodiscard]] uint64_t EmittedPackets() const noexcept { return emittedPackets_; }
    [[nodiscard]] uint64_t DroppedPackets() const noexcept { return droppedPackets_; }
    [[nodiscard]] size_t PendingScratchBytes() const noexcept { return scratchLen_; }

private:
    // Source is a cache-inhibited DMA mapping (Device memory on arm64) which
    // rejects unaligned NEON vector loads. std::memcpy compiles to
    // _platform_memmove which uses LDP Q0,Q1 → EXC_ARM_DA_ALIGN on any copy
    // ≥16B at a non-16-aligned offset. Mirror the scalar volatile byte loop
    // used in BufferRing.cpp:168 / DMAMemoryManager.cpp:305 for this reason.
    static void CopyFromDma(uint8_t* dst, const uint8_t* src, size_t n) noexcept {
        const auto* vsrc = reinterpret_cast<volatile const uint8_t*>(src);
        for (size_t i = 0; i < n; ++i) {
            dst[i] = vsrc[i];
        }
    }

    std::array<uint8_t, kMaxPacketBytes> scratch_{};
    size_t scratchLen_{0};
    size_t scratchPktSize_{0};
    uint64_t emittedPackets_{0};
    uint64_t droppedPackets_{0};
};

} // namespace ASFW::Isoch::Rx
