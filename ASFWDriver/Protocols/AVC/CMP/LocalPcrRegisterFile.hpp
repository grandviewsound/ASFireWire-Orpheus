#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace ASFW::CMP {

// ============================================================================
// LocalPcrRegisterFile — host-side (local) PCR backing store (gaps 3.3/3.4)
// ============================================================================
//
// When another FireWire node connects TO this node (host acting as a CMP
// TARGET — device→host-as-listener, peer-to-peer, broadcast), it READs and
// LOCKs (compare_swap) the host's plug control registers. A universal stack
// must serve those transactions from a real stored, lockable register — not a
// value computed on the fly.
//
// This mirrors Apple's IOFireWirePCRSpace, which keeps a raw 32-bit quadlet per
// plug (IOFireWireAVC `IOFireWirePCRSpace+0x58`) and services:
//   • READ  → return the stored quadlet
//   • WRITE → store the quadlet (doWrite @0x40aa)
//   • LOCK (compare_swap) → if stored==arg, store data; return prior (updatePlug
//     @0x4332: read stored, compare oldVal, store newVal on match, return prior)
//
// This class is the storage + atomic compare_swap ONLY. It is intentionally
// layout-agnostic (it stores opaque quadlets) so it is pure and trivially
// unit-testable; the 1394/CMP bit layout lives in PCRBits and is applied by the
// caller (CMPClient) when it seeds plug values. Address→(reg,index) mapping is
// the caller's responsibility (kept where the CSR layout already lives).
//
// Per IEC 1394 compare_swap: the swap is unconditional storage of `desired`
// when the current value equals `expected`; the prior value is always returned
// so the requester can see whether its compare matched. The register file does
// NOT interpret the quadlet — exactly like Apple's updatePlug, which stores
// newVal verbatim on a match.
class LocalPcrRegisterFile {
public:
    // 1394 allows up to 31 oPCR + 31 iPCR plus the two master plug registers.
    static constexpr uint8_t kMaxPlugs = 31;

    enum class Reg : uint8_t {
        kOutputMaster,  // oMPR
        kInputMaster,   // iMPR
        kOutputPlug,    // oPCR[index]
        kInputPlug,     // iPCR[index]
    };

    LocalPcrRegisterFile() = default;

    // Read the stored quadlet. nullopt for an out-of-range plug index.
    [[nodiscard]] std::optional<uint32_t> Read(Reg reg, uint8_t index) const noexcept {
        const uint32_t* slot = Slot(reg, index);
        if (slot == nullptr) {
            return std::nullopt;
        }
        return *slot;
    }

    // Host-initiated update (our own connect path sets a plug's register).
    // Returns false for an out-of-range plug index.
    bool Set(Reg reg, uint8_t index, uint32_t value) noexcept {
        uint32_t* slot = Slot(reg, index);
        if (slot == nullptr) {
            return false;
        }
        *slot = value;
        return true;
    }

    // 1394 compare_swap. Returns the PRIOR value (always, when the address is
    // valid); stores `desired` iff prior == `expected`. nullopt for an
    // out-of-range plug index. `swapped` (optional out) reports whether the
    // store happened.
    [[nodiscard]] std::optional<uint32_t> CompareSwap(Reg reg, uint8_t index,
                                                      uint32_t expected,
                                                      uint32_t desired,
                                                      bool* swapped = nullptr) noexcept {
        uint32_t* slot = Slot(reg, index);
        if (slot == nullptr) {
            if (swapped != nullptr) {
                *swapped = false;
            }
            return std::nullopt;
        }
        const uint32_t prior = *slot;
        const bool match = (prior == expected);
        if (match) {
            *slot = desired;
        }
        if (swapped != nullptr) {
            *swapped = match;
        }
        return prior;
    }

private:
    [[nodiscard]] uint32_t* Slot(Reg reg, uint8_t index) noexcept {
        switch (reg) {
            case Reg::kOutputMaster: return &oMpr_;
            case Reg::kInputMaster:  return &iMpr_;
            case Reg::kOutputPlug:   return index < kMaxPlugs ? &oPcr_[index] : nullptr;
            case Reg::kInputPlug:    return index < kMaxPlugs ? &iPcr_[index] : nullptr;
        }
        return nullptr;
    }

    [[nodiscard]] const uint32_t* Slot(Reg reg, uint8_t index) const noexcept {
        return const_cast<LocalPcrRegisterFile*>(this)->Slot(reg, index);
    }

    uint32_t oMpr_{0};
    uint32_t iMpr_{0};
    std::array<uint32_t, kMaxPlugs> oPcr_{};
    std::array<uint32_t, kMaxPlugs> iPcr_{};
};

} // namespace ASFW::CMP
