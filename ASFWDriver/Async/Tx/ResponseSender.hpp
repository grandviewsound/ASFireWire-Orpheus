#pragma once

#include <array>
#include <cstdint>
#include <memory>

#include "../ResponseCode.hpp"
#include "../Rx/PacketRouter.hpp"

namespace ASFW::Driver { class HardwareInterface; }

namespace ASFW::Async {

class DescriptorBuilder;
class ATResponseContext;
class IFireWireBusInfo;
namespace Bus { class GenerationTracker; }
namespace Engine { class ContextManager; }
namespace Tx { class Submitter; }

/// Utility to build and send Write Response (WrResp) packets for incoming AR requests.
class ResponseSender {
public:
    ResponseSender(DescriptorBuilder& builder,
                   Tx::Submitter& submitter,
                   Engine::ContextManager& ctxMgr,
                   Bus::GenerationTracker& generationTracker,
                   Driver::HardwareInterface& hw) noexcept;

    /// Build and transmit a WrResp for the given request packet.
    /// Skips transmission for broadcast requests (destID=0xFFFF).
    void SendWriteResponse(const ARPacketView& request, ResponseCode rcode) noexcept;

    /// Build and transmit a quadlet read response for the given request packet.
    /// Skips transmission for broadcast requests (destID=0xFFFF).
    void SendReadQuadletResponse(const ARPacketView& request,
                                 ResponseCode rcode,
                                 uint32_t quadletData) noexcept;

    /// Build and transmit a LOCK response (tCode 0xB) for an incoming compareSwap
    /// to a space we own (e.g. the local PCR/CSR). Carries the prior quadlet as a
    /// 4-byte data block (big-endian on the wire). Skips broadcast requests.
    void SendLockResponse(const ARPacketView& request,
                          ResponseCode rcode,
                          uint16_t extendedTCode,
                          uint32_t oldValue) noexcept;

private:
    DescriptorBuilder& builder_;
    Tx::Submitter& submitter_;
    Engine::ContextManager& ctxMgr_;
    Bus::GenerationTracker& generationTracker_;
    Driver::HardwareInterface& hw_;

    // Lock responses carry a small data block in a DMA buffer the OHCI reads during
    // TX. Responses are fire-and-forget (untracked), so we retain the last few
    // PayloadContexts in a ring: AT response TX completes in microseconds, so by the
    // time a slot is reused its prior transfer is long done — no completion hook
    // needed. Lock responses to our PCR are rare, so a small ring is ample.
    // shared_ptr<void> (via PayloadContext::IntoShared) keeps PayloadContext out of
    // this header — the type-correct deleter is captured at the call site.
    static constexpr std::size_t kLockResponsePayloadSlots = 8;
    std::array<std::shared_ptr<void>, kLockResponsePayloadSlots> lockRespPayloads_{};
    std::size_t lockRespPayloadIdx_{0};
};

} // namespace ASFW::Async
