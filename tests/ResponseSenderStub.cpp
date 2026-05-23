#include "ASFWDriver/Async/Tx/ResponseSender.hpp"

namespace ASFW::Driver { class HardwareInterface; }

namespace ASFW::Async {

ResponseSender::ResponseSender(DescriptorBuilder& builder,
                               Tx::Submitter& submitter,
                               Engine::ContextManager& ctxMgr,
                               Bus::GenerationTracker& generationTracker,
                               Driver::HardwareInterface& hw) noexcept
    : builder_(builder)
    , submitter_(submitter)
    , ctxMgr_(ctxMgr)
    , generationTracker_(generationTracker)
    , hw_(hw) {}

void ResponseSender::SendWriteResponse(const ARPacketView& request, ResponseCode rcode) noexcept {
    // Stub implementation
    (void)request;
    (void)rcode;
}

void ResponseSender::SendReadQuadletResponse(const ARPacketView& request,
                                             ResponseCode rcode,
                                             uint32_t quadletData) noexcept {
    // Stub implementation
    (void)request;
    (void)rcode;
    (void)quadletData;
}

void ResponseSender::SendLockResponse(const ARPacketView& request,
                                      ResponseCode rcode,
                                      uint16_t extendedTCode,
                                      uint32_t oldValue) noexcept {
    // Stub implementation
    (void)request;
    (void)rcode;
    (void)extendedTCode;
    (void)oldValue;
}

} // namespace ASFW::Async
