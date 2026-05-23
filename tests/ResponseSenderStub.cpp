#include "ASFWDriver/Async/Tx/ResponseSender.hpp"

namespace ASFW::Async {

ResponseSender::ResponseSender(DescriptorBuilder& builder,
                               Tx::Submitter& submitter,
                               Engine::ContextManager& ctxMgr,
                               Bus::GenerationTracker& generationTracker) noexcept
    : builder_(builder)
    , submitter_(submitter)
    , ctxMgr_(ctxMgr)
    , generationTracker_(generationTracker) {}

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

} // namespace ASFW::Async
