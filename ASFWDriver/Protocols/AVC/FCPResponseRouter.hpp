//
// FCPResponseRouter.hpp
// ASFWDriver - AV/C Protocol Layer
//
// FCP Response Router - routes incoming FCP responses to correct FCPTransport
// Integrates with PacketRouter for block write request handling
//

#pragma once

#include "../Ports/FireWireBusPort.hpp"
#include "../Ports/FireWireRxPort.hpp"
#include "AVCDiscovery.hpp"
#include <vector>

namespace ASFW::Protocols::AVC {

//==============================================================================
// FCP Response Router
//==============================================================================

class FCPResponseRouter {
  public:
    explicit FCPResponseRouter(AVCDiscovery& avcDiscovery,
                               Protocols::Ports::FireWireBusInfo& busInfo)
        : avcDiscovery_(avcDiscovery), busInfo_(busInfo) {}

    Protocols::Ports::BlockWriteDisposition
    RouteBlockWrite(const Protocols::Ports::BlockWriteRequestView& request) {
        const uint64_t destOffset = request.destOffset;

        if (destOffset != kFCPResponseAddress) {
            ASFW_LOG_V3(FCP, "⚠️  FCPResponseRouter: Not an FCP response (offset=0x%012llx)",
                        destOffset);
            return Protocols::Ports::BlockWriteDisposition::kAddressError;
        }

        const uint16_t srcNodeID = request.sourceID;
        const uint32_t generation = busInfo_.GetGeneration().value;
        const size_t pn = request.payload.size();
        auto pb = [&](size_t i) -> uint8_t { return i < pn ? request.payload[i] : 0xFFu; };
        ASFW_LOG_V1(FCP,
                    "FCPResponseRouter: FCP response src=0x%04x gen=%u len=%zu bytes=%02x %02x %02x %02x %02x %02x %02x %02x",
                    srcNodeID, generation, pn,
                    pb(0), pb(1), pb(2), pb(3), pb(4), pb(5), pb(6), pb(7));

        FCPTransport* transport = avcDiscovery_.GetFCPTransportForNodeID(srcNodeID);
        if (!transport) {
            ASFW_LOG_V1(FCP, "FCPResponseRouter: FCP response from unknown node 0x%04x", srcNodeID);
            return Protocols::Ports::BlockWriteDisposition::kComplete;
        }

        std::vector<uint8_t> payloadCopy(request.payload.begin(), request.payload.end());

        transport->OnFCPResponse(srcNodeID, generation,
                                 std::span<const uint8_t>(payloadCopy.data(), payloadCopy.size()));

        return Protocols::Ports::BlockWriteDisposition::kComplete;
    }

  private:
    AVCDiscovery& avcDiscovery_;
    Protocols::Ports::FireWireBusInfo& busInfo_;
};

} // namespace ASFW::Protocols::AVC
