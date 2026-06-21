//
//  IAVCDiscovery.hpp
//  ASFWDriver
//
//  Interface for AV/C Discovery
//  Decouples AVCHandler from concrete AVCDiscovery for testing.
//

#pragma once

#include <vector>
#include <cstdint>

namespace ASFW::Protocols::AVC {

class AVCUnit;
class FCPTransport;

class IAVCDiscovery {
public:
    virtual ~IAVCDiscovery() = default;

    /**
     * @brief Get all AV/C units
     * @return Vector of pointers to AVCUnit instances
     */
    virtual std::vector<AVCUnit*> GetAllAVCUnits() = 0;

    /**
     * @brief Re-scan all AV/C units
     * Triggers re-initialization for all discovered units.
     */
    virtual void ReScanAllUnits() = 0;

    /// Resolve live FCP transport for a node ID.
    virtual FCPTransport* GetFCPTransportForNodeID(uint16_t nodeID) = 0;

    /// U5 — device's negotiated max link speed as a CMP/isoch speed code
    /// (0=S100, 1=S200, 2=S400, 3=S800), taken from the Self-ID topology. Lets
    /// the isoch connection speed be derived per device instead of hardcoded
    /// S400. Returns 0 (S100) when the node/topology is unknown — callers should
    /// treat 0 as "unknown" and apply their own floor.
    [[nodiscard]] virtual uint8_t GetDeviceSpeedCode(uint16_t nodeID) const = 0;
};

} // namespace ASFW::Protocols::AVC
