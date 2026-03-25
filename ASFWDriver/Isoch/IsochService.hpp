#pragma once

#include <cstdint>
#include <memory>

#ifdef ASFW_HOST_TEST
#include "../Testing/HostDriverKitStubs.hpp"
#else
#include <DriverKit/OSSharedPtr.h>
#endif

#include "IsochReceiveContext.hpp"
#include "Core/ExternalSyncBridge.hpp"
#include "Transmit/IsochTransmitContext.hpp"

namespace ASFW {
namespace Discovery {
class DeviceManager;
}
namespace Protocols {
namespace AVC {
class AVCDiscovery;
}
}
namespace CMP {
class CMPClient;
}
}

namespace ASFW::Driver {

class HardwareInterface;

class IsochService {
public:
    IsochService() = default;
    ~IsochService() = default;

    kern_return_t StartReceive(uint8_t channel,
                               HardwareInterface& hardware,
                               ASFW::Discovery::DeviceManager* deviceManager,
                               ASFW::CMP::CMPClient* cmpClient,
                               ASFW::Protocols::AVC::AVCDiscovery* avcDiscovery = nullptr);

    kern_return_t StopReceive(ASFW::CMP::CMPClient* cmpClient);

    kern_return_t StartTransmit(uint8_t channel,
                                HardwareInterface& hardware,
                                ASFW::Discovery::DeviceManager* deviceManager,
                                ASFW::CMP::CMPClient* cmpClient,
                                ASFW::Protocols::AVC::AVCDiscovery* avcDiscovery);

    kern_return_t StopTransmit(ASFW::CMP::CMPClient* cmpClient);

    void StopAll();

    ASFW::Isoch::IsochReceiveContext* ReceiveContext() const { return isochReceiveContext_.get(); }
    ASFW::Isoch::IsochTransmitContext* TransmitContext() const { return isochTransmitContext_.get(); }

    // Reconnect oPCR on the IR channel after a device resume (e.g. BeBoB bus reset recovery).
    // No-op if StartReceive has not been called yet.
    void ReconnectOPCR(ASFW::CMP::CMPClient* cmpClient);

    // Reconnect iPCR on the IT channel after a device resume.
    // Called alongside ReconnectOPCR so both directions are re-established after a BeBoB bus reset.
    // No-op if StartTransmit has not been called yet.
    void ReconnectIPCR(ASFW::CMP::CMPClient* cmpClient);

private:
    ASFW::Isoch::Core::ExternalSyncBridge externalSyncBridge_{};
    OSSharedPtr<ASFW::Isoch::IsochReceiveContext> isochReceiveContext_;
    std::unique_ptr<ASFW::Isoch::IsochTransmitContext> isochTransmitContext_;
    uint8_t irChannel_{0xFF};  // channel passed to StartReceive; 0xFF = not started
    uint8_t itChannel_{0xFF};  // channel passed to StartTransmit; 0xFF = not started
};

} // namespace ASFW::Driver
