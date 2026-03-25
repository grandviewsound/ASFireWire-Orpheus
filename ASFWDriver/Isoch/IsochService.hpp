#pragma once

#include <cstdint>
#include <memory>

#ifdef ASFW_HOST_TEST
#include "../Testing/HostDriverKitStubs.hpp"
#else
#include <DriverKit/IOBufferMemoryDescriptor.h>
#include <DriverKit/IOMemoryMap.h>
#include <DriverKit/OSSharedPtr.h>
#endif

#include "IsochReceiveContext.hpp"
#include "Core/ExternalSyncBridge.hpp"
#include "Transmit/IsochTransmitContext.hpp"
#include "../Audio/Model/ASFWAudioDevice.hpp"
#include "../Common/DriverKitOwnership.hpp"

namespace ASFW::Driver {

class HardwareInterface;

struct IsochDuplexStartParams {
    uint64_t guid{0};

    uint8_t irChannel{0};  // device -> host
    uint8_t itChannel{0};  // host -> device
    uint8_t sid{0};        // local node number (6-bit)

    uint32_t sampleRateHz{0};

    uint32_t hostInputPcmChannels{0};
    uint32_t hostOutputPcmChannels{0};

    uint32_t deviceToHostAm824Slots{0};
    uint32_t hostToDeviceAm824Slots{0};

    ASFW::Audio::Model::StreamMode streamMode{ASFW::Audio::Model::StreamMode::kNonBlocking};

    OSSharedPtr<IOBufferMemoryDescriptor> rxQueueMemory{};
    uint64_t rxQueueBytes{0};
    OSSharedPtr<IOBufferMemoryDescriptor> txQueueMemory{};
    uint64_t txQueueBytes{0};

    void* zeroCopyBase{nullptr};
    uint64_t zeroCopyBytes{0};
    uint32_t zeroCopyFrames{0};
};

class IsochService {
public:
    IsochService() = default;
    ~IsochService() = default;

    kern_return_t StartReceive(uint8_t channel,
                               HardwareInterface& hardware,
                               OSSharedPtr<IOBufferMemoryDescriptor> rxQueueMemory,
                               uint64_t rxQueueBytes);

    kern_return_t StopReceive();

    kern_return_t StartTransmit(uint8_t channel,
                                HardwareInterface& hardware,
                                uint8_t sid,
                                uint32_t streamModeRaw,
                                uint32_t pcmChannels,
                                uint32_t am824Slots,
                                OSSharedPtr<IOBufferMemoryDescriptor> txQueueMemory,
                                uint64_t txQueueBytes,
                                void* zeroCopyBase,
                                uint64_t zeroCopyBytes,
                                uint32_t zeroCopyFrames);

    kern_return_t StopTransmit();

    kern_return_t StartDuplex(const IsochDuplexStartParams& params,
                              HardwareInterface& hardware);

    kern_return_t StopDuplex(uint64_t guid);

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
    struct SharedQueueMapping {
        OSSharedPtr<IOBufferMemoryDescriptor> memory{};
        OSSharedPtr<IOMemoryMap> map{};
        uint64_t bytes{0};

        void Reset() noexcept {
            map.reset();
            memory.reset();
            bytes = 0;
        }

        [[nodiscard]] void* BaseAddress() const noexcept {
            return map ? reinterpret_cast<void*>(static_cast<uintptr_t>(map->GetAddress())) : nullptr;
        }
    };

    ASFW::Isoch::Core::ExternalSyncBridge externalSyncBridge_{};
    OSSharedPtr<ASFW::Isoch::IsochReceiveContext> isochReceiveContext_;
    std::unique_ptr<ASFW::Isoch::IsochTransmitContext> isochTransmitContext_;
    uint8_t irChannel_{0xFF};  // channel passed to StartReceive; 0xFF = not started
    uint8_t itChannel_{0xFF};  // channel passed to StartTransmit; 0xFF = not started

    SharedQueueMapping rxQueue_{};
    SharedQueueMapping txQueue_{};

    uint64_t activeGuid_{0};
};

} // namespace ASFW::Driver
