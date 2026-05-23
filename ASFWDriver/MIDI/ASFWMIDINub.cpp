// SPDX-License-Identifier: LGPL-3.0-or-later
// Copyright (c) 2026 ASFireWire Project

#include "ASFWMIDINub.h"
#include "ASFWDriver.h"
#include "../Service/DriverContext.hpp"
#include "../Isoch/IsochService.hpp"
#include "../Logging/Logging.hpp"

#include <DriverKit/IOLib.h>

namespace {

constexpr uint32_t kMidiRxQueueCapacity = 128;
constexpr uint32_t kMidiRxQueueMask = kMidiRxQueueCapacity - 1;

void ReceiveMIDIBytesForNub(void* context, const uint8_t* bytes, uint8_t count) noexcept {
    auto* nub = static_cast<ASFWMIDINub*>(context);
    if (!nub || !bytes || count == 0 || count > 3) {
        return;
    }

    const uint32_t packed =
        static_cast<uint32_t>(bytes[0]) |
        (static_cast<uint32_t>(count > 1 ? bytes[1] : 0) << 8) |
        (static_cast<uint32_t>(count > 2 ? bytes[2] : 0) << 16);
    nub->QueueReceivedMIDIBytes(packed, count);
}

} // namespace

bool ASFWMIDINub::init()
{
    if (!super::init()) {
        ASFW_LOG_ERROR(Audio, "ASFWMIDINub: super::init() failed");
        return false;
    }

    ivars = IONewZero(ASFWMIDINub_IVars, 1);
    if (!ivars) {
        ASFW_LOG_ERROR(Audio, "ASFWMIDINub: Failed to allocate ivars");
        return false;
    }

    ivars->rxLock = IOLockAlloc();
    if (!ivars->rxLock) {
        ASFW_LOG_ERROR(Audio, "ASFWMIDINub: Failed to allocate RX lock");
        IOSafeDeleteNULL(ivars, ASFWMIDINub_IVars, 1);
        return false;
    }

    ASFW_LOG(Audio, "ASFWMIDINub: init() succeeded");
    return true;
}

void ASFWMIDINub::free()
{
    ASFW_LOG(Audio, "ASFWMIDINub: free()");
    if (ivars) {
        if (ivars->rxLock) {
            IOLockFree(ivars->rxLock);
            ivars->rxLock = nullptr;
        }
        IOSafeDeleteNULL(ivars, ASFWMIDINub_IVars, 1);
    }
    super::free();
}

kern_return_t IMPL(ASFWMIDINub, Start)
{
    const kern_return_t error = Start(provider, SUPERDISPATCH);
    if (error != kIOReturnSuccess) {
        ASFW_LOG_ERROR(Audio, "ASFWMIDINub: super::Start() failed: 0x%x", error);
        return error;
    }

    if (!ivars->parentDriver) {
        ivars->parentDriver = provider;
    }

    const kern_return_t registerError = RegisterService();
    if (registerError != kIOReturnSuccess) {
        ASFW_LOG_ERROR(Audio, "ASFWMIDINub: RegisterService() failed: 0x%x", registerError);
        return registerError;
    }

    ASFW_LOG(Audio,
             "ASFWMIDINub: Start provider=%p GUID=0x%016llx in=%u out=%u registered",
             provider,
             ivars->guid,
             ivars->inputPorts,
             ivars->outputPorts);
    return kIOReturnSuccess;
}

kern_return_t IMPL(ASFWMIDINub, Stop)
{
    ASFW_LOG(Audio, "ASFWMIDINub: Stop GUID=0x%016llx", ivars ? ivars->guid : 0);
    if (ivars) {
        if (ivars->rxSinkAttached && ivars->isochService) {
            auto* isochService = static_cast<ASFW::Driver::IsochService*>(ivars->isochService);
            isochService->SetReceiveMidiSink(nullptr, nullptr);
            ivars->rxSinkAttached = false;
        }
        ivars->parentDriver = nullptr;
        ivars->isochService = nullptr;
    }
    return Stop(provider, SUPERDISPATCH);
}

ASFWDriver* ASFWMIDINub::GetParentDriver() const
{
    if (!ivars || !ivars->parentDriver) {
        return nullptr;
    }
    return OSDynamicCast(ASFWDriver, ivars->parentDriver);
}

void ASFWMIDINub::SetParentDriver(IOService* parentDriver)
{
    if (!ivars) {
        return;
    }
    ivars->parentDriver = parentDriver;
    ASFW_LOG(Audio, "ASFWMIDINub: Parent driver set to %p", parentDriver);
}

void ASFWMIDINub::SetIsochService(void* isochService)
{
    if (!ivars) {
        return;
    }
    ivars->isochService = isochService;
    ASFW_LOG(Audio, "ASFWMIDINub: Isoch service set to %p", isochService);

    auto* service = static_cast<ASFW::Driver::IsochService*>(isochService);
    if (service) {
        service->SetReceiveMidiSink(this, ReceiveMIDIBytesForNub);
        ivars->rxSinkAttached = true;
        ASFW_LOG(Audio,
                 "ASFWMIDINub: RX MIDI bridge sink attached queueCapacity=%u",
                 kMidiRxQueueCapacity);
    }
}

void* ASFWMIDINub::GetIsochService() const
{
    return ivars ? ivars->isochService : nullptr;
}

void ASFWMIDINub::SetGuid(uint64_t guid)
{
    if (!ivars) {
        return;
    }
    ivars->guid = guid;
    ASFW_LOG(Audio, "ASFWMIDINub: GUID set to 0x%016llx", guid);
    if (ivars->rxSinkAttached) {
        ASFW_LOG(Audio, "ASFWMIDINub: RX MIDI bridge GUID updated to 0x%016llx", guid);
    }
}

uint64_t ASFWMIDINub::GetGuid() const
{
    return ivars ? ivars->guid : 0;
}

void ASFWMIDINub::SetVendorID(uint32_t vendorId)
{
    if (!ivars) {
        return;
    }
    ivars->vendorId = vendorId;
}

uint32_t ASFWMIDINub::GetVendorID() const
{
    return ivars ? ivars->vendorId : 0;
}

void ASFWMIDINub::SetModelID(uint32_t modelId)
{
    if (!ivars) {
        return;
    }
    ivars->modelId = modelId;
}

uint32_t ASFWMIDINub::GetModelID() const
{
    return ivars ? ivars->modelId : 0;
}

void ASFWMIDINub::SetPortCounts(uint32_t inputPorts, uint32_t outputPorts)
{
    if (!ivars) {
        return;
    }
    ivars->inputPorts = inputPorts;
    ivars->outputPorts = outputPorts;
    ASFW_LOG(Audio, "ASFWMIDINub: Port counts set to in=%u out=%u", inputPorts, outputPorts);
}

uint32_t ASFWMIDINub::GetInputPortCount() const
{
    return ivars ? ivars->inputPorts : 0;
}

uint32_t ASFWMIDINub::GetOutputPortCount() const
{
    return ivars ? ivars->outputPorts : 0;
}

void ASFWMIDINub::QueueReceivedMIDIBytes(uint32_t packedBytes, uint32_t byteCount)
{
    if (!ivars || !ivars->rxLock || byteCount == 0 || byteCount > 3) {
        return;
    }

    IOLockLock(ivars->rxLock);
    uint32_t nextHead = (ivars->rxHead + 1u) & kMidiRxQueueMask;
    if (nextHead == ivars->rxTail) {
        ivars->rxTail = (ivars->rxTail + 1u) & kMidiRxQueueMask;
        ++ivars->rxDropped;
    }

    ivars->rxPacked[ivars->rxHead] = packedBytes;
    ivars->rxByteCount[ivars->rxHead] = static_cast<uint8_t>(byteCount);
    ivars->rxHead = nextHead;
    IOLockUnlock(ivars->rxLock);
}

kern_return_t IMPL(ASFWMIDINub, PushTransmitMIDIBytes)
{
    if (!ivars || !outQueued || byteCount == 0 || byteCount > 3) {
        if (outQueued) {
            *outQueued = 0;
        }
        return kIOReturnBadArgument;
    }

    auto* parent = GetParentDriver();
    if (!parent) {
        ASFW_LOG(Audio,
                 "ASFWMIDINub: PushTransmitMIDIBytes failed - no parent driver GUID=0x%016llx",
                 ivars->guid);
        *outQueued = 0;
        return kIOReturnNotReady;
    }

    auto* context = static_cast<ServiceContext*>(parent->GetServiceContext());
    if (!context) {
        ASFW_LOG(Audio,
                 "ASFWMIDINub: PushTransmitMIDIBytes failed - no ServiceContext GUID=0x%016llx",
                 ivars->guid);
        *outQueued = 0;
        return kIOReturnNotReady;
    }

    uint8_t bytes[3] = {
        static_cast<uint8_t>(packedBytes & 0xffu),
        static_cast<uint8_t>((packedBytes >> 8) & 0xffu),
        static_cast<uint8_t>((packedBytes >> 16) & 0xffu),
    };

    *outQueued = context->isoch.PushTransmitMidiBytes(bytes, byteCount);
    ASFW_LOG(Audio,
             "ASFWMIDINub: TX MIDI bridge bytes=%02x %02x %02x count=%u queued=%u GUID=0x%016llx",
             bytes[0],
             byteCount > 1 ? bytes[1] : 0,
             byteCount > 2 ? bytes[2] : 0,
             byteCount,
             *outQueued,
             ivars->guid);
    return kIOReturnSuccess;
}

kern_return_t IMPL(ASFWMIDINub, PopReceivedMIDIBytes)
{
    if (!ivars || !outPackedBytes || !outByteCount || !outDroppedCount) {
        return kIOReturnBadArgument;
    }

    *outPackedBytes = 0;
    *outByteCount = 0;
    *outDroppedCount = 0;

    if (!ivars->rxLock) {
        return kIOReturnNotReady;
    }

    IOLockLock(ivars->rxLock);
    *outDroppedCount = ivars->rxDropped;
    ivars->rxDropped = 0;

    if (ivars->rxTail != ivars->rxHead) {
        *outPackedBytes = ivars->rxPacked[ivars->rxTail];
        *outByteCount = ivars->rxByteCount[ivars->rxTail];
        ivars->rxTail = (ivars->rxTail + 1u) & kMidiRxQueueMask;
    }
    IOLockUnlock(ivars->rxLock);

    return kIOReturnSuccess;
}
