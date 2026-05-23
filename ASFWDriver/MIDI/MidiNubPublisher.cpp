// SPDX-License-Identifier: LGPL-3.0-or-later
// Copyright (c) 2026 ASFireWire Project

#include "MidiNubPublisher.hpp"

#include <DriverKit/IOService.h>
#include <DriverKit/OSDictionary.h>
#include <DriverKit/OSNumber.h>
#include <DriverKit/OSSharedPtr.h>
#include <DriverKit/OSString.h>

#include "../Protocols/Audio/DeviceProtocolFactory.hpp"
#include "../Logging/Logging.hpp"
#include <com.kevinpeters.ASFW.ASFWDriver/ASFWMIDINub.h>

#include <cstdio>

namespace ASFW::MIDI {

namespace {

struct MidiIsochRegistry {
    IOLock* lock{nullptr};
    std::unordered_map<uint64_t, void*> servicesByGuid{};

    MidiIsochRegistry() noexcept
        : lock(IOLockAlloc()) {
        if (!lock) {
            ASFW_LOG_ERROR(Audio, "MidiIsochRegistry: Failed to allocate lock");
        }
    }

    ~MidiIsochRegistry() noexcept {
        if (lock) {
            IOLockFree(lock);
            lock = nullptr;
        }
    }
};

MidiIsochRegistry& SharedMidiIsochRegistry() noexcept {
    static MidiIsochRegistry registry;
    return registry;
}

OSSharedPtr<OSString> MakeDecimalGuidString(uint64_t guid) noexcept {
    char guidText[32];
    snprintf(guidText, sizeof(guidText), "%llu", guid);
    return OSSharedPtr(OSString::withCString(guidText), OSNoRetain);
}

bool PopulateMidiNubProperties(OSDictionary* properties,
                               const ASFW::Audio::Model::ASFWAudioDevice& config) noexcept {
    if (!properties) {
        return false;
    }

    auto deviceNameStr = OSSharedPtr(OSString::withCString(config.deviceName.c_str()), OSNoRetain);
    auto guidStr = MakeDecimalGuidString(config.guid);
    auto guidNum = OSSharedPtr(OSNumber::withNumber(config.guid, 64), OSNoRetain);
    auto vendorIdNum = OSSharedPtr(OSNumber::withNumber(config.vendorId, 32), OSNoRetain);
    auto modelIdNum = OSSharedPtr(OSNumber::withNumber(config.modelId, 32), OSNoRetain);
    auto editorPathStr = OSSharedPtr(OSString::withCString(""), OSNoRetain);
    auto iconFilePathStr = OSSharedPtr(OSString::withCString(""), OSNoRetain);
    auto inputPortsNum = OSSharedPtr(OSNumber::withNumber(config.midiInputPorts, 32), OSNoRetain);
    auto outputPortsNum = OSSharedPtr(OSNumber::withNumber(config.midiOutputPorts, 32), OSNoRetain);

    if (!deviceNameStr || !guidStr || !guidNum || !vendorIdNum || !modelIdNum ||
        !editorPathStr || !iconFilePathStr || !inputPortsNum || !outputPortsNum) {
        return false;
    }

    // Apple-visible AppleFWAudioMIDIDeviceNub property names.
    properties->setObject("DeviceName", deviceNameStr.get());
    properties->setObject("GUID", guidStr.get());
    properties->setObject("vendorID", vendorIdNum.get());
    properties->setObject("modelID", modelIdNum.get());
    properties->setObject("EditorPath", editorPathStr.get());
    properties->setObject("IconFilePath", iconFilePathStr.get());

    // ASFW match/routing properties.
    properties->setObject("ASFWDeviceName", deviceNameStr.get());
    properties->setObject("ASFWGUID", guidNum.get());
    properties->setObject("ASFWDeviceGUID", guidNum.get());
    properties->setObject("ASFWVendorID", vendorIdNum.get());
    properties->setObject("ASFWModelID", modelIdNum.get());
    properties->setObject("ASFWMidiInputPorts", inputPortsNum.get());
    properties->setObject("ASFWMidiOutputPorts", outputPortsNum.get());

    return true;
}

ASFW::Audio::Model::ASFWAudioDevice WithKnownDeviceMidiFallbacks(
    const ASFW::Audio::Model::ASFWAudioDevice& config) noexcept {
    auto effective = config;
    if (effective.vendorId == ASFW::Audio::DeviceProtocolFactory::kPrismSoundVendorId &&
        effective.modelId == ASFW::Audio::DeviceProtocolFactory::kOrpheusModelId) {
        if (effective.midiInputPorts == 0) {
            effective.midiInputPorts = 1;
        }
        if (effective.midiOutputPorts == 0) {
            effective.midiOutputPorts = 1;
        }
    }
    return effective;
}

} // namespace

void RegisterIsochServiceForGuid(uint64_t guid, void* isochService) noexcept {
    if (guid == 0 || !isochService) {
        return;
    }

    auto& registry = SharedMidiIsochRegistry();
    if (!registry.lock) {
        return;
    }

    IOLockLock(registry.lock);
    registry.servicesByGuid[guid] = isochService;
    IOLockUnlock(registry.lock);

    ASFW_LOG(Audio,
             "MidiIsochRegistry: Registered IsochService %p for GUID=%llx",
             isochService,
             guid);
}

void UnregisterIsochServiceForGuid(uint64_t guid) noexcept {
    if (guid == 0) {
        return;
    }

    auto& registry = SharedMidiIsochRegistry();
    if (!registry.lock) {
        return;
    }

    IOLockLock(registry.lock);
    registry.servicesByGuid.erase(guid);
    IOLockUnlock(registry.lock);

    ASFW_LOG(Audio,
             "MidiIsochRegistry: Unregistered IsochService for GUID=%llx",
             guid);
}

void* ResolveIsochServiceForGuid(uint64_t guid) noexcept {
    if (guid == 0) {
        return nullptr;
    }

    auto& registry = SharedMidiIsochRegistry();
    if (!registry.lock) {
        return nullptr;
    }

    IOLockLock(registry.lock);
    auto it = registry.servicesByGuid.find(guid);
    void* isochService = (it != registry.servicesByGuid.end()) ? it->second : nullptr;
    IOLockUnlock(registry.lock);
    return isochService;
}

MidiNubPublisher::MidiNubPublisher(IOService* driver, void* isochService) noexcept
    : driver_(driver)
    , isochService_(isochService) {
    lock_ = IOLockAlloc();
    if (!lock_) {
        ASFW_LOG_ERROR(Audio, "MidiNubPublisher: Failed to allocate lock");
    }
}

MidiNubPublisher::~MidiNubPublisher() noexcept {
    if (lock_) {
        IOLockFree(lock_);
        lock_ = nullptr;
    }
}

bool MidiNubPublisher::ReserveGuidLocked(uint64_t guid) noexcept {
    const auto [it, inserted] = nubsByGuid_.emplace(guid, nullptr);
    return inserted;
}

bool MidiNubPublisher::EnsureNub(uint64_t guid,
                                 const ASFW::Audio::Model::ASFWAudioDevice& config,
                                 const char* sourceTag) noexcept {
    if (!driver_ || !lock_ || guid == 0) {
        return false;
    }

    const auto effectiveConfig = WithKnownDeviceMidiFallbacks(config);
    if (effectiveConfig.midiInputPorts == 0 && effectiveConfig.midiOutputPorts == 0) {
        return true;
    }

    IOLockLock(lock_);
    {
        auto it = nubsByGuid_.find(guid);
        if (it != nubsByGuid_.end()) {
            IOLockUnlock(lock_);
            return true;
        }

        if (!ReserveGuidLocked(guid)) {
            IOLockUnlock(lock_);
            return true;
        }
    }
    IOLockUnlock(lock_);

    IOService* nubService = nullptr;
    kern_return_t kr = driver_->Create(
        driver_,
        "ASFWMIDINubProperties",
        &nubService);

    if (kr != kIOReturnSuccess || !nubService) {
        ASFW_LOG_ERROR(Audio,
                       "MidiNubPublisher[%{public}s]: Failed to create ASFWMIDINub (GUID=%llx kr=0x%x)",
                       sourceTag ? sourceTag : "unknown",
                       guid,
                       kr);
        IOLockLock(lock_);
        nubsByGuid_.erase(guid);
        IOLockUnlock(lock_);
        return false;
    }

    OSDictionary* propertiesRaw = nullptr;
    kr = nubService->CopyProperties(&propertiesRaw);
    OSSharedPtr<OSDictionary> properties(propertiesRaw, OSNoRetain);
    if (kr == kIOReturnSuccess && properties) {
        if (!PopulateMidiNubProperties(properties.get(), effectiveConfig)) {
            ASFW_LOG_ERROR(Audio,
                           "MidiNubPublisher[%{public}s]: Failed to populate properties (GUID=%llx)",
                           sourceTag ? sourceTag : "unknown",
                           guid);
        } else {
            nubService->SetProperties(properties.get());
            ASFW_LOG(Audio,
                     "MidiNubPublisher[%{public}s]: ASFWMIDINub properties set (GUID=%llx in=%u out=%u)",
                     sourceTag ? sourceTag : "unknown",
                     guid,
                     effectiveConfig.midiInputPorts,
                     effectiveConfig.midiOutputPorts);
        }
    }

    ASFWMIDINub* midiNub = OSDynamicCast(ASFWMIDINub, nubService);
    if (!midiNub) {
        ASFW_LOG_ERROR(Audio,
                       "MidiNubPublisher[%{public}s]: Created service is not ASFWMIDINub (GUID=%llx)",
                       sourceTag ? sourceTag : "unknown",
                       guid);
        IOLockLock(lock_);
        nubsByGuid_.erase(guid);
        IOLockUnlock(lock_);
        nubService->release();
        return false;
    }

    midiNub->SetParentDriver(driver_);
    midiNub->SetIsochService(isochService_);
    midiNub->SetGuid(effectiveConfig.guid);
    midiNub->SetVendorID(effectiveConfig.vendorId);
    midiNub->SetModelID(effectiveConfig.modelId);
    midiNub->SetPortCounts(effectiveConfig.midiInputPorts, effectiveConfig.midiOutputPorts);
    RegisterIsochServiceForGuid(effectiveConfig.guid, isochService_);

    IOLockLock(lock_);
    nubsByGuid_[guid] = midiNub;
    IOLockUnlock(lock_);

    nubService->release();

    ASFW_LOG(Audio,
             "✅ MidiNubPublisher[%{public}s]: ASFWMIDINub ready for GUID=%llx",
             sourceTag ? sourceTag : "unknown",
             guid);
    return true;
}

ASFWMIDINub* MidiNubPublisher::GetNub(uint64_t guid) const noexcept {
    if (!lock_ || guid == 0) return nullptr;

    IOLockLock(lock_);
    auto it = nubsByGuid_.find(guid);
    ASFWMIDINub* nub = (it != nubsByGuid_.end()) ? it->second : nullptr;
    IOLockUnlock(lock_);
    return nub;
}

void MidiNubPublisher::TerminateNub(uint64_t guid, const char* reasonTag) noexcept {
    if (!lock_ || guid == 0) return;

    ASFWMIDINub* nub = nullptr;
    IOLockLock(lock_);
    auto it = nubsByGuid_.find(guid);
    if (it != nubsByGuid_.end()) {
        nub = it->second;
        nubsByGuid_.erase(it);
    }
    IOLockUnlock(lock_);

    if (nub) {
        ASFW_LOG(Audio,
                 "MidiNubPublisher[%{public}s]: Terminating ASFWMIDINub for GUID=%llx",
                 reasonTag ? reasonTag : "unknown",
                 guid);
        nub->Terminate(0);
    }
    UnregisterIsochServiceForGuid(guid);
}

} // namespace ASFW::MIDI
