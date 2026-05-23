// SPDX-License-Identifier: LGPL-3.0-or-later
// Copyright (c) 2026 ASFireWire Project

#include "ASFWMIDIDriver.h"

#include "ASFWMIDINub.h"
#include "MidiNubPublisher.hpp"
#include "Midi1ToUMP.hpp"
#include "UMPToMidi1.hpp"
#include "ASFWDriver.h"
#include "../Service/DriverContext.hpp"
#include "../Logging/LogConfig.hpp"
#include "../Logging/Logging.hpp"

#include <DriverKit/OSDictionary.h>
#include <DriverKit/OSNumber.h>
#include <DriverKit/OSString.h>
#include <DriverKit/OSArray.h>
#include <DriverKit/IOLib.h>
#include <DriverKit/IOTimerDispatchSource.h>
#include <MIDIDriverKit/MIDIDriverKitTypes.h>

#include <cstdio>
#include <cstring>

namespace {

constexpr const char* kDefaultDeviceName = "FireWire MIDI";
constexpr const char* kAppleStyleSourceName = "MIDI O";
constexpr const char* kAppleStyleDestinationName = "MIDI I 1";
constexpr uint64_t kMidiRxPollIntervalUsec = 1000;
constexpr uint32_t kMidiRxPollBudget = 32;

uint64_t MicrosecondsToMachTicks(uint64_t usec) {
    static mach_timebase_info_data_t timebase{0, 0};
    if (timebase.denom == 0) {
        mach_timebase_info(&timebase);
    }

    const __uint128_t nanos = static_cast<__uint128_t>(usec) * 1000u;
    const __uint128_t scaled = nanos * timebase.denom;
    return static_cast<uint64_t>(scaled / timebase.numer);
}

void ScheduleMidiRxPoll(ASFWMIDIDriver_IVars* ivars) noexcept {
    if (!ivars || !ivars->rxPollActive || !ivars->rxPollTimer) {
        return;
    }

    const uint64_t now = mach_absolute_time();
    const uint64_t delta = MicrosecondsToMachTicks(kMidiRxPollIntervalUsec);
    (void)ivars->rxPollTimer->WakeAtTime(kIOTimerClockMachAbsoluteTime, now + delta, 0);
}

struct ParsedMidiDriverConfig {
    uint64_t guid{0};
    uint32_t vendorId{0};
    uint32_t modelId{0};
    uint32_t inputPorts{0};
    uint32_t outputPorts{0};
    char deviceName[128]{};
};

OSSharedPtr<OSString> MakeString(const char* text) noexcept {
    return OSSharedPtr(OSString::withCString(text ? text : ""), OSNoRetain);
}

OSSharedPtr<OSString> MakeFormattedString(const char* format, uint64_t value) noexcept {
    char text[96];
    snprintf(text, sizeof(text), format, value);
    return MakeString(text);
}

void ParseMidiConfigFromProvider(IOService* provider, ParsedMidiDriverConfig& outConfig) noexcept {
    strlcpy(outConfig.deviceName, kDefaultDeviceName, sizeof(outConfig.deviceName));
    if (!provider) {
        return;
    }

    OSDictionary* propsRaw = nullptr;
    if (provider->CopyProperties(&propsRaw) != kIOReturnSuccess || !propsRaw) {
        return;
    }

    OSSharedPtr<OSDictionary> props(propsRaw, OSNoRetain);
    if (auto* guid = OSDynamicCast(OSNumber, props->getObject("ASFWGUID"))) {
        outConfig.guid = guid->unsigned64BitValue();
    }
    if (auto* vendor = OSDynamicCast(OSNumber, props->getObject("ASFWVendorID"))) {
        outConfig.vendorId = vendor->unsigned32BitValue();
    }
    if (auto* model = OSDynamicCast(OSNumber, props->getObject("ASFWModelID"))) {
        outConfig.modelId = model->unsigned32BitValue();
    }
    if (auto* inputs = OSDynamicCast(OSNumber, props->getObject("ASFWMidiInputPorts"))) {
        outConfig.inputPorts = inputs->unsigned32BitValue();
    }
    if (auto* outputs = OSDynamicCast(OSNumber, props->getObject("ASFWMidiOutputPorts"))) {
        outConfig.outputPorts = outputs->unsigned32BitValue();
    }
    if (auto* name = OSDynamicCast(OSString, props->getObject("DeviceName"))) {
        strlcpy(outConfig.deviceName, name->getCStringNoCopy(), sizeof(outConfig.deviceName));
    } else if (auto* name = OSDynamicCast(OSString, props->getObject("ASFWDeviceName"))) {
        strlcpy(outConfig.deviceName, name->getCStringNoCopy(), sizeof(outConfig.deviceName));
    }
}

void LogMidiDiagnosticSnapshot(const ASFWMIDIDriver_IVars* ivars, const char* phase) noexcept {
    if (!ivars) {
        ASFW_LOG(Audio,
                 "ASFWMIDIDriver: MIDI DIAGNOSTIC phase=%{public}s status=no_ivars",
                 phase ? phase : "unknown");
        return;
    }

    const bool hasObjects = ivars->device && ivars->entity && ivars->source && ivars->destination;
    const bool hasPorts = (ivars->inputPorts > 0 || ivars->outputPorts > 0);
    const bool hasBridge = (ivars->isochService != nullptr || ivars->midiNubProvider != nullptr);
    const char* status = (hasObjects && hasPorts && hasBridge) ? "ready" : "partial";

    ASFW_LOG(Audio,
             "ASFWMIDIDriver: MIDI DIAGNOSTIC phase=%{public}s status=%{public}s device='%{public}s' source='%{public}s' destination='%{public}s' GUID=0x%016llx vendorID=%u modelID=%u portsIn=%u portsOut=%u objects=%{public}s bridge=%{public}s rxSelfTest=%{public}s txWords=%llu txDecoded=%llu txQueued=%llu txUnsupported=%llu rxBytes=%llu rxWords=%llu rxUnsupported=%llu rxSendFailures=%llu",
             phase ? phase : "unknown",
             status,
             ivars->deviceName,
             ivars->sourceName,
             ivars->destinationName,
             ivars->guid,
             ivars->vendorId,
             ivars->modelId,
             ivars->inputPorts,
             ivars->outputPorts,
             hasObjects ? "yes" : "no",
             hasBridge ? "yes" : "no",
             ivars->rxSelfTestSent ? "sent" : "pending",
             ivars->txWordsReceived,
             ivars->txBytesDecoded,
             ivars->txBytesQueued,
             ivars->txUnsupportedWords,
             ivars->rxBytesReceived,
             ivars->rxWordsSent,
             ivars->rxUnsupportedMessages,
             ivars->rxSendFailures);
}

kern_return_t RemoveMIDIObject(IOUserMIDIDriver* driver, IOUserMIDIObject* object) noexcept {
    if (!driver || !object) {
        return kIOReturnSuccess;
    }
    return driver->RemoveObject(object);
}

ASFW::Driver::IsochService* GetIsochServiceFromProvider(IOService* provider,
                                                        const ParsedMidiDriverConfig& config) noexcept {
    auto* nub = OSDynamicCast(ASFWMIDINub, provider);
    if (!nub) {
        if (auto* registered = static_cast<ASFW::Driver::IsochService*>(
                ASFW::MIDI::ResolveIsochServiceForGuid(config.guid))) {
            ASFW_LOG(Audio,
                     "ASFWMIDIDriver: Isoch service resolved from GUID registry GUID=0x%016llx service=%p",
                     config.guid,
                     registered);
            return registered;
        }
        ASFW_LOG(Audio,
                 "ASFWMIDIDriver: direct Isoch service unavailable across provider boundary GUID=0x%016llx; nub RPC bridge will be used if available",
                 config.guid);
        return nullptr;
    }

    if (auto* direct = static_cast<ASFW::Driver::IsochService*>(nub->GetIsochService())) {
        ASFW_LOG(Audio, "ASFWMIDIDriver: Isoch service resolved from MIDI nub direct pointer %p", direct);
        return direct;
    }

    ASFWDriver* parent = nub->GetParentDriver();
    if (!parent) {
        ASFW_LOG(Audio, "ASFWMIDIDriver: Isoch resolve failed - MIDI nub has no ASFWDriver parent");
        return nullptr;
    }

    auto* context = static_cast<ServiceContext*>(parent->GetServiceContext());
    if (!context) {
        ASFW_LOG(Audio, "ASFWMIDIDriver: Isoch resolve failed - ASFWDriver has no ServiceContext");
        return nullptr;
    }
    ASFW_LOG(Audio, "ASFWMIDIDriver: Isoch service resolved from parent driver context %p", &context->isoch);
    return &context->isoch;
}

void DeliverRxMidiBytesToSource(void* context, const uint8_t* bytes, uint8_t count) noexcept {
    auto* ivars = static_cast<ASFWMIDIDriver_IVars*>(context);
    if (!ivars || !bytes || count == 0) {
        return;
    }

    ivars->rxBytesReceived += count;
    const auto encoded = ASFW::MIDI::Midi1ToUMP::EncodeBytes(bytes, count);
    if (encoded.count == 0) {
        ++ivars->rxUnsupportedMessages;
        ASFW_LOG(Audio,
                 "ASFWMIDIDriver: RX MIDI unsupported bytes=%02x %02x %02x count=%u unsupportedTotal=%llu",
                 bytes[0],
                 count > 1 ? bytes[1] : 0,
                 count > 2 ? bytes[2] : 0,
                 count,
                 ivars->rxUnsupportedMessages);
        return;
    }

    if (!ivars->source) {
        ASFW_LOG(Audio, "ASFWMIDIDriver: RX MIDI dropped - source not ready");
        return;
    }

    const kern_return_t kr = ivars->source->Send(encoded.words, encoded.count);
    if (kr == kIOReturnSuccess) {
        ivars->rxWordsSent += encoded.count;
    } else {
        ++ivars->rxSendFailures;
    }
    ASFW_LOG(Audio,
             "ASFWMIDIDriver: RX MIDI bytes=%02x %02x %02x count=%u ump=0x%08x send=0x%x totals rxBytes=%llu rxWords=%llu unsupported=%llu sendFailures=%llu",
             bytes[0],
             count > 1 ? bytes[1] : 0,
             count > 2 ? bytes[2] : 0,
             count,
             encoded.words[0],
             kr,
             ivars->rxBytesReceived,
             ivars->rxWordsSent,
             ivars->rxUnsupportedMessages,
             ivars->rxSendFailures);
}

void DrainRxMidiBridge(ASFWMIDIDriver_IVars* ivars) noexcept {
    if (!ivars || !ivars->midiNubProvider) {
        return;
    }

    uint32_t delivered = 0;
    for (; delivered < kMidiRxPollBudget; ++delivered) {
        uint32_t packed = 0;
        uint32_t byteCount = 0;
        uint32_t dropped = 0;
        const kern_return_t kr =
            ivars->midiNubProvider->PopReceivedMIDIBytes(&packed, &byteCount, &dropped);
        ivars->rxBridgeDropped += dropped;

        if (kr != kIOReturnSuccess) {
            ASFW_LOG(Audio, "ASFWMIDIDriver: RX MIDI bridge pop failed kr=0x%x", kr);
            break;
        }
        if (byteCount == 0) {
            ++ivars->rxBridgeEmptyPolls;
            break;
        }

        uint8_t bytes[3] = {
            static_cast<uint8_t>(packed & 0xffu),
            static_cast<uint8_t>((packed >> 8) & 0xffu),
            static_cast<uint8_t>((packed >> 16) & 0xffu),
        };
        DeliverRxMidiBytesToSource(ivars, bytes, static_cast<uint8_t>(byteCount));
    }

    ++ivars->rxBridgePolls;
    if (delivered > 0 || ivars->rxBridgeDropped > 0) {
        ASFW_LOG(Audio,
                 "ASFWMIDIDriver: RX MIDI bridge poll delivered=%u totalPolls=%llu emptyPolls=%llu dropped=%llu",
                 delivered,
                 ivars->rxBridgePolls,
                 ivars->rxBridgeEmptyPolls,
                 ivars->rxBridgeDropped);
    }
}

void RunRxSelfTest(ASFWMIDIDriver_IVars* ivars) noexcept {
    if (!ivars || ivars->rxSelfTestSent || !ASFW::LogConfig::Shared().IsMIDIRxSelfTestEnabled()) {
        return;
    }

    constexpr uint8_t kNoteOn[] = {0x90, 0x3c, 0x40};
    constexpr uint8_t kNoteOff[] = {0x80, 0x3c, 0x00};
    ivars->rxSelfTestSent = true;

    ASFW_LOG(Audio, "ASFWMIDIDriver: RX self-test sending synthetic source note pair");
    DeliverRxMidiBytesToSource(ivars, kNoteOn, sizeof(kNoteOn));
    DeliverRxMidiBytesToSource(ivars, kNoteOff, sizeof(kNoteOff));
}

} // namespace

bool ASFWMIDIDriver::init()
{
    if (!super::init()) {
        ASFW_LOG_ERROR(Audio, "ASFWMIDIDriver: super::init() failed");
        return false;
    }

    ivars = IONewZero(ASFWMIDIDriver_IVars, 1);
    if (!ivars) {
        ASFW_LOG_ERROR(Audio, "ASFWMIDIDriver: Failed to allocate ivars");
        return false;
    }

    strlcpy(ivars->deviceName, kDefaultDeviceName, sizeof(ivars->deviceName));
    strlcpy(ivars->sourceName, "MIDI Source", sizeof(ivars->sourceName));
    strlcpy(ivars->destinationName, "MIDI Destination", sizeof(ivars->destinationName));
    ASFW_LOG(Audio, "ASFWMIDIDriver: init() succeeded");
    return true;
}

void ASFWMIDIDriver::free()
{
    ASFW_LOG(Audio, "ASFWMIDIDriver: free()");
    if (ivars) {
        ivars->rxPollActive = false;
        if (ivars->rxPollTimer) {
            ivars->rxPollTimer->SetEnableWithCompletion(false, nullptr);
        }
        ivars->rxPollAction.reset();
        ivars->rxPollTimer.reset();
        ivars->destination.reset();
        ivars->source.reset();
        ivars->entity.reset();
        ivars->device.reset();
        IOSafeDeleteNULL(ivars, ASFWMIDIDriver_IVars, 1);
    }
    super::free();
}

kern_return_t IMPL(ASFWMIDIDriver, Start)
{
    ASFW_LOG(Audio, "ASFWMIDIDriver: Start() - provider is ASFWMIDINub");

    kern_return_t kr = Start(provider, SUPERDISPATCH);
    if (kr != kIOReturnSuccess) {
        ASFW_LOG_ERROR(Audio, "ASFWMIDIDriver: super::Start() failed: 0x%x", kr);
        return kr;
    }

    ParsedMidiDriverConfig config{};
    ParseMidiConfigFromProvider(provider, config);
    ivars->guid = config.guid;
    ivars->vendorId = config.vendorId;
    ivars->modelId = config.modelId;
    ivars->inputPorts = config.inputPorts;
    ivars->outputPorts = config.outputPorts;
    // DriverKit exposes the provider across a service boundary, so this mirrors
    // ASFWAudioDriver: use the generated proxy type for RPC methods, not
    // OSDynamicCast/raw local ivar access.
    ivars->midiNubProvider = reinterpret_cast<ASFWMIDINub*>(provider);
    ivars->isochService = GetIsochServiceFromProvider(provider, config);
    strlcpy(ivars->deviceName, config.deviceName, sizeof(ivars->deviceName));

    auto driverName = MakeString("ASFW FireWire MIDI");
    if (driverName) {
        SetName(driverName.get());
    }

    auto deviceUID = MakeFormattedString("ASFW-MIDI-%llu", ivars->guid);
    auto modelUID = MakeFormattedString("ASFW-MIDI-MODEL-%llu", ivars->modelId);
    auto manufacturerUID = MakeString("ASFW FireWire");
    auto entityName = MakeString(ivars->deviceName);
    strlcpy(ivars->sourceName,
            (ivars->outputPorts > 0) ? kAppleStyleSourceName : "MIDI Source",
            sizeof(ivars->sourceName));
    strlcpy(ivars->destinationName,
            (ivars->inputPorts > 0) ? kAppleStyleDestinationName : "MIDI Destination",
            sizeof(ivars->destinationName));
    auto sourceName = MakeString(ivars->sourceName);
    auto destinationName = MakeString(ivars->destinationName);

    if (!deviceUID || !modelUID || !manufacturerUID || !entityName || !sourceName || !destinationName) {
        ASFW_LOG_ERROR(Audio, "ASFWMIDIDriver: Failed to allocate MIDI names");
        return kIOReturnNoMemory;
    }

    ivars->device = IOUserMIDIDevice::Create(this, deviceUID.get(), modelUID.get(), manufacturerUID.get());
    if (!ivars->device) {
        ASFW_LOG_ERROR(Audio, "ASFWMIDIDriver: Failed to create IOUserMIDIDevice");
        return kIOReturnNoMemory;
    }

    ivars->entity = IOUserMIDIEntity::Create(this,
                                             ivars->device.get(),
                                             entityName.get(),
                                             MIDIDriverKit::IOUserMIDIProtocolID::MIDIProtocol_1_0,
                                             0,
                                             0);
    ivars->source = IOUserMIDISource::Create(this,
                                             sourceName.get(),
                                             MIDIDriverKit::IOUserMIDIProtocolID::MIDIProtocol_1_0);
    ivars->destination = IOUserMIDIDestination::Create(this,
                                                       destinationName.get(),
                                                       MIDIDriverKit::IOUserMIDIProtocolID::MIDIProtocol_1_0);
    if (!ivars->entity || !ivars->source || !ivars->destination) {
        ASFW_LOG_ERROR(Audio, "ASFWMIDIDriver: Failed to create MIDI entity/source/destination");
        return kIOReturnNoMemory;
    }

    kr = ivars->destination->SetIOBlock(^(IOUserMIDIUMPWord const* umpWords, size_t numWords) {
        if (!umpWords || numWords == 0) {
            return kIOReturnSuccess;
        }

        uint32_t decodedBytesThisBlock = 0;
        uint32_t queuedBytesThisBlock = 0;
        uint32_t unsupportedThisBlock = 0;
        for (size_t i = 0; i < numWords; ++i) {
            const auto decoded = ASFW::MIDI::UMPToMidi1::DecodeWord(static_cast<uint32_t>(umpWords[i]));
            ++ivars->txWordsReceived;
            if (decoded.count == 0) {
                ++ivars->txUnsupportedWords;
                ++unsupportedThisBlock;
                continue;
            }
            ivars->txBytesDecoded += decoded.count;
            decodedBytesThisBlock += decoded.count;
            auto* isochService = static_cast<ASFW::Driver::IsochService*>(ivars->isochService);
            if (isochService) {
                const uint32_t queued = isochService->PushTransmitMidiBytes(decoded.bytes, decoded.count);
                ivars->txBytesQueued += queued;
                queuedBytesThisBlock += queued;
            } else if (ivars->midiNubProvider) {
                const uint32_t packed =
                    static_cast<uint32_t>(decoded.bytes[0]) |
                    (static_cast<uint32_t>(decoded.count > 1 ? decoded.bytes[1] : 0) << 8) |
                    (static_cast<uint32_t>(decoded.count > 2 ? decoded.bytes[2] : 0) << 16);
                uint32_t queued = 0;
                const kern_return_t bridgeKr =
                    ivars->midiNubProvider->PushTransmitMIDIBytes(packed, decoded.count, &queued);
                if (bridgeKr == kIOReturnSuccess) {
                    ivars->txBytesQueued += queued;
                    queuedBytesThisBlock += queued;
                } else {
                    ASFW_LOG(Audio,
                             "ASFWMIDIDriver: TX MIDI bridge failed kr=0x%x bytes=%02x %02x %02x count=%u",
                             bridgeKr,
                             decoded.bytes[0],
                             decoded.count > 1 ? decoded.bytes[1] : 0,
                             decoded.count > 2 ? decoded.bytes[2] : 0,
                             decoded.count);
                }
            }
        }
        ASFW_LOG(Audio,
                 "ASFWMIDIDriver: TX UMP words=%zu decodedBytes=%u queuedBytes=%u unsupported=%u totals words=%llu decoded=%llu queued=%llu unsupported=%llu",
                 numWords,
                 decodedBytesThisBlock,
                 queuedBytesThisBlock,
                 unsupportedThisBlock,
                 ivars->txWordsReceived,
                 ivars->txBytesDecoded,
                 ivars->txBytesQueued,
                 ivars->txUnsupportedWords);
        return kIOReturnSuccess;
    });
    if (kr != kIOReturnSuccess) {
        ASFW_LOG_ERROR(Audio, "ASFWMIDIDriver: SetIOBlock failed: 0x%x", kr);
        return kr;
    }

    kr = ivars->entity->AddSource(ivars->source.get());
    if (kr != kIOReturnSuccess) {
        ASFW_LOG_ERROR(Audio, "ASFWMIDIDriver: AddSource failed: 0x%x", kr);
        return kr;
    }
    kr = ivars->entity->AddDestination(ivars->destination.get());
    if (kr != kIOReturnSuccess) {
        ASFW_LOG_ERROR(Audio, "ASFWMIDIDriver: AddDestination failed: 0x%x", kr);
        return kr;
    }
    kr = ivars->device->AddEntity(ivars->entity.get());
    if (kr != kIOReturnSuccess) {
        ASFW_LOG_ERROR(Audio, "ASFWMIDIDriver: AddEntity failed: 0x%x", kr);
        return kr;
    }

    if ((kr = AddObject(ivars->device.get())) != kIOReturnSuccess ||
        (kr = AddObject(ivars->entity.get())) != kIOReturnSuccess ||
        (kr = AddObject(ivars->source.get())) != kIOReturnSuccess ||
        (kr = AddObject(ivars->destination.get())) != kIOReturnSuccess) {
        ASFW_LOG_ERROR(Audio, "ASFWMIDIDriver: AddObject failed: 0x%x", kr);
        return kr;
    }

    if (auto* isochService = static_cast<ASFW::Driver::IsochService*>(ivars->isochService)) {
        isochService->SetReceiveMidiSink(ivars, DeliverRxMidiBytesToSource);
        ASFW_LOG(Audio, "ASFWMIDIDriver: RX MIDI sink attached directly to IsochService");
    } else if (ivars->midiNubProvider) {
        auto workQueue = GetWorkQueue();
        if (!workQueue) {
            ASFW_LOG(Audio, "ASFWMIDIDriver: RX MIDI bridge polling not started - no work queue");
        } else {
            IOTimerDispatchSource* timerSource = nullptr;
            kern_return_t timerKr = IOTimerDispatchSource::Create(workQueue.get(), &timerSource);
            if (timerKr != kIOReturnSuccess || !timerSource) {
                ASFW_LOG(Audio,
                         "ASFWMIDIDriver: RX MIDI bridge polling timer create failed kr=0x%x",
                         timerKr);
            } else {
                ivars->rxPollTimer = OSSharedPtr(timerSource, OSNoRetain);

                OSAction* timerAction = nullptr;
                timerKr = CreateActionMidiRxPollTimerFired(0, &timerAction);
                if (timerKr != kIOReturnSuccess || !timerAction) {
                    ASFW_LOG(Audio,
                             "ASFWMIDIDriver: RX MIDI bridge polling action create failed kr=0x%x",
                             timerKr);
                    ivars->rxPollTimer.reset();
                } else {
                    ivars->rxPollAction = OSSharedPtr(timerAction, OSNoRetain);
                    timerKr = ivars->rxPollTimer->SetHandler(ivars->rxPollAction.get());
                    if (timerKr != kIOReturnSuccess) {
                        ASFW_LOG(Audio,
                                 "ASFWMIDIDriver: RX MIDI bridge polling SetHandler failed kr=0x%x",
                                 timerKr);
                        ivars->rxPollAction.reset();
                        ivars->rxPollTimer.reset();
                    } else if ((timerKr = ivars->rxPollTimer->SetEnableWithCompletion(true, nullptr)) !=
                               kIOReturnSuccess) {
                        ASFW_LOG(Audio,
                                 "ASFWMIDIDriver: RX MIDI bridge polling enable failed kr=0x%x",
                                 timerKr);
                        ivars->rxPollAction.reset();
                        ivars->rxPollTimer.reset();
                    } else {
                        ivars->rxPollActive = true;
                        ScheduleMidiRxPoll(ivars);
                        ASFW_LOG(Audio,
                                 "ASFWMIDIDriver: RX MIDI bridge polling attached intervalUsec=%llu budget=%u",
                                 kMidiRxPollIntervalUsec,
                                 kMidiRxPollBudget);
                    }
                }
            }
        }
    } else {
        ASFW_LOG(Audio, "ASFWMIDIDriver: RX MIDI sink not attached - no direct service or nub bridge");
    }

    ASFW_LOG(Audio,
             "✅ ASFWMIDIDriver: Started - device '%{public}s' GUID=0x%016llx midiIn=%u midiOut=%u",
             ivars->deviceName,
             ivars->guid,
             ivars->inputPorts,
             ivars->outputPorts);
    RunRxSelfTest(ivars);
    LogMidiDiagnosticSnapshot(ivars, "Start");
    return kIOReturnSuccess;
}

kern_return_t IMPL(ASFWMIDIDriver, Stop)
{
    ASFW_LOG(Audio, "ASFWMIDIDriver: Stop()");
    if (ivars) {
        ivars->rxPollActive = false;
        if (ivars->rxPollTimer) {
            ivars->rxPollTimer->SetEnableWithCompletion(false, nullptr);
        }
        ivars->rxPollAction.reset();
        ivars->rxPollTimer.reset();
        if (auto* isochService = static_cast<ASFW::Driver::IsochService*>(ivars->isochService)) {
            isochService->SetReceiveMidiSink(nullptr, nullptr);
        }
        RemoveMIDIObject(this, ivars->destination.get());
        RemoveMIDIObject(this, ivars->source.get());
        RemoveMIDIObject(this, ivars->entity.get());
        RemoveMIDIObject(this, ivars->device.get());

        ivars->destination.reset();
        ivars->source.reset();
        ivars->entity.reset();
        ivars->device.reset();
        ivars->isochService = nullptr;
        ivars->midiNubProvider = nullptr;
    }
    return Stop(provider, SUPERDISPATCH);
}

kern_return_t IMPL(ASFWMIDIDriver, NewUserClient)
{
    ASFW_LOG(Audio, "ASFWMIDIDriver: NewUserClient(type=%u)", type);
    if (type == kIOUserMIDIDriverUserClientType) {
        return super::NewUserClient(type, userClient, SUPERDISPATCH);
    }
    return kIOReturnBadArgument;
}

kern_return_t ASFWMIDIDriver::StartIO(OSArray* deviceList)
{
    const uint32_t deviceCount = deviceList ? deviceList->getCount() : 0;
    ASFW_LOG(Audio, "ASFWMIDIDriver: StartIO(deviceListCount=%u)", deviceCount);
    const kern_return_t kr = super::StartIO(deviceList);
    if (kr == kIOReturnSuccess) {
        RunRxSelfTest(ivars);
    }
    LogMidiDiagnosticSnapshot(ivars, (kr == kIOReturnSuccess) ? "StartIO" : "StartIOFailed");
    return kr;
}

kern_return_t ASFWMIDIDriver::StopIO()
{
    ASFW_LOG(Audio, "ASFWMIDIDriver: StopIO()");
    LogMidiDiagnosticSnapshot(ivars, "StopIO");
    return super::StopIO();
}

void ASFWMIDIDriver::MidiRxPollTimerFired_Impl([[maybe_unused]] OSAction* action,
                                               [[maybe_unused]] uint64_t time)
{
    if (!ivars || !ivars->rxPollActive) {
        return;
    }

    DrainRxMidiBridge(ivars);
    ScheduleMidiRxPoll(ivars);
}
