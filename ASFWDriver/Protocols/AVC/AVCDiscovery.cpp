//
// AVCDiscovery.cpp
// ASFWDriver - AV/C Protocol Layer
//
// AV/C Discovery implementation
//

#include "AVCDiscovery.hpp"
#include "../../Logging/Logging.hpp"
#include "../../Audio/Model/ASFWAudioDevice.hpp"
#include "../Audio/DeviceProtocolFactory.hpp"
#include "../Audio/BeBoB/BeBoBTypes.hpp"
#include "../Audio/Oxford/Apogee/ApogeeDuetProtocol.hpp"
#include "../Audio/DeviceStreamModeQuirks.hpp"
#include "../../Discovery/DeviceRegistry.hpp"
#include "../../Discovery/DiscoveryTypes.hpp"
#include "AVCCommand.hpp"
#include "Music/MusicSubunit.hpp"
#include "StreamFormats/AVCSignalFormatCommand.hpp"
#include "StreamFormats/AVCStreamFormatCommands.hpp"
#include <DriverKit/IOService.h>
#include <DriverKit/OSSharedPtr.h>
#include <DriverKit/OSString.h>
#include <DriverKit/OSNumber.h>
#include <DriverKit/OSArray.h>
#include <DriverKit/OSDictionary.h>
#include <set>
#include <algorithm>
#include <atomic>
#include <functional>

using namespace ASFW::Protocols::AVC;

namespace {

ASFW::Audio::Model::StreamMode ResolveStreamMode(
    const ASFW::Protocols::AVC::Music::MusicSubunitCapabilities& caps,
    uint32_t vendorId,
    uint32_t modelId,
    const char*& reason) noexcept {
    if (auto forced = ASFW::Audio::Quirks::LookupForcedStreamMode(vendorId, modelId); forced.has_value()) {
        reason = "quirk";
        ASFW_LOG_WARNING(Audio,
                         "AVCDiscovery: QUIRK OVERRIDE stream mode vendor=0x%06x model=0x%06x forced=%{public}s",
                         vendorId, modelId,
                         ASFW::Audio::Quirks::StreamModeToString(*forced));
        return *forced;
    }

    // Use transmit capability as mode selection signal. This mode is currently
    // used by the host IT stream and is expected to match RX in practical devices.
    const bool supportsBlocking = caps.SupportsBlockingTransmit();
    const bool supportsNonBlocking = caps.SupportsNonBlockingTransmit();

    if (supportsBlocking && !supportsNonBlocking) {
        reason = "avc-blocking-only";
        return ASFW::Audio::Model::StreamMode::kBlocking;
    }

    if (supportsNonBlocking) {
        reason = supportsBlocking ? "avc-both-prefer-nonblocking" : "avc-nonblocking-only";
        return ASFW::Audio::Model::StreamMode::kNonBlocking;
    }

    reason = "default-nonblocking";
    return ASFW::Audio::Model::StreamMode::kNonBlocking;
}

struct PlugChannelSummary {
    uint32_t inputAudioMaxChannels{0};   // Subunit input audio stream width
    uint32_t outputAudioMaxChannels{0};  // Subunit output audio stream width
    uint32_t inputAudioPlugs{0};
    uint32_t outputAudioPlugs{0};
};

[[nodiscard]] uint32_t ExtractPlugChannelCount(
    const ASFW::Protocols::AVC::StreamFormats::PlugInfo& plug) noexcept {
    if (!plug.currentFormat.has_value()) {
        return 0;
    }

    const auto& fmt = *plug.currentFormat;
    if (fmt.totalChannels > 0) {
        return fmt.totalChannels;
    }

    uint32_t sum = 0;
    for (const auto& block : fmt.channelFormats) {
        sum += block.channelCount;
    }
    return sum;
}

[[nodiscard]] PlugChannelSummary SummarizePlugChannels(
    const std::vector<ASFW::Protocols::AVC::StreamFormats::PlugInfo>& plugs) noexcept {
    PlugChannelSummary summary{};
    for (const auto& plug : plugs) {
        if (plug.type != ASFW::Protocols::AVC::StreamFormats::MusicPlugType::kAudio) {
            continue;
        }

        const uint32_t channels = ExtractPlugChannelCount(plug);
        if (channels == 0) {
            continue;
        }

        if (plug.IsInput()) {
            ++summary.inputAudioPlugs;
            summary.inputAudioMaxChannels = std::max(summary.inputAudioMaxChannels, channels);
        } else if (plug.IsOutput()) {
            ++summary.outputAudioPlugs;
            summary.outputAudioMaxChannels = std::max(summary.outputAudioMaxChannels, channels);
        }
    }
    return summary;
}

[[nodiscard]] std::vector<uint8_t> FindRawFormatBlockForRate(
    const std::vector<ASFW::Protocols::AVC::StreamFormats::PlugInfo>& plugs,
    bool wantInput,
    uint32_t targetRateHz) {
    for (const auto& plug : plugs) {
        if (plug.type != ASFW::Protocols::AVC::StreamFormats::MusicPlugType::kAudio ||
            plug.IsInput() != wantInput) {
            continue;
        }

        for (const auto& format : plug.supportedFormats) {
            if (format.GetSampleRateHz() == targetRateHz && !format.rawFormatBlock.empty()) {
                return format.rawFormatBlock;
            }
        }

        if (plug.currentFormat.has_value() &&
            plug.currentFormat->GetSampleRateHz() == targetRateHz &&
            !plug.currentFormat->rawFormatBlock.empty()) {
            return plug.currentFormat->rawFormatBlock;
        }
    }

    return {};
}

struct ParsedDiscoveryFormatRecord {
    uint8_t subfunction{0};
    ASFW::Protocols::AVC::StreamFormats::AudioStreamFormat format;
};

[[nodiscard]] std::optional<ParsedDiscoveryFormatRecord> ParseDiscoveryFormatRecord(
    const ASFW::Protocols::AVC::AVCUnit::AppleDiscoveryFormatRecord& record) {
    if (!record.valid ||
        record.rawResponse.size() < ASFW::Protocols::AVC::kAVCFrameMinSize ||
        record.rawResponse.size() > ASFW::Protocols::AVC::kAVCFrameMaxSize) {
        return std::nullopt;
    }

    ASFW::Protocols::AVC::FCPFrame frame;
    frame.length = record.rawResponse.size();
    std::copy(record.rawResponse.begin(), record.rawResponse.end(), frame.data.begin());

    auto cdb = ASFW::Protocols::AVC::AVCCdb::Decode(frame);
    if (!cdb.has_value() || cdb->operandLength == 0) {
        return std::nullopt;
    }

    const uint8_t subfunction = cdb->operands[0];
    size_t formatOffset = 0;
    if (subfunction == ASFW::Protocols::AVC::StreamFormats::kStreamFormatSubfunc_Current) {
        formatOffset = 7;
    } else if (subfunction == ASFW::Protocols::AVC::StreamFormats::kStreamFormatSubfunc_Supported) {
        formatOffset = 8;
    } else {
        return std::nullopt;
    }

    if (cdb->operandLength <= formatOffset) {
        return std::nullopt;
    }

    auto format = ASFW::Protocols::AVC::StreamFormats::StreamFormatParser::Parse(
        cdb->operands.data() + formatOffset,
        cdb->operandLength - formatOffset);
    if (!format.has_value()) {
        return std::nullopt;
    }

    return ParsedDiscoveryFormatRecord{
        .subfunction = subfunction,
        .format = *format,
    };
}

void CollectUnitIsochSampleRates(const ASFW::Protocols::AVC::AVCUnit& avcUnit,
                                 std::set<double>& rateSet) {
    for (const auto& record : avcUnit.GetAppleDiscoveryUnitIsochFormats()) {
        auto parsed = ParseDiscoveryFormatRecord(record);
        if (!parsed.has_value() ||
            parsed->subfunction != ASFW::Protocols::AVC::StreamFormats::kStreamFormatSubfunc_Supported) {
            continue;
        }

        const uint32_t rateHz = parsed->format.GetSampleRateHz();
        if (rateHz > 0) {
            rateSet.insert(static_cast<double>(rateHz));
        }
    }
}

[[nodiscard]] std::optional<uint32_t> FindCurrentRateFromUnitIsochFormats(
    const ASFW::Protocols::AVC::AVCUnit& avcUnit) {
    for (const auto& record : avcUnit.GetAppleDiscoveryUnitIsochFormats()) {
        auto parsed = ParseDiscoveryFormatRecord(record);
        if (!parsed.has_value() ||
            parsed->subfunction != ASFW::Protocols::AVC::StreamFormats::kStreamFormatSubfunc_Current) {
            continue;
        }

        const uint32_t rateHz = parsed->format.GetSampleRateHz();
        if (rateHz > 0) {
            return rateHz;
        }
    }

    return std::nullopt;
}

[[nodiscard]] std::vector<uint8_t> FindRawUnitIsochFormatBlockForRate(
    const ASFW::Protocols::AVC::AVCUnit& avcUnit,
    bool wantInput,
    uint32_t targetRateHz) {
    const uint8_t direction = wantInput ? 0x00 : 0x01;

    for (const auto& record : avcUnit.GetAppleDiscoveryUnitIsochFormats()) {
        if (record.direction != direction) {
            continue;
        }

        auto parsed = ParseDiscoveryFormatRecord(record);
        if (!parsed.has_value() ||
            parsed->subfunction != ASFW::Protocols::AVC::StreamFormats::kStreamFormatSubfunc_Supported) {
            continue;
        }

        if (parsed->format.GetSampleRateHz() == targetRateHz &&
            !parsed->format.rawFormatBlock.empty()) {
            return parsed->format.rawFormatBlock;
        }
    }

    for (const auto& record : avcUnit.GetAppleDiscoveryUnitIsochFormats()) {
        if (record.direction != direction) {
            continue;
        }

        auto parsed = ParseDiscoveryFormatRecord(record);
        if (!parsed.has_value() ||
            parsed->subfunction != ASFW::Protocols::AVC::StreamFormats::kStreamFormatSubfunc_Current) {
            continue;
        }

        if (parsed->format.GetSampleRateHz() == targetRateHz &&
            !parsed->format.rawFormatBlock.empty()) {
            return parsed->format.rawFormatBlock;
        }
    }

    return {};
}

[[nodiscard]] bool IsPrismOrpheus(uint32_t vendorId, uint32_t modelId) noexcept {
    return vendorId == ASFW::Audio::BeBoB::kPrismSoundVendorId &&
           modelId == ASFW::Audio::BeBoB::kOrpheusModelId;
}

} // namespace

//==============================================================================
// Constants
//==============================================================================

/// 1394 Trade Association spec ID (24-bit)
constexpr uint32_t kAVCSpecID = 0x00A02D;
constexpr uint32_t kDuetPrefetchTimeoutMs = 1200;

/// Post-reset stabilization delay for devices that need time before accepting
/// AV/C commands (e.g., Prism Sound Orpheus — hardware initialization window).
constexpr uint32_t kOrpheusInitDelayMs = 5000;
constexpr uint32_t kClassIdPhantomPower = static_cast<uint32_t>('phan');
constexpr uint32_t kClassIdPhaseInvert = static_cast<uint32_t>('phsi');
constexpr uint32_t kScopeInput = static_cast<uint32_t>('inpt');
constexpr uint32_t kDuetPhantomMask = 0x3u;

void ConfigureDuetPhantomOverrides(
    ASFW::Audio::Model::ASFWAudioDevice& config,
    const std::optional<ASFW::Audio::Oxford::Apogee::InputParams>& inputParams) {
    config.hasPhantomOverride = true;
    config.phantomSupportedMask = kDuetPhantomMask;

    uint32_t initialMask = 0;
    if (inputParams.has_value()) {
        const auto& params = *inputParams;
        for (uint32_t index = 0; index < 2; ++index) {
            if (params.phantomPowerings[index]) {
                initialMask |= (1u << index);
            }
        }
    }
    config.phantomInitialMask = initialMask;

    config.boolControlOverrides.clear();
    config.boolControlOverrides.reserve(4);
    for (uint32_t element = 1; element <= 2; ++element) {
        const uint32_t bit = 1u << (element - 1u);
        bool polarityInitial = false;
        if (inputParams.has_value()) {
            polarityInitial = inputParams->polarities[element - 1u];
        }
        config.boolControlOverrides.push_back({
            .classIdFourCC = kClassIdPhantomPower,
            .scopeFourCC = kScopeInput,
            .element = element,
            .isSettable = true,
            .initialValue = (initialMask & bit) != 0u,
        });
        config.boolControlOverrides.push_back({
            .classIdFourCC = kClassIdPhaseInvert,
            .scopeFourCC = kScopeInput,
            .element = element,
            .isSettable = true,
            .initialValue = polarityInitial,
        });
    }
}

//==============================================================================
// Constructor / Destructor
//==============================================================================

AVCDiscovery::AVCDiscovery(IOService* driver,
                           Discovery::IDeviceManager& deviceManager,
                           Discovery::DeviceRegistry& registry,
                           Protocols::Ports::FireWireBusOps& busOps,
                           Protocols::Ports::FireWireBusInfo& busInfo,
                           ASFW::Audio::IAVCAudioConfigListener* audioConfigListener)
    : driver_(driver)
    , deviceManager_(deviceManager)
    , registry_(registry)
    , busOps_(busOps)
    , busInfo_(busInfo)
    , audioConfigListener_(audioConfigListener) {

    // Allocate lock
    lock_ = IOLockAlloc();
    if (!lock_) {
        os_log_error(log_, "AVCDiscovery: Failed to allocate lock");
    }

    IODispatchQueue* queue = nullptr;
    auto kr = IODispatchQueue::Create("com.asfw.avc.rescan", 0, 0, &queue);
    if (kr == kIOReturnSuccess && queue) {
        rescanQueue_ = OSSharedPtr(queue, OSNoRetain);
    } else if (kr != kIOReturnSuccess) {
        os_log_error(log_, "AVCDiscovery: Failed to create rescan queue (0x%x)", kr);
    }

    // Register as discovery observers
    deviceManager_.RegisterUnitObserver(this);
    deviceManager_.RegisterDeviceObserver(this);

    os_log_info(log_, "AVCDiscovery: Initialized");
}

AVCDiscovery::~AVCDiscovery() {
    // Unregister discovery observers
    deviceManager_.UnregisterDeviceObserver(this);
    deviceManager_.UnregisterUnitObserver(this);

    // Clean up lock
    if (lock_) {
        IOLockFree(lock_);
        lock_ = nullptr;
    }

    os_log_info(log_, "AVCDiscovery: Destroyed");
}

//==============================================================================
// IUnitObserver Interface
//==============================================================================

void AVCDiscovery::OnUnitPublished(std::shared_ptr<Discovery::FWUnit> unit) {
    if (!IsAVCUnit(unit)) {
        return;
    }

    uint64_t guid = GetUnitGUID(unit);

    ASFW_LOG(Async,
             "✅ AV/C DETECTED: GUID=%llx, specID=0x%06x - SCANNING...",
             guid, unit->GetUnitSpecID());

    // Get parent device
    auto device = unit->GetDevice();
    if (!device) {
        os_log_error(log_, "AVCDiscovery: Unit has no parent device");
        return;
    }

    // Create AVCUnit
    auto avcUnit = std::make_shared<AVCUnit>(device, unit, busOps_, busInfo_);

    // Store AVCUnit (before async Initialize — OnUnitResumed needs it)
    IOLockLock(lock_);
    units_[guid] = avcUnit;
    IOLockUnlock(lock_);

    // Rebuild node ID map (unit now has transport)
    RebuildNodeIDMap();

    // Check if this device needs a stabilization delay before AVC probing.
    // The Orpheus (and potentially other BeBoB devices) cannot handle AV/C
    // commands during its post-reset hardware initialization window.  Give it
    // time to finish booting before we send SUBUNIT_INFO, UNIT_INFO, etc.
    const uint32_t vid = device->GetVendorID();
    const uint32_t mid = device->GetModelID();
    const bool needsInitDelay =
        (vid == Audio::DeviceProtocolFactory::kPrismSoundVendorId &&
         mid == Audio::DeviceProtocolFactory::kOrpheusModelId);

    auto initWork = [this, avcUnit, guid, needsInitDelay]() {
        if (needsInitDelay) {
            ASFW_LOG(Audio,
                     "AVCDiscovery: Orpheus detected — waiting %ums for device to stabilize before AVC discovery GUID=%llx",
                     kOrpheusInitDelayMs, guid);
            IOSleep(kOrpheusInitDelayMs);

            // Enable bus-reset retry: the bus may still be settling after the
            // delay, so allow FCP to retry across generation changes.
            avcUnit->GetFCPTransport().SetAllowBusResetRetry(true);
        }

        // Use Apple's exact ~174-command discovery sequence (synchronous).
        // This replaces the old async probe chain and ensures the device sees
        // the same command sequence Apple's AppleFWAudio sends on cold attach.
        bool success = avcUnit->InitializeWithAppleDiscovery();

        if (needsInitDelay) {
            avcUnit->GetFCPTransport().SetAllowBusResetRetry(false);
        }

        if (!success) {
            os_log_error(log_,
                         "AVCDiscovery: AVCUnit Apple discovery failed: GUID=%llx — "
                         "falling back to async probe",
                         guid);

            // Fallback to old async path if Apple discovery fails
            avcUnit->Initialize([this, avcUnit, guid](bool asyncSuccess) {
                if (!asyncSuccess) {
                    os_log_error(log_,
                                 "AVCDiscovery: AVCUnit async init also failed: GUID=%llx",
                                 guid);
                    return;
                }
                HandleInitializedUnit(guid, avcUnit);
            });
            return;
        }

        HandleInitializedUnit(guid, avcUnit);
    };

    // Always run on rescan queue — AppleDiscoverySequence is synchronous and
    // blocks the calling thread for the full discovery duration (~5-30 s).
    if (rescanQueue_) {
        rescanQueue_->DispatchAsync(^{ initWork(); });
    } else {
        initWork();
    }
}

void AVCDiscovery::HandleInitializedUnit(uint64_t guid, const std::shared_ptr<AVCUnit>& avcUnit) {
    if (!avcUnit) {
        return;
    }

    auto device = avcUnit->GetDevice();
    if (!device) {
        os_log_error(log_, "AVCDiscovery: AVCUnit missing parent device: GUID=%llx", guid);
        return;
    }

    os_log_info(log_,
                "AVCDiscovery: AVCUnit initialized: GUID=%llx, "
                "%zu subunits, %d inputs, %d outputs",
                guid,
                avcUnit->GetSubunits().size(),
                avcUnit->IsInitialized() ? 2 : 0,  // Placeholder
                avcUnit->IsInitialized() ? 2 : 0); // Placeholder

    // Look for Music Subunit with audio capability
    Music::MusicSubunit* musicSubunit = nullptr;
    for (auto& subunit : avcUnit->GetSubunits()) {
        ASFW_LOG(Audio, "AVCDiscovery: Checking subunit type=0x%02x (kMusic=0x%02x)",
                 static_cast<uint8_t>(subunit->GetType()),
                 static_cast<uint8_t>(AVCSubunitType::kMusic));

        // Check if this is a Music subunit by type
        // Some devices report 0x0C, others 0x1C (both are valid Music subunit types)
        if (subunit->GetType() == AVCSubunitType::kMusic ||
            subunit->GetType() == AVCSubunitType::kMusic0C) {
            auto* music = static_cast<Music::MusicSubunit*>(subunit.get());
            const auto& caps = music->GetCapabilities();

            ASFW_LOG(Audio, "AVCDiscovery: Found Music subunit - hasAudioCapability=%d",
                     caps.HasAudioCapability());

            if (caps.HasAudioCapability()) {
                musicSubunit = music;
                break;
            }
        }
    }

    if (!musicSubunit) {
        os_log_debug(log_,
                     "AVCDiscovery: No audio-capable music subunit found (GUID=%llx)",
                     guid);
        // For known devices, fall back to the hardcoded audio nub rather than
        // silently dropping.  This covers the case where SUBUNIT_INFO timed out
        // during bus-reset recovery and the re-probe on resume also failed to
        // yield a Music Subunit (e.g. Prism Sound Orpheus).
        if (device) {
            const uint32_t vid = device->GetVendorID();
            const uint32_t mid = device->GetModelID();
            if (ASFW::Audio::DeviceProtocolFactory::IsKnownDevice(vid, mid)) {
                Discovery::DeviceRecord rec;
                rec.guid = guid;
                rec.vendorId = vid;
                rec.modelId = mid;
                ASFW_LOG(Audio,
                         "AVCDiscovery: Known device — using hardcoded audio nub fallback (GUID=%llx)",
                         guid);
                EnsureHardcodedAudioNubForDevice(rec);
                // Mark this device so OnUnitResumed won't re-probe endlessly.
                // AVC discovery always fails for this device (FCP transport errors
                // cause bus resets), so stop trying once the hardcoded nub is up.
                if (lock_) {
                    IOLockLock(lock_);
                    hardcodedNubGuids_.insert(guid);
                    IOLockUnlock(lock_);
                }
            }
        }
        return;
    }

    if (!musicSubunit->HasCompleteDescriptorParse()) {
        ASFW_LOG(Audio,
                 "AVCDiscovery: MusicSubunit descriptor incomplete - scheduling re-scan (GUID=%llx)",
                 guid);
        // TODO: Remove this duct-tape once AV/C discovery is reliable.
        ScheduleRescan(guid, avcUnit);
        return;
    }

    if (lock_) {
        IOLockLock(lock_);
        rescanAttempts_.erase(guid);
        IOLockUnlock(lock_);
    }

    //======================================================================
    // Populate MusicSubunitCapabilities with discovery data
    //======================================================================

    auto* devicePtr = device.get();
    auto& mutableCaps = const_cast<Music::MusicSubunitCapabilities&>(musicSubunit->GetCapabilities());

    // Populate device identity from FWDevice (Config ROM)
    mutableCaps.guid = guid;
    mutableCaps.vendorName = std::string(devicePtr->GetVendorName());
    mutableCaps.modelName = std::string(devicePtr->GetModelName());

    // Extract sample rates from supported formats (deduplicated, sorted)
    std::set<double> rateSet;
    for (const auto& plug : musicSubunit->GetPlugs()) {
        for (const auto& format : plug.supportedFormats) {
            uint32_t rateHz = format.GetSampleRateHz();
            if (rateHz > 0) {
                rateSet.insert(static_cast<double>(rateHz));
            }
        }
    }
    CollectUnitIsochSampleRates(*avcUnit, rateSet);
    mutableCaps.supportedSampleRates.assign(rateSet.begin(), rateSet.end());
    if (mutableCaps.supportedSampleRates.empty()) {
        mutableCaps.supportedSampleRates = {44100.0, 48000.0};  // Fallback
    }

    // Extract plug names (first input/output found, or keep defaults)
    // NOTE: Subunit perspective vs Host perspective are opposite!
    // - MusicSubunit "Input" = audio FROM host to device = HOST OUTPUT
    // - MusicSubunit "Output" = audio TO host from device = HOST INPUT
    for (const auto& plug : musicSubunit->GetPlugs()) {
        if (plug.IsInput() && !plug.name.empty() && mutableCaps.outputPlugName == "Output") {
            // Subunit Input (from host) = Host Output
            mutableCaps.outputPlugName = plug.name;
        }
        if (plug.IsOutput() && !plug.name.empty() && mutableCaps.inputPlugName == "Input") {
            // Subunit Output (to host) = Host Input
            mutableCaps.inputPlugName = plug.name;
        }
    }

    // Extract current sample rate from first plug's current format
    // The device reports its active sample rate in the currentFormat
    bool foundCurrentRate = false;
    for (const auto& plug : musicSubunit->GetPlugs()) {
        if (plug.currentFormat.has_value()) {
            uint32_t rateHz = plug.currentFormat->GetSampleRateHz();
            if (rateHz > 0) {
                mutableCaps.currentSampleRate = static_cast<double>(rateHz);
                ASFW_LOG(Audio, "AVCDiscovery: Current sample rate from plug %u: %u Hz",
                         plug.plugID, rateHz);
                foundCurrentRate = true;
                break;  // Use first found
            }
        }
    }

    // Fallback: Use first supported sample rate if no current rate found
    if (!foundCurrentRate) {
        if (auto unitRateHz = FindCurrentRateFromUnitIsochFormats(*avcUnit); unitRateHz.has_value()) {
            mutableCaps.currentSampleRate = static_cast<double>(*unitRateHz);
            ASFW_LOG(Audio, "AVCDiscovery: Current sample rate from unit isoch format: %u Hz",
                     *unitRateHz);
            foundCurrentRate = true;
        }
    }

    if (!foundCurrentRate && !mutableCaps.supportedSampleRates.empty()) {
        mutableCaps.currentSampleRate = mutableCaps.supportedSampleRates[0];
        ASFW_LOG(Audio, "AVCDiscovery: Using first supported rate as current: %.0f Hz",
                 mutableCaps.currentSampleRate);
    }

    // Use GetAudioDeviceConfiguration() for device creation.
    auto audioConfig = mutableCaps.GetAudioDeviceConfiguration();
    std::string deviceName = audioConfig.GetDeviceName();

    // Derive transport channel width from current plug formats.
    // For the host TX path, subunit input plugs are the authoritative view.
    const auto plugSummary = SummarizePlugChannels(musicSubunit->GetPlugs());
    const uint32_t plugsDerivedMax = std::max(plugSummary.inputAudioMaxChannels,
                                              plugSummary.outputAudioMaxChannels);
    uint32_t channelCount = plugsDerivedMax;
    const char* channelCountSource = "audio-plug-max-channels";

    if (channelCount == 0) {
        channelCount = audioConfig.GetMaxChannelCount();
        channelCountSource = "capability-fallback";
    }

    if (plugSummary.inputAudioMaxChannels > 0) {
        mutableCaps.maxAudioInputChannels = static_cast<uint16_t>(
            std::min<uint32_t>(plugSummary.inputAudioMaxChannels, 0xFFFFu));
    }
    if (plugSummary.outputAudioMaxChannels > 0) {
        mutableCaps.maxAudioOutputChannels = static_cast<uint16_t>(
            std::min<uint32_t>(plugSummary.outputAudioMaxChannels, 0xFFFFu));
    }

    ASFW_LOG(Audio,
             "AVCDiscovery: audio plug summary in=max%u/%u plugs out=max%u/%u plugs -> selected=%u (%{public}s)",
             plugSummary.inputAudioMaxChannels, plugSummary.inputAudioPlugs,
             plugSummary.outputAudioMaxChannels, plugSummary.outputAudioPlugs,
             channelCount, channelCountSource);

    const uint32_t vendorId = devicePtr->GetVendorID();
    const uint32_t modelId = devicePtr->GetModelID();

    //======================================================================
    // Phase 1.5: Choose the rate we publish into the audio nub
    //======================================================================
    // When a device protocol owns attach-time format programming, discovery must
    // not issue its own generic 0x19 sample-rate switch as well. That splits
    // ownership across two layers and can leave the single-pending FCP transport
    // busy right as attach-time bring-up starts.
    constexpr double kTargetSampleRate = 48000.0;
    bool protocolOwnsAttachTimeFormat = false;
    if (const auto* record = registry_.FindByGuid(guid); record && record->protocol) {
        protocolOwnsAttachTimeFormat = record->protocol->OwnsAttachTimeFormatProgramming();
    }

    // Check if device supports 48kHz
    bool supports48k = false;
    for (double rate : mutableCaps.supportedSampleRates) {
        if (rate == kTargetSampleRate) {
            supports48k = true;
            break;
        }
    }

    if (supports48k && mutableCaps.currentSampleRate != kTargetSampleRate) {
        if (protocolOwnsAttachTimeFormat) {
            ASFW_LOG(Audio,
                     "AVCDiscovery: Publishing preferred 48kHz "
                     "(current %.0f Hz); protocol-owned attach-time format programming "
                     "will program the device",
                     mutableCaps.currentSampleRate);
            mutableCaps.currentSampleRate = kTargetSampleRate;
        } else {
            ASFW_LOG(Audio, "AVCDiscovery: Switching sample rate from %.0f Hz to %.0f Hz (fire-and-forget)",
                     mutableCaps.currentSampleRate, kTargetSampleRate);

            // Use Unit Plug Signal Format (Oxford/Linux style) - opcode 0x19
            // This sets the format on Unit Plug 0 which controls both input and output
            using namespace ASFW::Protocols::AVC;

            // Build AV/C CDB for INPUT PLUG SIGNAL FORMAT (0x19) CONTROL command
            // Subunit 0xFF = Unit level (not Music Subunit)
            AVCCdb cdb;
            cdb.ctype = static_cast<uint8_t>(AVCCommandType::kControl);
            cdb.subunit = 0xFF;  // Unit level (not Music Subunit 0x60)
            cdb.opcode = 0x19;   // INPUT PLUG SIGNAL FORMAT (Oxford/Linux style)
            cdb.operands[0] = 0x00;  // Plug 0
            cdb.operands[1] = 0x90;  // AM824 format
            cdb.operands[2] = 0x02;  // 48kHz (SFC code per IEC 61883-6)
            cdb.operands[3] = 0xFF;  // Padding/Sync
            cdb.operands[4] = 0xFF;  // Padding/Sync
            cdb.operandLength = 5;

            // Create command with shared ownership (required by AVCCommand)
            auto setRateCmd = std::make_shared<AVCCommand>(
                avcUnit->GetFCPTransport(),
                cdb
            );

            // Fire-and-forget: Submit and assume success
            // Don't wait - the device will switch asynchronously
            // The callback just logs the result
            setRateCmd->Submit([setRateCmd](AVCResult result, const AVCCdb&) {
                // Capture setRateCmd to keep it alive until completion
                if (IsSuccess(result)) {
                    ASFW_LOG(Audio, "✅ AVCDiscovery: Sample rate change command accepted");
                } else {
                    ASFW_LOG_WARNING(Audio, "AVCDiscovery: Sample rate change command response: %d",
                                     static_cast<int>(result));
                }
            });

            // Assume success - set to 48kHz
            // The device typically switches within milliseconds
            mutableCaps.currentSampleRate = kTargetSampleRate;
            ASFW_LOG(Audio, "AVCDiscovery: Assuming 48kHz - nub will use this rate");
        }
    } else if (!supports48k) {
        ASFW_LOG(Audio, "AVCDiscovery: Device does not support 48kHz, using %.0f Hz",
                 mutableCaps.currentSampleRate);
    } else {
        ASFW_LOG(Audio, "AVCDiscovery: Device already at 48kHz");
    }

    // Build sample rates vector for OSArray (need uint32_t for OSNumber)
    // IMPORTANT: Put the current/target sample rate FIRST so CoreAudio HAL selects it
    std::vector<uint32_t> sampleRates;

    // First, add the current sample rate (48kHz if we switched)
    uint32_t currentRate = static_cast<uint32_t>(mutableCaps.currentSampleRate);
    sampleRates.push_back(currentRate);

    // Then add other supported rates (excluding the one already added)
    for (double rate : mutableCaps.supportedSampleRates) {
        uint32_t rateHz = static_cast<uint32_t>(rate);
        if (rateHz != currentRate) {
            sampleRates.push_back(rateHz);
        }
    }

    const char* streamModeReason = "default-nonblocking";
    const auto streamMode = ResolveStreamMode(mutableCaps, vendorId, modelId, streamModeReason);

    ASFW_LOG(Audio,
             "AVCDiscovery: stream mode selected vendor=0x%06x model=0x%06x mode=%{public}s reason=%{public}s",
             vendorId, modelId,
             ASFW::Audio::Quirks::StreamModeToString(streamMode),
             streamModeReason);
    uint32_t publishedAggregateChannels = channelCount;
    uint32_t publishedInputChannels =
        (plugSummary.outputAudioMaxChannels > 0) ? plugSummary.outputAudioMaxChannels : channelCount;
    uint32_t publishedOutputChannels =
        (plugSummary.inputAudioMaxChannels > 0) ? plugSummary.inputAudioMaxChannels : channelCount;

    if (IsPrismOrpheus(vendorId, modelId)) {
        publishedAggregateChannels = ASFW::Audio::BeBoB::kOrpheusInputAudioChannels;
        publishedInputChannels = ASFW::Audio::BeBoB::kOrpheusOutputAudioChannels;
        publishedOutputChannels = ASFW::Audio::BeBoB::kOrpheusInputAudioChannels;

        // Match Apple's naming: "Orpheus (0818)" where suffix is last 2 GUID bytes as decimal
        uint16_t deviceNum = static_cast<uint16_t>(guid & 0xFFFF);
        char nameBuf[64];
        snprintf(nameBuf, sizeof(nameBuf), "Orpheus (%04u)", deviceNum);
        deviceName = nameBuf;

        ASFW_LOG(Audio,
                 "AVCDiscovery: Applying Orpheus overrides "
                 "name=%{public}s rawIn=%u rawOut=%u publishedIn=%u publishedOut=%u aggregate=%u",
                 deviceName.c_str(),
                 plugSummary.outputAudioMaxChannels,
                 plugSummary.inputAudioMaxChannels,
                 publishedInputChannels,
                 publishedOutputChannels,
                 publishedAggregateChannels);
    }

    ASFW_LOG(Audio,
             "AVCDiscovery: Publishing audio configuration for GUID=%llx: %{public}s, %u channels, %zu sample rates",
             guid, deviceName.c_str(), publishedAggregateChannels, sampleRates.size());

    ASFW::Audio::Model::ASFWAudioDevice audioDeviceConfig;
    audioDeviceConfig.guid = guid;
    audioDeviceConfig.vendorId = vendorId;
    audioDeviceConfig.modelId = modelId;
    audioDeviceConfig.deviceName = deviceName;
    audioDeviceConfig.channelCount = publishedAggregateChannels;
    audioDeviceConfig.inputChannelCount = publishedInputChannels;
    audioDeviceConfig.outputChannelCount = publishedOutputChannels;
    audioDeviceConfig.sampleRates = sampleRates;
    audioDeviceConfig.currentSampleRate = currentRate;
    audioDeviceConfig.inputPlugName = mutableCaps.inputPlugName;
    audioDeviceConfig.outputPlugName = mutableCaps.outputPlugName;
    audioDeviceConfig.playback48kRawFormatBlock =
        FindRawFormatBlockForRate(musicSubunit->GetPlugs(), /*wantInput=*/true, 48000);
    audioDeviceConfig.capture48kRawFormatBlock =
        FindRawFormatBlockForRate(musicSubunit->GetPlugs(), /*wantInput=*/false, 48000);
    if (audioDeviceConfig.playback48kRawFormatBlock.empty()) {
        audioDeviceConfig.playback48kRawFormatBlock =
            FindRawUnitIsochFormatBlockForRate(*avcUnit, /*wantInput=*/true, 48000);
    }
    if (audioDeviceConfig.capture48kRawFormatBlock.empty()) {
        audioDeviceConfig.capture48kRawFormatBlock =
            FindRawUnitIsochFormatBlockForRate(*avcUnit, /*wantInput=*/false, 48000);
    }
    audioDeviceConfig.streamMode = streamMode;

    ASFW_LOG(Audio,
             "AVCDiscovery: Cached raw 48k format blocks playback=%zu capture=%zu "
             "GUID=%llx",
             audioDeviceConfig.playback48kRawFormatBlock.size(),
             audioDeviceConfig.capture48kRawFormatBlock.size(),
             guid);

    if (IsApogeeDuet(*devicePtr)) {
        ConfigureDuetPhantomOverrides(audioDeviceConfig, std::nullopt);
        ASFW_LOG(Audio,
                 "AVCDiscovery: Apogee Duet detected (GUID=%llx) - prefetching vendor config before publishing config",
                 guid);
        PrefetchDuetStateAndCreateNub(guid, avcUnit, audioDeviceConfig);
        return;
    }

    if (!audioConfigListener_) {
        ASFW_LOG_ERROR(Audio,
                       "AVCDiscovery: no audio config listener; dropping config for GUID=%llx",
                       guid);
        return;
    }

    audioConfigListener_->OnAVCAudioConfigurationReady(guid, audioDeviceConfig);
}

void AVCDiscovery::PrefetchDuetStateAndCreateNub(
    uint64_t guid,
    const std::shared_ptr<AVCUnit>& avcUnit,
    const Audio::Model::ASFWAudioDevice& config) {
    if (!avcUnit) {
        if (!audioConfigListener_) {
            ASFW_LOG_ERROR(Audio,
                           "AVCDiscovery: no audio config listener; dropping Duet fallback config for GUID=%llx",
                           guid);
            return;
        }
        audioConfigListener_->OnAVCAudioConfigurationReady(guid, config);
        return;
    }

    auto device = avcUnit->GetDevice();
    if (!device) {
        if (!audioConfigListener_) {
            ASFW_LOG_ERROR(Audio,
                           "AVCDiscovery: no audio config listener; dropping Duet fallback config for GUID=%llx",
                           guid);
            return;
        }
        audioConfigListener_->OnAVCAudioConfigurationReady(guid, config);
        return;
    }

    auto protocol = std::make_shared<Audio::Oxford::Apogee::ApogeeDuetProtocol>(
        busOps_,
        busInfo_,
        device->GetNodeID(),
        &avcUnit->GetFCPTransport());
    auto state = std::make_shared<DuetPrefetchState>();
    auto completed = std::make_shared<std::atomic<bool>>(false);
    auto finish = std::make_shared<std::function<void(const char*)>>();

    *finish = [this, guid, config, state, completed](const char* reason) {
        if (completed->exchange(true)) {
            return;
        }

        if (lock_) {
            IOLockLock(lock_);
            duetPrefetchByGuid_[guid] = *state;
            IOLockUnlock(lock_);
        }

        ASFW_LOG(Audio,
                 "AVCDiscovery: Duet prefetch complete GUID=%llx reason=%{public}s input=%d mixer=%d output=%d display=%d fw=%d hw=%d timedOut=%d",
                 guid,
                 reason ? reason : "unknown",
                 state->inputParams.has_value(),
                 state->mixerParams.has_value(),
                 state->outputParams.has_value(),
                 state->displayParams.has_value(),
                 state->firmwareId.has_value(),
                 state->hardwareId.has_value(),
                 state->timedOut);

        auto finalConfig = config;
        ConfigureDuetPhantomOverrides(finalConfig, state->inputParams);

        if (!audioConfigListener_) {
            ASFW_LOG_ERROR(Audio,
                           "AVCDiscovery: no audio config listener; dropping Duet config for GUID=%llx",
                           guid);
            return;
        }
        audioConfigListener_->OnAVCAudioConfigurationReady(guid, finalConfig);
    };

    if (rescanQueue_) {
        auto timeoutState = state;
        auto timeoutDone = completed;
        auto timeoutFinish = finish;
        rescanQueue_->DispatchAsync(^{
            IOSleep(kDuetPrefetchTimeoutMs);
            if (timeoutDone->load()) {
                return;
            }
            timeoutState->timedOut = true;
            ASFW_LOG_WARNING(Audio,
                             "AVCDiscovery: Duet prefetch timeout GUID=%llx after %u ms (continuing)",
                             guid, kDuetPrefetchTimeoutMs);
            (*timeoutFinish)("timeout");
        });
    } else {
        ASFW_LOG_WARNING(Audio,
                         "AVCDiscovery: no rescan queue for Duet timeout guard (GUID=%llx) - using fallback defaults",
                         guid);
        state->timedOut = true;
        (*finish)("missing-timeout-queue");
        return;
    }

    protocol->GetInputParams([guid, protocol, state, completed, finish](
                                 IOReturn status,
                                 Audio::Oxford::Apogee::InputParams params) {
        if (completed->load()) {
            return;
        }
        if (status == kIOReturnSuccess) {
            state->inputParams = params;
        } else {
            ASFW_LOG_WARNING(Audio,
                             "AVCDiscovery: Duet input prefetch failed GUID=%llx status=0x%x",
                             guid, status);
        }

        protocol->GetMixerParams([guid, protocol, state, completed, finish](
                                     IOReturn mixerStatus,
                                     Audio::Oxford::Apogee::MixerParams mixerParams) {
            if (completed->load()) {
                return;
            }
            if (mixerStatus == kIOReturnSuccess) {
                state->mixerParams = mixerParams;
            } else {
                ASFW_LOG_WARNING(Audio,
                                 "AVCDiscovery: Duet mixer prefetch failed GUID=%llx status=0x%x",
                                 guid, mixerStatus);
            }

            protocol->GetOutputParams([guid, protocol, state, completed, finish](
                                          IOReturn outputStatus,
                                          Audio::Oxford::Apogee::OutputParams outputParams) {
                if (completed->load()) {
                    return;
                }
                if (outputStatus == kIOReturnSuccess) {
                    state->outputParams = outputParams;
                } else {
                    ASFW_LOG_WARNING(Audio,
                                     "AVCDiscovery: Duet output prefetch failed GUID=%llx status=0x%x",
                                     guid, outputStatus);
                }

                protocol->GetDisplayParams(
                    [guid, protocol, state, completed, finish](
                        IOReturn displayStatus,
                        Audio::Oxford::Apogee::DisplayParams displayParams) {
                        if (completed->load()) {
                            return;
                        }
                        if (displayStatus == kIOReturnSuccess) {
                            state->displayParams = displayParams;
                        } else {
                            ASFW_LOG_WARNING(Audio,
                                             "AVCDiscovery: Duet display prefetch failed GUID=%llx status=0x%x",
                                             guid, displayStatus);
                        }

                        protocol->GetFirmwareId([guid, protocol, state, completed, finish](
                                                    IOReturn fwStatus,
                                                    uint32_t firmwareId) {
                            if (completed->load()) {
                                return;
                            }
                            if (fwStatus == kIOReturnSuccess) {
                                state->firmwareId = firmwareId;
                            } else {
                                ASFW_LOG_WARNING(Audio,
                                                 "AVCDiscovery: Duet firmware-id prefetch failed GUID=%llx status=0x%x",
                                                 guid, fwStatus);
                            }

                            protocol->GetHardwareId(
                                [guid, state, completed, finish](
                                    IOReturn hwStatus,
                                    uint32_t hardwareId) {
                                    if (completed->load()) {
                                        return;
                                    }
                                    if (hwStatus == kIOReturnSuccess) {
                                        state->hardwareId = hardwareId;
                                    } else {
                                        ASFW_LOG_WARNING(Audio,
                                                         "AVCDiscovery: Duet hardware-id prefetch failed GUID=%llx status=0x%x",
                                                         guid, hwStatus);
                                    }
                                    (*finish)("complete");
                                });
                        });
                    });
            });
        });
    });
}

void AVCDiscovery::ScheduleRescan(uint64_t guid, const std::shared_ptr<AVCUnit>& avcUnit) {
    if (!avcUnit) {
        return;
    }

    constexpr uint8_t kMaxAutoRescanAttempts = 1;
    constexpr uint32_t kRescanDelayMs = 250;

    uint8_t attempt = 0;
    IOLockLock(lock_);
    auto& count = rescanAttempts_[guid];
    if (count >= kMaxAutoRescanAttempts) {
        IOLockUnlock(lock_);
        ASFW_LOG(Audio,
                 "AVCDiscovery: Auto re-scan limit reached for GUID=%llx (attempts=%u)",
                 guid, count);
        return;
    }
    count++;
    attempt = count;
    IOLockUnlock(lock_);

    auto unit = avcUnit;
    auto rescanWork = [this, guid, attempt, unit]() {
        if (kRescanDelayMs > 0) {
            IOSleep(kRescanDelayMs);
        }

        ASFW_LOG(Audio, "AVCDiscovery: Auto re-scan attempt %u for GUID=%llx", attempt, guid);
        unit->ReScan([this, guid, unit](bool success) {
            if (!success) {
                os_log_error(log_,
                             "AVCDiscovery: AVCUnit re-scan failed: GUID=%llx",
                             guid);
                return;
            }

            HandleInitializedUnit(guid, unit);
        });
    };

    if (rescanQueue_) {
        rescanQueue_->DispatchAsync(^{ rescanWork(); });
    } else {
        rescanWork();
    }
}

void AVCDiscovery::OnUnitSuspended(std::shared_ptr<Discovery::FWUnit> unit) {
    uint64_t guid = GetUnitGUID(unit);

    IOLockLock(lock_);
    auto it = units_.find(guid);
    if (it != units_.end()) {
        os_log_info(log_,
                    "AVCDiscovery: AV/C unit suspended: GUID=%llx",
                    guid);
        // Unit remains in map but operations will fail until resumed
    }
    duetPrefetchByGuid_.erase(guid);
    IOLockUnlock(lock_);

    // Rebuild node ID map (suspended units removed from routing)
    RebuildNodeIDMap();
}

void AVCDiscovery::OnUnitResumed(std::shared_ptr<Discovery::FWUnit> unit) {
    uint64_t guid = GetUnitGUID(unit);

    std::shared_ptr<AVCUnit> avcUnit;
    bool needsReprobe = false;

    IOLockLock(lock_);
    auto it = units_.find(guid);
    if (it != units_.end()) {
        avcUnit = it->second;
        // If the initial probing failed (0 subunits — typically due to FCP
        // timeouts during bus-reset turmoil), re-initialize now that the
        // device has stabilized.  BUT if a hardcoded nub is already in place,
        // do NOT re-probe — FCP commands cause bus resets on this device,
        // creating an infinite reset loop (fix 38).
        if (avcUnit && avcUnit->GetSubunits().empty() &&
            hardcodedNubGuids_.find(guid) == hardcodedNubGuids_.end()) {
            needsReprobe = true;
        }
        os_log_info(log_,
                    "AVCDiscovery: AV/C unit resumed: GUID=%llx needsReprobe=%d",
                    guid, needsReprobe);
    }
    IOLockUnlock(lock_);

    // Rebuild node ID map (resumed units back in routing)
    RebuildNodeIDMap();

    if (needsReprobe && avcUnit) {
        ASFW_LOG(Audio,
                 "AVCDiscovery: Re-probing subunits on resume (initial probe had 0 subunits) GUID=%llx",
                 guid);
        // Clear previous rescan attempt counter so the post-init rescan path
        // is available if the descriptor parse is still incomplete.
        if (lock_) {
            IOLockLock(lock_);
            rescanAttempts_.erase(guid);
            IOLockUnlock(lock_);
        }

        // Determine stabilization delay: Orpheus needs longer (5s) due to
        // hardware initialization window; other devices use 500ms.
        auto device = avcUnit->GetDevice();
        uint32_t delayMs = 500;
        if (device) {
            const uint32_t vid = device->GetVendorID();
            const uint32_t mid = device->GetModelID();
            if (vid == Audio::DeviceProtocolFactory::kPrismSoundVendorId &&
                mid == Audio::DeviceProtocolFactory::kOrpheusModelId) {
                delayMs = kOrpheusInitDelayMs;
            }
        }

        auto rescanWork = [this, avcUnit, guid, delayMs]() {
            ASFW_LOG(Audio,
                     "AVCDiscovery: Waiting %ums for device to stabilize before re-probe GUID=%llx",
                     delayMs, guid);
            IOSleep(delayMs);

            // Enable bus-reset retry for the re-probe.  The bus may still be
            // resetting, so allow the FCP transport to retry across generation
            // changes.
            avcUnit->GetFCPTransport().SetAllowBusResetRetry(true);

            ASFW_LOG(Audio,
                     "AVCDiscovery: Starting delayed ReScan for GUID=%llx",
                     guid);

            // Must use ReScan() (not Initialize()) — Initialize() is a no-op
            // when initialized_==true.
            avcUnit->ReScan([this, avcUnit, guid](bool success) {
                // Restore generation-locked FCP for normal operation.
                avcUnit->GetFCPTransport().SetAllowBusResetRetry(false);

                if (!success) {
                    ASFW_LOG_ERROR(Audio,
                                   "AVCDiscovery: Re-probe on resume failed GUID=%llx",
                                   guid);
                    return;
                }
                HandleInitializedUnit(guid, avcUnit);
            });
        };

        if (rescanQueue_) {
            rescanQueue_->DispatchAsync(^{ rescanWork(); });
        } else {
            rescanWork();
        }
    }
}

void AVCDiscovery::OnUnitTerminated(std::shared_ptr<Discovery::FWUnit> unit) {
    uint64_t guid = GetUnitGUID(unit);

    IOLockLock(lock_);

    auto it = units_.find(guid);
    if (it != units_.end()) {
        os_log_info(log_,
                    "AVCDiscovery: AV/C unit terminated: GUID=%llx",
                    guid);
        units_.erase(it);
    }
    rescanAttempts_.erase(guid);
    duetPrefetchByGuid_.erase(guid);
    IOLockUnlock(lock_);

    // Rebuild node ID map (terminated unit removed)
    RebuildNodeIDMap();
}

void AVCDiscovery::OnDeviceAdded(std::shared_ptr<Discovery::FWDevice> device) {
    (void)device;
}

void AVCDiscovery::OnDeviceResumed(std::shared_ptr<Discovery::FWDevice> device) {
    // Called (under DeviceManager mutex) when a suspended device reappears —
    // e.g. after the mandatory BeBoB bus reset that follows SetFormat.
    if (deviceResumedCallback_) {
        deviceResumedCallback_(device);
    }
}

void AVCDiscovery::SetDeviceResumedCallback(
    std::function<void(std::shared_ptr<Discovery::FWDevice>)> cb)
{
    deviceResumedCallback_ = std::move(cb);
}

void AVCDiscovery::OnDeviceSuspended(std::shared_ptr<Discovery::FWDevice> device) {
    (void)device;
}

void AVCDiscovery::OnDeviceRemoved(Discovery::Guid64 guid) {
    IOLockLock(lock_);

    units_.erase(guid);
    rescanAttempts_.erase(guid);
    duetPrefetchByGuid_.erase(guid);
    IOLockUnlock(lock_);

    RebuildNodeIDMap();
}

//==============================================================================
// Public API
//==============================================================================

AVCUnit* AVCDiscovery::GetAVCUnit(uint64_t guid) {
    IOLockLock(lock_);

    auto it = units_.find(guid);
    AVCUnit* result = (it != units_.end()) ? it->second.get() : nullptr;

    IOLockUnlock(lock_);

    return result;
}

AVCUnit* AVCDiscovery::GetAVCUnit(std::shared_ptr<Discovery::FWUnit> unit) {
    if (!unit) {
        return nullptr;
    }

    uint64_t guid = GetUnitGUID(unit);
    return GetAVCUnit(guid);
}

std::vector<AVCUnit*> AVCDiscovery::GetAllAVCUnits() {
    IOLockLock(lock_);

    std::vector<AVCUnit*> result;
    result.reserve(units_.size());

    for (auto& [guid, avcUnit] : units_) {
        result.push_back(avcUnit.get());
    }

    IOLockUnlock(lock_);

    return result;
}

void AVCDiscovery::EnsureHardcodedAudioNubForDevice(const Discovery::DeviceRecord& deviceRecord) {
    if (!driver_ || deviceRecord.guid == 0) {
        return;
    }

    if (!audioConfigListener_) {
        ASFW_LOG_ERROR(Audio,
                       "AVCDiscovery: no audio config listener; dropping hardcoded config for GUID=%llx",
                       deviceRecord.guid);
        return;
    }

    ASFW::Audio::Model::ASFWAudioDevice hardcoded;
    hardcoded.guid = deviceRecord.guid;
    hardcoded.vendorId = deviceRecord.vendorId;
    hardcoded.modelId = deviceRecord.modelId;
    {
        // Match Apple's naming: "Orpheus (0818)" where suffix is last 2 GUID bytes as decimal
        uint16_t deviceNum = static_cast<uint16_t>(deviceRecord.guid & 0xFFFF);
        char nameBuf[64];
        snprintf(nameBuf, sizeof(nameBuf), "Orpheus (%04u)", deviceNum);
        hardcoded.deviceName = nameBuf;
    }

    // Hardcoded bring-up profile (v1):
    // - advertise single 48kHz / 24-bit stream format
    // Orpheus channel layout (asymmetric, from AVC discovery + macOS 11 ioreg):
    //   10 ins  = 8 analog + 2 S/PDIF  (device oPCR sends DBS=11: 10 audio + 1 MIDI)
    //   12 outs = 8 analog + 2 S/PDIF + 2 headphone  (device iPCR expects DBS=13: 12 audio + 1 MIDI)
    // fw_diag confirmed: Orpheus returns NOT_IMPLEMENTED for SetFormat CONTROL.
    hardcoded.channelCount = 12;
    hardcoded.inputChannelCount = 10;   // 10 audio channels (device oPCR DBS=11 = 10 audio + 1 MIDI)
    hardcoded.outputChannelCount = 12;  // 12 audio channels (device iPCR DBS=13 = 12 audio + 1 MIDI)
    hardcoded.sampleRates = {48000};
    hardcoded.currentSampleRate = 48000;
    hardcoded.inputPlugName = "Orpheus Input";
    hardcoded.outputPlugName = "Orpheus Output";
    hardcoded.streamMode = Audio::Model::StreamMode::kBlocking;

    ASFW_LOG(Audio,
             "AVCDiscovery[Hardcoded]: ensuring audio nub for GUID=%llx (%{public}s)",
             deviceRecord.guid,
             hardcoded.deviceName.c_str());

    audioConfigListener_->OnAVCAudioConfigurationReady(deviceRecord.guid, hardcoded);
}


void AVCDiscovery::ReScanAllUnits() {
    IOLockLock(lock_);
    
    os_log_info(log_, "AVCDiscovery: Re-scanning all %zu units", units_.size());
    rescanAttempts_.clear();

    for (auto& [guid, avcUnit] : units_) {
        if (avcUnit) {
            // Trigger re-scan (async)
            avcUnit->ReScan([guid](bool success) {
                // Logging handled inside AVCUnit
            });
        }
    }

    IOLockUnlock(lock_);
}

FCPTransport* AVCDiscovery::GetFCPTransportForNodeID(uint16_t nodeID) {
    IOLockLock(lock_);

    // Normalize to node number (low 6 bits) to match map keys
    const uint16_t nodeNumber = static_cast<uint16_t>(nodeID & 0x3Fu);

    auto it = fcpTransportsByNodeID_.find(nodeNumber);
    FCPTransport* result = (it != fcpTransportsByNodeID_.end())
                               ? it->second
                               : nullptr;

    IOLockUnlock(lock_);

    return result;
}

//==============================================================================
// Bus Reset Handling
//==============================================================================

void AVCDiscovery::OnBusReset(uint32_t newGeneration) {
    os_log_info(log_,
                "AVCDiscovery: Bus reset (generation %u)",
                newGeneration);

    // Notify all AVCUnits of bus reset
    IOLockLock(lock_);

    for (auto& [guid, avcUnit] : units_) {
        avcUnit->OnBusReset(newGeneration);
    }

    IOLockUnlock(lock_);

    // Rebuild node ID map (node IDs changed)
    RebuildNodeIDMap();
}

//==============================================================================
// Private Helpers
//==============================================================================

bool AVCDiscovery::IsAVCUnit(std::shared_ptr<Discovery::FWUnit> unit) const {
    if (!unit) {
        return false;
    }

    // Check unit spec ID (24-bit, should be 0x00A02D for AV/C)
    uint32_t specID = unit->GetUnitSpecID() & 0xFFFFFF;

    return specID == kAVCSpecID;
}

bool AVCDiscovery::IsApogeeDuet(const Discovery::FWDevice& device) const noexcept {
    return device.GetVendorID() == Audio::DeviceProtocolFactory::kApogeeVendorId &&
           device.GetModelID() == Audio::DeviceProtocolFactory::kApogeeDuetModelId;
}

uint64_t AVCDiscovery::GetUnitGUID(std::shared_ptr<Discovery::FWUnit> unit) const {
    if (!unit) {
        return 0;
    }

    auto device = unit->GetDevice();
    if (!device) {
        return 0;
    }

    return device->GetGUID();
}

void AVCDiscovery::RebuildNodeIDMap() {
    IOLockLock(lock_);

    // Clear old mappings
    fcpTransportsByNodeID_.clear();

    // Rebuild from current units
    for (auto& [guid, avcUnit] : units_) {
        auto device = avcUnit->GetDevice();
        if (!device) {
            continue;  // Device destroyed
        }

        auto unit = avcUnit->GetFWUnit();
        if (!unit || !unit->IsReady()) {
            continue;  // Unit suspended or terminated
        }

        // Normalize to node number (low 6 bits) to tolerate full vs short IDs
        const uint16_t fullNodeID = device->GetNodeID();
        const uint16_t nodeNumber = static_cast<uint16_t>(fullNodeID & 0x3Fu);
        
        fcpTransportsByNodeID_[nodeNumber] = &avcUnit->GetFCPTransport();

        os_log_debug(log_,
                     "AVCDiscovery: Mapped fullNodeID=0x%04x (node=%u) → FCPTransport (GUID=%llx)",
                     fullNodeID, nodeNumber, guid);
    }

    IOLockUnlock(lock_);
}

void AVCDiscovery::SetTransmitRingBufferOnNubs(void* ringBuffer) {
    // This method is deprecated - shared TX queue is now in ASFWAudioNub
    // Kept for backwards compatibility logging
    IOLockLock(lock_);

    os_log_info(log_,
                "AVCDiscovery: SetTransmitRingBufferOnNubs called (deprecated - using shared queue now)");

    IOLockUnlock(lock_);
}
