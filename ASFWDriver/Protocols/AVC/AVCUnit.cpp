//
// AVCUnit.cpp
// ASFWDriver - AV/C Protocol Layer
//
// AV/C Unit implementation
//

#include "AVCUnit.hpp"
#include "AppleDiscoverySequence.hpp"
#include "Music/MusicSubunit.hpp"
#include "Audio/AudioSubunit.hpp"
#include "Camera/CameraSubunit.hpp"
#include "../../Common/CallbackUtils.hpp"
#include "../../Logging/Logging.hpp"
#include "Descriptors/DescriptorAccessor.hpp"
#include "AVCSignalFormatCommand.hpp"

using namespace ASFW::Protocols::AVC;

//==============================================================================
// Constructor / Destructor
//==============================================================================

AVCUnit::AVCUnit(std::shared_ptr<Discovery::FWDevice> device,
                 std::shared_ptr<Discovery::FWUnit> unit,
                 Protocols::Ports::FireWireBusOps& busOps,
                 Protocols::Ports::FireWireBusInfo& busInfo)
    : device_(device),
      unit_(unit),
      busOps_(busOps),
      busInfo_(busInfo) {

    // Check for custom FCP addresses in Config ROM (optional)
    // For now, use standard addresses
    FCPTransportConfig config;
    config.commandAddress = kFCPCommandAddress;
    config.responseAddress = kFCPResponseAddress;
    config.timeoutMs = kFCPTimeoutInitial;
    config.interimTimeoutMs = kFCPTimeoutAfterInterim;
    config.maxRetries = kFCPMaxRetries;
    config.allowBusResetRetry = false;  // Default: generation-locked

    // Create FCP transport
    fcpTransport_ = OSSharedPtr(new FCPTransport, OSNoRetain);
    if (fcpTransport_) {
        fcpTransport_->init(&busOps_, &busInfo_, device.get(), config);

        // Create DescriptorAccessor for unit-level descriptors (Phase 5)
        descriptorAccessor_ = std::make_shared<DescriptorAccessor>(*fcpTransport_, kAVCSubunitUnit);

        if (!descriptorAccessor_) {
            ASFW_LOG_ERROR(Discovery, "AVCUnit: Failed to allocate DescriptorAccessor");
        }
    } else {
        ASFW_LOG_V1(AVC, "AVCUnit: Failed to allocate FCPTransport");
    }

    ASFW_LOG_V1(AVC,
                "AVCUnit: Created for device GUID=%llx, specID=0x%06x",
                GetGUID(), GetSpecID());
}

void AVCUnit::ProbeUnitInfo(std::function<void(bool)> completion) {
    auto completionState = Common::ShareCallback(std::move(completion));
    // UNIT_INFO: [STATUS, unit, 0x30, FF FF FF FF FF]
    // AV/C General Spec (TA 1999008) §9.2.1: 5 operand bytes, all 0xFF
    AVCCdb cdb{};
    cdb.ctype = static_cast<uint8_t>(AVCCommandType::kStatus);
    cdb.subunit = kAVCSubunitUnit;
    cdb.opcode = static_cast<uint8_t>(AVCOpcode::kUnitInfo);
    cdb.operands[0] = 0xFF;
    cdb.operands[1] = 0xFF;
    cdb.operands[2] = 0xFF;
    cdb.operands[3] = 0xFF;
    cdb.operands[4] = 0xFF;
    cdb.operandLength = 5;

    SubmitCommand(cdb, [this, completionState](AVCResult result, const AVCCdb&) {
        if (!IsSuccess(result)) {
            ASFW_LOG_V1(AVC, "AVCUnit: UNIT_INFO failed (result=%d) - continuing anyway",
                         static_cast<int>(result));
            // Continue anyway for devices that don't fully support UNIT_INFO
        }
        ASFW_LOG_V2(AVC, "AVCUnit: UNIT_INFO succeeded");
        Common::InvokeSharedCallback(completionState, true);
    });
}

AVCUnit::~AVCUnit() {
    ASFW_LOG_V1(AVC, "AVCUnit: Destroyed (GUID=%llx)", GetGUID());
}

//==============================================================================
// Initialization
//==============================================================================

void AVCUnit::Initialize(std::function<void(bool)> completion) {
    auto completionState = Common::ShareCallback(std::move(completion));
    if (initialized_) {
        ASFW_LOG_V2(AVC, "AVCUnit: Already initialized");
        Common::InvokeSharedCallback(completionState, true);
        return;
    }

    ASFW_LOG_V1(AVC, "AVCUnit: Initializing...");

    ProbeDescriptorMechanism([this, completionState](bool descriptorOk) {
        ProbeSignalFormat([this, completionState](bool signalFormatOk) {
            ProbeUnitInfo([this, completionState](bool unitOk) {
                if (!unitOk) {
                    ASFW_LOG_V1(AVC, "AVCUnit: UNIT_INFO probe failed");
                    Common::InvokeSharedCallback(completionState, false);
                    return;
                }

            ProbeSubunits([this, completionState](bool subunitOk) {
                if (!subunitOk) {
                    ASFW_LOG_V1(AVC, "AVCUnit: Subunit probe failed");
                    Common::InvokeSharedCallback(completionState, false);
                    return;
                }

                ProbePlugs([this, completionState](bool plugsOk) {
                    if (!plugsOk) {
                        ASFW_LOG_V1(AVC, "AVCUnit: Plug probe failed");
                        initialized_ = false;
                        Common::InvokeSharedCallback(completionState, false);
                        return;
                    }

                    // Fix 55b (Apr 12 2026): Apple phase 10 — QuerySyncPlugReconnect
                    // at UNIT for each external/iso plug. Apple runs this 11 times
                    // during cold attach and Orpheus may gate DAC arming on it.
                    // Non-fatal: replicate Apple exactly and continue regardless
                    // of per-plug result. See apple-discovery-sequence-full-174.md
                    // phase 10 and sequence-differences-vs-apple.md bug 2 sub-bug 2.
                    ProbeSyncPlugReconnect([this, completionState](bool /*qsprOk*/) {
                        initialized_ = true;

                        ASFW_LOG_V1(AVC,
                                   "AVCUnit: Initialized - "
                                   "%zu subunits, %u/%u ISO plugs, "
                                   "descriptor support: %s",
                                   subunits_.size(),
                                   plugCounts_.isoInputPlugs,
                                   plugCounts_.isoOutputPlugs,
                                   descriptorInfo_.descriptorMechanismSupported ?
                                       "YES" : "NO");

                        Common::InvokeSharedCallback(completionState, true);
                    });
                });
            });
        });
    });
    });
}

//==============================================================================
// Apple Discovery Initialization (synchronous)
//==============================================================================

bool AVCUnit::InitializeWithAppleDiscovery() {
    if (initialized_) {
        ASFW_LOG_V2(AVC, "AVCUnit: Already initialized (Apple path)");
        return true;
    }

    if (!fcpTransport_) {
        ASFW_LOG_V1(AVC, "AVCUnit: No FCP transport (Apple path)");
        return false;
    }

    ASFW_LOG_V1(AVC, "AVCUnit: InitializeWithAppleDiscovery starting...");

    appleDiscoveryUnitIsochFormats_.clear();

    AppleDiscoverySequence discovery(*fcpTransport_);
    auto result = discovery.RunSync();

    if (!result.success) {
        ASFW_LOG_V1(AVC, "AVCUnit: AppleDiscoverySequence failed");
        return false;
    }

    // ── Populate plug counts from Phase 2 ────────────────────────────────
    plugCounts_.isoInputPlugs  = result.isoInputPlugs;
    plugCounts_.isoOutputPlugs = result.isoOutputPlugs;
    plugCounts_.extInputPlugs  = result.extInputPlugs;
    plugCounts_.extOutputPlugs = result.extOutputPlugs;

    // ── Create subunits from Phase 2 ─────────────────────────────────────
    subunits_.clear();

    for (const auto& entry : result.subunits) {
        for (uint8_t id = 0; id <= entry.maxId; id++) {
            std::shared_ptr<Subunit> subunit;
            auto type = static_cast<AVCSubunitType>(entry.type);

            if (type == AVCSubunitType::kMusic || type == AVCSubunitType::kMusic0C) {
                subunit = std::make_shared<Music::MusicSubunit>(type, id);
            } else if (type == AVCSubunitType::kAudio) {
                subunit = std::make_shared<Audio::AudioSubunit>(type, id);
            } else {
                // Generic subunit for others
                class GenericSubunit : public Subunit {
                public:
                    GenericSubunit(AVCSubunitType t, uint8_t i) : Subunit(t, i) {}
                    std::string GetName() const override { return "Generic"; }
                };
                subunit = std::make_shared<GenericSubunit>(type, id);
            }

            if (subunit) {
                subunits_.push_back(subunit);
                ASFW_LOG_V2(AVC,
                            "AVCUnit: [Apple] Subunit: type=0x%02x id=%d (%{public}s)",
                            entry.type, id, subunit->GetName().c_str());
            }
        }
    }

    // ── Load discovery data into subunits ─────────────────────────────────
    for (auto& subunit : subunits_) {
        auto type = subunit->GetType();

        if (type == AVCSubunitType::kMusic || type == AVCSubunitType::kMusic0C) {
            auto* music = static_cast<Music::MusicSubunit*>(subunit.get());
            music->LoadFromDiscovery(result.musicDestPlugs,
                                     result.musicSrcPlugs,
                                     result.musicDescriptorData);
            for (const auto& formatResult : result.musicSubunitFormats) {
                if (!formatResult.valid) {
                    continue;
                }
                music->ApplyDiscoveryFormatResponse(formatResult.plugNum,
                                                    formatResult.direction == 0,
                                                    formatResult.rawResponse);
            }
        } else if (type == AVCSubunitType::kAudio) {
            auto* audio = static_cast<Audio::AudioSubunit*>(subunit.get());
            audio->LoadFromDiscovery(result.audioDestPlugs,
                                     result.audioSrcPlugs,
                                     result.audioDescriptorData);
        }
    }

    for (const auto& formatResult : result.unitIsochFormats) {
        appleDiscoveryUnitIsochFormats_.push_back({
            .valid = formatResult.valid,
            .direction = formatResult.direction,
            .plugNum = formatResult.plugNum,
            .rawResponse = formatResult.rawResponse,
        });
    }

    // ── Mark descriptor mechanism as supported ────────────────────────────
    descriptorInfo_.descriptorMechanismSupported =
        !result.musicDescriptorData.empty() || !result.audioDescriptorData.empty();

    initialized_ = true;

    ASFW_LOG_V1(AVC,
                "AVCUnit: InitializeWithAppleDiscovery complete — "
                "%zu subunits, %u/%u ISO plugs, QSPR %u/%u",
                subunits_.size(),
                plugCounts_.isoInputPlugs,
                plugCounts_.isoOutputPlugs,
                result.syncPlugsAccepted,
                result.syncPlugsTotal);

    return true;
}

void AVCUnit::ReScan(std::function<void(bool)> completion) {
    ASFW_LOG_V1(AVC, "AVCUnit: Re-scan requested (GUID=%llx)", GetGUID());
    
    // Reset state
    initialized_ = false;
    subunits_.clear();
    plugCounts_ = {};
    descriptorInfo_ = {};
    appleDiscoveryUnitIsochFormats_.clear();
    
    // Re-initialize
    Initialize(completion);
}

//==============================================================================
// Subunit Probing
//==============================================================================

void AVCUnit::ProbeSubunits(std::function<void(bool)> completion) {
    auto completionState = Common::ShareCallback(std::move(completion));
    // Create SUBUNIT_INFO command (page 0)
    auto cmd = std::make_shared<AVCSubunitInfoCommand>(*fcpTransport_, 0);

    // Submit command
    cmd->Submit([this, completionState, cmd](AVCResult result,
                                         const AVCSubunitInfoCommand::SubunitInfo& info) {
        if (!IsSuccess(result)) {
            ASFW_LOG_V1(AVC, "AVCUnit: SUBUNIT_INFO failed (result=%d) - continuing without subunit info",
                         static_cast<int>(result));
            // Non-fatal: device may still be booting after BeBoB format reset
            Common::InvokeSharedCallback(completionState, true);
            return;
        }

        // Store subunit info
        StoreSubunitInfo(info);

        ASFW_LOG_V1(AVC, "AVCUnit: Found %zu subunits", subunits_.size());

        // Now parse capabilities for each subunit
        ParseSubunitCapabilities(0, *completionState);
    });
}

void AVCUnit::StoreSubunitInfo(const AVCSubunitInfoCommand::SubunitInfo& info) {
    subunits_.clear();

    for (const auto& entry : info.subunits) {
        // For each subunit type reported, enumerate instances
        for (uint8_t id = 0; id <= entry.maxID; id++) {
            std::shared_ptr<Subunit> subunit;
            AVCSubunitType type = static_cast<AVCSubunitType>(entry.type);

            // Factory logic
            if (type == AVCSubunitType::kMusic || type == AVCSubunitType::kMusic0C) {
                subunit = std::make_shared<Music::MusicSubunit>(type, id);
            } else if (type == AVCSubunitType::kCamera) {
                subunit = std::make_shared<Camera::CameraSubunit>(type, id);
            } else if (type == AVCSubunitType::kAudio) {
                // Fix 55a (Apr 12 2026): do NOT skip the audio subunit when a music
                // subunit is present. Orpheus advertises BOTH (GetSubUnitInfo returns
                // 08 60 ff ff) and Apple's AppleFWAudio probes the audio subunit
                // heavily during cold attach (GetPlugInfo, Open/ReadDescriptor,
                // per-plug GetExtendedStreamFormat, GetChannelVolumeInfo, GetChannelMute).
                // Skipping it meant we never ran Apple's phase-3/phase-6 discovery,
                // and the DAC never armed. See apple-discovery-sequence-full-174.md.
                subunit = std::make_shared<Audio::AudioSubunit>(type, id);
            } else {
                // Generic subunit for others
                class GenericSubunit : public Subunit {
                public:
                    GenericSubunit(AVCSubunitType type, uint8_t id) : Subunit(type, id) {}
                    std::string GetName() const override { return "Generic"; }
                };
                subunit = std::make_shared<GenericSubunit>(type, id);
            }

            if (subunit) {
                subunits_.push_back(subunit);
                ASFW_LOG_V2(AVC, "AVCUnit: Subunit %zu: type=0x%02x, id=%d (%{public}s)",
                            subunits_.size() - 1, entry.type, id, subunit->GetName().c_str());
            }
        }
    }
}


void AVCUnit::ParseSubunitCapabilities(size_t index, std::function<void(bool)> completion) {
    auto completionState = Common::ShareCallback(std::move(completion));
    if (index >= subunits_.size()) {
        // All done
        Common::InvokeSharedCallback(completionState, true);
        return;
    }

    auto subunit = subunits_[index];
    subunit->ParseCapabilities(*this, [this, index, completionState](bool success) {
        if (!success) {
            ASFW_LOG_V2(AVC, "AVCUnit: Failed to parse capabilities for subunit %zu", index);
            // Continue anyway? Yes, partial success is better than failure.
        }
        // Next
        ParseSubunitCapabilities(index + 1, *completionState);
    });
}


//==============================================================================
// Plug Probing
//==============================================================================

void AVCUnit::ProbePlugs(std::function<void(bool)> completion) {
    auto completionState = Common::ShareCallback(std::move(completion));
    // Query unit-level plugs (subunit = 0xFF) using new command (Opcode 0x02 Subfunction 0x00)
    auto cmd = std::make_shared<AVCUnitPlugInfoCommand>(*this);

    cmd->Submit([this, completionState](AVCResult result,
                                         const UnitPlugCounts& info) {
        if (!IsSuccess(result)) {
            ASFW_LOG_V1(AVC,
                         "AVCUnit: PLUG_INFO failed (result=%d) - using defaults (1 iso in/out)",
                         static_cast<int>(result));
            // Non-fatal: assume 1 iso in + 1 iso out; the BeBoB hardcoded path
            // doesn't need accurate plug counts to set up streaming.
            plugCounts_.isoInputPlugs = 1;
            plugCounts_.isoOutputPlugs = 1;
            Common::InvokeSharedCallback(completionState, true);
            return;
        }

        // Store plug info
        plugCounts_ = info;

        ASFW_LOG_V2(AVC,
                    "AVCUnit: Unit plugs: %u iso in, %u iso out, %u ext in, %u ext out",
                    info.isoInputPlugs, info.isoOutputPlugs,
                    info.extInputPlugs, info.extOutputPlugs);

        Common::InvokeSharedCallback(completionState, true);
    });
}

//==============================================================================
// Sync Plug Reconnect (Apple phase 10)
//==============================================================================

// Fix 55b (Apr 12 2026): QuerySyncPlugReconnect × 11 at UNIT.
//
// Apple's cold-attach dtrace shows AppleFWAudio issues 11 QuerySyncPlugReconnect
// commands during discovery (opcode 0x1A subfunction 0x0F) BEFORE any CONTROL
// format sets. The bytes are verified in apple-discovery-sequence-full-174.md
// phase 10 — 10 for unit plugs 0x00, 0x01, 0x80..0x87 and 1 for music subunit
// plug 0x08. All commands are addressed to UNIT (0xFF) with CTYPE=0x02
// (SPECIFIC INQUIRY); the target subunit + plug are encoded in the operands.
//
// Operand layout per command: [0x0F, <target_subunit>, <target_plug>, 0x60, 0x07]
//   - 0x0F    = SIGNAL_SOURCE subfunction "sync plug reconnect"
//   - target_subunit = 0xFF for unit plugs, 0x60 for music subunit plug
//   - target_plug    = plug number (0x00, 0x01, 0x80..0x87, or 0x08)
//   - 0x60 0x07 = Apple's fixed trailing bytes (meaning unclear — may be the
//                 "source" plug specifier the device is expected to validate
//                 against; we replicate byte-for-byte).
//
// Non-fatal: device may NotImplement individual plugs. Continue regardless.
void AVCUnit::ProbeSyncPlugReconnect(std::function<void(bool)> completion) {
    struct PlugAddr {
        uint8_t targetSubunit;
        uint8_t targetPlug;
    };

    // Apple's exact order from phase 10 byte log.
    static constexpr std::array<PlugAddr, 11> kPlugs = {{
        {0xFF, 0x00},  // unit plug 0 (iso input 0)
        {0xFF, 0x01},  // unit plug 1 (iso input 1)
        {0xFF, 0x80},  // ext plug 0
        {0xFF, 0x81},  // ext plug 1
        {0xFF, 0x82},
        {0xFF, 0x83},
        {0xFF, 0x84},
        {0xFF, 0x85},
        {0xFF, 0x86},
        {0xFF, 0x87},
        {0x60, 0x08},  // music subunit plug 8
    }};

    struct QsprState {
        size_t index{0};
        size_t acceptedCount{0};
        std::function<void(bool)> completion;
        std::function<void()> queryNext;
    };

    auto state = std::make_shared<QsprState>();
    state->completion = std::move(completion);

    state->queryNext = [this, state]() {
        if (state->index >= kPlugs.size()) {
            ASFW_LOG_V1(Discovery,
                        "AVCUnit: ProbeSyncPlugReconnect complete — %zu/%zu accepted",
                        state->acceptedCount, kPlugs.size());
            auto completion = state->completion;
            state->queryNext = nullptr; // break reference cycle
            if (completion) {
                completion(true);
            }
            return;
        }

        const auto& plugAddr = kPlugs[state->index];

        AVCCdb cdb{};
        cdb.ctype = static_cast<uint8_t>(AVCCommandType::kInquiry); // 0x02 SPECIFIC INQUIRY
        cdb.subunit = kAVCSubunitUnit;                              // 0xFF
        cdb.opcode = 0x1A;                                          // SIGNAL_SOURCE
        cdb.operands[0] = 0x0F;                                     // sync plug reconnect subfunction
        cdb.operands[1] = plugAddr.targetSubunit;
        cdb.operands[2] = plugAddr.targetPlug;
        cdb.operands[3] = 0x60;                                     // Apple fixed trailer
        cdb.operands[4] = 0x07;
        cdb.operandLength = 5;

        const size_t currentIndex = state->index;

        SubmitCommand(cdb, [this, state, currentIndex](AVCResult result,
                                                       const AVCCdb& /*response*/) {
            if (IsSuccess(result)) {
                state->acceptedCount++;
                ASFW_LOG_V3(Discovery,
                            "AVCUnit: QSPR plug %zu accepted (subunit=0x%02x plug=0x%02x)",
                            currentIndex,
                            kPlugs[currentIndex].targetSubunit,
                            kPlugs[currentIndex].targetPlug);
            } else {
                ASFW_LOG_V3(Discovery,
                            "AVCUnit: QSPR plug %zu not accepted (subunit=0x%02x plug=0x%02x result=%d) — continuing",
                            currentIndex,
                            kPlugs[currentIndex].targetSubunit,
                            kPlugs[currentIndex].targetPlug,
                            static_cast<int>(result));
            }
            state->index++;
            state->queryNext();
        });
    };

    ASFW_LOG_V2(Discovery, "AVCUnit: ProbeSyncPlugReconnect starting (%zu plugs)", kPlugs.size());
    state->queryNext();
}

void AVCUnit::ProbeSignalFormat(std::function<void(bool)> completion) {
    auto completionState = Common::ShareCallback(std::move(completion));
    // Query Plug 0
    auto cmd = std::make_shared<AVCOutputPlugSignalFormatCommand>(*fcpTransport_, 0);

    cmd->Submit([this, completionState](AVCResult result,
                                   const AVCOutputPlugSignalFormatCommand::SignalFormat& fmt) {
        if (IsSuccess(result)) {
            ASFW_LOG_INFO(Discovery, "Received Signal Format: Format=0x%02x, RateCode=0x%02x", fmt.formatHierarchy, fmt.formatSync);

            if (fmt.formatHierarchy == 0x90) {
                ASFW_LOG_INFO(Discovery, "Detected Apogee AM824 Format (0x90).");
                
                // Use Music Subunit helper to interpret rate code (0x01 = 44.1kHz, etc.)
                auto rate = StreamFormats::MusicSubunitCodeToSampleRate(fmt.formatSync);
                uint32_t freqHz = StreamFormats::SampleRateToHz(rate);
                
                if (freqHz > 0) { // NOSONAR(cpp:S3923): branches log different diagnostic messages
                    ASFW_LOG_INFO(Discovery, "Device is locked to %u Hz (Code 0x%02x).", freqHz, fmt.formatSync);
                } else {
                    ASFW_LOG_INFO(Discovery, "Device is locked to Unknown Rate (Code 0x%02x).", fmt.formatSync);
                }
            }
        } else {
            ASFW_LOG_ERROR(Discovery, "Failed to send Signal Format Query: result=%d", static_cast<int>(result));
        }
        // Always continue
        Common::InvokeSharedCallback(completionState, true);
    });
}

bool AVCUnit::ParseUnitIdentifier(const std::vector<uint8_t>& data) {
    // Minimum size check: descriptor_length(2) + generation_ID(1) + 3 size fields = 6
    if (data.size() < 6) {
        ASFW_LOG_V1(AVC, "AVCUnit: Unit Identifier too short (need at least 6 bytes)");
        return false;
    }

    // Parse descriptor_length (bytes 0-1)
    // Note: DescriptorAccessor includes this in the returned data
    uint16_t descriptorLength = (data[0] << 8) | data[1];
    ASFW_LOG_V3(AVC, "AVCUnit: Unit Identifier length = %d bytes", descriptorLength);

    // Validate length matches actual data size
    if (descriptorLength + 2 != data.size()) {
        ASFW_LOG_V2(AVC,
                        "AVCUnit: Descriptor length mismatch (declared=%d, actual=%zu)",
                        descriptorLength, data.size() - 2);
        // Continue anyway - some devices may have padding
    }

    // Parse fields (Section 6.2.1 of TA 2002013)
    descriptorInfo_.generationID = data[2];
    descriptorInfo_.sizeOfListID = data[3];
    descriptorInfo_.sizeOfObjectID = data[4];
    descriptorInfo_.sizeOfEntryPosition = data[5];

    // Validate sizes are reasonable (spec says 0-8 bytes typical)
    if (descriptorInfo_.sizeOfListID > 8 ||
        descriptorInfo_.sizeOfObjectID > 8 ||
        descriptorInfo_.sizeOfEntryPosition > 8) {
        ASFW_LOG_V1(AVC, "AVCUnit: Suspicious descriptor sizes (one or more > 8 bytes)");
        return false;
    }

    // Parse number_of_root_object_lists (offset 6, 2 bytes)
    if (data.size() < 8) {
        // No root lists section present
        descriptorInfo_.numberOfRootObjectLists = 0;
        descriptorInfo_.rootListIDs.clear();
        return true;
    }

    descriptorInfo_.numberOfRootObjectLists = (data[6] << 8) | data[7];

    // Parse root_list_ID array
    size_t listIdSize = (descriptorInfo_.sizeOfListID > 0) ?
        descriptorInfo_.sizeOfListID : 2;  // Default to 2 bytes if size is 0

    size_t arraySize = descriptorInfo_.numberOfRootObjectLists * listIdSize;
    size_t arrayOffset = 8;

    if (data.size() < arrayOffset + arraySize) {
        ASFW_LOG_V1(AVC, "AVCUnit: Data too short for root_list_ID array");
        return false;
    }

    // Extract root list IDs (MSB first encoding)
    descriptorInfo_.rootListIDs.clear();
    descriptorInfo_.rootListIDs.reserve(descriptorInfo_.numberOfRootObjectLists);

    const uint8_t* arrayPtr = data.data() + arrayOffset;
    for (uint16_t i = 0; i < descriptorInfo_.numberOfRootObjectLists; ++i) {
        uint64_t listId = 0;
        // Read listIdSize bytes in MSB-first order
        for (size_t byteIdx = 0; byteIdx < listIdSize; ++byteIdx) {
            listId = (listId << 8) | arrayPtr[byteIdx];
        }
        descriptorInfo_.rootListIDs.push_back(listId);
        arrayPtr += listIdSize;

        ASFW_LOG_V3(AVC, "AVCUnit: Root list [%d] = 0x%llx", i, listId);
    }

    return true;
}

void AVCUnit::ProbeDescriptorMechanism(std::function<void(bool)> completion) {
    auto completionState = Common::ShareCallback(std::move(completion));
    ASFW_LOG_V2(AVC, "AVCUnit: Probing descriptor mechanism (Status Descriptor 0x80)...");

    if (!descriptorAccessor_) {
        ASFW_LOG_V2(AVC, "AVCUnit: No DescriptorAccessor, skipping descriptors");
        descriptorInfo_.descriptorMechanismSupported = false;
        Common::InvokeSharedCallback(completionState, true);
        return;
    }

    // Use 0x80 (Status Descriptor) as Apple does for Music Subunits
    auto specifier = DescriptorSpecifier();
    specifier.type = static_cast<DescriptorSpecifierType>(0x80);
    auto self = shared_from_this();

    descriptorAccessor_->readWithOpenCloseSequence(
        specifier,
        [this, self, completionState](const DescriptorAccessor::ReadDescriptorResult& result) {
            if (!result.success) {
                ASFW_LOG_V2(AVC, "AVCUnit: Status Descriptor read failed: %d",
                             static_cast<int>(result.avcResult));
                descriptorInfo_.descriptorMechanismSupported = false;
                Common::InvokeSharedCallback(completionState, true);  // Continue despite failure
                return;
            }

            // Note: The response is a Status Descriptor, not a Unit Identifier.
            // Standard ParseUnitIdentifier won't work here because the format is different.
            // We just mark support as true if we got data.
            // The specific parsing (Info Blocks) is handled by MusicSubunit.
            
            if (!result.data.empty()) {
                descriptorInfo_.descriptorMechanismSupported = true;
                ASFW_LOG_V1(AVC, "AVCUnit: Descriptor mechanism SUPPORTED (Status Descriptor 0x80 read success, %zu bytes)", result.data.size());
            } else {
                descriptorInfo_.descriptorMechanismSupported = false;
            }
            
            // Skip TraverseRootLists for Music Subunits using Status Descriptor model
            Common::InvokeSharedCallback(completionState, true);
        });
}

void AVCUnit::TraverseRootLists(size_t listIndex,
                                std::function<void(bool)> completion) {
    auto completionState = Common::ShareCallback(std::move(completion));
    if (listIndex >= descriptorInfo_.rootListIDs.size()) {
        // All lists traversed
        ASFW_LOG_V2(AVC,
                     "AVCUnit: Traversed all %zu root object lists",
                     descriptorInfo_.rootListContents.size());
        Common::InvokeSharedCallback(completionState, true);
        return;
    }

    uint64_t listID = descriptorInfo_.rootListIDs[listIndex];
    ASFW_LOG_V3(AVC,
                  "AVCUnit: Traversing root list [%zu]: ID=0x%llx",
                  listIndex, listID);

    auto self = shared_from_this();
    ReadRootObjectList(listID,
        [this, self, listIndex, listID, completionState]
        (bool success, std::vector<uint64_t> objectIDs) {

            if (success) {
                UnitDescriptorInfo::RootListContents contents;
                contents.listID = listID;
                contents.objectIDs = std::move(objectIDs);
                descriptorInfo_.rootListContents.push_back(std::move(contents));

                ASFW_LOG_V3(AVC,
                              "AVCUnit: Root list 0x%llx contains %zu objects",
                              listID,
                              descriptorInfo_.rootListContents.back().objectIDs.size());
            } else {
                ASFW_LOG_V2(AVC,
                                "AVCUnit: Failed to read root list 0x%llx (continuing)",
                                listID);
            }

            // Continue to next list (graceful degradation)
            TraverseRootLists(listIndex + 1, *completionState);
        });
}

void AVCUnit::ReadRootObjectList(
    uint64_t listID,
    std::function<void(bool success, std::vector<uint64_t> objectIDs)> completion) {
    auto completionState = Common::ShareCallback(std::move(completion));

    if (!descriptorAccessor_) {
        Common::InvokeSharedCallback(completionState, false, std::vector<uint64_t>{});
        return;
    }

    // Build descriptor specifier for list_ID (type 0x10)
    size_t listIdSize = descriptorInfo_.sizeOfListID > 0 ?
        descriptorInfo_.sizeOfListID : 2;

    std::vector<uint8_t> operands;
    operands.reserve(listIdSize);

    // Encode listID as MSB-first bytes
    for (size_t i = 0; i < listIdSize; ++i) {
        size_t shiftAmount = (listIdSize - 1 - i) * 8;
        operands.push_back(static_cast<uint8_t>((listID >> shiftAmount) & 0xFF));
    }

    auto specifier = DescriptorSpecifier::forListID(operands);
    auto self = shared_from_this();

    descriptorAccessor_->readWithOpenCloseSequence(
        specifier,
        [this, self, listID, completionState]
        (const DescriptorAccessor::ReadDescriptorResult& result) {

            if (!result.success) {
                ASFW_LOG_V2(AVC,
                                "AVCUnit: Failed to read list 0x%llx: result=%d",
                                listID, static_cast<int>(result.avcResult));
                Common::InvokeSharedCallback(completionState, false, std::vector<uint64_t>{});
                return;
            }

            // Parse object list descriptor
            const auto& data = result.data;
            if (data.size() < 4) {
                ASFW_LOG_V1(AVC, "AVCUnit: List descriptor too short");
                Common::InvokeSharedCallback(completionState, false, std::vector<uint64_t>{});
                return;
            }

            uint16_t descriptorLength = (data[0] << 8) | data[1];
            uint16_t numEntries = (data[2] << 8) | data[3];

            ASFW_LOG_V3(AVC,
                          "AVCUnit: List 0x%llx: length=%d, entries=%d",
                          listID, descriptorLength, numEntries);

            // Parse object IDs
            size_t objectIdSize = descriptorInfo_.sizeOfObjectID > 0 ?
                descriptorInfo_.sizeOfObjectID : 2;
            size_t arrayOffset = 4;
            size_t expectedSize = arrayOffset + (numEntries * objectIdSize);

            if (data.size() < expectedSize) {
                ASFW_LOG_V1(AVC, "AVCUnit: List data too short for entries");
                Common::InvokeSharedCallback(completionState, false, std::vector<uint64_t>{});
                return;
            }

            std::vector<uint64_t> objectIDs;
            objectIDs.reserve(numEntries);

            const uint8_t* ptr = data.data() + arrayOffset;
            for (uint16_t i = 0; i < numEntries; ++i) {
                uint64_t objectID = 0;
                for (size_t b = 0; b < objectIdSize; ++b) {
                    objectID = (objectID << 8) | ptr[b];
                }
                objectIDs.push_back(objectID);
                ptr += objectIdSize;
            }

            Common::InvokeSharedCallback(completionState, true, std::move(objectIDs));
        });
}

//==============================================================================
// Command Submission
//==============================================================================

// Implement IAVCCommandSubmitter
void AVCUnit::SubmitCommand(const AVCCdb& cdb, AVCCompletion completion) {
    if (!fcpTransport_) {
        completion(AVCResult::kTransportError, cdb);
        return;
    }

    // Create AVCCommand to handle the transaction
    // Note: AVCCommand manages its own lifetime via shared_from_this during the transaction
    auto cmd = std::make_shared<AVCCommand>(*fcpTransport_, cdb);
    cmd->Submit(completion);
}


void AVCUnit::GetPlugInfo(std::function<void(AVCResult, const UnitPlugCounts&)> completion) {
    if (initialized_) {
        // Return cached result
        completion(AVCResult::kImplementedStable, plugCounts_);
        return;
    }

    // Query device
    auto cmd = std::make_shared<AVCUnitPlugInfoCommand>(*this);

    cmd->Submit(completion);
}

//==============================================================================
// Bus Reset Handling
//==============================================================================

void AVCUnit::OnBusReset(uint32_t newGeneration) {
    ASFW_LOG_V2(AVC,
                "AVCUnit: Bus reset (generation %u)",
                newGeneration);

    // Forward to FCP transport (will handle pending commands)
    fcpTransport_->OnBusReset(newGeneration);

    // v1: Keep cached state (subunits, plugs rarely change)
    // Caller can re-Initialize() if topology changed

    // v2 improvement: Could invalidate cache on topology change
    // and re-probe automatically
}

//==============================================================================
// Accessors
//==============================================================================

uint64_t AVCUnit::GetGUID() const {
    auto device = device_.lock();
    if (!device) {
        return 0;
    }
    return device->GetGUID();
}

uint32_t AVCUnit::GetSpecID() const {
    auto unit = unit_.lock();
    if (!unit) {
        return 0;
    }
    return unit->GetUnitSpecID();
}
