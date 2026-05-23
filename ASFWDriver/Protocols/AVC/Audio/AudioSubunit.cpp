//
// AudioSubunit.cpp
// ASFWDriver - AV/C Protocol Layer
//
// Audio Subunit implementation
//

#include "AudioSubunit.hpp"
#include "../AVCUnit.hpp"
#include "../AVCCommands.hpp"
#include "../AudioFunctionBlockCommand.hpp"
#include "../Descriptors/AVCInfoBlock.hpp"
#include "../StreamFormats/AVCStreamFormatCommands.hpp"
#include "../StreamFormats/StreamFormatParser.hpp"
#include "../../../Common/CallbackUtils.hpp"
#include "../../../Logging/Logging.hpp"

#include <algorithm>
#include <cstring>
#include <map>
#include <unordered_map>

using namespace ASFW::Protocols::AVC::Audio;

namespace {

constexpr uint16_t kInfoName = 0x000B;
constexpr uint16_t kInfoRawText = 0x000A;
constexpr uint16_t kMusicGeneralStatus = 0x8100;
constexpr uint16_t kMusicRoutingStatus = 0x8108;
constexpr uint16_t kMusicSubunitPlugInfo = 0x8109;
constexpr uint16_t kClusterInfo = 0x810A;
constexpr uint16_t kMusicPlugInfo = 0x810B;

uint16_t ReadBE16Local(const uint8_t* p) {
    return (static_cast<uint16_t>(p[0]) << 8) | p[1];
}

std::optional<ASFW::Protocols::AVC::StreamFormats::AudioStreamFormat>
ParseDiscoveryFormatResponse(const std::vector<uint8_t>& rawResponse, uint8_t& outSubfunction) {
    using namespace ASFW::Protocols::AVC;
    if (rawResponse.size() < kAVCFrameMinSize || rawResponse.size() > kAVCFrameMaxSize) {
        return std::nullopt;
    }

    FCPFrame frame;
    frame.length = rawResponse.size();
    std::copy(rawResponse.begin(), rawResponse.end(), frame.data.begin());

    auto cdb = AVCCdb::Decode(frame);
    if (!cdb.has_value() || cdb->operandLength == 0) {
        return std::nullopt;
    }

    outSubfunction = cdb->operands[0];

    size_t formatOffset = 0;
    if (outSubfunction == ASFW::Protocols::AVC::StreamFormats::kStreamFormatSubfunc_Current) {
        formatOffset = 7;
    } else if (outSubfunction == ASFW::Protocols::AVC::StreamFormats::kStreamFormatSubfunc_Supported) {
        formatOffset = 8;
    } else {
        return std::nullopt;
    }

    if (cdb->operandLength <= formatOffset) {
        return std::nullopt;
    }

    return ASFW::Protocols::AVC::StreamFormats::StreamFormatParser::Parse(
        cdb->operands.data() + formatOffset,
        cdb->operandLength - formatOffset);
}

ASFW::Protocols::AVC::StreamFormat ToLegacyStreamFormat(
    const ASFW::Protocols::AVC::StreamFormats::AudioStreamFormat& parsed) {
    ASFW::Protocols::AVC::StreamFormat out{};
    out.formatType = static_cast<uint8_t>(parsed.formatHierarchy);
    out.formatSubtype = static_cast<uint8_t>(parsed.subtype);
    out.sampleRate = static_cast<uint8_t>(parsed.sampleRate);
    out.syncMode = parsed.syncMode == ASFW::Protocols::AVC::StreamFormats::SyncMode::kSynchronized;
    out.numChannels = parsed.totalChannels;
    out.rawData = parsed.rawFormatBlock;
    return out;
}

std::string TextFromNameBlock(const ASFW::Protocols::AVC::Descriptors::AVCInfoBlock& block) {
    using ASFW::Protocols::AVC::Descriptors::AVCInfoBlock;

    std::vector<uint8_t> textBytes;
    if (block.GetType() == kInfoRawText) {
        textBytes = block.GetPrimaryData();
    } else if (auto rawText = block.FindNestedRecursive(kInfoRawText)) {
        textBytes = rawText->GetPrimaryData();
    }

    const auto& bytes = textBytes;
    if (bytes.empty()) {
        return {};
    }

    size_t len = bytes.size();
    while (len > 0 && bytes[len - 1] == 0) {
        --len;
    }
    return std::string(reinterpret_cast<const char*>(bytes.data()), len);
}

std::string ExtractNameRecursive(const ASFW::Protocols::AVC::Descriptors::AVCInfoBlock& block) {
    if (block.GetType() == kInfoName || block.GetType() == kInfoRawText) {
        auto text = TextFromNameBlock(block);
        if (!text.empty()) {
            return text;
        }
    }

    for (const auto& child : block.GetNestedBlocks()) {
        auto text = ExtractNameRecursive(child);
        if (!text.empty()) {
            return text;
        }
    }

    return {};
}

std::unordered_map<uint16_t, std::string>
CollectMusicPlugNames(const ASFW::Protocols::AVC::Descriptors::AVCInfoBlock& root) {
    std::unordered_map<uint16_t, std::string> names;
    for (const auto& block : root.FindAllNestedRecursive(kMusicPlugInfo)) {
        const auto& primary = block.GetPrimaryData();
        if (primary.size() < 3) {
            continue;
        }

        // Apple's GetChannelNameFromPlugId compares the BE16 at primary[1..2].
        const uint16_t plugId = ReadBE16Local(primary.data() + 1);
        auto name = ExtractNameRecursive(block);
        if (!name.empty()) {
            names[plugId] = std::move(name);
        }
    }
    return names;
}

std::vector<const ASFW::Protocols::AVC::Descriptors::AVCInfoBlock*>
ImmediateChildrenOfType(const ASFW::Protocols::AVC::Descriptors::AVCInfoBlock& block, uint16_t type) {
    std::vector<const ASFW::Protocols::AVC::Descriptors::AVCInfoBlock*> out;
    for (const auto& child : block.GetNestedBlocks()) {
        if (child.GetType() == type) {
            out.push_back(&child);
        }
    }
    return out;
}

const ASFW::Protocols::AVC::Descriptors::AVCInfoBlock*
FindFirstRecursivePtr(const ASFW::Protocols::AVC::Descriptors::AVCInfoBlock& block, uint16_t type) {
    if (block.GetType() == type) {
        return &block;
    }

    for (const auto& child : block.GetNestedBlocks()) {
        if (const auto* found = FindFirstRecursivePtr(child, type)) {
            return found;
        }
    }

    return nullptr;
}

const ASFW::Protocols::AVC::Descriptors::AVCInfoBlock*
FindRoutingBlockLikeApple(const ASFW::Protocols::AVC::Descriptors::AVCInfoBlock& root) {
    if (root.GetType() == kMusicRoutingStatus) {
        return &root;
    }

    if (root.GetType() == kMusicGeneralStatus) {
        for (const auto& child : root.GetNestedBlocks()) {
            if (child.GetType() == kMusicRoutingStatus) {
                return &child;
            }
        }
    }

    return FindFirstRecursivePtr(root, kMusicRoutingStatus);
}

void CountInfoBlockTypes(const ASFW::Protocols::AVC::Descriptors::AVCInfoBlock& block,
                         std::map<uint16_t, size_t>& counts) {
    counts[block.GetType()]++;
    for (const auto& child : block.GetNestedBlocks()) {
        CountInfoBlockTypes(child, counts);
    }
}

size_t CountInfoBlocks(const ASFW::Protocols::AVC::Descriptors::AVCInfoBlock& block) {
    size_t count = 1;
    for (const auto& child : block.GetNestedBlocks()) {
        count += CountInfoBlocks(child);
    }
    return count;
}

void LogAppleDescriptorShape(const std::vector<ASFW::Protocols::AVC::Descriptors::AVCInfoBlock>& roots,
                             uint16_t declaredLength,
                             size_t actualLength) {
    std::map<uint16_t, size_t> counts;
    size_t totalBlocks = 0;
    for (const auto& root : roots) {
        CountInfoBlockTypes(root, counts);
        totalBlocks += CountInfoBlocks(root);
    }

    ASFW_LOG_INFO(Discovery,
                  "AudioSubunit: Apple audio descriptor shape declared=%u actual=%zu roots=%zu blocks=%zu "
                  "routing=%zu subunitPlug=%zu cluster=%zu musicPlug=%zu",
                  declaredLength,
                  actualLength,
                  roots.size(),
                  totalBlocks,
                  counts[kMusicRoutingStatus],
                  counts[kMusicSubunitPlugInfo],
                  counts[kClusterInfo],
                  counts[kMusicPlugInfo]);

    const size_t maxRootsToLog = std::min<size_t>(roots.size(), 4);
    for (size_t i = 0; i < maxRootsToLog; ++i) {
        const auto& root = roots[i];
        ASFW_LOG_INFO(Discovery,
                      "AudioSubunit: Apple audio descriptor root[%zu] type=0x%04x primary=%u children=%zu",
                      i,
                      root.GetType(),
                      root.GetPrimaryFieldsLength(),
                      root.GetNestedBlocks().size());
    }
}

const ASFW::Protocols::AVC::Descriptors::AVCInfoBlock*
GetPlugInfoLikeApple(const ASFW::Protocols::AVC::Descriptors::AVCInfoBlock& root,
                     bool isInput,
                     uint8_t plugIndex) {
    const auto* routing = FindRoutingBlockLikeApple(root);
    if (!routing) {
        return nullptr;
    }

    const auto plugBlocks = ImmediateChildrenOfType(*routing, kMusicSubunitPlugInfo);
    if (plugBlocks.empty()) {
        return nullptr;
    }

    size_t appleIndex = plugIndex;
    if (!isInput) {
        const auto& primary = routing->GetPrimaryData();
        if (primary.empty()) {
            return nullptr;
        }
        appleIndex += primary[0]; // Dest plugs precede source plugs.
    }

    if (appleIndex >= plugBlocks.size()) {
        return nullptr;
    }

    return plugBlocks[appleIndex];
}

void ApplyApplePlugInfo(const ASFW::Protocols::AVC::Descriptors::AVCInfoBlock& root,
                        AudioPlugInfo& plug,
                        const std::unordered_map<uint16_t, std::string>& musicPlugNames) {
    const auto* plugBlock = GetPlugInfoLikeApple(root, plug.isInput, plug.plugNumber);
    if (!plugBlock) {
        return;
    }

    auto plugName = ExtractNameRecursive(*plugBlock);
    if (!plugName.empty()) {
        plug.name = std::move(plugName);
    }

    plug.channelMap.clear();
    plug.channelMusicPlugIDs.clear();
    plug.channelNames.clear();
    plug.channelsPerStream.clear();
    plug.audioStreamCount = 0;
    plug.midiStreamCount = 0;

    for (const auto& cluster : plugBlock->FindAllNestedRecursive(kClusterInfo)) {
        const auto& primary = cluster.GetPrimaryData();
        if (primary.size() < 3) {
            continue;
        }

        const uint8_t signalCount = primary[2];
        plug.channelsPerStream.push_back(signalCount);
        if (signalCount > 0) {
            ++plug.audioStreamCount;
        }

        for (uint8_t i = 0; i < signalCount && (3u + (static_cast<size_t>(i) + 1u) * 4u) <= primary.size(); ++i) {
            const size_t off = 3u + static_cast<size_t>(i) * 4u;
            const uint16_t musicPlugId = ReadBE16Local(primary.data() + off);
            const uint8_t channelPosition = primary[off + 2];
            plug.channelMusicPlugIDs.push_back(musicPlugId);
            plug.channelMap.push_back(channelPosition);

            auto nameIt = musicPlugNames.find(musicPlugId);
            if (nameIt != musicPlugNames.end()) {
                plug.channelNames.push_back(nameIt->second);
            } else if (!plug.name.empty()) {
                plug.channelNames.push_back(plug.name);
            } else {
                plug.channelNames.push_back({});
            }
        }
    }
}

void ApplyAppleStreamFormatDetails(
    AudioPlugInfo& plug,
    const ASFW::Protocols::AVC::StreamFormats::AudioStreamFormat& parsed) {
    using ASFW::Protocols::AVC::StreamFormats::StreamFormatCode;

    // Preserve descriptor-derived routing if descriptor 0x00/0x80 provided it.
    // Apple falls back to stream-format counts when AVCInfoBlock::GetFromPlug
    // cannot provide channels-per-stream/audio-stream-count.
    if (!plug.channelsPerStream.empty() || !plug.channelMap.empty() || !plug.channelMusicPlugIDs.empty()) {
        return;
    }

    plug.channelMap.clear();
    plug.channelMusicPlugIDs.clear();
    plug.channelNames.clear();
    plug.channelsPerStream.clear();
    plug.audioStreamCount = 0;
    plug.midiStreamCount = 0;

    uint16_t syntheticMusicPlugID = 0;
    auto appendAudioChannels = [&](uint8_t count,
                                   const std::vector<ASFW::Protocols::AVC::StreamFormats::ChannelFormatInfo::ChannelDetail>& details) {
        if (count == 0) {
            return;
        }

        plug.channelsPerStream.push_back(count);
        ++plug.audioStreamCount;

        for (uint8_t i = 0; i < count; ++i) {
            uint16_t musicPlugID = syntheticMusicPlugID;
            uint8_t position = i;
            std::string name;

            if (i < details.size()) {
                musicPlugID = details[i].musicPlugID;
                position = details[i].position;
                name = details[i].name;
            }

            plug.channelMusicPlugIDs.push_back(musicPlugID);
            plug.channelMap.push_back(position);
            plug.channelNames.push_back(std::move(name));
            ++syntheticMusicPlugID;
        }
    };

    if (!parsed.channelFormats.empty()) {
        for (const auto& format : parsed.channelFormats) {
            if (format.formatCode == StreamFormatCode::kMIDI) {
                plug.midiStreamCount = static_cast<uint8_t>(
                    std::min<uint16_t>(255, static_cast<uint16_t>(plug.midiStreamCount) + format.channelCount));
                continue;
            }
            appendAudioChannels(format.channelCount, format.channels);
        }
    } else {
        appendAudioChannels(parsed.totalChannels, {});
    }
}

} // namespace

void AudioSubunit::ParseCapabilities(AVCUnit& unit, std::function<void(bool)> completion) {
    auto completionState = Common::ShareCallback(std::move(completion));
    ASFW_LOG_INFO(Discovery, "AudioSubunit: Parsing capabilities for Audio subunit (id=%d)", GetID());
    
    auto unitPtr = unit.shared_from_this();
    
    QueryPlugCounts(unit, [this, unitPtr, completionState](bool success) {
        if (!success) {
            ASFW_LOG_WARNING(Discovery, "AudioSubunit: Failed to query plug counts");
            Common::InvokeSharedCallback(completionState, false);
            return;
        }
        
        ASFW_LOG_INFO(Discovery, "AudioSubunit: Found %d input plugs, %d output plugs",
                     numInputPlugs_, numOutputPlugs_);
        
        inputPlugs_.clear();
        outputPlugs_.clear();
        
        if (numInputPlugs_ > 0) {
            inputPlugs_.resize(numInputPlugs_);
            for (size_t i = 0; i < numInputPlugs_; ++i) {
                inputPlugs_[i].plugNumber = i;
                inputPlugs_[i].isInput = true;
            }
            QueryPlugFormats(*unitPtr, 0, true, *completionState);
        } else if (numOutputPlugs_ > 0) {
            outputPlugs_.resize(numOutputPlugs_);
            for (size_t i = 0; i < numOutputPlugs_; ++i) {
                outputPlugs_[i].plugNumber = i;
                outputPlugs_[i].isInput = false;
            }
            QueryPlugFormats(*unitPtr, 0, false, *completionState);
        } else {
            ASFW_LOG_INFO(Discovery, "AudioSubunit: No plugs to query");
            Common::InvokeSharedCallback(completionState, true);
        }
    });
}

void AudioSubunit::QueryPlugCounts(AVCUnit& unit, std::function<void(bool)> completion) {
    auto completionState = Common::ShareCallback(std::move(completion));
    uint8_t subunitAddr = (static_cast<uint8_t>(GetType()) << 3) | (GetID() & 0x07);
    
    auto cmd = std::make_shared<AVCPlugInfoCommand>(unit.GetFCPTransport(), subunitAddr);
    
    cmd->Submit([this, completionState, cmd](AVCResult result, const AVCPlugInfoCommand::PlugInfo& info) {
        if (IsSuccess(result)) {
            numInputPlugs_ = info.numDestPlugs;
            numOutputPlugs_ = info.numSrcPlugs;
            Common::InvokeSharedCallback(completionState, true);
        } else {
            ASFW_LOG_ERROR(Discovery, "AudioSubunit: PLUG_INFO failed: result=%d",
                          static_cast<int>(result));
            Common::InvokeSharedCallback(completionState, false);
        }
    });
}

void AudioSubunit::QueryPlugFormats(AVCUnit& unit, size_t plugIndex, bool isInput,
                                   std::function<void(bool)> completion) {
    auto completionState = Common::ShareCallback(std::move(completion));
    auto& plugs = isInput ? inputPlugs_ : outputPlugs_;
    
    if (plugIndex >= plugs.size()) {
        if (isInput && numOutputPlugs_ > 0) {
            outputPlugs_.resize(numOutputPlugs_);
            for (size_t i = 0; i < numOutputPlugs_; ++i) {
                outputPlugs_[i].plugNumber = i;
                outputPlugs_[i].isInput = false;
            }
            QueryPlugFormats(unit, 0, false, *completionState);
        } else {
            ASFW_LOG_INFO(Discovery, "AudioSubunit: Finished querying all plug formats");
            Common::InvokeSharedCallback(completionState, true);
        }
        return;
    }
    
    auto unitPtr = unit.shared_from_this();
    
    uint8_t subunitAddr = (static_cast<uint8_t>(GetType()) << 3) | (GetID() & 0x07);
    uint8_t plugNum = plugs[plugIndex].plugNumber;
    
    auto cmd = std::make_shared<AVCStreamFormatCommand>(unit.GetFCPTransport(),
                                                        subunitAddr, plugNum, isInput);
    
    cmd->Submit([this, unitPtr, plugIndex, isInput, completionState, cmd](
                AVCResult result, const std::optional<StreamFormat>& format) {
        auto& plugs = isInput ? inputPlugs_ : outputPlugs_;
        
        if (IsSuccess(result) && format) {
            plugs[plugIndex].currentFormat = *format;
            ASFW_LOG_INFO(Discovery, "AudioSubunit: Plug %d (%{public}s) current format: type=0x%02x",
                         plugs[plugIndex].plugNumber,
                         isInput ? "input" : "output",
                         format->formatType);
        } else {
            ASFW_LOG_WARNING(Discovery, "AudioSubunit: Failed to query current format for plug %d (%{public}s)",
                           plugs[plugIndex].plugNumber, isInput ? "input" : "output");
        }
        
        QueryPlugFormats(*unitPtr, plugIndex + 1, isInput, *completionState);
    });
}

//==============================================================================
// LoadFromDiscovery — populate from AppleDiscoverySequence results
//==============================================================================

void AudioSubunit::LoadFromDiscovery(uint8_t destPlugs, uint8_t srcPlugs,
                                      const std::vector<uint8_t>& descriptorData) {
    ASFW_LOG_INFO(Discovery,
                  "AudioSubunit: LoadFromDiscovery dest=%u src=%u descriptorLen=%zu",
                  destPlugs, srcPlugs, descriptorData.size());

    // Populate base-class plug counts so AVCUnit wire serialization and UI see them.
    SetPlugCounts({.dest = destPlugs, .src = srcPlugs});

    numInputPlugs_ = destPlugs;
    numOutputPlugs_ = srcPlugs;

    inputPlugs_.clear();
    inputPlugs_.resize(numInputPlugs_);
    for (size_t i = 0; i < numInputPlugs_; ++i) {
        inputPlugs_[i].plugNumber = i;
        inputPlugs_[i].isInput = true;
    }

    outputPlugs_.clear();
    outputPlugs_.resize(numOutputPlugs_);
    for (size_t i = 0; i < numOutputPlugs_; ++i) {
        outputPlugs_[i].plugNumber = i;
        outputPlugs_[i].isInput = false;
    }

    if (!descriptorData.empty()) {
        statusDescriptorData_ = descriptorData;
        ParseAppleAudioDescriptor();
    } else {
        statusDescriptorData_.reset();
    }

    ASFW_LOG_INFO(Discovery,
                  "AudioSubunit: LoadFromDiscovery complete — %u in, %u out",
                  numInputPlugs_, numOutputPlugs_);
}

void AudioSubunit::ParseAppleAudioDescriptor() {
    using ASFW::Protocols::AVC::Descriptors::AVCInfoBlock;

    if (!statusDescriptorData_.has_value() || statusDescriptorData_->size() < 8) {
        return;
    }

    const auto& data = *statusDescriptorData_;
    const uint16_t declaredLength = ReadBE16Local(data.data());
    const size_t advertisedEnd = std::min(data.size(), static_cast<size_t>(declaredLength));

    size_t offset = 2;
    size_t parsedBlockCount = 0;
    std::vector<AVCInfoBlock> roots;

    while (offset + 6 <= advertisedEnd) {
        const uint16_t compoundLength = ReadBE16Local(data.data() + offset);
        const size_t blockSize = static_cast<size_t>(compoundLength) + 2u;
        if (blockSize < 6 || compoundLength == 0xFFFF) {
            offset += 2;
            continue;
        }

        size_t consumed = 0;
        auto parsed = AVCInfoBlock::Parse(data.data() + offset, advertisedEnd - offset, consumed);
        if (!parsed || consumed == 0) {
            offset += 2;
            continue;
        }

        roots.push_back(std::move(*parsed));
        ++parsedBlockCount;
        offset += consumed;
    }

    if (roots.empty()) {
        ASFW_LOG_WARNING(Discovery,
                         "AudioSubunit: Apple descriptor parse found no info blocks (declared=%u actual=%zu)",
                         declaredLength,
                         data.size());
        return;
    }

    LogAppleDescriptorShape(roots, declaredLength, data.size());

    for (const auto& root : roots) {
        auto musicPlugNames = CollectMusicPlugNames(root);
        for (auto& plug : inputPlugs_) {
            ApplyApplePlugInfo(root, plug, musicPlugNames);
        }
        for (auto& plug : outputPlugs_) {
            ApplyApplePlugInfo(root, plug, musicPlugNames);
        }
    }

    size_t namedPlugs = 0;
    size_t mappedChannels = 0;
    for (const auto& plug : inputPlugs_) {
        namedPlugs += !plug.name.empty() ? 1u : 0u;
        mappedChannels += plug.channelMap.size();
    }
    for (const auto& plug : outputPlugs_) {
        namedPlugs += !plug.name.empty() ? 1u : 0u;
        mappedChannels += plug.channelMap.size();
    }

    ASFW_LOG_INFO(Discovery,
                  "AudioSubunit: Apple descriptor parse roots=%zu namedPlugs=%zu mappedChannels=%zu",
                  parsedBlockCount,
                  namedPlugs,
                  mappedChannels);
}

void AudioSubunit::ApplyDiscoveryFormatResponse(uint8_t plugId, bool isInput,
                                                const std::vector<uint8_t>& rawResponse) {
    auto& plugs = isInput ? inputPlugs_ : outputPlugs_;
    auto plugIt = std::find_if(plugs.begin(), plugs.end(),
                               [plugId](const AudioPlugInfo& plug) {
                                   return plug.plugNumber == plugId;
                               });
    if (plugIt == plugs.end()) {
        return;
    }

    uint8_t subfunction = 0;
    auto parsed = ParseDiscoveryFormatResponse(rawResponse, subfunction);
    if (!parsed.has_value()) {
        return;
    }

    if (subfunction == ASFW::Protocols::AVC::StreamFormats::kStreamFormatSubfunc_Current) {
        plugIt->currentFormat = ToLegacyStreamFormat(*parsed);
        ApplyAppleStreamFormatDetails(*plugIt, *parsed);
        ASFW_LOG_INFO(Discovery,
                      "AudioSubunit: Applied Apple discovery current format plug=%u %{public}s rate=0x%02x channels=%u audioStreams=%u midiStreams=%u details=%zu",
                      plugId,
                      isInput ? "input" : "output",
                      static_cast<uint8_t>(parsed->sampleRate),
                      parsed->totalChannels,
                      plugIt->audioStreamCount,
                      plugIt->midiStreamCount,
                      plugIt->channelMap.size());
        return;
    }

    if (subfunction == ASFW::Protocols::AVC::StreamFormats::kStreamFormatSubfunc_Supported) {
        const auto alreadyPresent = std::any_of(
            plugIt->supportedFormats.begin(),
            plugIt->supportedFormats.end(),
            [&parsed](const ASFW::Protocols::AVC::StreamFormat& existing) {
                return existing.sampleRate == static_cast<uint8_t>(parsed->sampleRate) &&
                       existing.numChannels == parsed->totalChannels &&
                       existing.rawData == parsed->rawFormatBlock;
            });
        if (!alreadyPresent) {
            plugIt->supportedFormats.push_back(ToLegacyStreamFormat(*parsed));
        }
    }
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void AudioSubunit::SetAudioVolume(AVCUnit& unit, uint8_t plugId, int16_t volume, std::function<void(bool)> completion) {
    auto completionState = Common::ShareCallback(std::move(completion));
    uint8_t subunitAddr = (static_cast<uint8_t>(GetType()) << 3) | (GetID() & 0x07);
    
    // Volume data: 2 bytes, big endian
    std::vector<uint8_t> data;
    data.push_back(static_cast<uint8_t>((volume >> 8) & 0xFF));
    data.push_back(static_cast<uint8_t>(volume & 0xFF));
    
    auto cmd = std::make_shared<AudioFunctionBlockCommand>(
        unit, // AVCUnit implements IAVCCommandSubmitter
        subunitAddr,
        AudioFunctionBlockCommand::CommandType::kControl,
        plugId,
        AudioFunctionBlockCommand::ControlSelector::kVolume,
        data
    );
    
    cmd->Submit([completionState, cmd](AVCResult result, const std::vector<uint8_t>&) {
        if (IsSuccess(result)) {
            ASFW_LOG_V1(AVC, "AudioSubunit: Set volume success");
            Common::InvokeSharedCallback(completionState, true);
        } else {
            ASFW_LOG_ERROR(AVC, "AudioSubunit: Set volume failed: result=%d", static_cast<int>(result));
            Common::InvokeSharedCallback(completionState, false);
        }
    });
}

void AudioSubunit::SetAudioMute(AVCUnit& unit, uint8_t plugId, bool mute, std::function<void(bool)> completion) {
    auto completionState = Common::ShareCallback(std::move(completion));
    uint8_t subunitAddr = (static_cast<uint8_t>(GetType()) << 3) | (GetID() & 0x07);
    
    // Mute data: 1 byte (0x70 = Mute, 0x60 = Unmute) - typical for Audio Subunit
    // Wait, spec says:
    // Mute: 0x70 (On), 0x60 (Off)
    uint8_t muteVal = mute ? 0x70 : 0x60;
    
    auto cmd = std::make_shared<AudioFunctionBlockCommand>(
        unit,
        subunitAddr,
        AudioFunctionBlockCommand::CommandType::kControl,
        plugId,
        AudioFunctionBlockCommand::ControlSelector::kMute,
        std::vector<uint8_t>{muteVal}
    );
    
    cmd->Submit([completionState, cmd](AVCResult result, const std::vector<uint8_t>&) {
        if (IsSuccess(result)) {
            ASFW_LOG_V1(AVC, "AudioSubunit: Set mute success");
            Common::InvokeSharedCallback(completionState, true);
        } else {
            ASFW_LOG_ERROR(AVC, "AudioSubunit: Set mute failed: result=%d", static_cast<int>(result));
            Common::InvokeSharedCallback(completionState, false);
        }
    });
}
