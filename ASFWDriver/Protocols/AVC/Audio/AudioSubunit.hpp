//
// AudioSubunit.hpp
// ASFWDriver - AV/C Protocol Layer
//
// Audio Subunit (type 0x01) implementation
//

#pragma once

#include "../Subunit.hpp"
#include "../AVCStreamFormatCommand.hpp"
#include <string>
#include <vector>
#include <optional>

namespace ASFW::Protocols::AVC::Audio {

/// Audio plug information
struct AudioPlugInfo {
    uint8_t plugNumber{0};
    bool isInput{false};
    std::string name;
    std::vector<uint16_t> channelMusicPlugIDs;
    std::vector<uint8_t> channelMap;
    std::vector<std::string> channelNames;
    std::vector<uint8_t> channelsPerStream;
    uint8_t audioStreamCount{0};
    uint8_t midiStreamCount{0};
    std::optional<StreamFormat> currentFormat;
    std::vector<StreamFormat> supportedFormats;
};

/// Audio Subunit class
class AudioSubunit : public Subunit {
public:
    AudioSubunit(AVCSubunitType type, uint8_t id)
        : Subunit(type, id) {}
    
    std::string GetName() const override { return "Audio"; }
    
    void ParseCapabilities(AVCUnit& unit, std::function<void(bool)> completion) override;
    
    // Accessors
    uint8_t GetNumInputPlugs() const { return numInputPlugs_; }
    uint8_t GetNumOutputPlugs() const { return numOutputPlugs_; }
    const std::vector<AudioPlugInfo>& GetInputPlugs() const { return inputPlugs_; }
    const std::vector<AudioPlugInfo>& GetOutputPlugs() const { return outputPlugs_; }
    const std::optional<std::vector<uint8_t>>& GetStatusDescriptorData() const {
        return statusDescriptorData_;
    }

    /// Populate from AppleDiscoverySequence results (synchronous path).
    /// Sets plug counts from Phase 3. Descriptor data is Audio descriptor 0x00,
    /// matching AppleFWAudioDevice's type-0x01 AM824AVC::GetSubunitDescriptor path.
    void LoadFromDiscovery(uint8_t destPlugs, uint8_t srcPlugs,
                           const std::vector<uint8_t>& descriptorData);

    /// Apply a raw StreamFormat response captured during Apple-style discovery.
    /// Apple reads Audio subunit output plugs first, then input plugs, using
    /// GetExtendedStreamFormat status requests at subunit address 0x08.
    void ApplyDiscoveryFormatResponse(uint8_t plugId, bool isInput,
                                      const std::vector<uint8_t>& rawResponse);

private:
    uint8_t numInputPlugs_{0};
    uint8_t numOutputPlugs_{0};
    std::vector<AudioPlugInfo> inputPlugs_;
    std::vector<AudioPlugInfo> outputPlugs_;
    std::optional<std::vector<uint8_t>> statusDescriptorData_;
    
    void QueryPlugCounts(AVCUnit& unit, std::function<void(bool)> completion);
    void QueryPlugFormats(AVCUnit& unit, size_t plugIndex, bool isInput,
                         std::function<void(bool)> completion);
    void ParseAppleAudioDescriptor();

    /// Set volume for a function block (plug)
    /// @param unit AVCUnit for command submission
    /// @param plugId Plug ID (Function Block ID)
    /// @param volume Volume level (0x7FFF = 0dB, etc.)
    /// @param completion Callback
    void SetAudioVolume(AVCUnit& unit, uint8_t plugId, int16_t volume, std::function<void(bool)> completion);

    /// Set mute for a function block (plug)
    /// @param unit AVCUnit for command submission
    /// @param plugId Plug ID (Function Block ID)
    /// @param mute True to mute, false to unmute
    /// @param completion Callback
    void SetAudioMute(AVCUnit& unit, uint8_t plugId, bool mute, std::function<void(bool)> completion);
};

} // namespace ASFW::Protocols::AVC::Audio
