//
// AudioFunctionBlockCommand.cpp
// ASFWDriver - AV/C Protocol Layer
//

#include "AudioFunctionBlockCommand.hpp"

namespace ASFW::Protocols::AVC {

AudioFunctionBlockCommand::AudioFunctionBlockCommand(IAVCCommandSubmitter& submitter,
                                                     uint8_t subunitAddr,
                                                     CommandType type,
                                                     uint8_t functionBlockId,
                                                     ControlSelector selector,
                                                     std::vector<uint8_t> data)
    : submitter_(submitter)
    , cdb_(BuildCdb(subunitAddr, type, functionBlockId, selector, data)) {}

void AudioFunctionBlockCommand::Submit(std::function<void(AVCResult, const std::vector<uint8_t>&)> completion) {
    submitter_.SubmitCommand(cdb_, [completion](AVCResult result, const AVCCdb& response) {
        if (IsSuccess(result)) {
            // Apple AM824 feature responses place selector data after:
            // type, blockID, infoType, pathLength, channel, selector, selectorAttr.
            std::vector<uint8_t> responseData;
            if (response.operandLength > 7) {
                for (size_t i = 7; i < response.operandLength; ++i) {
                    responseData.push_back(response.operands[i]);
                }
            }
            completion(result, responseData);
        } else {
            completion(result, {});
        }
    });
}

AVCCdb AudioFunctionBlockCommand::BuildCdb(uint8_t subunitAddr,
                                           CommandType type,
                                           uint8_t functionBlockId,
                                           ControlSelector selector,
                                           const std::vector<uint8_t>& data) {
    AVCCdb cdb;
    cdb.ctype = static_cast<uint8_t>(type == CommandType::kControl ? AVCCommandType::kControl : AVCCommandType::kStatus);
    cdb.subunit = subunitAddr;
    cdb.opcode = 0xB8; // FUNCTION BLOCK

    size_t offset = 0;
    const uint8_t kCurrentInfoType = 0x10;
    const uint8_t kChannelPathLength = 0x02;
    const uint8_t kMasterChannel = 0x00;

    // Function Block Type: Feature (0x81)
    cdb.operands[offset++] = 0x81;

    // Function Block ID
    cdb.operands[offset++] = functionBlockId;

    // Apple AM824AVC::SetChannelVolume/SetChannelMute use infoType 0x10
    // (current value), a one-byte channel path, and then selector metadata.
    cdb.operands[offset++] = kCurrentInfoType;
    cdb.operands[offset++] = kChannelPathLength;
    cdb.operands[offset++] = kMasterChannel;

    // Control Selector
    cdb.operands[offset++] = static_cast<uint8_t>(selector);

    // Selector attribute/value length. analysis shows 0x02 for volume and 0x01
    // for mute; those are the data lengths we pass from the call sites.
    cdb.operands[offset++] = static_cast<uint8_t>(data.size());

    // Control Data
    for (uint8_t byte : data) {
        cdb.operands[offset++] = byte;
    }

    cdb.operandLength = offset;
    return cdb;
}

} // namespace ASFW::Protocols::AVC
