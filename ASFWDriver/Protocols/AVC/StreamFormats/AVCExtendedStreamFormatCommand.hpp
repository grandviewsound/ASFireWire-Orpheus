// SPDX-License-Identifier: LGPL-3.0-or-later
//
// AVCExtendedStreamFormatCommand.hpp
// ASFWDriver - AV/C Protocol Layer
//
// Extended Stream Format Information command (opcode 0xBF, subfunction 0xC0)
// addressed to the UNIT (subunit byte 0xFF), address_mode = unit isochronous plug.
//
// Fix 53: byte-level dtrace of AppleFWAudio (macOS 11) during Orpheus cold attach
// captured Apple's exact CONTROL wire bytes. Three state-mutating commands in 170
// total — all SetExtendedStreamFormat at the UNIT, not the music subunit.
// Prior fixes (51 at 0x60 / 0x2F, 52 at 0x60 / 0xBF) addressed the wrong subunit
// and had the wrong operand layout; Orpheus rejected both.
//
// Reference: TA 2001002 AV/C Stream Format Information Specification §5
//            IEC 61883-6 AM824 compound format

#pragma once

#include "../AVCCommand.hpp"
#include <functional>
#include <vector>

namespace ASFW::Protocols::AVC::StreamFormats {

//==============================================================================
// Extended Stream Format Command (0xBF) — Unit Isochronous Plug
//==============================================================================

constexpr uint8_t kOpcodeExtStreamFormat = 0xBF;

/// Sub-function: SINGLE REQUEST — read current (STATUS) or set current (CONTROL).
/// Subfunction 0xC1 is LIST REQUEST (STATUS only, reads an entry from the
/// supported-format list); do not use with CONTROL.
constexpr uint8_t kSubFuncSingle = 0xC0;

/// Plug direction at unit level (TA 2001002 Table 5-3)
constexpr uint8_t kPlugDirInput  = 0x00;  // iPCR: host→device (playback)
constexpr uint8_t kPlugDirOutput = 0x01;  // oPCR: device→host (recording)

/// Address mode: 0x00 = unit isochronous plug
constexpr uint8_t kAddrModeUnitIsoch = 0x00;

/// AM824 compound format info header
constexpr uint8_t kFmtAM824     = 0x90;   // format_hierarchy_root
constexpr uint8_t kFmtCompound  = 0x40;   // format_hierarchy_level_1
constexpr uint8_t kExtSFC_48000 = 0x04;   // compound sfc: 48 kHz (TA 2001002 Table 5-7)

/// Rate control byte observed on Apple's AppleFWAudio cold-attach capture.
/// Device-specific; Orpheus accepts these exact values per direction.
constexpr uint8_t kRateCtrlPlayback  = 0xFE;  // iPCR direction
constexpr uint8_t kRateCtrlRecording = 0xFA;  // oPCR direction

/// Stream format entry codes (IEC 61883-6)
constexpr uint8_t kStreamFmtMBLA = 0x06;  // Multi-bit Linear Audio (PCM)
constexpr uint8_t kStreamFmtMIDI = 0x0D;  // MIDI conformant data

/// Sends Extended Stream Format CONTROL or STATUS at unit level.
///
/// CONTROL sets the plug format; STATUS queries it. The command includes the
/// full AM824 compound format info block with per-cluster channel layout.
///
/// Wire layout (CONTROL, playback direction, 6 MBLA pairs + MIDI — 29 bytes):
/// ```
///   00 ff bf c0 00 00 00 00 ff ff 90 40 04 fe 07 02 06 ...(×6)... 01 0d
///   ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^
///   |  |  |  |  |  |  |  |  |  |  |  |  |  |  number_of_stream_format_info_fields
///   |  |  |  |  |  |  |  |  |  |  |  |  |  rate_control (0xFE iPCR / 0xFA oPCR)
///   |  |  |  |  |  |  |  |  |  |  |  |  sfc (compound): 0x04 = 48 kHz
///   |  |  |  |  |  |  |  |  |  |  |  format_hierarchy_level_1 = compound
///   |  |  |  |  |  |  |  |  |  |  format_hierarchy_root = AM824
///   |  |  |  |  |  |  |  |  |  ext_length_lo
///   |  |  |  |  |  |  |  |  ext_length_hi
///   |  |  |  |  |  |  |  status
///   |  |  |  |  |  |  plug number
///   |  |  |  |  |  address_mode (0x00 = unit isoch plug)
///   |  |  |  |  plug_direction (0x00 input / 0x01 output)
///   |  |  |  subfunction (0xC0 = SINGLE REQUEST)
///   |  |  opcode (0xBF)
///   |  subunit address (0xFF = unit)
///   ctype (0x00 = CONTROL)
/// ```
class AVCExtendedStreamFormatCommand : public AVC::AVCCommand {
public:
    /// Construct a CONTROL command that sets the plug format to an AM824
    /// compound 48 kHz stream with the given per-cluster layout.
    ///
    /// @param transport  FCP transport
    /// @param isInput    true  = iPCR (dest plug, host→device playback, direction 0x00)
    ///                   false = oPCR (source plug, device→host recording, direction 0x01)
    /// @param plugId     Isoch plug number (usually 0)
    /// @param audioPairs Stereo MBLA pair count (6 for iPCR DBS 13, 5 for oPCR DBS 11)
    /// @param hasMidi    true to append a 1-ch MIDI conformant data entry
    AVCExtendedStreamFormatCommand(AVC::FCPTransport& transport,
                                    bool isInput,
                                    uint8_t plugId,
                                    uint8_t audioPairs,
                                    bool hasMidi)
        : AVCCommand(transport, BuildControlCdb(isInput, plugId, audioPairs, hasMidi)) {}

    /// Construct a CONTROL command that replays a previously discovered raw
    /// Extended Stream Format block.
    AVCExtendedStreamFormatCommand(AVC::FCPTransport& transport,
                                    bool isInput,
                                    uint8_t plugId,
                                    const std::vector<uint8_t>& rawFormatBlock)
        : AVCCommand(transport, BuildControlCdb(isInput, plugId, rawFormatBlock)) {}

    /// Construct a STATUS query command for a unit isoch plug.
    AVCExtendedStreamFormatCommand(AVC::FCPTransport& transport,
                                    bool isInput,
                                    uint8_t plugId)
        : AVCCommand(transport, BuildStatusCdb(isInput, plugId)) {}

    /// Submit with simplified result callback
    void Submit(std::function<void(AVC::AVCResult)> completion) {
        AVCCommand::Submit([completion](AVC::AVCResult result, const AVC::AVCCdb&) {
            completion(result);
        });
    }

private:
    static AVC::AVCCdb BuildControlCdb(bool isInput, uint8_t plugId,
                                        uint8_t audioPairs, bool hasMidi) {
        AVC::AVCCdb cdb{};
        cdb.ctype   = static_cast<uint8_t>(AVC::AVCCommandType::kControl);
        cdb.subunit = kAVCSubunitUnit;   // 0xFF — unit addressing, NOT music subunit
        cdb.opcode  = kOpcodeExtStreamFormat;

        const uint8_t numEntries = audioPairs + (hasMidi ? 1 : 0);
        const uint8_t plugDir    = isInput ? kPlugDirInput : kPlugDirOutput;
        const uint8_t rateCtrl   = isInput ? kRateCtrlPlayback : kRateCtrlRecording;

        size_t ix = 0;
        cdb.operands[ix++] = kSubFuncSingle;
        cdb.operands[ix++] = plugDir;
        cdb.operands[ix++] = kAddrModeUnitIsoch;
        cdb.operands[ix++] = plugId;
        cdb.operands[ix++] = 0x00;          // status
        cdb.operands[ix++] = 0xFF;          // ext_length_hi
        cdb.operands[ix++] = 0xFF;          // ext_length_lo

        // AM824 compound format info block
        cdb.operands[ix++] = kFmtAM824;
        cdb.operands[ix++] = kFmtCompound;
        cdb.operands[ix++] = kExtSFC_48000;
        cdb.operands[ix++] = rateCtrl;
        cdb.operands[ix++] = numEntries;

        for (uint8_t pp = 0; pp < audioPairs; ++pp) {
            cdb.operands[ix++] = 0x02;           // 2 channels (stereo)
            cdb.operands[ix++] = kStreamFmtMBLA; // MBLA (PCM)
        }

        if (hasMidi) {
            cdb.operands[ix++] = 0x01;           // 1 channel
            cdb.operands[ix++] = kStreamFmtMIDI;
        }

        cdb.operandLength = ix;
        return cdb;
    }

    static AVC::AVCCdb BuildControlCdb(bool isInput,
                                        uint8_t plugId,
                                        const std::vector<uint8_t>& rawFormatBlock) {
        AVC::AVCCdb cdb{};
        cdb.ctype   = static_cast<uint8_t>(AVC::AVCCommandType::kControl);
        cdb.subunit = kAVCSubunitUnit;
        cdb.opcode  = kOpcodeExtStreamFormat;

        const uint8_t plugDir = isInput ? kPlugDirInput : kPlugDirOutput;

        size_t ix = 0;
        cdb.operands[ix++] = kSubFuncSingle;
        cdb.operands[ix++] = plugDir;
        cdb.operands[ix++] = kAddrModeUnitIsoch;
        cdb.operands[ix++] = plugId;
        cdb.operands[ix++] = 0x00;          // status
        cdb.operands[ix++] = 0xFF;          // ext_length_hi
        cdb.operands[ix++] = 0xFF;          // ext_length_lo

        const size_t maxRawBytes =
            (AVC::kAVCOperandMaxLength > ix) ? (AVC::kAVCOperandMaxLength - ix) : 0;
        const size_t rawBytes =
            (rawFormatBlock.size() < maxRawBytes) ? rawFormatBlock.size() : maxRawBytes;
        for (size_t i = 0; i < rawBytes; ++i) {
            cdb.operands[ix++] = rawFormatBlock[i];
        }

        cdb.operandLength = ix;
        return cdb;
    }

    static AVC::AVCCdb BuildStatusCdb(bool isInput, uint8_t plugId) {
        AVC::AVCCdb cdb{};
        cdb.ctype   = static_cast<uint8_t>(AVC::AVCCommandType::kStatus);
        cdb.subunit = kAVCSubunitUnit;
        cdb.opcode  = kOpcodeExtStreamFormat;

        cdb.operands[0] = kSubFuncSingle;
        cdb.operands[1] = isInput ? kPlugDirInput : kPlugDirOutput;
        cdb.operands[2] = kAddrModeUnitIsoch;
        cdb.operands[3] = plugId;
        cdb.operands[4] = 0x00;              // status
        cdb.operands[5] = 0xFF;              // ext_length_hi
        cdb.operands[6] = 0xFF;              // ext_length_lo
        cdb.operandLength = 7;
        return cdb;
    }
};

} // namespace ASFW::Protocols::AVC::StreamFormats
