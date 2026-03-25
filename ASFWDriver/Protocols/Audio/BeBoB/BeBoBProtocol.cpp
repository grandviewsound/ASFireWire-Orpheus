// SPDX-License-Identifier: LGPL-3.0-or-later

#include "BeBoBProtocol.hpp"
#include "BeBoBTypes.hpp"
#include "../../../Logging/Logging.hpp"
#include "../../Ports/FireWireBusPort.hpp"

#include <array>

namespace ASFW::Audio::BeBoB {

BeBoBProtocol::BeBoBProtocol(Protocols::Ports::FireWireBusOps& busOps,
                             Protocols::Ports::FireWireBusInfo& busInfo,
                             uint16_t nodeId)
    : busOps_(busOps), busInfo_(busInfo), mNodeId(nodeId)
{
    ASFW_LOG(Audio, "BeBoBProtocol: Created for Prism Sound Orpheus (node=0x%04x)", nodeId);
}

IOReturn BeBoBProtocol::Initialize()
{
    ASFW_LOG(Audio, "BeBoBProtocol: Initialize (node=0x%04x)", mNodeId);
    return kIOReturnSuccess;
}

IOReturn BeBoBProtocol::Shutdown()
{
    ASFW_LOG(Audio, "BeBoBProtocol: Shutdown");
    return kIOReturnSuccess;
}

IOReturn BeBoBProtocol::StartDuplex48k()
{
    ASFW_LOG(Audio, "BeBoBProtocol: Configuring Orpheus for 48kHz duplex "
             "oPCR=%u+%uMIDI iPCR=%u+%uMIDI",
             kOrpheusOutputAudioChannels, kOrpheusOutputMidiChannels,
             kOrpheusInputAudioChannels, kOrpheusInputMidiChannels);

    // oPCR[0]: device output plug (device→host, recording) — 10 audio + 1 MIDI
    SendSetFormatCommand(kPlugDirOutput, 0x00, kSFC_48000,
                         kOrpheusOutputAudioChannels, kOrpheusOutputMidiChannels);

    // iPCR[0]: device input plug (host→device, playback) — 12 audio + 1 MIDI
    SendSetFormatCommand(kPlugDirInput, 0x00, kSFC_48000,
                         kOrpheusInputAudioChannels, kOrpheusInputMidiChannels);

    return kIOReturnSuccess;
}

void BeBoBProtocol::SendSetFormatCommand(uint8_t plugDir, uint8_t plugNum,
                                          uint8_t sfcCode, uint8_t audioChannels,
                                          uint8_t midiChannels)
{
    // AV/C Extended Stream Format Information - CONTROL (set current format)
    //
    // Byte layout:
    //   [0] ctype        = 0x00 (CONTROL)
    //   [1] subunit      = 0xFF (UNIT: type=0x1F, id=0x7)
    //   [2] opcode       = 0x2F (EXTENDED STREAM FORMAT INFORMATION)
    //   [3] subfunction  = 0xC0 (set/get currently active format)
    //   [4] addr_type    = 0x00 (unit PCR plugs)
    //   [5] direction    = 0x00 output (oPCR) | 0x01 input (iPCR)
    //   [6] reserved     = 0xFF
    //   [7] plug_num     = plug index
    //   --- IEC 61883-6 Compound AM824 format compound ---
    //   [8]  = 0x90  hierarchy root: AM824 compound
    //   [9]  = sfc   sample frequency code (0x02 = 48000 Hz per IEC 61883-6)
    //   [10] = 0x00  reserved
    //   [11] = N     number of format info entries (2 for audio+MIDI)
    //   [12] = 0x06  label: MBLA (Multi-Bit Linear Audio / PCM)
    //   [13] = n     audio channel count
    //   [14] = 0x0D  label: MIDI conformant  (Fix 29)
    //   [15] = m     MIDI channel count       (Fix 29)
    //
    // Reference: AV/C Extended Stream Format spec TA 2001007
    //            Linux sound/firewire/lib.c avc_stream_set_format()
    //            Linux sound/firewire/bebob/bebob_stream.c

    const uint8_t entryCount = (midiChannels > 0) ? 2 : 1;
    const uint32_t cmdSize = (midiChannels > 0)
        ? kSetFormat2EntryCommandSize : kSetFormat1EntryCommandSize;

    std::array<uint8_t, kSetFormat2EntryCommandSize> cmd = {{
        // AV/C frame header
        0x00,                         // ctype: CONTROL
        0xFF,                         // subunit: UNIT
        kAVCOpcodeExtStreamFmt,       // opcode: 0x2F
        kExtStreamFmtSubfuncCurrent,  // subfunction: 0xC0
        kPlugAddrUnit,                // address type: unit PCR
        plugDir,                      // plug direction
        0xFF,                         // reserved
        plugNum,                      // plug number
        // IEC 61883-6 Compound AM824 format compound
        kAM824CompoundMarker,         // 0x90
        sfcCode,                      // sample frequency code
        0x00,                         // reserved
        entryCount,                   // number of format info entries
        kAM824LabelMBLA,              // 0x06: MBLA (PCM)
        audioChannels,                // audio channel count
        kAM824LabelMIDI,              // 0x0D: MIDI conformant
        midiChannels,                 // MIDI channel count
    }};

    const auto gen = busInfo_.GetGeneration();
    const Async::FWAddress addr{
        Async::FWAddress::AddressParts{
            static_cast<uint16_t>((kFCPCommandAddress >> 32) & 0xFFFF),
            static_cast<uint32_t>(kFCPCommandAddress & 0xFFFFFFFF)
        }
    };

    // WriteBlock copies the payload into DMA-backed storage before returning,
    // so cmd[] does not need to outlive this call.
    busOps_.WriteBlock(gen,
        FW::NodeId(mNodeId),
        addr,
        std::span<const uint8_t>{cmd.data(), cmdSize},
        FW::FwSpeed::S400,
        [this, plugDir, plugNum](Async::AsyncStatus status,
                                 std::span<const uint8_t>) {
            if (status != Async::AsyncStatus::kSuccess) {
                ASFW_LOG(Audio,
                    "BeBoBProtocol: SetFormat write failed (dir=%u plug=%u)",
                    plugDir, plugNum);
            } else {
                ASFW_LOG(Audio,
                    "BeBoBProtocol: SetFormat write OK (dir=%u plug=%u)",
                    plugDir, plugNum);
                // iPCR (input plug) is the last SetFormat sent. Mark format
                // as done so the recovery path knows to skip retrying it.
                if (plugDir == kPlugDirInput) {
                    mFormatDone_ = true;
                    ASFW_LOG(Audio,
                        "BeBoBProtocol: SetFormat complete - BeBoB bus reset expected");
                }
            }
        });
}

} // namespace ASFW::Audio::BeBoB
