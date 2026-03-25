// SPDX-License-Identifier: LGPL-3.0-or-later
// BeBoB protocol handler for Prism Sound Orpheus.
//
// Implements IDeviceProtocol using standard AV/C Extended Stream Format
// commands (opcode 0x2F) over FCP, with no device-specific quirks.
//
// Reference: Linux sound/firewire/bebob/ (spec_normal)

#pragma once

#include "../IDeviceProtocol.hpp"
#include "../../Ports/FireWireBusPort.hpp"

namespace ASFW::Audio::BeBoB {

/// IDeviceProtocol implementation for BridgeCo BeBoB devices.
/// Currently targets: Prism Sound Orpheus (vendor 0x00001198, model 0x00010048)
class BeBoBProtocol final : public IDeviceProtocol {
public:
    BeBoBProtocol(Protocols::Ports::FireWireBusOps& busOps,
                  Protocols::Ports::FireWireBusInfo& busInfo,
                  uint16_t nodeId);

    IOReturn Initialize() override;
    IOReturn Shutdown() override;
    const char* GetName() const override { return "Prism Sound Orpheus"; }

    /// Configure device for 48kHz stereo duplex via AV/C Extended Stream Format.
    /// Sends CONTROL commands setting oPCR[0] and iPCR[0] to AM824 2ch 48kHz.
    /// Fire-and-forget: submits async FCP writes and returns immediately.
    IOReturn StartDuplex48k() override;

    /// Returns true once the iPCR SetFormat write has succeeded at least once.
    /// False if SetFormat was never sent or failed (e.g., device was mid-reset at init).
    bool IsFormatDone() const override { return mFormatDone_; }

    /// Update the node ID used for AVC writes. Called from bus-reset recovery
    /// before retrying StartDuplex48k() so writes reach the correct node.
    void UpdateRuntimeContext(uint16_t nodeId,
                              Protocols::AVC::FCPTransport*) override {
        mNodeId = nodeId;
    }

private:
    /// Build and send an AV/C Extended Stream Format CONTROL command.
    /// @param plugDir       kPlugDirOutput (oPCR) or kPlugDirInput (iPCR)
    /// @param plugNum       Plug index (0 = first plug)
    /// @param sfcCode       IEC 61883-6 sample frequency code (e.g. kSFC_48000)
    /// @param audioChannels Number of PCM audio channels
    /// @param midiChannels  Number of MIDI channels (0 = no MIDI entry in compound)
    void SendSetFormatCommand(uint8_t plugDir, uint8_t plugNum,
                              uint8_t sfcCode, uint8_t audioChannels,
                              uint8_t midiChannels = 0);

    Protocols::Ports::FireWireBusOps& busOps_;
    Protocols::Ports::FireWireBusInfo& busInfo_;
    uint16_t mNodeId;
    bool mFormatDone_{false};
};

} // namespace ASFW::Audio::BeBoB
