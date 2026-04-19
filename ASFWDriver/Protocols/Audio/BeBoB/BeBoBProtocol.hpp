// SPDX-License-Identifier: LGPL-3.0-or-later
// BeBoB protocol handler for Prism Sound Orpheus.
//
// Fix 53: mirrors Apple's byte-level SetExtendedStreamFormat sequence captured
// via fbt dtrace of AppleFWAudio on macOS 11 during Orpheus cold attach.
// Apple issues exactly three state-mutating CONTROL commands, all at the UNIT
// (subunit byte 0xFF), opcode 0xBF, subfunction 0xC0:
//
//   1. During initHardware (pre-CMP):      iPCR  (host→device, 6 MBLA + MIDI)
//   2. During StartStream/SetUpOutputConn: iPCR  (same payload, re-send)
//   3. During StartStream/SetUpOutputConn: oPCR  (device→host, 5 MBLA + MIDI)
//
// Apple never sends 0x18/0x19 plug-signal-format CONTROL, and never sends any
// vendor commands — SetSampleRate IS a re-send of Extended Stream Format with
// the new sfc byte. StartDuplex48k handles step 1; PreparePlaybackPath handles
// steps 2+3 after CMP connect.
//
// Reference: TA-2001002 AV/C Stream Format Information, IEC 61883-6

#pragma once

#include "../IDeviceProtocol.hpp"
#include "../../Ports/FireWireBusPort.hpp"
#include "BeBoBTypes.hpp"

#include <atomic>
#include <functional>

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

    /// Report Orpheus channel topology: asymmetric — 10 PCM in, 12 PCM out.
    bool GetRuntimeAudioStreamCaps(AudioStreamRuntimeCaps& outCaps) const override {
        outCaps.hostInputPcmChannels = kOrpheusOutputAudioChannels;  // 10 (device→host, DBS=11)
        outCaps.hostOutputPcmChannels = kOrpheusInputAudioChannels;  // 12 (host→device, DBS=13)
        outCaps.deviceToHostAm824Slots = kOrpheusOutputChannels;     // 11 (DBS incl MIDI)
        outCaps.hostToDeviceAm824Slots = kOrpheusInputChannels;      // 13 (DBS incl MIDI)
        outCaps.sampleRateHz = 48000;
        return true;
    }

    /// Pre-CMP Extended Stream Format CONTROL (0xBF/0xC0) at UNIT, iPCR plug.
    /// Mirrors Apple's initHardware-direct SetExtendedStreamFormat call.
    IOReturn StartDuplex48k() override;

    /// Returns true once stream format setup is complete.
    bool IsFormatDone() const override { return mFormatDone_.load(std::memory_order_acquire); }

    /// Returns true while the pre-CMP ExtStreamFormat CONTROL is still pending.
    bool IsFormatInFlight() const override {
        return mFormatInFlight_.load(std::memory_order_acquire);
    }

    /// Update the node ID / FCP transport used for AVC writes across bus resets.
    void UpdateRuntimeContext(uint16_t nodeId,
                              Protocols::AVC::FCPTransport* transport) override;

    /// Post-CMP Extended Stream Format CONTROL (0xBF/0xC0) at UNIT — iPCR then
    /// oPCR. Mirrors Apple's StartStream/SetUpOutputConnection pair. Apple's
    /// SetSampleRate implementation is itself a re-send of this command, so no
    /// separate 0x18/0x19 call is issued.
    IOReturn PreparePlaybackPath() override;

private:
    using FormatCompletion = std::function<void(IOReturn)>;

    /// Submit an Extended Stream Format CONTROL (0xBF/0xC0) at UNIT level for
    /// a unit isochronous plug. Completion fires on the FCP response.
    void SendExtStreamFormatControl(uint32_t sequence,
                                     bool isInput,
                                     uint8_t plugId,
                                     FormatCompletion completion);

    void FailFormatVerification(uint32_t sequence, const char* stage, IOReturn status);
    void CompleteFormatVerification(uint32_t sequence);

    Protocols::Ports::FireWireBusOps& busOps_;
    Protocols::Ports::FireWireBusInfo& busInfo_;
    std::atomic<uint16_t> mNodeId_{0};
    std::atomic<Protocols::AVC::FCPTransport*> transport_{nullptr};
    std::atomic<bool> mFormatDone_{false};
    std::atomic<bool> mFormatInFlight_{false};
    std::atomic<bool> mOutputFormatVerified_{false};
    std::atomic<bool> mInputFormatVerified_{false};
    std::atomic<uint32_t> mFormatSequence_{0};
};

} // namespace ASFW::Audio::BeBoB
