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

#include <array>
#include <atomic>
#include <functional>
#include <vector>

namespace ASFW::Audio::BeBoB {

/// Mirror of the Prism Control Panel's internal device-state struct, populated
/// from the same read paths. Source: analysis of
/// Orpheus::Device::{Get(13), Sync()} (May 2026).
///
/// Path A — vendor STATUS opcode 0xBF at music subunit returns 8 bytes packed
/// with: master vol, master enabled, mute/lock/meters/source/wordclock/adat/
/// headphone/brightness. This matches the panel's `Get(13)` bulk read.
///
/// Path B — AV/C SignalSource STATUS at UNIT returns the sync source via
/// resp[5] decode. This matches the panel's `Sync()` getter.
struct OrpheusDeviceState {
    bool     bulkReadValid{false};      // 0xBF STATUS succeeded
    bool     signalSourceValid{false};  // SignalSource STATUS succeeded
    bool     adatReadValid{false};      // 0xB3 STATUS succeeded
    bool     unknownB6Valid{false};     // 0xB6 STATUS succeeded

    // From 0xBF bulk STATUS. Field offsets per analysis of
    // Orpheus::Device::Get(int) case 13 response decoder at 0x10001b1cc.
    int16_t  masterVolValue{0};         // resp[7..8] s16 BE
    int16_t  masterVolEnabled{0};       // resp[9..10] s16 BE
    bool     masterMute{false};         // resp[11] bit 0
    bool     masterLock{false};         // resp[11] bit 1
    bool     unknownBit11_2{false};     // resp[11] bit 2 (struct +0x1b0)
    uint8_t  metersValue{0};            // resp[11] bits 4-7
    uint8_t  outputSource{0xFF};         // resp[12] low nibble — Unit::Src
    uint8_t  wordclockValue{0xFF};      // resp[12] high nibble
    uint8_t  adatModeBulk{0xFF};        // resp[13]
    uint8_t  headphoneMix{0xFF};        // resp[14] low nibble
    uint8_t  metersBrightness{0xFF};    // resp[14] high nibble
    std::array<uint8_t, 12> bulkRaw{};  // raw 0xBF response operand bytes

    // From SignalSource STATUS (separate sync-source control, not 0xBF source)
    uint8_t  syncSource_avc{0xFF};      // decoded enum from resp[5]
    uint8_t  signalSourceDestPlug{0};   // 0x07 (no ADAT) or 0x08 (ADAT)
    uint8_t  signalSourceRespByte4{0};  // raw resp[4] for diagnosis
    uint8_t  signalSourceRespByte5{0};  // raw resp[5] for diagnosis
    bool     signalSourceUsedAdatPlug{false};

    // From 0xB3 ADAT STATUS — full ADAT struct (operand layout TBD)
    std::array<uint8_t, 12> adatRaw{};

    // From 0xB6 STATUS — single quadlet stored at device+0x134 by panel
    std::array<uint8_t, 12> unknownB6Raw{};
};

struct OrpheusMixOutputState {
    bool     valid{false};
    uint8_t  outputIndex{0};
    int16_t  outputGain{0};
    bool     muted{false};
    uint16_t soloMask{0};
    bool     defeated{false};
    std::array<int16_t, kOrpheusMixInputCount> inputGain{};
    std::array<bool, kOrpheusMixInputCount> inputMuted{};
    std::array<bool, kOrpheusMixInputCount> inputUsesBalance{};
    std::array<int8_t, kOrpheusMixInputCount> inputPanBalance{};
    std::array<uint8_t, kOrpheusMixBulkOperandLength> raw{};
};

struct OrpheusAnalogState {
    bool    valid{false};
    uint8_t channelIndex{0};
    uint8_t type{0};
    uint8_t allFlags{0};
    uint8_t filter{0};
    uint8_t gain1{0};
    uint8_t gain2{0};
    bool    inputGainFlag{false};   // `all` bit 0, per Analog::GetAll()
    bool    outputGainFlag{false};  // `all` bit 1, per Analog::GetAll()
    bool    bit2{false};
    bool    bit3{false};
    bool    bit5{false};
    bool    bit6{false};
    std::array<uint8_t, 12> raw{};
};

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

    bool OwnsAttachTimeFormatProgramming() const override { return true; }

    void UpdateDiscoveredStreamFormatBlocks(
        const std::vector<uint8_t>& playback48kRawFormatBlock,
        const std::vector<uint8_t>& capture48kRawFormatBlock) override;

    /// Returns true once stream format setup is complete.
    bool IsFormatDone() const override { return mFormatDone_.load(std::memory_order_acquire); }

    /// Returns true while the pre-CMP ExtStreamFormat CONTROL is still pending.
    bool IsFormatInFlight() const override {
        return mFormatInFlight_.load(std::memory_order_acquire);
    }

    /// Update the node ID / FCP transport used for AVC writes across bus resets.
    void UpdateRuntimeContext(uint16_t nodeId,
                              Protocols::AVC::FCPTransport* transport) override;

    /// AppleFWAudioDevice::StartAllStreams keys its input-first/output-first
    /// path from the selected clock source. For Orpheus, the Prism panel's
    /// SignalSource STATUS path exposes that live sync source.
    std::optional<AudioStartOrderHint> GetAppleStartOrderHint() override;

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

    /// Send a vendor-dependent CONTROL command (opcode 0x00) at the audio
    /// subunit (0x08) with the Prism Sound OUI (00 11 98). Used to flip the
    /// Orpheus DAC source switch to FireWire (cmd 0xB1, value 0x01). Blocks
    /// up to ~250 ms for the device response.
    /// Reference: working old-laptop XML <src>1</src>; OrpheusModels.swift
    /// buildDeviceFrame() in the control app.
    IOReturn SendVendorDeviceCommand(uint8_t cmdByte, uint8_t value);

    /// Apply the same 0xBF bulk unit-state CONTROL path used by the Prism
    /// control panel after XML load (`Orpheus::Device::Set(13, 0)`). Reads the
    /// current 0xBF state, preserves every raw payload byte except the Unit::Src
    /// low nibble, and writes it back with source forced to FireWire.
    IOReturn ApplyVendorBulkSource(uint8_t source);

    /// analysis-backed Prism mix output commands (`Orpheus::Device::SetMix`).
    /// These send 15-byte vendor CONTROL frames and are used narrowly to make
    /// Line 1/2 listen to the same DAW slots that our Apple-style stream sends.
    IOReturn SendVendorMixOutputByteCommand(uint8_t cmdByte,
                                            uint8_t outputIndex,
                                            uint8_t value);
    IOReturn SendVendorMixOutputGainCommand(uint8_t outputIndex, int16_t gain);
    IOReturn SendVendorMixInputByteCommand(uint8_t cmdByte,
                                           uint8_t outputIndex,
                                           uint8_t inputIndex,
                                           uint8_t value);
    IOReturn SendVendorMixInputGainCommand(uint8_t outputIndex,
                                           uint8_t inputIndex,
                                           int16_t gain);
    IOReturn EnsureLine12DawPlaybackMix();

    /// Send a vendor-dependent STATUS query (opcode 0x00, ctype 0x01) at the
    /// audio subunit. Mirrors SendVendorDeviceCommand wire layout but with
    /// ctype=STATUS and `param` (instead of `value`) at operand[4]; remaining
    /// operand bytes are 0xFF padding. On success, copies the response's 12
    /// operand bytes into outOperands. Used by diagnostics for vendor state
    /// reads that do not need custom request padding.
    IOReturn QueryVendorDeviceStatus(uint8_t cmdByte,
                                      uint8_t param,
                                      std::array<uint8_t, 12>& outOperands);

    /// Vendor STATUS opcode 0xBF at music subunit — bulk state read. Mirrors
    /// the panel's `Orpheus::Device::Get(13)` (analysis addr 0x10001b1cc, vendor
    /// opcode 0xBF). Returns master vol, master enabled, mute/lock, meters,
    /// output source, wordclock, ADAT mode, headphone mix, brightness — all in
    /// one 8-byte response payload. Populates the `OrpheusDeviceState` fields
    /// prefixed `*_bulk` / direct field names. Read-only.
    IOReturn QueryVendorBulkState(OrpheusDeviceState& out);

    /// AV/C SignalSource STATUS at UNIT, destination External plug 0x60/0x07
    /// (or 0x60/0x08 if ADAT input present per `useAdatPlug`). Mirrors the
    /// panel's `Orpheus::Device::Sync()` (analysis addr 0x1cb78). Populates
    /// `*_avc` fields. Read-only.
    IOReturn QuerySignalSourceSync(bool useAdatPlug, OrpheusDeviceState& out);

    /// Vendor STATUS opcode 0xB3 — full ADAT-section state. Mirrors panel's
    /// `Orpheus::Device::Get(8)` AVC path. Operand layout currently logged
    /// as raw bytes (decode TBD). Read-only.
    IOReturn QueryAdatStatus(OrpheusDeviceState& out);

    /// Vendor STATUS opcode 0xB6 — single-quadlet read used by panel's
    /// `Orpheus::Device::Get(10)`; result stored at `device+0x134` (purpose
    /// undocumented). Captured for cataloging; logged raw. Read-only.
    IOReturn QueryUnknownB6(OrpheusDeviceState& out);

    /// Vendor STATUS opcode 0xEF — full 74-byte output mixer read. Mirrors
    /// `Orpheus::Device::GetMix(output, 8)` and decodes output gain/mute/solo/
    /// defeated plus 12 per-output input gains and pan/balance bytes. Read-only.
    IOReturn QueryMixOutputState(uint8_t outputIndex, OrpheusMixOutputState& out);

    /// Vendor STATUS opcode 0xCF — full analog channel read. Mirrors
    /// `Orpheus::Device::GetAnalog(channel, 10)` and decodes the response bytes
    /// consumed by `Analog::SetType`, `SetAll`, `SetFilter`, and `SetGain`.
    /// Read-only.
    IOReturn QueryAnalogState(uint8_t channelIndex, OrpheusAnalogState& out);

    /// Diagnostic: snapshot the full device state via all four read paths and
    /// log every decoded field. Called
    /// from `StartDuplex48k` (attach time) and `PreparePlaybackPath` (pre/post
    /// source writes) so we get a baseline + side-effect comparison. Read-only.
    void LogFullDeviceState(const char* phaseTag);

    /// Diagnostic: read all six Orpheus mixer output blocks once after source
    /// apply. This is intentionally separate from LogFullDeviceState to keep
    /// attach-time FCP traffic modest.
    void LogMixOutputStates(const char* phaseTag);

    /// Diagnostic: read all eight Orpheus analog channel blocks via 0xCF after
    /// the playback path is prepared. Read-only.
    void LogAnalogStates(const char* phaseTag);

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
    std::vector<uint8_t> playback48kRawFormatBlock_{};
    std::vector<uint8_t> capture48kRawFormatBlock_{};
};

} // namespace ASFW::Audio::BeBoB
