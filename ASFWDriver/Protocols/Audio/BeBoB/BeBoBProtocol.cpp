// SPDX-License-Identifier: LGPL-3.0-or-later
//
// Fix 53: byte-level mirror of Apple's SetExtendedStreamFormat sequence.
//
// Apple's AppleFWAudio (dtrace, macOS 11, Orpheus cold attach) issues exactly
// three state-mutating AVC commands — all Extended Stream Format CONTROL at
// UNIT addressing (subunit 0xFF), opcode 0xBF, subfunction 0xC0:
//
//   1. initHardware direct      → iPCR (playback, 6 MBLA + MIDI, rate_ctrl 0xFE)
//   2. StartStream → SetUpOutCon → iPCR (same payload, re-send on stream start)
//   3. StartStream → SetUpOutCon → oPCR (recording, 5 MBLA + MIDI, rate_ctrl 0xFA)
//
// StartDuplex48k issues command #1 before CMP connect. PreparePlaybackPath
// issues commands #2 and #3 after iPCR connect. Apple never sends 0x18/0x19
// plug-signal-format CONTROL and never sends vendor commands; SetSampleRate is
// itself a re-send of Extended Stream Format with a new sfc byte.

#include "BeBoBProtocol.hpp"
#include "BeBoBTypes.hpp"
#include "../../../Logging/Logging.hpp"
#include "../../AVC/AVCCommand.hpp"
#include "../../AVC/AVCDefs.hpp"
#include "../../AVC/FCPTransport.hpp"
#include "../../AVC/StreamFormats/AVCExtendedStreamFormatCommand.hpp"
#include "../../Ports/FireWireBusPort.hpp"

#include <DriverKit/IOLib.h>
#include <algorithm>
#include <array>
#include <cstdio>
#include <memory>
#include <vector>

namespace ASFW::Audio::BeBoB {

using Protocols::AVC::AVCCommand;
using Protocols::AVC::AVCCdb;
using Protocols::AVC::AVCResult;
using Protocols::AVC::StreamFormats::AVCExtendedStreamFormatCommand;

namespace {

constexpr uint32_t kVendorCommandPollMs = 5;
constexpr uint32_t kVendorCommandTimeoutMs = 250;
constexpr uint8_t kLine12MixOutput = 0;
constexpr uint8_t kDawInputLeft = 0;
constexpr uint8_t kDawInputRight = 1;
constexpr int16_t kUnityMixGain = 0;
constexpr int8_t kFullLeftPan = -20;
constexpr int8_t kFullRightPan = 20;

[[nodiscard]] const char* PlugDirectionString(bool isInput) noexcept
{
    return isInput ? "iPCR" : "oPCR";
}

[[nodiscard]] const char* AVCResultToString(AVCResult result) noexcept
{
    switch (result) {
        case AVCResult::kAccepted:          return "accepted";
        case AVCResult::kImplementedStable: return "stable";
        case AVCResult::kChanged:           return "changed";
        case AVCResult::kInTransition:      return "in-transition";
        case AVCResult::kInterim:           return "interim";
        case AVCResult::kNotImplemented:    return "not-implemented";
        case AVCResult::kRejected:          return "rejected";
        case AVCResult::kInvalidResponse:   return "invalid-response";
        case AVCResult::kTimeout:           return "timeout";
        case AVCResult::kBusReset:          return "bus-reset";
        case AVCResult::kTransportError:    return "transport-error";
        case AVCResult::kBusy:              return "busy";
    }
    return "unknown";
}

[[nodiscard]] IOReturn MapAVCResultToIOReturn(AVCResult result) noexcept
{
    switch (result) {
        case AVCResult::kAccepted:
        case AVCResult::kImplementedStable:
        case AVCResult::kChanged:
            return kIOReturnSuccess;
        case AVCResult::kNotImplemented:
            return kIOReturnUnsupported;
        case AVCResult::kInTransition:
        case AVCResult::kInterim:
        case AVCResult::kBusy:
            return kIOReturnBusy;
        case AVCResult::kTimeout:
            return kIOReturnTimeout;
        case AVCResult::kBusReset:
            return kIOReturnNotResponding;
        default:
            return kIOReturnError;
    }
}

[[nodiscard]] IOReturn SubmitBlockingAVCCommand(Protocols::AVC::FCPTransport& transport,
                                                const AVCCdb& cdb,
                                                AVCCdb* responseOut = nullptr)
{
    struct CommandState {
        std::atomic<bool> done{false};
        std::atomic<IOReturn> status{kIOReturnTimeout};
        AVCCdb response{};
    };
    auto state = std::make_shared<CommandState>();

    auto cmd = std::make_shared<AVCCommand>(transport, cdb);
    cmd->Submit([state](AVCResult result, const AVCCdb& response) {
        state->response = response;
        state->status.store(MapAVCResultToIOReturn(result), std::memory_order_release);
        state->done.store(true, std::memory_order_release);
    });

    for (uint32_t waited = 0; waited < kVendorCommandTimeoutMs; waited += kVendorCommandPollMs) {
        if (state->done.load(std::memory_order_acquire)) {
            break;
        }
        IOSleep(kVendorCommandPollMs);
    }

    if (responseOut != nullptr) {
        *responseOut = state->response;
    }
    return state->status.load(std::memory_order_acquire);
}

[[nodiscard]] bool RawFormatBlockLooksUsable(const std::vector<uint8_t>& rawFormatBlock) noexcept
{
    return rawFormatBlock.size() >= 3 && rawFormatBlock[0] == 0x90;
}

// Sync-source enum decode — confirmed against the Prism Control Panel's
// `Orpheus::Device::SetSyncAvc(int)` switch builder + `Sync()` response decoder
// (Apr 26 2026).
[[nodiscard]] const char* DecodeSyncSource(uint8_t value) noexcept
{
    switch (value) {
        case kOrpheusSyncLocal:     return "Local (Internal master)";
        case kOrpheusSyncFreeRun:   return "Free-run (no source)";
        case kOrpheusSyncWordclock: return "Wordclock (external)";
        case kOrpheusSyncSPDIF:     return "S/PDIF (external)";
        case kOrpheusSyncADAT:      return "ADAT (external)";
        case kOrpheusSyncSlave:     return "Slave (PC-DAW slave mode)";
        default:                    return "unknown";
    }
}

// 0xBF bulk state low nibble at response frame[12]. analysis shows
// Orpheus::Device::Get(13) stores this into Unit::Src, not the sync-source
// enum. The sync source is the separate AV/C SignalSource path below.
[[nodiscard]] const char* DecodeOutputSource(uint8_t value) noexcept
{
    switch (value) {
        case kOrpheusSourceFireWire: return "FireWire/DAW";
        default:                     return "unknown";
    }
}

[[nodiscard]] const char* SyncSourceMuteRisk(uint8_t value) noexcept
{
    switch (value) {
        case kOrpheusSyncLocal:
        case kOrpheusSyncFreeRun:
        case kOrpheusSyncSlave:
            return "safe (internal/free-run/slave)";
        case kOrpheusSyncWordclock:
        case kOrpheusSyncSPDIF:
        case kOrpheusSyncADAT:
            return "EXTERNAL — mutes if cable absent";
        default:
            return "unknown";
    }
}

[[nodiscard]] bool IsAppleExternalClockSource(uint8_t value) noexcept
{
    switch (value) {
        case kOrpheusSyncWordclock:
        case kOrpheusSyncSPDIF:
        case kOrpheusSyncADAT:
            return true;
        default:
            return false;
    }
}

[[nodiscard]] const char* AppleStartOrderReasonForSync(uint8_t value) noexcept
{
    switch (value) {
        case kOrpheusSyncWordclock:
            return "orpheus-signalsource-wordclock-external";
        case kOrpheusSyncSPDIF:
            return "orpheus-signalsource-spdif-external";
        case kOrpheusSyncADAT:
            return "orpheus-signalsource-adat-external";
        case kOrpheusSyncSlave:
            return "orpheus-signalsource-slave-output-first";
        case kOrpheusSyncLocal:
            return "orpheus-signalsource-local-output-first";
        case kOrpheusSyncFreeRun:
            return "orpheus-signalsource-freerun-output-first";
        default:
            return "orpheus-signalsource-unknown";
    }
}

// Decode SignalSource STATUS resp[5] back to sync-source enum. Mirrors the
// switch logic in `Orpheus::Device::Sync()` at analysis addr 0x1ccae (no-ADAT path)
// and the ADAT jump table at 0x1ccc7. resp[4] is checked separately:
//   resp[4] == 0x60 → enum 0 (Local default)
//   resp[4] == 0xFF → decode resp[5] via this table.
[[nodiscard]] uint8_t DecodeSignalSourceResponse(uint8_t respByte4,
                                                  uint8_t respByte5,
                                                  bool hasAdatInput) noexcept
{
    if (respByte4 == 0x60) {
        return kOrpheusSyncLocal;
    }
    if (respByte4 != 0xFF) {
        return 0xFF;  // unrecognized response shape
    }

    if (hasAdatInput) {
        // Panel ADAT path: source byte for plug 8 ranges 0x85-0x88. Layout per
        // SetSyncAvc: enum 2→0x88, 3→0x85, 4→0x86, 5→0x87. (Mirror of write.)
        switch (respByte5) {
            case 0x00: return kOrpheusSyncFreeRun;
            case 0x85: return kOrpheusSyncSPDIF;
            case 0x86: return kOrpheusSyncADAT;
            case 0x87: return kOrpheusSyncSlave;
            case 0x88: return kOrpheusSyncWordclock;
            default:   return 0xFF;
        }
    }

    // No-ADAT path: source byte for plug 7. Layout per SetSyncAvc:
    // enum 2→0x87, 3→0x85, 5→0x86. enum 4 (ADAT) is invalid here.
    switch (respByte5) {
        case 0x00: return kOrpheusSyncFreeRun;
        case 0x85: return kOrpheusSyncSPDIF;
        case 0x86: return kOrpheusSyncSlave;
        case 0x87: return kOrpheusSyncWordclock;
        default:   return 0xFF;
    }
}

void FormatOperandsHex(const std::array<uint8_t, 12>& ops, char* out, size_t outLen) noexcept
{
    if (outLen == 0) {
        return;
    }
    out[0] = '\0';
    size_t pos = 0;
    for (size_t i = 0; i < ops.size() && pos + 4 < outLen; ++i) {
        const int written = std::snprintf(out + pos, outLen - pos,
                                          (i == 0) ? "%02x" : " %02x",
                                          ops[i]);
        if (written <= 0) {
            break;
        }
        pos += static_cast<size_t>(written);
    }
}

void FormatInt16List(const std::array<int16_t, kOrpheusMixInputCount>& values,
                     char* out,
                     size_t outLen) noexcept
{
    if (outLen == 0) {
        return;
    }
    out[0] = '\0';
    size_t pos = 0;
    for (size_t i = 0; i < values.size() && pos + 8 < outLen; ++i) {
        const int written = std::snprintf(out + pos, outLen - pos,
                                          (i == 0) ? "%d" : ",%d",
                                          static_cast<int>(values[i]));
        if (written <= 0) {
            break;
        }
        pos += static_cast<size_t>(written);
    }
}

void FormatInt8List(const std::array<int8_t, kOrpheusMixInputCount>& values,
                    char* out,
                    size_t outLen) noexcept
{
    if (outLen == 0) {
        return;
    }
    out[0] = '\0';
    size_t pos = 0;
    for (size_t i = 0; i < values.size() && pos + 6 < outLen; ++i) {
        const int written = std::snprintf(out + pos, outLen - pos,
                                          (i == 0) ? "%d" : ",%d",
                                          static_cast<int>(values[i]));
        if (written <= 0) {
            break;
        }
        pos += static_cast<size_t>(written);
    }
}

template <typename BoolArray>
[[nodiscard]] uint16_t BoolArrayMask(const BoolArray& values) noexcept
{
    uint16_t mask = 0;
    for (size_t i = 0; i < values.size() && i < 16; ++i) {
        if (values[i]) {
            mask |= static_cast<uint16_t>(1u << i);
        }
    }
    return mask;
}

} // namespace

BeBoBProtocol::BeBoBProtocol(Protocols::Ports::FireWireBusOps& busOps,
                             Protocols::Ports::FireWireBusInfo& busInfo,
                             uint16_t nodeId)
    : busOps_(busOps), busInfo_(busInfo), mNodeId_(nodeId)
{
    ASFW_LOG(Audio, "BeBoBProtocol: Created for Prism Sound Orpheus (node=0x%04x)", nodeId);
}

IOReturn BeBoBProtocol::Initialize()
{
    ASFW_LOG(Audio, "BeBoBProtocol: Initialize (node=0x%04x)", mNodeId_.load(std::memory_order_acquire));
    return kIOReturnSuccess;
}

IOReturn BeBoBProtocol::Shutdown()
{
    ASFW_LOG(Audio, "BeBoBProtocol: Shutdown");
    return kIOReturnSuccess;
}

// ---------------------------------------------------------------------------
// StartDuplex48k — Apple cmd #1: Extended Stream Format CONTROL, iPCR, pre-CMP
// ---------------------------------------------------------------------------
// Mirrors AppleFWAudio's initHardware-direct SetExtendedStreamFormat: a single
// CONTROL on the playback (iPCR) unit isoch plug, before any CMP traffic.

IOReturn BeBoBProtocol::StartDuplex48k()
{
    auto* transport = transport_.load(std::memory_order_acquire);
    if (!transport) {
        ASFW_LOG_ERROR(Audio,
                       "BeBoBProtocol: Cannot set stream format without FCP transport "
                       "(node=0x%04x)",
                       mNodeId_.load(std::memory_order_acquire));
        return kIOReturnNotReady;
    }

    if (mFormatDone_.load(std::memory_order_acquire)) {
        ASFW_LOG(Audio,
                 "BeBoBProtocol: Stream format already set "
                 "(node=0x%04x)",
                 mNodeId_.load(std::memory_order_acquire));
        return kIOReturnSuccess;
    }

    bool expected = false;
    if (!mFormatInFlight_.compare_exchange_strong(expected, true,
                                                  std::memory_order_acq_rel,
                                                  std::memory_order_acquire)) {
        ASFW_LOG(Audio,
                 "BeBoBProtocol: Stream format setup already in flight "
                 "(node=0x%04x)",
                 mNodeId_.load(std::memory_order_acquire));
        return kIOReturnSuccess;
    }

    mFormatDone_.store(false, std::memory_order_release);
    mOutputFormatVerified_.store(false, std::memory_order_release);
    mInputFormatVerified_.store(false, std::memory_order_release);

    const uint32_t sequence =
        mFormatSequence_.fetch_add(1, std::memory_order_acq_rel) + 1;

    ASFW_LOG(Audio,
             "BeBoBProtocol: Pre-CMP ExtStreamFormat CONTROL (iPCR) "
             "(seq=%u node=0x%04x)",
             sequence,
             mNodeId_.load(std::memory_order_acquire));

    // Attach-time baseline: snapshot full device state via vendor 0xBF bulk
    // STATUS + AV/C SignalSource STATUS (both plug variants) + 0xB3 ADAT +
    // 0xB6. Read-only. Captures persistent device state (sync source, master
    // vol, mute, ADAT mode, etc.) before we touch anything.
    LogFullDeviceState("attach-baseline");

    SendExtStreamFormatControl(sequence, /*isInput=*/true, 0x00,
        [this, sequence](IOReturn status) {
            if (status != kIOReturnSuccess) {
                FailFormatVerification(sequence, "pre-cmp-ext-stream-format", status);
                return;
            }
            mInputFormatVerified_.store(true, std::memory_order_release);
            mOutputFormatVerified_.store(true, std::memory_order_release);
            CompleteFormatVerification(sequence);
        });

    return kIOReturnSuccess;
}

void BeBoBProtocol::UpdateRuntimeContext(uint16_t nodeId,
                                         Protocols::AVC::FCPTransport* transport)
{
    mNodeId_.store(nodeId, std::memory_order_release);
    transport_.store(transport, std::memory_order_release);
}

void BeBoBProtocol::UpdateDiscoveredStreamFormatBlocks(
    const std::vector<uint8_t>& playback48kRawFormatBlock,
    const std::vector<uint8_t>& capture48kRawFormatBlock)
{
    playback48kRawFormatBlock_ = playback48kRawFormatBlock;
    capture48kRawFormatBlock_ = capture48kRawFormatBlock;

    ASFW_LOG(Audio,
             "BeBoBProtocol: Updated discovered 48k raw format blocks "
             "(playback=%zu bytes capture=%zu bytes)",
             playback48kRawFormatBlock_.size(),
             capture48kRawFormatBlock_.size());
}

void BeBoBProtocol::UpdateDiscoveredRateFormatBlocks(
    const std::vector<RateStreamFormat>& rateBlocks)
{
    rateFormatBlocks_ = rateBlocks;
    ASFW_LOG(Audio,
             "BeBoBProtocol: stored per-rate ExtStreamFormat blocks for %zu rates",
             rateFormatBlocks_.size());
}

IOReturn BeBoBProtocol::SetSampleRate(uint32_t rateHz)
{
    auto* transport = transport_.load(std::memory_order_acquire);
    if (!transport) {
        ASFW_LOG_ERROR(Audio, "BeBoBProtocol: SetSampleRate requires FCP transport");
        return kIOReturnNotReady;
    }

    // Locate the discovered ExtStreamFormat blocks for the requested rate.
    const RateStreamFormat* match = nullptr;
    for (const auto& entry : rateFormatBlocks_) {
        if (entry.rateHz == rateHz) {
            match = &entry;
            break;
        }
    }
    if (match == nullptr) {
        ASFW_LOG_ERROR(Audio,
                       "BeBoBProtocol: SetSampleRate(%u) — no discovered format block for that rate",
                       rateHz);
        return kIOReturnUnsupported;
    }

    ASFW_LOG(Audio,
             "BeBoBProtocol: SetSampleRate(%u) — re-sending ExtStreamFormat CONTROL "
             "(iPCR playback=%zuB, oPCR capture=%zuB)",
             rateHz, match->playbackRawFormatBlock.size(),
             match->captureRawFormatBlock.size());

    struct CommandState {
        std::atomic<bool> done{false};
        std::atomic<IOReturn> status{kIOReturnTimeout};
    };

    // Send one direction's ExtStreamFormat CONTROL at the target rate, blocking.
    auto submitOne = [&](bool isInput, const std::vector<uint8_t>& rawFormatBlock,
                         const char* tag) -> IOReturn {
        if (!RawFormatBlockLooksUsable(rawFormatBlock)) {
            ASFW_LOG_ERROR(Audio,
                           "BeBoBProtocol: SetSampleRate(%u) %{public}s block unusable",
                           rateHz, tag);
            return kIOReturnUnsupported;
        }
        auto state = std::make_shared<CommandState>();
        auto command = std::make_shared<AVCExtendedStreamFormatCommand>(
            *transport, isInput, /*plugId=*/0x00, rawFormatBlock);
        command->Submit([state](AVCResult result) {
            state->status.store(MapAVCResultToIOReturn(result), std::memory_order_release);
            state->done.store(true, std::memory_order_release);
        });
        for (uint32_t waited = 0; waited < kVendorCommandTimeoutMs; waited += kVendorCommandPollMs) {
            if (state->done.load(std::memory_order_acquire)) break;
            IOSleep(kVendorCommandPollMs);
        }
        const IOReturn kr = state->status.load(std::memory_order_acquire);
        ASFW_LOG(Audio, "BeBoBProtocol: SetSampleRate(%u) %{public}s CONTROL result=0x%x",
                 rateHz, tag, kr);
        return kr;
    };

    const IOReturn inputKr = submitOne(/*isInput=*/true, match->playbackRawFormatBlock, "iPCR");
    const IOReturn outputKr = submitOne(/*isInput=*/false, match->captureRawFormatBlock, "oPCR");
    if (inputKr != kIOReturnSuccess) {
        return inputKr;
    }
    if (outputKr != kIOReturnSuccess) {
        return outputKr;
    }

    // STATUS read-back: ask the device what rate it now reports, so a HW log
    // proves the device actually adopted the switch (not just ACCEPTED it).
    auto verifyOne = [&](bool isInput, const char* tag) {
        auto state = std::make_shared<CommandState>();
        auto responseOps = std::make_shared<std::vector<uint8_t>>();
        auto statusCmd = std::make_shared<AVCExtendedStreamFormatCommand>(
            *transport, isInput, /*plugId=*/0x00);
        statusCmd->AVCCommand::Submit(
            [state, responseOps](AVCResult result, const AVCCdb& response) {
                if (response.operandLength > 0) {
                    responseOps->assign(response.operands.begin(),
                                        response.operands.begin() + response.operandLength);
                }
                state->status.store(Protocols::AVC::IsSuccess(result) ? kIOReturnSuccess
                                                                      : MapAVCResultToIOReturn(result),
                                    std::memory_order_release);
                state->done.store(true, std::memory_order_release);
            });
        for (uint32_t waited = 0; waited < kVendorCommandTimeoutMs; waited += kVendorCommandPollMs) {
            if (state->done.load(std::memory_order_acquire)) break;
            IOSleep(kVendorCommandPollMs);
        }
        const auto& ops = *responseOps;
        // operand[5] is the AM824 nominal-rate control field in the Extended
        // Stream Format response; log it raw so the device's adopted rate is
        // visible without decoding here.
        const int rateField = ops.size() > 5 ? static_cast<int>(ops[5]) : -1;
        ASFW_LOG(Audio,
                 "BeBoBProtocol: SetSampleRate(%u) STATUS %{public}s ops=%zu rateField=0x%02x",
                 rateHz, tag, ops.size(), rateField);
    };
    verifyOne(/*isInput=*/true, "iPCR");
    verifyOne(/*isInput=*/false, "oPCR");

    return kIOReturnSuccess;
}

void BeBoBProtocol::SetDiscoveredAudioCaps(const AudioStreamRuntimeCaps& caps)
{
    // Adopt only a complete, self-consistent set. DBS (AM824 wire slots) must be
    // at least the PCM width — a smaller value means discovery lost the wire
    // format, in which case the Orpheus fallback is safer than a bad DBS.
    const bool complete =
        caps.hostInputPcmChannels > 0 && caps.hostOutputPcmChannels > 0 &&
        caps.deviceToHostAm824Slots >= caps.hostInputPcmChannels &&
        caps.hostToDeviceAm824Slots >= caps.hostOutputPcmChannels &&
        caps.sampleRateHz > 0;

    if (!complete) {
        ASFW_LOG(Audio,
                 "BeBoBProtocol: ignoring partial discovered caps "
                 "(in=%u out=%u dbsIn=%u dbsOut=%u rate=%u) — keeping fallback",
                 caps.hostInputPcmChannels, caps.hostOutputPcmChannels,
                 caps.deviceToHostAm824Slots, caps.hostToDeviceAm824Slots,
                 caps.sampleRateHz);
        return;
    }

    mHostInputPcmChannels_.store(caps.hostInputPcmChannels, std::memory_order_relaxed);
    mHostOutputPcmChannels_.store(caps.hostOutputPcmChannels, std::memory_order_relaxed);
    mDeviceToHostAm824Slots_.store(caps.deviceToHostAm824Slots, std::memory_order_relaxed);
    mHostToDeviceAm824Slots_.store(caps.hostToDeviceAm824Slots, std::memory_order_relaxed);
    mSampleRateHz_.store(caps.sampleRateHz, std::memory_order_relaxed);
    mCapsValid_.store(true, std::memory_order_release);

    ASFW_LOG(Audio,
             "BeBoBProtocol: adopted device-driven caps in=%u out=%u "
             "dbsIn=%u dbsOut=%u rate=%u",
             caps.hostInputPcmChannels, caps.hostOutputPcmChannels,
             caps.deviceToHostAm824Slots, caps.hostToDeviceAm824Slots,
             caps.sampleRateHz);
}

std::optional<AudioStartOrderHint> BeBoBProtocol::GetAppleStartOrderHint()
{
    auto* transport = transport_.load(std::memory_order_acquire);
    if (!transport) {
        ASFW_LOG_WARNING(Audio,
                         "BeBoBProtocol: Apple start order hint requires FCP transport");
        return std::nullopt;
    }

    OrpheusDeviceState ssNoAdat;
    const IOReturn ssKr7 = QuerySignalSourceSync(/*useAdatPlug=*/false, ssNoAdat);
    OrpheusDeviceState ssAdat;
    const IOReturn ssKr8 = QuerySignalSourceSync(/*useAdatPlug=*/true, ssAdat);

    uint8_t syncSource = 0xFF;
    uint8_t plug = 0;
    uint8_t raw4 = 0;
    uint8_t raw5 = 0;

    if (ssKr7 == kIOReturnSuccess && ssNoAdat.syncSource_avc != 0xFF) {
        syncSource = ssNoAdat.syncSource_avc;
        plug = ssNoAdat.signalSourceDestPlug;
        raw4 = ssNoAdat.signalSourceRespByte4;
        raw5 = ssNoAdat.signalSourceRespByte5;
    } else if (ssKr8 == kIOReturnSuccess && ssAdat.syncSource_avc != 0xFF) {
        syncSource = ssAdat.syncSource_avc;
        plug = ssAdat.signalSourceDestPlug;
        raw4 = ssAdat.signalSourceRespByte4;
        raw5 = ssAdat.signalSourceRespByte5;
    }

    if (syncSource == 0xFF) {
        ASFW_LOG_WARNING(Audio,
                         "BeBoBProtocol: Apple start order hint unavailable "
                         "SignalSource plug7=0x%x/0x%x plug8=0x%x/0x%x",
                         ssKr7,
                         ssNoAdat.syncSource_avc,
                         ssKr8,
                         ssAdat.syncSource_avc);
        return std::nullopt;
    }

    const bool inputFirst = IsAppleExternalClockSource(syncSource);
    const char* reason = AppleStartOrderReasonForSync(syncSource);
    ASFW_LOG(Audio,
             "BeBoBProtocol: Apple start order hint SignalSource plug=0x%02x "
             "raw4=0x%02x raw5=0x%02x sync=0x%x (%{public}s) -> %{public}s "
             "reason=%{public}s",
             plug,
             raw4,
             raw5,
             syncSource,
             DecodeSyncSource(syncSource),
             inputFirst ? "input-first" : "output-first",
             reason);
    // For BeBoB, external-clock and input-first are the same predicate
    // (IsAppleExternalClockSource: Wordclock/SPDIF/ADAT). externalClock drives
    // the transmit SYT discipline (Apple externalSync regime); inputFirst drives
    // StartAllStreams order.
    return AudioStartOrderHint{inputFirst, reason, /*externalClock=*/inputFirst};
}

// ---------------------------------------------------------------------------
// PreparePlaybackPath — Apple cmds #2/#3: post-iPCR-connect re-send
// ---------------------------------------------------------------------------
// After both CMP connects complete, Apple's StartStream path issues
// SetUpOutputConnection which calls SetSampleRate, which IS a re-send of
// SetExtendedStreamFormat. Sequence on the wire (Apr 2026 dtrace):
//   #2  iPCR (playback, 6 MBLA + MIDI, rate_ctrl 0xFE)
//   #3  oPCR (recording, 5 MBLA + MIDI, rate_ctrl 0xFA)
// Both are subfunction 0xC0 (single/set), unit addressing (0xFF). No 0x18/0x19,
// no vendor commands.

IOReturn BeBoBProtocol::PreparePlaybackPath()
{
    auto* transport = transport_.load(std::memory_order_acquire);
    if (!transport) {
        ASFW_LOG_ERROR(Audio, "BeBoBProtocol: PreparePlaybackPath requires FCP transport");
        return kIOReturnNotReady;
    }

    ASFW_LOG(Audio,
             "BeBoBProtocol: Post-CMP ExtStreamFormat re-send (iPCR + oPCR)");

    struct CommandState {
        std::atomic<bool> done{false};
        std::atomic<IOReturn> status{kIOReturnTimeout};
    };

    auto submitOne = [&](bool isInput, uint8_t audioPairs, const char* tag) -> IOReturn {
        auto state = std::make_shared<CommandState>();
        const auto& rawFormatBlock =
            isInput ? playback48kRawFormatBlock_ : capture48kRawFormatBlock_;

        std::shared_ptr<AVCExtendedStreamFormatCommand> command;
        if (RawFormatBlockLooksUsable(rawFormatBlock)) {
            command = std::make_shared<AVCExtendedStreamFormatCommand>(
                *transport, isInput, /*plugId=*/0x00, rawFormatBlock);
            ASFW_LOG(Audio,
                     "BeBoBProtocol: Post-CMP ExtStreamFormat %{public}s using discovered "
                     "raw block (%zu bytes)",
                     tag,
                     rawFormatBlock.size());
        } else {
            command = std::make_shared<AVCExtendedStreamFormatCommand>(
                *transport, isInput, /*plugId=*/0x00, audioPairs, /*hasMidi=*/true);
            ASFW_LOG_WARNING(Audio,
                             "BeBoBProtocol: Post-CMP ExtStreamFormat %{public}s falling back "
                             "to synthetic Orpheus payload",
                             tag);
        }
        command->Submit([state](AVCResult result) {
            state->status.store(MapAVCResultToIOReturn(result), std::memory_order_release);
            state->done.store(true, std::memory_order_release);
        });

        for (uint32_t waited = 0; waited < kVendorCommandTimeoutMs; waited += kVendorCommandPollMs) {
            if (state->done.load(std::memory_order_acquire)) break;
            IOSleep(kVendorCommandPollMs);
        }

        const IOReturn kr = state->status.load(std::memory_order_acquire);
        ASFW_LOG(Audio,
                 "BeBoBProtocol: Post-CMP ExtStreamFormat %{public}s result=0x%x",
                 tag, kr);
        return kr;
    };

    // #2: iPCR — playback direction, 6 stereo MBLA pairs + 1 MIDI
    const IOReturn inputKr =
        submitOne(/*isInput=*/true, kOrpheusInputMBLAPairs, "iPCR");

    // #3: oPCR — recording direction, 5 stereo MBLA pairs + 1 MIDI
    const IOReturn outputKr =
        submitOne(/*isInput=*/false, kOrpheusOutputMBLAPairs, "oPCR");

    if (inputKr != kIOReturnSuccess) {
        return inputKr;
    }
    if (outputKr != kIOReturnSuccess) {
        return outputKr;
    }

    // Fix 92: STATUS readback of the format we just programmed. The device
    // returns 0x09 ACCEPTED on CONTROL even if it silently overrides our
    // requested format with its own preferred AM824 layout. STATUS returns the
    // authoritative current format. Mismatch between what we sent and what
    // device reports back is a load-bearing silence candidate (May 17 — wire
    // is byte-equivalent to Apple golden but DAW Feed meter still reads 0).
    //
    // Read-only: we log raw operands but do not fail the bring-up; if the
    // device disagrees we still want to see what it disagrees on.
    auto verifyOne = [&](bool isInput, const std::vector<uint8_t>& sentBlock, const char* tag) {
        auto state = std::make_shared<CommandState>();
        auto responseOps = std::make_shared<std::vector<uint8_t>>();

        auto statusCmd = std::make_shared<AVCExtendedStreamFormatCommand>(
            *transport, isInput, /*plugId=*/0x00);

        // Explicit qualification: derived class's simplified Submit hides the
        // base's (response,result) overload; we want the response CDB here.
        statusCmd->AVCCommand::Submit(
            [state, responseOps](AVCResult result, const AVCCdb& response) {
                if (response.operandLength > 0) {
                    responseOps->assign(response.operands.begin(),
                                        response.operands.begin() + response.operandLength);
                }
                state->status.store(Protocols::AVC::IsSuccess(result) ? kIOReturnSuccess
                                                                      : MapAVCResultToIOReturn(result),
                                    std::memory_order_release);
                state->done.store(true, std::memory_order_release);
            });

        for (uint32_t waited = 0; waited < kVendorCommandTimeoutMs; waited += kVendorCommandPollMs) {
            if (state->done.load(std::memory_order_acquire)) break;
            IOSleep(kVendorCommandPollMs);
        }

        const IOReturn kr = state->status.load(std::memory_order_acquire);
        if (kr != kIOReturnSuccess) {
            ASFW_LOG_WARNING(Audio,
                             "BeBoBProtocol: Post-CMP ExtStreamFormat %{public}s STATUS readback failed kr=0x%x — cannot verify device-committed format",
                             tag, kr);
            return;
        }

        // STATUS response operand layout mirrors CONTROL:
        //   [0]=subfunc 0xC0  [1]=dir  [2]=addr_mode  [3]=plug  [4]=status
        //   [5]=ext_len_hi    [6]=ext_len_lo
        //   [7+] = AM824 compound format info block (this is what `sentBlock`
        //          stores — starts at the format_hierarchy_root byte 0x90).
        constexpr size_t kBlockStart = 7;
        const std::vector<uint8_t> reported(
            responseOps->size() > kBlockStart
                ? std::vector<uint8_t>(responseOps->begin() + kBlockStart, responseOps->end())
                : std::vector<uint8_t>{});

        // Format both blocks as compact hex for log inspection.
        auto hexOf = [](const std::vector<uint8_t>& v) {
            char buf[3 * 64 + 1];
            size_t pos = 0;
            const size_t n = std::min<size_t>(v.size(), 32);
            for (size_t i = 0; i < n; ++i) {
                pos += std::snprintf(buf + pos, sizeof(buf) - pos, "%02x ", v[i]);
            }
            if (v.size() > 32) {
                std::snprintf(buf + pos, sizeof(buf) - pos, "...(+%zu)", v.size() - 32);
            }
            return std::string{buf};
        };

        const bool matches = (reported == sentBlock);
        ASFW_LOG(Audio,
                 "BeBoBProtocol: Post-CMP ExtStreamFormat %{public}s STATUS readback ctype=ImplementedStable matches=%{public}s (sent=%zuB reported=%zuB)",
                 tag, matches ? "YES" : "NO",
                 sentBlock.size(), reported.size());
        ASFW_LOG(Audio,
                 "BeBoBProtocol: Post-CMP ExtStreamFormat %{public}s sent:     [%{public}s]",
                 tag, hexOf(sentBlock).c_str());
        ASFW_LOG(Audio,
                 "BeBoBProtocol: Post-CMP ExtStreamFormat %{public}s reported: [%{public}s]",
                 tag, hexOf(reported).c_str());
        if (!matches) {
            ASFW_LOG_WARNING(Audio,
                             "BeBoBProtocol: Post-CMP ExtStreamFormat %{public}s device committed a DIFFERENT format than requested — first divergence at byte %zu",
                             tag,
                             [&]() -> size_t {
                                 const size_t n = std::min(sentBlock.size(), reported.size());
                                 for (size_t i = 0; i < n; ++i) {
                                     if (sentBlock[i] != reported[i]) return i;
                                 }
                                 return n;
                             }());
        }
    };

    verifyOne(/*isInput=*/true,  playback48kRawFormatBlock_, "iPCR");
    verifyOne(/*isInput=*/false, capture48kRawFormatBlock_,  "oPCR");

    // Read-only diagnostic: snapshot source / sync / clock state BEFORE we
    // touch the DAC source selector.
    LogFullDeviceState("pre-source-apply");

    // Vendor: select FireWire as the DAC input source. Working old-laptop
    // control panel XML pins <src>1</src>; without it the DAC routes analog
    // inputs and ignores incoming isochronous audio. Apr 25 passive sniff
    // proved our slot encoding matches Apple, so this is the remaining
    // device-side switch needed to make speakers play.
    const IOReturn sourceKr =
        SendVendorDeviceCommand(kOrpheusCmdSource, kOrpheusSourceFireWire);
    if (sourceKr != kIOReturnSuccess) {
        ASFW_LOG_ERROR(Audio,
                       "BeBoBProtocol: Vendor source=FireWire (0xB1/0x01) failed kr=0x%x",
                       sourceKr);
        return sourceKr;
    }

    const IOReturn bulkSourceKr = ApplyVendorBulkSource(kOrpheusSourceFireWire);
    if (bulkSourceKr != kIOReturnSuccess) {
        ASFW_LOG_ERROR(Audio,
                       "BeBoBProtocol: Vendor bulk source=FireWire "
                       "(0xBF Unit::Src=0x01) failed kr=0x%x",
                       bulkSourceKr);
        return bulkSourceKr;
    }

    // Re-read after source writes to catch the exact persistent device state.
    LogFullDeviceState("post-source-apply");
    LogMixOutputStates("post-source-apply");

    // Read-only follow-up diagnostics. The 0xEF mixer block's 12 input strips
    // are analog 1-8, digital L/R, and DAW L/R per the Orpheus manual/control
    // panel model, so do not mutate slots 0/1 as if they were DAW playback.
    LogAnalogStates("post-source-apply");

    return kIOReturnSuccess;
}

// ---------------------------------------------------------------------------
// SendVendorDeviceCommand — 15-byte vendor-dependent CONTROL at audio subunit
// ---------------------------------------------------------------------------
// Wire layout (matches ASFW/Models/OrpheusModels.swift buildDeviceFrame):
//   [0]=ctype CONTROL  [1]=subunit 0x08  [2]=opcode 0x00 VendorDependent
//   [3..5]=Prism OUI 00 11 98  [6]=cmdByte  [7]=value  [8..14]=0xFF padding

IOReturn BeBoBProtocol::SendVendorDeviceCommand(uint8_t cmdByte, uint8_t value)
{
    auto* transport = transport_.load(std::memory_order_acquire);
    if (!transport) {
        return kIOReturnNotReady;
    }

    AVCCdb cdb;
    cdb.ctype = static_cast<uint8_t>(Protocols::AVC::AVCCommandType::kControl);
    cdb.subunit = kAVCSubunitAudio;
    cdb.opcode = kAVCOpcodeVendorDep;
    cdb.operands[0] = kPrismOUI0;
    cdb.operands[1] = kPrismOUI1;
    cdb.operands[2] = kPrismOUI2;
    cdb.operands[3] = cmdByte;
    cdb.operands[4] = value;
    for (size_t i = 5; i < kVendorDeviceCmdSize - 3; ++i) {
        cdb.operands[i] = 0xFF;
    }
    cdb.operandLength = kVendorDeviceCmdSize - 3;  // 12 operand bytes after [ctype][subunit][opcode]

    struct CommandState {
        std::atomic<bool> done{false};
        std::atomic<IOReturn> status{kIOReturnTimeout};
    };
    auto state = std::make_shared<CommandState>();

    auto cmd = std::make_shared<AVCCommand>(*transport, cdb);
    cmd->Submit([state](AVCResult result, const AVCCdb&) {
        state->status.store(MapAVCResultToIOReturn(result), std::memory_order_release);
        state->done.store(true, std::memory_order_release);
    });

    for (uint32_t waited = 0; waited < kVendorCommandTimeoutMs; waited += kVendorCommandPollMs) {
        if (state->done.load(std::memory_order_acquire)) break;
        IOSleep(kVendorCommandPollMs);
    }

    const IOReturn kr = state->status.load(std::memory_order_acquire);
    ASFW_LOG(Audio,
             "BeBoBProtocol: Vendor cmd 0x%02x value=0x%02x kr=0x%x",
             cmdByte, value, kr);
    return kr;
}

// ---------------------------------------------------------------------------
// ApplyVendorBulkSource — Prism 0xBF bulk CONTROL preserving raw device state
// ---------------------------------------------------------------------------
// analysis evidence:
//   Orpheus::Device::Set(int,int) case 13 (0x10001d9c4) writes opcode 0xBF.
//   Orpheus::Device::Load(NSString*) calls Set(13, 0) after parsing XML,
//   including <src>. That makes 0xBF the control panel's "apply unit state"
//   path. We preserve the current raw 0xBF payload and change only Unit::Src.

IOReturn BeBoBProtocol::ApplyVendorBulkSource(uint8_t source)
{
    auto* transport = transport_.load(std::memory_order_acquire);
    if (!transport) {
        return kIOReturnNotReady;
    }

    OrpheusDeviceState state;
    const IOReturn readKr = QueryVendorBulkState(state);
    if (readKr != kIOReturnSuccess) {
        ASFW_LOG_WARNING(Audio,
                         "BeBoBProtocol: 0xBF bulk source apply could not read "
                         "current state kr=0x%x",
                         readKr);
        return readKr;
    }

    std::array<uint8_t, 12> operands = state.bulkRaw;
    if (operands[0] != kPrismOUI0 || operands[1] != kPrismOUI1 ||
        operands[2] != kPrismOUI2 || operands[3] != kOrpheusCmdBulkState) {
        char rawBuf[64];
        FormatOperandsHex(operands, rawBuf, sizeof(rawBuf));
        ASFW_LOG_ERROR(Audio,
                       "BeBoBProtocol: 0xBF bulk source apply saw unexpected "
                       "bulk header raw=[%{public}s]",
                       rawBuf);
        return kIOReturnError;
    }

    const uint8_t previousSource = static_cast<uint8_t>(operands[9] & 0x0F);
    operands[9] = static_cast<uint8_t>((operands[9] & 0xF0) | (source & 0x0F));

    AVCCdb cdb;
    cdb.ctype = static_cast<uint8_t>(Protocols::AVC::AVCCommandType::kControl);
    cdb.subunit = kAVCSubunitAudio;
    cdb.opcode = kAVCOpcodeVendorDep;
    std::copy_n(operands.begin(), operands.size(), cdb.operands.begin());
    cdb.operandLength = operands.size();

    struct CommandState {
        std::atomic<bool> done{false};
        std::atomic<IOReturn> status{kIOReturnTimeout};
    };
    auto commandState = std::make_shared<CommandState>();

    auto cmd = std::make_shared<AVCCommand>(*transport, cdb);
    cmd->Submit([commandState](AVCResult result, const AVCCdb&) {
        commandState->status.store(MapAVCResultToIOReturn(result), std::memory_order_release);
        commandState->done.store(true, std::memory_order_release);
    });

    for (uint32_t waited = 0; waited < kVendorCommandTimeoutMs; waited += kVendorCommandPollMs) {
        if (commandState->done.load(std::memory_order_acquire)) {
            break;
        }
        IOSleep(kVendorCommandPollMs);
    }

    const IOReturn kr = commandState->status.load(std::memory_order_acquire);
    char rawBuf[64];
    FormatOperandsHex(operands, rawBuf, sizeof(rawBuf));
    ASFW_LOG(Audio,
             "BeBoBProtocol: Vendor bulk source apply 0xBF source 0x%x->0x%x "
             "(%{public}s) raw=[%{public}s] kr=0x%x",
             previousSource,
             source,
             DecodeOutputSource(source),
             rawBuf,
             kr);
    return kr;
}

// ---------------------------------------------------------------------------
// Prism mix CONTROL helpers — Orpheus::Device::SetMix opcodes
// ---------------------------------------------------------------------------
// Observed behavior:
//   SetMix(output,param,value) builds [00 08 00 00 11 98 cmd output value...]
//   SetMix(output,input,param,value) builds
//   [00 08 00 00 11 98 cmd output input value...].
// The control panel accepts ctype 0x09 for these 15-byte commands. Padding is
// zeroed here because the vector-backed output SetMix path zero-fills it.

IOReturn BeBoBProtocol::SendVendorMixOutputByteCommand(uint8_t cmdByte,
                                                       uint8_t outputIndex,
                                                       uint8_t value)
{
    if (outputIndex >= kOrpheusMixOutputCount) {
        return kIOReturnBadArgument;
    }
    auto* transport = transport_.load(std::memory_order_acquire);
    if (transport == nullptr) {
        return kIOReturnNotReady;
    }

    AVCCdb cdb;
    cdb.ctype = static_cast<uint8_t>(Protocols::AVC::AVCCommandType::kControl);
    cdb.subunit = kAVCSubunitAudio;
    cdb.opcode = kAVCOpcodeVendorDep;
    cdb.operands[0] = kPrismOUI0;
    cdb.operands[1] = kPrismOUI1;
    cdb.operands[2] = kPrismOUI2;
    cdb.operands[3] = cmdByte;
    cdb.operands[4] = outputIndex;
    cdb.operands[5] = value;
    cdb.operandLength = kVendorDeviceCmdSize - 3;

    AVCCdb response;
    const IOReturn kr = SubmitBlockingAVCCommand(*transport, cdb, &response);
    ASFW_LOG(Audio,
             "BeBoBProtocol: Vendor mix output byte cmd=0x%02x output=%u value=%u "
             "kr=0x%x respCtype=0x%02x",
             cmdByte,
             static_cast<unsigned>(outputIndex),
             static_cast<unsigned>(value),
             kr,
             response.ctype);
    return kr;
}

IOReturn BeBoBProtocol::SendVendorMixOutputGainCommand(uint8_t outputIndex, int16_t gain)
{
    if (outputIndex >= kOrpheusMixOutputCount) {
        return kIOReturnBadArgument;
    }
    auto* transport = transport_.load(std::memory_order_acquire);
    if (transport == nullptr) {
        return kIOReturnNotReady;
    }

    const uint16_t rawGain = static_cast<uint16_t>(gain);

    AVCCdb cdb;
    cdb.ctype = static_cast<uint8_t>(Protocols::AVC::AVCCommandType::kControl);
    cdb.subunit = kAVCSubunitAudio;
    cdb.opcode = kAVCOpcodeVendorDep;
    cdb.operands[0] = kPrismOUI0;
    cdb.operands[1] = kPrismOUI1;
    cdb.operands[2] = kPrismOUI2;
    cdb.operands[3] = kOrpheusCmdMixOutGain;
    cdb.operands[4] = outputIndex;
    cdb.operands[5] = static_cast<uint8_t>((rawGain >> 8) & 0xFF);
    cdb.operands[6] = static_cast<uint8_t>(rawGain & 0xFF);
    cdb.operandLength = kVendorDeviceCmdSize - 3;

    AVCCdb response;
    const IOReturn kr = SubmitBlockingAVCCommand(*transport, cdb, &response);
    ASFW_LOG(Audio,
             "BeBoBProtocol: Vendor mix output gain output=%u gain=%d kr=0x%x "
             "respCtype=0x%02x",
             static_cast<unsigned>(outputIndex),
             static_cast<int>(gain),
             kr,
             response.ctype);
    return kr;
}

IOReturn BeBoBProtocol::SendVendorMixInputByteCommand(uint8_t cmdByte,
                                                      uint8_t outputIndex,
                                                      uint8_t inputIndex,
                                                      uint8_t value)
{
    if (outputIndex >= kOrpheusMixOutputCount || inputIndex >= kOrpheusMixInputCount) {
        return kIOReturnBadArgument;
    }
    auto* transport = transport_.load(std::memory_order_acquire);
    if (transport == nullptr) {
        return kIOReturnNotReady;
    }

    AVCCdb cdb;
    cdb.ctype = static_cast<uint8_t>(Protocols::AVC::AVCCommandType::kControl);
    cdb.subunit = kAVCSubunitAudio;
    cdb.opcode = kAVCOpcodeVendorDep;
    cdb.operands[0] = kPrismOUI0;
    cdb.operands[1] = kPrismOUI1;
    cdb.operands[2] = kPrismOUI2;
    cdb.operands[3] = cmdByte;
    cdb.operands[4] = outputIndex;
    cdb.operands[5] = inputIndex;
    cdb.operands[6] = value;
    cdb.operandLength = kVendorDeviceCmdSize - 3;

    AVCCdb response;
    const IOReturn kr = SubmitBlockingAVCCommand(*transport, cdb, &response);
    ASFW_LOG(Audio,
             "BeBoBProtocol: Vendor mix input byte cmd=0x%02x output=%u input=%u "
             "value=%u kr=0x%x respCtype=0x%02x",
             cmdByte,
             static_cast<unsigned>(outputIndex),
             static_cast<unsigned>(inputIndex),
             static_cast<unsigned>(value),
             kr,
             response.ctype);
    return kr;
}

IOReturn BeBoBProtocol::SendVendorMixInputGainCommand(uint8_t outputIndex,
                                                      uint8_t inputIndex,
                                                      int16_t gain)
{
    if (outputIndex >= kOrpheusMixOutputCount || inputIndex >= kOrpheusMixInputCount) {
        return kIOReturnBadArgument;
    }
    auto* transport = transport_.load(std::memory_order_acquire);
    if (transport == nullptr) {
        return kIOReturnNotReady;
    }

    const uint16_t rawGain = static_cast<uint16_t>(gain);

    AVCCdb cdb;
    cdb.ctype = static_cast<uint8_t>(Protocols::AVC::AVCCommandType::kControl);
    cdb.subunit = kAVCSubunitAudio;
    cdb.opcode = kAVCOpcodeVendorDep;
    cdb.operands[0] = kPrismOUI0;
    cdb.operands[1] = kPrismOUI1;
    cdb.operands[2] = kPrismOUI2;
    cdb.operands[3] = kOrpheusCmdMixInGain;
    cdb.operands[4] = outputIndex;
    cdb.operands[5] = inputIndex;
    cdb.operands[6] = static_cast<uint8_t>((rawGain >> 8) & 0xFF);
    cdb.operands[7] = static_cast<uint8_t>(rawGain & 0xFF);
    cdb.operandLength = kVendorDeviceCmdSize - 3;

    AVCCdb response;
    const IOReturn kr = SubmitBlockingAVCCommand(*transport, cdb, &response);
    ASFW_LOG(Audio,
             "BeBoBProtocol: Vendor mix input gain output=%u input=%u gain=%d "
             "kr=0x%x respCtype=0x%02x",
             static_cast<unsigned>(outputIndex),
             static_cast<unsigned>(inputIndex),
             static_cast<int>(gain),
             kr,
             response.ctype);
    return kr;
}

IOReturn BeBoBProtocol::EnsureLine12DawPlaybackMix()
{
    OrpheusMixOutputState mix;
    IOReturn kr = QueryMixOutputState(kLine12MixOutput, mix);
    if (kr != kIOReturnSuccess) {
        ASFW_LOG_WARNING(Audio,
                         "BeBoBProtocol: Line 1/2 DAW mix read failed before apply kr=0x%x",
                         kr);
        return kr;
    }

    ASFW_LOG(Audio,
             "BeBoBProtocol: Line 1/2 DAW mix before apply gain=%d mute=%d "
             "defeated=%d in0(g=%d,m=%d,p=%d) in1(g=%d,m=%d,p=%d)",
             static_cast<int>(mix.outputGain),
             mix.muted,
             mix.defeated,
             static_cast<int>(mix.inputGain[kDawInputLeft]),
             mix.inputMuted[kDawInputLeft],
             static_cast<int>(mix.inputPanBalance[kDawInputLeft]),
             static_cast<int>(mix.inputGain[kDawInputRight]),
             mix.inputMuted[kDawInputRight],
             static_cast<int>(mix.inputPanBalance[kDawInputRight]));

    auto run = [](IOReturn stepKr, IOReturn& aggregate) {
        if (aggregate == kIOReturnSuccess && stepKr != kIOReturnSuccess) {
            aggregate = stepKr;
        }
    };

    IOReturn result = kIOReturnSuccess;
    if (mix.defeated) {
        run(SendVendorMixOutputByteCommand(kOrpheusCmdMixOutDefeat,
                                           kLine12MixOutput,
                                           0),
            result);
    }
    if (mix.muted) {
        run(SendVendorMixOutputByteCommand(kOrpheusCmdMixOutMute,
                                           kLine12MixOutput,
                                           0),
            result);
    }
    if (mix.outputGain != kUnityMixGain) {
        run(SendVendorMixOutputGainCommand(kLine12MixOutput, kUnityMixGain), result);
    }

    if (mix.inputGain[kDawInputLeft] != kUnityMixGain) {
        run(SendVendorMixInputGainCommand(kLine12MixOutput,
                                          kDawInputLeft,
                                          kUnityMixGain),
            result);
    }
    if (mix.inputMuted[kDawInputLeft]) {
        run(SendVendorMixInputByteCommand(kOrpheusCmdMixInMute,
                                          kLine12MixOutput,
                                          kDawInputLeft,
                                          0),
            result);
    }
    if (mix.inputPanBalance[kDawInputLeft] != kFullLeftPan || mix.inputUsesBalance[kDawInputLeft]) {
        run(SendVendorMixInputByteCommand(kOrpheusCmdMixInPan,
                                          kLine12MixOutput,
                                          kDawInputLeft,
                                          static_cast<uint8_t>(kFullLeftPan)),
            result);
    }

    if (mix.inputGain[kDawInputRight] != kUnityMixGain) {
        run(SendVendorMixInputGainCommand(kLine12MixOutput,
                                          kDawInputRight,
                                          kUnityMixGain),
            result);
    }
    if (mix.inputMuted[kDawInputRight]) {
        run(SendVendorMixInputByteCommand(kOrpheusCmdMixInMute,
                                          kLine12MixOutput,
                                          kDawInputRight,
                                          0),
            result);
    }
    if (mix.inputPanBalance[kDawInputRight] != kFullRightPan ||
        mix.inputUsesBalance[kDawInputRight]) {
        run(SendVendorMixInputByteCommand(kOrpheusCmdMixInPan,
                                          kLine12MixOutput,
                                          kDawInputRight,
                                          static_cast<uint8_t>(kFullRightPan)),
            result);
    }

    if (result != kIOReturnSuccess) {
        return result;
    }

    OrpheusMixOutputState after;
    kr = QueryMixOutputState(kLine12MixOutput, after);
    if (kr != kIOReturnSuccess) {
        return kr;
    }

    ASFW_LOG(Audio,
             "BeBoBProtocol: Line 1/2 DAW mix after apply gain=%d mute=%d "
             "defeated=%d in0(g=%d,m=%d,p=%d) in1(g=%d,m=%d,p=%d)",
             static_cast<int>(after.outputGain),
             after.muted,
             after.defeated,
             static_cast<int>(after.inputGain[kDawInputLeft]),
             after.inputMuted[kDawInputLeft],
             static_cast<int>(after.inputPanBalance[kDawInputLeft]),
             static_cast<int>(after.inputGain[kDawInputRight]),
             after.inputMuted[kDawInputRight],
             static_cast<int>(after.inputPanBalance[kDawInputRight]));

    const bool line12Open =
        !after.defeated &&
        !after.muted &&
        after.outputGain == kUnityMixGain &&
        after.inputGain[kDawInputLeft] == kUnityMixGain &&
        after.inputGain[kDawInputRight] == kUnityMixGain &&
        !after.inputMuted[kDawInputLeft] &&
        !after.inputMuted[kDawInputRight] &&
        !after.inputUsesBalance[kDawInputLeft] &&
        !after.inputUsesBalance[kDawInputRight] &&
        after.inputPanBalance[kDawInputLeft] == kFullLeftPan &&
        after.inputPanBalance[kDawInputRight] == kFullRightPan;

    return line12Open ? kIOReturnSuccess : kIOReturnError;
}

// ---------------------------------------------------------------------------
// QueryVendorDeviceStatus — 15-byte vendor-dependent STATUS at audio subunit
// ---------------------------------------------------------------------------
// Wire layout (mirrors SendVendorDeviceCommand with ctype=STATUS):
//   [0]=ctype STATUS  [1]=subunit 0x08  [2]=opcode 0x00 VendorDependent
//   [3..5]=Prism OUI 00 11 98  [6]=cmdByte  [7]=param  [8..14]=0xFF padding
// Response operands include device state bytes; layout is per-cmd. Caller
// receives the raw 12-byte response operand buffer for decoding.

IOReturn BeBoBProtocol::QueryVendorDeviceStatus(uint8_t cmdByte,
                                                  uint8_t param,
                                                  std::array<uint8_t, 12>& outOperands)
{
    outOperands.fill(0);

    auto* transport = transport_.load(std::memory_order_acquire);
    if (transport == nullptr) {
        return kIOReturnNotReady;
    }

    AVCCdb cdb;
    cdb.ctype = static_cast<uint8_t>(Protocols::AVC::AVCCommandType::kStatus);
    cdb.subunit = kAVCSubunitAudio;
    cdb.opcode = kAVCOpcodeVendorDep;
    cdb.operands[0] = kPrismOUI0;
    cdb.operands[1] = kPrismOUI1;
    cdb.operands[2] = kPrismOUI2;
    cdb.operands[3] = cmdByte;
    cdb.operands[4] = param;
    for (size_t i = 5; i < kVendorDeviceCmdSize - 3; ++i) {
        cdb.operands[i] = 0xFF;
    }
    cdb.operandLength = kVendorDeviceCmdSize - 3;

    struct CommandState {
        std::atomic<bool> done{false};
        std::atomic<IOReturn> status{kIOReturnTimeout};
        AVCCdb response{};
    };
    auto state = std::make_shared<CommandState>();

    auto cmd = std::make_shared<AVCCommand>(*transport, cdb);
    cmd->Submit([state](AVCResult result, const AVCCdb& response) {
        state->response = response;
        state->status.store(MapAVCResultToIOReturn(result), std::memory_order_release);
        state->done.store(true, std::memory_order_release);
    });

    for (uint32_t waited = 0; waited < kVendorCommandTimeoutMs; waited += kVendorCommandPollMs) {
        if (state->done.load(std::memory_order_acquire)) {
            break;
        }
        IOSleep(kVendorCommandPollMs);
    }

    const IOReturn kr = state->status.load(std::memory_order_acquire);
    if (kr == kIOReturnSuccess) {
        const size_t copyLen = std::min<size_t>(state->response.operandLength,
                                                  outOperands.size());
        std::copy_n(state->response.operands.begin(), copyLen, outOperands.begin());
    }
    return kr;
}

// ---------------------------------------------------------------------------
// QueryVendorBulkState — vendor 0xBF STATUS bulk state read (panel Get(13))
// ---------------------------------------------------------------------------
// Mirrors the Prism Control Panel's `Orpheus::Device::Get(13)`.
// Wire layout — exact panel byte sequence:
//   [0]=0x01 STATUS  [1]=0x08 music subunit  [2]=0x00 VENDOR-DEPENDENT
//   [3..5]=Prism OUI 00 11 98               [6]=0xBF bulk-state opcode
//   [7..14]=0x00 (panel sends uninitialized stack; AV/C ignores unread bytes)
// Response layout (decoded at panel's response-handler 0x1b27c):
//   resp[0]   = 0x0C (IMPLEMENTED/STABLE)
//   resp[7..8]   s16 BE  → MasterVol value
//   resp[9..10]  s16 BE  → MasterVol enabled flags
//   resp[11] bit 0  = master mute (panel struct +0x1b6)
//   resp[11] bit 1  = master lock (panel struct +0x1b7)
//   resp[11] bit 2  = unknown bit (panel struct +0x1b0)
//   resp[11] bits 4-7 = meters value (right-shift by 4 → SetValue)
//   resp[12] low nib  = output source selector (panel Unit::Src)
//   resp[12] high nib = wordclock value (right-shift by 4 → SetValue)
//   resp[13]          = ADAT mode
//   resp[14] low nib  = headphone mix (low nibble per panel)
//   resp[14] high nib = meters brightness (right-shift by 4)
// AVCCdb operands index = frame index − 3, so resp[7] = operands[4], etc.

IOReturn BeBoBProtocol::QueryVendorBulkState(OrpheusDeviceState& out)
{
    auto* transport = transport_.load(std::memory_order_acquire);
    if (transport == nullptr) {
        return kIOReturnNotReady;
    }

    AVCCdb cdb;
    cdb.ctype = static_cast<uint8_t>(Protocols::AVC::AVCCommandType::kStatus);
    cdb.subunit = kAVCSubunitAudio;
    cdb.opcode = kAVCOpcodeVendorDep;
    cdb.operands[0] = kPrismOUI0;
    cdb.operands[1] = kPrismOUI1;
    cdb.operands[2] = kPrismOUI2;
    cdb.operands[3] = kOrpheusCmdBulkState;
    // Panel uses zero-init stack memory for cmd[7..14]; mirror with 0x00.
    for (size_t i = 4; i < kVendorDeviceCmdSize - 3; ++i) {
        cdb.operands[i] = 0x00;
    }
    cdb.operandLength = kVendorDeviceCmdSize - 3;

    struct CommandState {
        std::atomic<bool> done{false};
        std::atomic<IOReturn> status{kIOReturnTimeout};
        AVCCdb response{};
    };
    auto state = std::make_shared<CommandState>();

    auto cmd = std::make_shared<AVCCommand>(*transport, cdb);
    cmd->Submit([state](AVCResult result, const AVCCdb& response) {
        state->response = response;
        state->status.store(MapAVCResultToIOReturn(result), std::memory_order_release);
        state->done.store(true, std::memory_order_release);
    });

    for (uint32_t waited = 0; waited < kVendorCommandTimeoutMs; waited += kVendorCommandPollMs) {
        if (state->done.load(std::memory_order_acquire)) break;
        IOSleep(kVendorCommandPollMs);
    }

    const IOReturn kr = state->status.load(std::memory_order_acquire);
    if (kr != kIOReturnSuccess) {
        return kr;
    }

    // Capture raw operand buffer for diagnostic logging.
    const size_t copyLen = std::min<size_t>(state->response.operandLength,
                                             out.bulkRaw.size());
    std::copy_n(state->response.operands.begin(), copyLen, out.bulkRaw.begin());

    // Decode per panel's response handler. Note frame[N] = operands[N-3]:
    //   frame[7]=op[4]  frame[8]=op[5]  frame[9]=op[6]  ...
    if (state->response.operandLength < 12) {
        // Defensive: panel expects ≥12 operand bytes (15-byte total frame).
        return kIOReturnUnderrun;
    }
    const auto& op = state->response.operands;

    out.masterVolValue   = static_cast<int16_t>((op[4] << 8) | op[5]);
    out.masterVolEnabled = static_cast<int16_t>((op[6] << 8) | op[7]);

    const uint8_t pack11 = op[8];   // frame[11]
    out.masterMute       = (pack11 & 0x01) != 0;
    out.masterLock       = (pack11 & 0x02) != 0;
    out.unknownBit11_2   = (pack11 & 0x04) != 0;
    out.metersValue      = static_cast<uint8_t>(pack11 >> 4);

    const uint8_t pack12 = op[9];   // frame[12]
    out.outputSource     = static_cast<uint8_t>(pack12 & 0x0F);
    out.wordclockValue   = static_cast<uint8_t>(pack12 >> 4);

    out.adatModeBulk     = op[10];  // frame[13]

    const uint8_t pack14 = op[11];  // frame[14]
    out.headphoneMix     = static_cast<uint8_t>(pack14 & 0x0F);
    out.metersBrightness = static_cast<uint8_t>(pack14 >> 4);

    out.bulkReadValid = true;
    return kIOReturnSuccess;
}

// ---------------------------------------------------------------------------
// QuerySignalSourceSync — AV/C SignalSource STATUS at UNIT (panel Sync())
// ---------------------------------------------------------------------------
// Mirrors the Prism Control Panel's `Orpheus::Device::Sync()`.
// 8-byte AV/C General SignalSource STATUS, addressing the
// device's sync-input external plug.
//   [0]=0x01 STATUS  [1]=0xFF UNIT  [2]=0x1A SIGNAL_SOURCE
//   [3]=0x0F reserved   [4..5]=0xFF 0xFF source plug query (wildcard)
//   [6]=0x60 dest plug type (external)
//   [7]=0x07 (no ADAT input) or 0x08 (ADAT input present)
// Response:
//   resp[0] = 0x0C (IMPLEMENTED/STABLE)
//   resp[4] = source plug type byte (0xFF or 0x60)
//   resp[5] = source plug ID byte → decode via DecodeSignalSourceResponse

IOReturn BeBoBProtocol::QuerySignalSourceSync(bool useAdatPlug,
                                                OrpheusDeviceState& out)
{
    auto* transport = transport_.load(std::memory_order_acquire);
    if (transport == nullptr) {
        return kIOReturnNotReady;
    }

    const uint8_t destPlugLow = useAdatPlug ? kSignalSourceSyncPlugAdat
                                            : kSignalSourceSyncPlugNoAdat;

    AVCCdb cdb;
    cdb.ctype = static_cast<uint8_t>(Protocols::AVC::AVCCommandType::kStatus);
    cdb.subunit = Protocols::AVC::kAVCSubunitUnit;  // 0xFF
    cdb.opcode = kAVCOpcodeSignalSource;
    cdb.operands[0] = kSignalSourceReserved0F;  // frame[3]
    cdb.operands[1] = kSignalSourceQueryHi;     // frame[4]
    cdb.operands[2] = kSignalSourceQueryLo;     // frame[5]
    cdb.operands[3] = kSignalSourceDestPlugHi;  // frame[6]
    cdb.operands[4] = destPlugLow;              // frame[7]
    cdb.operandLength = kSignalSourceCommandSize - 3;  // 5 operand bytes

    struct CommandState {
        std::atomic<bool> done{false};
        std::atomic<IOReturn> status{kIOReturnTimeout};
        AVCCdb response{};
    };
    auto state = std::make_shared<CommandState>();

    auto cmd = std::make_shared<AVCCommand>(*transport, cdb);
    cmd->Submit([state](AVCResult result, const AVCCdb& response) {
        state->response = response;
        state->status.store(MapAVCResultToIOReturn(result), std::memory_order_release);
        state->done.store(true, std::memory_order_release);
    });

    for (uint32_t waited = 0; waited < kVendorCommandTimeoutMs; waited += kVendorCommandPollMs) {
        if (state->done.load(std::memory_order_acquire)) break;
        IOSleep(kVendorCommandPollMs);
    }

    const IOReturn kr = state->status.load(std::memory_order_acquire);
    if (kr != kIOReturnSuccess) {
        return kr;
    }

    if (state->response.operandLength < 5) {
        return kIOReturnUnderrun;
    }

    // resp[4] = operands[1], resp[5] = operands[2]
    out.signalSourceRespByte4 = state->response.operands[1];
    out.signalSourceRespByte5 = state->response.operands[2];
    out.signalSourceDestPlug = destPlugLow;
    out.signalSourceUsedAdatPlug = useAdatPlug;
    out.syncSource_avc = DecodeSignalSourceResponse(out.signalSourceRespByte4,
                                                      out.signalSourceRespByte5,
                                                      useAdatPlug);
    out.signalSourceValid = true;
    return kIOReturnSuccess;
}

// ---------------------------------------------------------------------------
// QueryAdatStatus — vendor 0xB3 STATUS (panel Get(8))
// ---------------------------------------------------------------------------
// Mirrors panel's `Orpheus::Device::Get(8)` AVC path. Captures the full
// ADAT-section state. Operand layout TBD — saved raw for now.

IOReturn BeBoBProtocol::QueryAdatStatus(OrpheusDeviceState& out)
{
    auto* transport = transport_.load(std::memory_order_acquire);
    if (transport == nullptr) {
        return kIOReturnNotReady;
    }

    AVCCdb cdb;
    cdb.ctype = static_cast<uint8_t>(Protocols::AVC::AVCCommandType::kStatus);
    cdb.subunit = kAVCSubunitAudio;
    cdb.opcode = kAVCOpcodeVendorDep;
    cdb.operands[0] = kPrismOUI0;
    cdb.operands[1] = kPrismOUI1;
    cdb.operands[2] = kPrismOUI2;
    cdb.operands[3] = kOrpheusCmdADAT;
    for (size_t i = 4; i < kVendorDeviceCmdSize - 3; ++i) {
        cdb.operands[i] = 0x00;
    }
    cdb.operandLength = kVendorDeviceCmdSize - 3;

    struct CommandState {
        std::atomic<bool> done{false};
        std::atomic<IOReturn> status{kIOReturnTimeout};
        AVCCdb response{};
    };
    auto state = std::make_shared<CommandState>();

    auto cmd = std::make_shared<AVCCommand>(*transport, cdb);
    cmd->Submit([state](AVCResult result, const AVCCdb& response) {
        state->response = response;
        state->status.store(MapAVCResultToIOReturn(result), std::memory_order_release);
        state->done.store(true, std::memory_order_release);
    });

    for (uint32_t waited = 0; waited < kVendorCommandTimeoutMs; waited += kVendorCommandPollMs) {
        if (state->done.load(std::memory_order_acquire)) break;
        IOSleep(kVendorCommandPollMs);
    }

    const IOReturn kr = state->status.load(std::memory_order_acquire);
    if (kr != kIOReturnSuccess) {
        return kr;
    }

    const size_t copyLen = std::min<size_t>(state->response.operandLength,
                                             out.adatRaw.size());
    std::copy_n(state->response.operands.begin(), copyLen, out.adatRaw.begin());
    out.adatReadValid = true;
    return kIOReturnSuccess;
}

// ---------------------------------------------------------------------------
// QueryUnknownB6 — vendor 0xB6 STATUS (panel Get(10))
// ---------------------------------------------------------------------------
// Single-quadlet read whose response panel stores at device+0x134. Purpose
// undocumented — captured for cataloging.

IOReturn BeBoBProtocol::QueryUnknownB6(OrpheusDeviceState& out)
{
    auto* transport = transport_.load(std::memory_order_acquire);
    if (transport == nullptr) {
        return kIOReturnNotReady;
    }

    AVCCdb cdb;
    cdb.ctype = static_cast<uint8_t>(Protocols::AVC::AVCCommandType::kStatus);
    cdb.subunit = kAVCSubunitAudio;
    cdb.opcode = kAVCOpcodeVendorDep;
    cdb.operands[0] = kPrismOUI0;
    cdb.operands[1] = kPrismOUI1;
    cdb.operands[2] = kPrismOUI2;
    cdb.operands[3] = kOrpheusCmdUnknownB6;
    for (size_t i = 4; i < kVendorDeviceCmdSize - 3; ++i) {
        cdb.operands[i] = 0x00;
    }
    cdb.operandLength = kVendorDeviceCmdSize - 3;

    struct CommandState {
        std::atomic<bool> done{false};
        std::atomic<IOReturn> status{kIOReturnTimeout};
        AVCCdb response{};
    };
    auto state = std::make_shared<CommandState>();

    auto cmd = std::make_shared<AVCCommand>(*transport, cdb);
    cmd->Submit([state](AVCResult result, const AVCCdb& response) {
        state->response = response;
        state->status.store(MapAVCResultToIOReturn(result), std::memory_order_release);
        state->done.store(true, std::memory_order_release);
    });

    for (uint32_t waited = 0; waited < kVendorCommandTimeoutMs; waited += kVendorCommandPollMs) {
        if (state->done.load(std::memory_order_acquire)) break;
        IOSleep(kVendorCommandPollMs);
    }

    const IOReturn kr = state->status.load(std::memory_order_acquire);
    if (kr != kIOReturnSuccess) {
        return kr;
    }

    const size_t copyLen = std::min<size_t>(state->response.operandLength,
                                             out.unknownB6Raw.size());
    std::copy_n(state->response.operands.begin(), copyLen, out.unknownB6Raw.begin());
    out.unknownB6Valid = true;
    return kIOReturnSuccess;
}

// ---------------------------------------------------------------------------
// QueryMixOutputState — vendor 0xEF STATUS, full output mixer block
// ---------------------------------------------------------------------------
// Mirrors `Orpheus::Device::GetMix(output, 8)` at 0x10001bec8. The Prism
// panel sends a 74-byte STATUS frame:
//   [01][08][00][00 11 98][EF][output][zero padding...]
// Response frame:
//   [7]=output [8..9]=output gain [10]=mute [11..12]=solo [13]=defeated
//   [14..73]=12 inputs, five bytes each:
//      gain_hi, gain_lo, mute, 0=pan/1=balance, pan_or_balance

IOReturn BeBoBProtocol::QueryMixOutputState(uint8_t outputIndex, OrpheusMixOutputState& out)
{
    if (outputIndex >= kOrpheusMixOutputCount) {
        return kIOReturnBadArgument;
    }

    auto* transport = transport_.load(std::memory_order_acquire);
    if (transport == nullptr) {
        return kIOReturnNotReady;
    }

    AVCCdb cdb;
    cdb.ctype = static_cast<uint8_t>(Protocols::AVC::AVCCommandType::kStatus);
    cdb.subunit = kAVCSubunitAudio;
    cdb.opcode = kAVCOpcodeVendorDep;
    cdb.operands[0] = kPrismOUI0;
    cdb.operands[1] = kPrismOUI1;
    cdb.operands[2] = kPrismOUI2;
    cdb.operands[3] = kOrpheusCmdMixBulk;
    cdb.operands[4] = outputIndex;
    for (size_t i = 5; i < kOrpheusMixBulkOperandLength; ++i) {
        cdb.operands[i] = 0x00;
    }
    cdb.operandLength = kOrpheusMixBulkOperandLength;

    struct CommandState {
        std::atomic<bool> done{false};
        std::atomic<IOReturn> status{kIOReturnTimeout};
        AVCCdb response{};
    };
    auto state = std::make_shared<CommandState>();

    auto cmd = std::make_shared<AVCCommand>(*transport, cdb);
    cmd->Submit([state](AVCResult result, const AVCCdb& response) {
        state->response = response;
        state->status.store(MapAVCResultToIOReturn(result), std::memory_order_release);
        state->done.store(true, std::memory_order_release);
    });

    for (uint32_t waited = 0; waited < kVendorCommandTimeoutMs; waited += kVendorCommandPollMs) {
        if (state->done.load(std::memory_order_acquire)) {
            break;
        }
        IOSleep(kVendorCommandPollMs);
    }

    const IOReturn kr = state->status.load(std::memory_order_acquire);
    if (kr != kIOReturnSuccess) {
        return kr;
    }
    if (state->response.operandLength < kOrpheusMixBulkOperandLength) {
        return kIOReturnUnderrun;
    }

    const size_t copyLen = std::min<size_t>(state->response.operandLength, out.raw.size());
    std::copy_n(state->response.operands.begin(), copyLen, out.raw.begin());

    const auto& op = state->response.operands;
    if (op[0] != kPrismOUI0 || op[1] != kPrismOUI1 ||
        op[2] != kPrismOUI2 || op[3] != kOrpheusCmdMixBulk ||
        op[4] != outputIndex) {
        return kIOReturnError;
    }

    out.outputIndex = outputIndex;
    out.outputGain = static_cast<int16_t>((op[5] << 8) | op[6]);
    out.muted = op[7] != 0;
    out.soloMask = static_cast<uint16_t>((op[8] << 8) | op[9]);
    out.defeated = op[10] != 0;

    for (size_t input = 0; input < kOrpheusMixInputCount; ++input) {
        const size_t base = 11 + input * 5;
        out.inputGain[input] = static_cast<int16_t>((op[base] << 8) | op[base + 1]);
        out.inputMuted[input] = op[base + 2] != 0;
        out.inputUsesBalance[input] = op[base + 3] != 0;
        out.inputPanBalance[input] = static_cast<int8_t>(op[base + 4]);
    }

    out.valid = true;
    return kIOReturnSuccess;
}

// ---------------------------------------------------------------------------
// QueryAnalogState — vendor 0xCF STATUS, analog channel block
// ---------------------------------------------------------------------------
// Mirrors `Orpheus::Device::GetAnalog(channel, 10)` at 0x10001ba1c. The Prism
// panel sends a 15-byte STATUS frame:
//   [01][08][00][00 11 98][CF][channel][zero padding...]
// Response frame bytes consumed by the panel:
//   [8]=type [9]=all flags [10]=filter [11]=gain1 [12]=gain2

IOReturn BeBoBProtocol::QueryAnalogState(uint8_t channelIndex, OrpheusAnalogState& out)
{
    if (channelIndex >= kOrpheusAnalogChannelCount) {
        return kIOReturnBadArgument;
    }

    auto* transport = transport_.load(std::memory_order_acquire);
    if (transport == nullptr) {
        return kIOReturnNotReady;
    }

    AVCCdb cdb;
    cdb.ctype = static_cast<uint8_t>(Protocols::AVC::AVCCommandType::kStatus);
    cdb.subunit = kAVCSubunitAudio;
    cdb.opcode = kAVCOpcodeVendorDep;
    cdb.operands[0] = kPrismOUI0;
    cdb.operands[1] = kPrismOUI1;
    cdb.operands[2] = kPrismOUI2;
    cdb.operands[3] = kOrpheusCmdAnalogBulk;
    cdb.operands[4] = channelIndex;
    cdb.operandLength = kVendorDeviceCmdSize - 3;

    AVCCdb response;
    const IOReturn kr = SubmitBlockingAVCCommand(*transport, cdb, &response);
    if (kr != kIOReturnSuccess) {
        return kr;
    }
    if (response.operandLength < (kVendorDeviceCmdSize - 3)) {
        return kIOReturnUnderrun;
    }

    std::copy_n(response.operands.begin(), out.raw.size(), out.raw.begin());

    const auto& op = response.operands;
    if (op[0] != kPrismOUI0 || op[1] != kPrismOUI1 ||
        op[2] != kPrismOUI2 || op[3] != kOrpheusCmdAnalogBulk ||
        op[4] != channelIndex) {
        return kIOReturnError;
    }

    out.channelIndex = channelIndex;
    out.type = op[5];
    out.allFlags = op[6];
    out.filter = op[7];
    out.gain1 = op[8];
    out.gain2 = op[9];
    out.inputGainFlag = (out.allFlags & 0x01) != 0;
    out.outputGainFlag = (out.allFlags & 0x02) != 0;
    out.bit2 = (out.allFlags & 0x04) != 0;
    out.bit3 = (out.allFlags & 0x08) != 0;
    out.bit5 = (out.allFlags & 0x20) != 0;
    out.bit6 = (out.allFlags & 0x40) != 0;
    out.valid = true;
    return kIOReturnSuccess;
}

// ---------------------------------------------------------------------------
// LogMixOutputStates — read-only Orpheus mixer/output diagnostics
// ---------------------------------------------------------------------------

void BeBoBProtocol::LogMixOutputStates(const char* phaseTag)
{
    auto* transport = transport_.load(std::memory_order_acquire);
    if (transport == nullptr) {
        ASFW_LOG_WARNING(Audio,
                         "BeBoBProtocol: LogMixOutputStates[%{public}s] no FCP transport, skipping",
                         phaseTag);
        return;
    }

    ASFW_LOG(Audio, "BeBoBProtocol: LogMixOutputStates[%{public}s] BEGIN", phaseTag);

    for (uint8_t output = 0; output < kOrpheusMixOutputCount; ++output) {
        OrpheusMixOutputState mix;
        const IOReturn kr = QueryMixOutputState(output, mix);
        if (kr != kIOReturnSuccess) {
            ASFW_LOG_WARNING(Audio,
                             "BeBoBProtocol: LogMixOutputStates[%{public}s] output=%u "
                             "0xEF read FAILED kr=0x%x",
                             phaseTag,
                             static_cast<unsigned>(output),
                             kr);
            continue;
        }

        char gains[128];
        char panBalance[96];
        FormatInt16List(mix.inputGain, gains, sizeof(gains));
        FormatInt8List(mix.inputPanBalance, panBalance, sizeof(panBalance));

        ASFW_LOG(Audio,
                 "BeBoBProtocol: LogMixOutputStates[%{public}s] output=%u "
                 "gain=%d mute=%d solo=0x%04x defeated=%d "
                 "inputMuteMask=0x%03x balanceMask=0x%03x",
                 phaseTag,
                 static_cast<unsigned>(mix.outputIndex),
                 static_cast<int>(mix.outputGain),
                 mix.muted,
                 mix.soloMask,
                 mix.defeated,
                 BoolArrayMask(mix.inputMuted),
                 BoolArrayMask(mix.inputUsesBalance));
        ASFW_LOG(Audio,
                 "BeBoBProtocol: LogMixOutputStates[%{public}s] output=%u "
                 "inputGain=[%{public}s] panBalance=[%{public}s]",
                 phaseTag,
                 static_cast<unsigned>(mix.outputIndex),
                 gains,
                 panBalance);
    }

    ASFW_LOG(Audio, "BeBoBProtocol: LogMixOutputStates[%{public}s] END", phaseTag);
}

// ---------------------------------------------------------------------------
// LogAnalogStates — read-only Orpheus analog diagnostics
// ---------------------------------------------------------------------------

void BeBoBProtocol::LogAnalogStates(const char* phaseTag)
{
    auto* transport = transport_.load(std::memory_order_acquire);
    if (transport == nullptr) {
        ASFW_LOG_WARNING(Audio,
                         "BeBoBProtocol: LogAnalogStates[%{public}s] no FCP transport, skipping",
                         phaseTag);
        return;
    }

    ASFW_LOG(Audio, "BeBoBProtocol: LogAnalogStates[%{public}s] BEGIN", phaseTag);

    for (uint8_t channel = 0; channel < kOrpheusAnalogChannelCount; ++channel) {
        OrpheusAnalogState analog;
        const IOReturn kr = QueryAnalogState(channel, analog);
        if (kr != kIOReturnSuccess) {
            ASFW_LOG_WARNING(Audio,
                             "BeBoBProtocol: LogAnalogStates[%{public}s] channel=%u "
                             "0xCF read FAILED kr=0x%x",
                             phaseTag,
                             static_cast<unsigned>(channel),
                             kr);
            continue;
        }

        char rawBuf[64];
        FormatOperandsHex(analog.raw, rawBuf, sizeof(rawBuf));
        ASFW_LOG(Audio,
                 "BeBoBProtocol: LogAnalogStates[%{public}s] channel=%u "
                 "type=%u all=0x%02x inGainFlag=%d outGainFlag=%d "
                 "bit2=%d bit3=%d bit5=%d bit6=%d filter=%u gain1=%u gain2=%u "
                 "raw=[%{public}s]",
                 phaseTag,
                 static_cast<unsigned>(analog.channelIndex),
                 static_cast<unsigned>(analog.type),
                 static_cast<unsigned>(analog.allFlags),
                 analog.inputGainFlag,
                 analog.outputGainFlag,
                 analog.bit2,
                 analog.bit3,
                 analog.bit5,
                 analog.bit6,
                 static_cast<unsigned>(analog.filter),
                 static_cast<unsigned>(analog.gain1),
                 static_cast<unsigned>(analog.gain2),
                 rawBuf);
    }

    ASFW_LOG(Audio, "BeBoBProtocol: LogAnalogStates[%{public}s] END", phaseTag);
}

// ---------------------------------------------------------------------------
// LogFullDeviceState — diagnostic snapshot via vendor and SignalSource reads
// ---------------------------------------------------------------------------
// Captures the device state via all four query paths: 0xBF bulk STATUS,
// AV/C SignalSource STATUS (with both plug variants), 0xB3 ADAT STATUS,
// 0xB6 STATUS. Logs every decoded field.
//
// SignalSource path: panel decides plug 7 vs 8 from `HasAdatInput()`. We don't
// have that bit until the bulk read populates `adatModeBulk`, so we issue
// SignalSource twice on the first call (once with plug 7, once with plug 8)
// and use whichever returns a sensible enum to populate `syncSource_avc`.

void BeBoBProtocol::LogFullDeviceState(const char* phaseTag)
{
    auto* transport = transport_.load(std::memory_order_acquire);
    if (transport == nullptr) {
        ASFW_LOG_WARNING(Audio,
                         "BeBoBProtocol: LogFullDeviceState[%{public}s] no FCP transport, skipping",
                         phaseTag);
        return;
    }

    ASFW_LOG(Audio, "BeBoBProtocol: LogFullDeviceState[%{public}s] BEGIN", phaseTag);

    OrpheusDeviceState st;
    char hexBuf[64];

    // --- Path A: vendor 0xBF bulk STATUS read --------------------------------
    const IOReturn bulkKr = QueryVendorBulkState(st);
    if (bulkKr != kIOReturnSuccess) {
        ASFW_LOG_WARNING(Audio,
                         "BeBoBProtocol: LogFullDeviceState[%{public}s] 0xBF bulk read FAILED kr=0x%x",
                         phaseTag, bulkKr);
    } else {
        FormatOperandsHex(st.bulkRaw, hexBuf, sizeof(hexBuf));
        ASFW_LOG(Audio,
                 "BeBoBProtocol: LogFullDeviceState[%{public}s] 0xBF raw=[%{public}s]",
                 phaseTag, hexBuf);
        ASFW_LOG(Audio,
                 "BeBoBProtocol: LogFullDeviceState[%{public}s] bulk: "
                 "masterVol=%d enabled=0x%04x mute=%d lock=%d bit11_2=%d "
                 "meters=0x%x source=0x%x (%{public}s) "
                 "wordclock=0x%x adat=0x%02x headphone=0x%x brightness=0x%x",
                 phaseTag,
                 static_cast<int>(st.masterVolValue),
                 static_cast<unsigned>(st.masterVolEnabled & 0xFFFF),
                 st.masterMute, st.masterLock, st.unknownBit11_2,
                 st.metersValue,
                 st.outputSource, DecodeOutputSource(st.outputSource),
                 st.wordclockValue,
                 st.adatModeBulk,
                 st.headphoneMix, st.metersBrightness);
    }

    // --- Path B: AV/C SignalSource STATUS at UNIT (both plug variants) -------
    // Try plug 7 first (no-ADAT case). If it succeeds and the decoded enum is
    // valid (≠ 0xFF), use it. Otherwise try plug 8 (ADAT-input case).
    OrpheusDeviceState ssNoAdat = st;  // copy bulk fields, will overwrite SS fields
    const IOReturn ssKr7 = QuerySignalSourceSync(/*useAdatPlug=*/false, ssNoAdat);
    if (ssKr7 != kIOReturnSuccess) {
        ASFW_LOG_WARNING(Audio,
                         "BeBoBProtocol: LogFullDeviceState[%{public}s] SignalSource plug=0x07 "
                         "FAILED kr=0x%x",
                         phaseTag, ssKr7);
    } else {
        ASFW_LOG(Audio,
                 "BeBoBProtocol: LogFullDeviceState[%{public}s] SignalSource plug=0x07: "
                 "resp[4]=0x%02x resp[5]=0x%02x → sync=0x%x (%{public}s)",
                 phaseTag,
                 ssNoAdat.signalSourceRespByte4, ssNoAdat.signalSourceRespByte5,
                 ssNoAdat.syncSource_avc,
                 DecodeSyncSource(ssNoAdat.syncSource_avc));
    }

    OrpheusDeviceState ssAdat = st;
    const IOReturn ssKr8 = QuerySignalSourceSync(/*useAdatPlug=*/true, ssAdat);
    if (ssKr8 != kIOReturnSuccess) {
        ASFW_LOG_WARNING(Audio,
                         "BeBoBProtocol: LogFullDeviceState[%{public}s] SignalSource plug=0x08 "
                         "FAILED kr=0x%x",
                         phaseTag, ssKr8);
    } else {
        ASFW_LOG(Audio,
                 "BeBoBProtocol: LogFullDeviceState[%{public}s] SignalSource plug=0x08: "
                 "resp[4]=0x%02x resp[5]=0x%02x → sync=0x%x (%{public}s)",
                 phaseTag,
                 ssAdat.signalSourceRespByte4, ssAdat.signalSourceRespByte5,
                 ssAdat.syncSource_avc,
                 DecodeSyncSource(ssAdat.syncSource_avc));
    }

    // Pick the first sensible SignalSource result. 0xBF Unit::Src is a routing
    // selector, not the sync-source enum, so it is not a cross-check key.
    if (ssKr7 == kIOReturnSuccess && ssNoAdat.syncSource_avc != 0xFF) {
        st.signalSourceValid = true;
        st.syncSource_avc = ssNoAdat.syncSource_avc;
        st.signalSourceDestPlug = ssNoAdat.signalSourceDestPlug;
        st.signalSourceRespByte4 = ssNoAdat.signalSourceRespByte4;
        st.signalSourceRespByte5 = ssNoAdat.signalSourceRespByte5;
        st.signalSourceUsedAdatPlug = false;
    } else if (ssKr8 == kIOReturnSuccess && ssAdat.syncSource_avc != 0xFF) {
        st.signalSourceValid = true;
        st.syncSource_avc = ssAdat.syncSource_avc;
        st.signalSourceDestPlug = ssAdat.signalSourceDestPlug;
        st.signalSourceRespByte4 = ssAdat.signalSourceRespByte4;
        st.signalSourceRespByte5 = ssAdat.signalSourceRespByte5;
        st.signalSourceUsedAdatPlug = true;
    } else if (ssKr7 == kIOReturnSuccess) {
        // Bulk read failed or no match — record plug-7 result as best effort.
        st.signalSourceValid = true;
        st.syncSource_avc = ssNoAdat.syncSource_avc;
        st.signalSourceDestPlug = ssNoAdat.signalSourceDestPlug;
        st.signalSourceRespByte4 = ssNoAdat.signalSourceRespByte4;
        st.signalSourceRespByte5 = ssNoAdat.signalSourceRespByte5;
        st.signalSourceUsedAdatPlug = false;
    } else if (ssKr8 == kIOReturnSuccess) {
        st.signalSourceValid = true;
        st.syncSource_avc = ssAdat.syncSource_avc;
        st.signalSourceDestPlug = ssAdat.signalSourceDestPlug;
        st.signalSourceRespByte4 = ssAdat.signalSourceRespByte4;
        st.signalSourceRespByte5 = ssAdat.signalSourceRespByte5;
        st.signalSourceUsedAdatPlug = true;
    }

    // --- Routing source + sync source: report distinct controls -------------
    if (st.bulkReadValid && st.signalSourceValid) {
        ASFW_LOG(Audio,
                 "BeBoBProtocol: LogFullDeviceState[%{public}s] routing/sync: "
                 "bulk(0xBF Unit::Src)=0x%x (%{public}s) "
                 "SignalSource(0x1A sync)=0x%x (%{public}s — %{public}s)",
                 phaseTag,
                 st.outputSource, DecodeOutputSource(st.outputSource),
                 st.syncSource_avc, DecodeSyncSource(st.syncSource_avc),
                 SyncSourceMuteRisk(st.syncSource_avc));
    }

    // --- 0xB3 ADAT STATUS ----------------------------------------------------
    const IOReturn adatKr = QueryAdatStatus(st);
    if (adatKr != kIOReturnSuccess) {
        ASFW_LOG_WARNING(Audio,
                         "BeBoBProtocol: LogFullDeviceState[%{public}s] 0xB3 ADAT STATUS "
                         "FAILED kr=0x%x",
                         phaseTag, adatKr);
    } else {
        FormatOperandsHex(st.adatRaw, hexBuf, sizeof(hexBuf));
        ASFW_LOG(Audio,
                 "BeBoBProtocol: LogFullDeviceState[%{public}s] 0xB3 adat raw=[%{public}s]",
                 phaseTag, hexBuf);
    }

    // --- 0xB6 unknown STATUS -------------------------------------------------
    const IOReturn b6Kr = QueryUnknownB6(st);
    if (b6Kr != kIOReturnSuccess) {
        ASFW_LOG_WARNING(Audio,
                         "BeBoBProtocol: LogFullDeviceState[%{public}s] 0xB6 STATUS "
                         "FAILED kr=0x%x",
                         phaseTag, b6Kr);
    } else {
        FormatOperandsHex(st.unknownB6Raw, hexBuf, sizeof(hexBuf));
        ASFW_LOG(Audio,
                 "BeBoBProtocol: LogFullDeviceState[%{public}s] 0xB6 raw=[%{public}s]",
                 phaseTag, hexBuf);
    }

    ASFW_LOG(Audio, "BeBoBProtocol: LogFullDeviceState[%{public}s] END", phaseTag);
}

// ---------------------------------------------------------------------------
// SendExtStreamFormatControl — 0xBF CONTROL at unit isoch plug
// ---------------------------------------------------------------------------

void BeBoBProtocol::SendExtStreamFormatControl(uint32_t sequence,
                                                bool isInput,
                                                uint8_t plugId,
                                                FormatCompletion completion)
{
    auto* transport = transport_.load(std::memory_order_acquire);
    if (!transport) {
        completion(kIOReturnNotReady);
        return;
    }

    // Orpheus channel layout:
    //   source plug (oPCR): 5 stereo MBLA pairs + 1 MIDI = DBS 11
    //   dest plug (iPCR):   6 stereo MBLA pairs + 1 MIDI = DBS 13
    const uint8_t audioPairs = isInput ? kOrpheusInputMBLAPairs : kOrpheusOutputMBLAPairs;
    const char* plugStr = isInput ? "dest" : "source";
    const char* dirStr = PlugDirectionString(isInput);
    const auto& rawFormatBlock =
        isInput ? playback48kRawFormatBlock_ : capture48kRawFormatBlock_;

    std::shared_ptr<AVCExtendedStreamFormatCommand> command;
    if (RawFormatBlockLooksUsable(rawFormatBlock)) {
        command = std::make_shared<AVCExtendedStreamFormatCommand>(
            *transport, isInput, plugId, rawFormatBlock);
    } else {
        command = std::make_shared<AVCExtendedStreamFormatCommand>(
            *transport, isInput, plugId, audioPairs, /*hasMidi=*/true);
    }

    ASFW_LOG(Audio,
             "BeBoBProtocol: Sending ExtStreamFormat CONTROL to unit isoch plug "
             "(%s plug %u, %s, source=%{public}s, payload=%zu bytes)",
             plugStr,
             plugId,
             dirStr,
             RawFormatBlockLooksUsable(rawFormatBlock) ? "discovered-raw" : "synthetic-fallback",
             RawFormatBlockLooksUsable(rawFormatBlock) ? rawFormatBlock.size() : 0u);

    command->Submit([this, sequence, plugStr, dirStr, completion = std::move(completion)](
                        AVCResult result) mutable {
        if (sequence != mFormatSequence_.load(std::memory_order_acquire)) {
            return;
        }

        const IOReturn status = MapAVCResultToIOReturn(result);
        if (status != kIOReturnSuccess) {
            ASFW_LOG_ERROR(Audio,
                           "BeBoBProtocol: ExtStreamFormat CONTROL failed "
                           "(%s plug, %s) result=%{public}s",
                           plugStr, dirStr, AVCResultToString(result));
        } else {
            ASFW_LOG(Audio,
                     "BeBoBProtocol: ExtStreamFormat CONTROL accepted "
                     "(%s plug, %s) result=%{public}s",
                     plugStr, dirStr, AVCResultToString(result));
        }

        completion(status);
    });
}

// ---------------------------------------------------------------------------
// Format verification helpers
// ---------------------------------------------------------------------------

void BeBoBProtocol::FailFormatVerification(uint32_t sequence,
                                           const char* stage,
                                           IOReturn status)
{
    if (sequence != mFormatSequence_.load(std::memory_order_acquire)) {
        return;
    }

    mFormatDone_.store(false, std::memory_order_release);
    mOutputFormatVerified_.store(false, std::memory_order_release);
    mInputFormatVerified_.store(false, std::memory_order_release);
    mFormatInFlight_.store(false, std::memory_order_release);

    ASFW_LOG_ERROR(Audio,
                   "BeBoBProtocol: Stream format setup failed at %{public}s "
                   "(seq=%u node=0x%04x kr=0x%x)",
                   stage, sequence,
                   mNodeId_.load(std::memory_order_acquire),
                   status);
}

void BeBoBProtocol::CompleteFormatVerification(uint32_t sequence)
{
    if (sequence != mFormatSequence_.load(std::memory_order_acquire)) {
        return;
    }

    mFormatDone_.store(true, std::memory_order_release);
    mFormatInFlight_.store(false, std::memory_order_release);

    ASFW_LOG(Audio,
             "BeBoBProtocol: Stream format setup complete via StreamFormat (0xBF) "
             "(seq=%u node=0x%04x output=%d input=%d)",
             sequence,
             mNodeId_.load(std::memory_order_acquire),
             mOutputFormatVerified_.load(std::memory_order_acquire) ? 1 : 0,
             mInputFormatVerified_.load(std::memory_order_acquire) ? 1 : 0);
}

} // namespace ASFW::Audio::BeBoB
