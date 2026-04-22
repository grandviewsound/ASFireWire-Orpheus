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

[[nodiscard]] bool RawFormatBlockLooksUsable(const std::vector<uint8_t>& rawFormatBlock) noexcept
{
    return rawFormatBlock.size() >= 3 && rawFormatBlock[0] == 0x90;
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
    return kIOReturnSuccess;
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
