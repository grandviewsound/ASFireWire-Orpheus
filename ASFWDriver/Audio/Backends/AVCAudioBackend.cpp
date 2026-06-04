// SPDX-License-Identifier: LGPL-3.0-or-later
// Copyright (c) 2026 ASFireWire Project

#include "AVCAudioBackend.hpp"

#include "../../Common/DriverKitOwnership.hpp"
#include "../../Logging/Logging.hpp"
#include "../../Protocols/Audio/IDeviceProtocol.hpp"
#include "../../Protocols/AVC/AVCDefs.hpp"
#include "../../Protocols/AVC/IAVCDiscovery.hpp"
#include "../../IRM/IRMTypes.hpp"   // CalculateBandwidthUnits / BandwidthUnitsRequest

#include <DriverKit/IOLib.h>
#include <DriverKit/IOBufferMemoryDescriptor.h>
#include <DriverKit/OSSharedPtr.h>
#include <optional>
#include <com.kevinpeters.ASFW.ASFWDriver/ASFWAudioNub.h>

namespace ASFW::Audio {

namespace {

constexpr uint8_t kInvalidIsochChannel = 0xFF;
constexpr uint8_t kConnectionSpeed  = 2; // S400 — Orpheus max link speed
// The backend must never impose a shorter wait than the transport's own AV/C
// completion window. Under the current Apple-like cold-attach sequence, the
// Orpheus can answer CMD A near the end of the FCP transport window.
constexpr uint32_t kFormatVerificationTimeoutMs =
    ASFW::Protocols::AVC::kFCPTimeoutAfterInterim + 1000;
constexpr uint32_t kFormatVerificationPollMs = 5;
// Async reads/locks use a 500 ms base timeout and can extend three times by
// 250 ms in AwaitingAR recovery. Give CMP enough budget to observe the real
// callback result instead of aborting while Async still owns the transaction.
constexpr uint32_t kCmpCompletionTimeoutMs = 1750;

inline uint8_t ReadLocalSid(Driver::HardwareInterface& hw) noexcept {
    // OHCI NodeID register: low 6 bits are node number.
    return static_cast<uint8_t>(hw.ReadNodeID() & 0x3Fu);
}

bool WaitForFormatVerification(IDeviceProtocol& protocol, uint32_t timeoutMs) noexcept {
    for (uint32_t waited = 0; waited < timeoutMs; waited += kFormatVerificationPollMs) {
        if (protocol.IsFormatDone()) {
            return true;
        }
        if (!protocol.IsFormatInFlight()) {
            return false;
        }
        IOSleep(kFormatVerificationPollMs);
    }
    return protocol.IsFormatDone();
}

// RAII marker that publishes "a BringUpPipeline is in flight for this GUID" for
// the lifetime of one bring-up call, then clears it on every exit path. The
// marker lets a CoreAudio HAL StartStreaming that arrives mid-bring-up no-op
// instead of launching a second, racing BringUpPipeline (which double-connected
// CMP plugs onto colliding isoch channels). Apple brings the pipeline up exactly
// once, synchronously, inside initHardware before the engine is exposed to the
// HAL — so its StartIO equivalent never participates in bring-up. Our attach is
// asynchronous, so we reproduce that invariant with this marker instead.
class BringUpInProgressMarker {
public:
    BringUpInProgressMarker(IOLock* lock, uint64_t& slot, uint64_t guid) noexcept
        : lock_(lock), slot_(slot) {
        if (lock_ != nullptr) {
            IOLockLock(lock_);
            slot_ = guid;
            IOLockUnlock(lock_);
        }
    }
    ~BringUpInProgressMarker() noexcept {
        if (lock_ != nullptr) {
            IOLockLock(lock_);
            slot_ = 0;
            IOLockUnlock(lock_);
        }
    }
    BringUpInProgressMarker(const BringUpInProgressMarker&) = delete;
    BringUpInProgressMarker& operator=(const BringUpInProgressMarker&) = delete;
    BringUpInProgressMarker(BringUpInProgressMarker&&) = delete;
    BringUpInProgressMarker& operator=(BringUpInProgressMarker&&) = delete;

private:
    IOLock* lock_;
    uint64_t& slot_;
};

} // namespace

AVCAudioBackend::AVCAudioBackend(AudioNubPublisher& publisher,
                                 Discovery::DeviceRegistry& registry,
                                 Driver::IsochService& isoch,
                                 Driver::HardwareInterface& hardware) noexcept
    : publisher_(publisher)
    , registry_(registry)
    , isoch_(isoch)
    , hardware_(hardware) {
    lock_ = IOLockAlloc();
    if (!lock_) {
        ASFW_LOG_ERROR(Audio, "AVCAudioBackend: Failed to allocate lock");
    }
}

AVCAudioBackend::~AVCAudioBackend() noexcept {
    if (lock_) {
        IOLockFree(lock_);
        lock_ = nullptr;
    }
}

void AVCAudioBackend::OnAudioConfigurationReady(uint64_t guid, const Model::ASFWAudioDevice& config) noexcept {
    if (guid == 0) return;

    if (lock_) {
        IOLockLock(lock_);
        configByGuid_[guid] = config;
        IOLockUnlock(lock_);
    }

    (void)publisher_.EnsureNub(guid, config, "AVC");

    // Apr 13 2026: Bring the full isoch pipeline up NOW, at attach time,
    // not when CoreAudio opens the engine. Mirrors Apple's
    // AppleFWAudioDevice::initHardware which fires CMD A (standalone
    // ExtStreamFormat CONTROL), then StartAllStreams — all before any
    // user-level play command. The DAC relay should click ONCE here and
    // then stay engaged for the rest of the device's lifetime on the bus.
    const IOReturn bringUpKr = BringUpPipeline(guid);
    if (bringUpKr != kIOReturnSuccess) {
        ASFW_LOG_ERROR(Audio,
                       "AVCAudioBackend: Attach-time pipeline bring-up failed "
                       "GUID=0x%016llx kr=0x%x — device will be silent",
                       guid,
                       bringUpKr);
    }
}

void AVCAudioBackend::OnDeviceRemoved(uint64_t guid) noexcept {
    if (guid == 0) return;

    TearDownPipeline(guid);
    publisher_.TerminateNub(guid, "AVC-Removed");

    if (lock_) {
        IOLockLock(lock_);
        configByGuid_.erase(guid);
        IOLockUnlock(lock_);
    }
}

bool AVCAudioBackend::WaitForCMP(std::atomic<bool>& done,
                                 std::atomic<ASFW::CMP::CMPStatus>& status,
                                 uint32_t timeoutMs) noexcept {
    constexpr uint32_t kPollMs = 5;
    for (uint32_t waited = 0; waited < timeoutMs; waited += kPollMs) {
        if (done.load(std::memory_order_acquire)) {
            return status.load(std::memory_order_acquire) == ASFW::CMP::CMPStatus::Success;
        }
        IOSleep(kPollMs);
    }
    return false;
}

bool AVCAudioBackend::WaitForIRM(std::atomic<bool>& done,
                                 std::atomic<ASFW::IRM::AllocationStatus>& status,
                                 uint32_t timeoutMs) noexcept {
    constexpr uint32_t kPollMs = 5;
    for (uint32_t waited = 0; waited < timeoutMs; waited += kPollMs) {
        if (done.load(std::memory_order_acquire)) {
            return status.load(std::memory_order_acquire) == ASFW::IRM::AllocationStatus::Success;
        }
        IOSleep(kPollMs);
    }
    return false;
}

// ---------------------------------------------------------------------------
// IRM isochronous bandwidth for the host-talker IT (host→device) channel.
// Apple's working driver reserves BANDWIDTH_AVAILABLE (0xF0000220) in addition to
// claiming the channel; ours historically claimed only the channel. A talker must
// reserve bus bandwidth for the channel it transmits on — faithful + correct for the
// universal remake. Best-effort: failure here NEVER blocks streaming (the device
// ingests regardless of host bandwidth bookkeeping), so we log and continue.
// ---------------------------------------------------------------------------
void AVCAudioBackend::AllocateItBandwidth(uint16_t payloadQuadlets,
                                          uint8_t itChannel) noexcept {
    if (irmClient_ == nullptr || payloadQuadlets == 0 || activeItBandwidthUnits_ != 0) {
        return;
    }
    // Worst case = one max-size data packet every isochronous cycle (8000 Hz).
    // payloadQuadlets already = AM824 data blocks + 2 CIP quadlets (the on-wire payload).
    // TODO(universality): derive speedMbps from the negotiated connection speed rather
    // than hard-coding S400 (current Orpheus connection speed; see kConnectionSpeed).
    const uint32_t pktBytes   = static_cast<uint32_t>(payloadQuadlets) * 4U;
    const uint32_t bitsPerSec = pktBytes * 8U * 8000U;
    const uint32_t bwUnits = ASFW::IRM::CalculateBandwidthUnits(
        {bitsPerSec, /*speedMbps S400*/ 400U, /*overheadPercent*/ 10U});
    if (bwUnits == 0) {
        return;
    }

    std::atomic<bool> done{false};
    std::atomic<ASFW::IRM::AllocationStatus> status{ASFW::IRM::AllocationStatus::Failed};
    irmClient_->AllocateBandwidth(bwUnits,
        [&done, &status](ASFW::IRM::AllocationStatus s) {
            status.store(s, std::memory_order_release);
            done.store(true, std::memory_order_release);
        });
    if (WaitForIRM(done, status, 250)) {
        activeItBandwidthUnits_ = bwUnits;
        ASFW_LOG(Audio,
                 "AVCAudioBackend: IRM allocated %u bandwidth units for IT channel %u "
                 "(payload=%u quadlets, S400)",
                 bwUnits, itChannel, payloadQuadlets);
    } else {
        ASFW_LOG(Audio,
                 "AVCAudioBackend: IRM bandwidth alloc (%u units) for IT channel %u did not "
                 "complete — continuing without it (non-fatal)",
                 bwUnits, itChannel);
    }
}

void AVCAudioBackend::ReleaseItBandwidth() noexcept {
    if (irmClient_ == nullptr || activeItBandwidthUnits_ == 0) {
        return;
    }
    const uint32_t units = activeItBandwidthUnits_;
    std::atomic<bool> done{false};
    std::atomic<ASFW::IRM::AllocationStatus> status{ASFW::IRM::AllocationStatus::Failed};
    irmClient_->ReleaseBandwidth(units,
        [&done, &status](ASFW::IRM::AllocationStatus s) {
            status.store(s, std::memory_order_release);
            done.store(true, std::memory_order_release);
        });
    (void)WaitForIRM(done, status, 250);
    activeItBandwidthUnits_ = 0;
    ASFW_LOG(Audio, "AVCAudioBackend: IRM released %u IT-channel bandwidth units", units);
}

// ---------------------------------------------------------------------------
// StartStreaming / StopStreaming — idempotent sample-gating entry points.
// ---------------------------------------------------------------------------
// Apr 13 2026: The real bring-up lives at attach time (OnAudioConfigurationReady
// → BringUpPipeline). These two calls arrive from CoreAudio HAL via
// ASFWAudioNub::StartAudioStreaming / StopAudioStreaming, and must NOT re-run
// CMP/IT/IR setup or the DAC relay will click on every play/stop and the
// device will never reach its streaming state (see dac-click-timing-insight).

IOReturn AVCAudioBackend::StartStreaming(uint64_t guid) noexcept {
    if (guid == 0) return kIOReturnBadArgument;

    uint64_t liveGuid = 0;
    uint64_t inProgressGuid = 0;
    if (lock_) {
        IOLockLock(lock_);
        liveGuid = pipelineGuid_;
        inProgressGuid = bringUpInProgressGuid_;
        IOLockUnlock(lock_);
    }

    if (liveGuid == guid) {
        ASFW_LOG(Audio,
                 "AVCAudioBackend: StartStreaming no-op, pipeline already live "
                 "GUID=0x%016llx (brought up at attach)",
                 guid);
        return kIOReturnSuccess;
    }

    // The attach-time bring-up is still running (CoreAudio opened the engine
    // during BringUpPipeline's settle window). No-op and let it finish: Apple's
    // HAL StartIO equivalent never participates in bring-up, and re-running it
    // here is what double-connected the CMP plugs onto colliding isoch channels.
    if (inProgressGuid == guid) {
        ASFW_LOG(Audio,
                 "AVCAudioBackend: StartStreaming no-op, attach-time bring-up "
                 "already in flight GUID=0x%016llx",
                 guid);
        return kIOReturnSuccess;
    }

    // Genuine recovery path: pipeline is neither live nor being brought up, so
    // the attach-time bring-up failed or never ran (e.g. OnAudioConfigurationReady
    // fired before dependencies were wired up). Bring it up now so playback still
    // works. This is a deviation from Apple's synchronous initHardware bring-up,
    // kept only as a safety net for our asynchronous attach.
    ASFW_LOG_WARNING(Audio,
                     "AVCAudioBackend: StartStreaming fallback — pipeline neither "
                     "live nor in flight for GUID=0x%016llx (live=0x%016llx), "
                     "bringing up now",
                     guid,
                     liveGuid);
    return BringUpPipeline(guid);
}

IOReturn AVCAudioBackend::StopStreaming(uint64_t guid) noexcept {
    if (guid == 0) return kIOReturnBadArgument;

    // Intentional no-op. The pipeline stays hot until the device leaves
    // the bus (OnDeviceRemoved → TearDownPipeline). Apple does the same:
    // Play/Stop in CoreAudio is just sample-gating on an already-running
    // IT context. Tearing down here would click the DAC on every CoreAudio
    // close and put the device back into the wrong-order state that
    // caused silence in fixes 53–57.
    ASFW_LOG(Audio,
             "AVCAudioBackend: StopStreaming no-op, pipeline stays hot until detach "
             "GUID=0x%016llx",
             guid);
    return kIOReturnSuccess;
}

// ---------------------------------------------------------------------------
// BringUpPipeline — attach-time isoch pipeline setup in Apple's order.
// ---------------------------------------------------------------------------
// Order (Apr 12 dtrace + Apr 11 function trace):
//
//   1. Resolve device + FCP transport        (preconditions)
//   2. StartDuplex48k        — CMD A         (initHardware+0x4d0 direct)
//   3. IRM alloc IR channel                  (input side of StartAllStreams)
//   4. oPCR CMP connect      — RMW×2         (SetUpInputConnection)
//   5. StartReceive          — IR DMA live   (NuDCLRead::StartStream)
//   6. IRM alloc IT channel                  (inside NuDCLWrite::Start)
//   7. PreparePlaybackPath   — CMDs B + C    (inside SetUpOutputConnection)
//   8. iPCR CMP connect      — RMW×2         (inside SetUpOutputConnection)
//   9. StartTransmit         — IT DMA live   (NuDCLWrite::Start returns)
//
// Previous ordering (fix 56) did IR DMA before oPCR lock and IT DMA before
// CMDs B/C, so packets hit the bus before the device knew the format.
// Changes here should be justified against apr12-dtrace-full-sequence.md.

IOReturn AVCAudioBackend::BringUpPipeline(uint64_t guid) noexcept {
    if (guid == 0) return kIOReturnBadArgument;

    // Publish "bring-up in flight" for the whole of this call so a CoreAudio HAL
    // StartStreaming that arrives during the attach-time settle window no-ops
    // rather than launching a second, channel-colliding BringUpPipeline. Cleared
    // on every exit path; on success pipelineGuid_ (set below) takes over as the
    // liveness marker, so StartStreaming keeps no-opping after this returns.
    BringUpInProgressMarker inProgress(lock_, bringUpInProgressGuid_, guid);

    uint64_t entryLiveGuid = 0;
    if (lock_) {
        IOLockLock(lock_);
        entryLiveGuid = pipelineGuid_;
        IOLockUnlock(lock_);
    }
    ASFW_LOG(Audio,
             "AVCAudioBackend: BringUpPipeline ENTER GUID=0x%016llx liveGuid=0x%016llx",
             guid,
             entryLiveGuid);

    if (!cmpClient_ || !irmClient_) {
        ASFW_LOG_ERROR(Audio,
                       "AVCAudioBackend: BringUpPipeline missing CMP/IRM client "
                       "GUID=0x%016llx",
                       guid);
        ASFW_LOG(Audio,
                 "AVCAudioBackend: BringUpPipeline RETURN GUID=0x%016llx kr=0x%x reason=missing-client",
                 guid,
                 kIOReturnNotReady);
        return kIOReturnNotReady;
    }

    // --- Step 1: Preconditions ---------------------------------------------
    Model::ASFWAudioDevice config{};
    bool hasConfig = false;
    if (lock_) {
        IOLockLock(lock_);
        auto it = configByGuid_.find(guid);
        if (it != configByGuid_.end()) {
            config = it->second;
            hasConfig = true;
        }
        IOLockUnlock(lock_);
    }
    if (!hasConfig) {
        ASFW_LOG(Audio, "AVCAudioBackend: BringUpPipeline no config GUID=0x%016llx", guid);
        return kIOReturnNotReady;
    }

    const auto* record = registry_.FindByGuid(guid);
    if (!record) {
        ASFW_LOG(Audio, "AVCAudioBackend: BringUpPipeline no device record GUID=0x%016llx", guid);
        return kIOReturnNotReady;
    }

    ASFW::Protocols::AVC::FCPTransport* transport = nullptr;
    if (record->protocol) {
        if (avcDiscovery_) {
            transport = avcDiscovery_->GetFCPTransportForNodeID(record->nodeId);
        }
        record->protocol->UpdateRuntimeContext(record->nodeId, transport);
        record->protocol->UpdateDiscoveredStreamFormatBlocks(
            config.playback48kRawFormatBlock,
            config.capture48kRawFormatBlock);
    }

    cmpClient_->SetDeviceNode(static_cast<uint8_t>(record->nodeId),
                              static_cast<ASFW::IRM::Generation>(record->gen));
    irmClient_->SetIRMNode(static_cast<uint8_t>(record->nodeId),
                           static_cast<ASFW::IRM::Generation>(record->gen));

    // analysis evidence: AppleFWAudioDevice::initHardware waits
    // 900ms + 4ms * (nodeID & 0x0f) before it starts the AV/C/isoc attach
    // sequence. Preserve that bus-settle window before PCR snapshots,
    // stream-format CONTROLs, CMP locks, or DMA start.
    const uint32_t appleSettleDelayMs =
        900u + (4u * (static_cast<uint32_t>(record->nodeId) & 0x0Fu));
    ASFW_LOG(Audio,
             "AVCAudioBackend: Apple initHardware settle delay %ums "
             "GUID=0x%016llx node=0x%04x",
             appleSettleDelayMs,
             guid,
             record->nodeId);
    IOSleep(appleSettleDelayMs);

    // AppleFWAudio::StartAllDirectionStreams starts the primary stream object
    // and then walks a stream collection for any additional streams of the
    // same direction. Before we risk opening extra DMA paths, snapshot every
    // remote PCR the device advertised so the next hardware log tells us which
    // iPCR/oPCRs are online, their channel hints, and their payload sizes.
    // Returns std::nullopt on read failure, or the raw 32-bit PCR value on
    // success. Logs the PCR breakdown either way. Fix 93 needs the actual
    // value (not just a log line) so bringUpInputSide can read the device's
    // advertised channel before allocating its own.
    auto readRemotePCRValue = [&](bool inputPlug, uint8_t plugNum,
                                  const char* stage) -> std::optional<uint32_t> {
        std::atomic<bool> done{false};
        bool ok = false;
        uint32_t value = 0;

        auto callback = [&done, &ok, &value](bool success, uint32_t raw) {
            ok = success;
            value = raw;
            done.store(true, std::memory_order_release);
        };

        if (inputPlug) {
            cmpClient_->ReadIPCR(plugNum, callback);
        } else {
            cmpClient_->ReadOPCR(plugNum, callback);
        }

        constexpr uint32_t kPCRSnapshotTimeoutMs = 500;
        constexpr uint32_t kPCRSnapshotPollMs = 5;
        uint32_t waited = 0;
        while (!done.load(std::memory_order_acquire) && waited < kPCRSnapshotTimeoutMs) {
            IOSleep(kPCRSnapshotPollMs);
            waited += kPCRSnapshotPollMs;
        }

        const char* pcrName = inputPlug ? "iPCR" : "oPCR";
        if (!done.load(std::memory_order_acquire) || !ok) {
            ASFW_LOG(Audio,
                     "AVCAudioBackend: PCRSnapshot[%{public}s] remote %{public}s[%u] read failed",
                     stage,
                     pcrName,
                     plugNum);
            return std::nullopt;
        }

        ASFW_LOG(Audio,
                 "AVCAudioBackend: PCRSnapshot[%{public}s] remote %{public}s[%u]=0x%08x "
                 "online=%u p2p=%u ch=%u speed=%u payloadQ=%u",
                 stage,
                 pcrName,
                 plugNum,
                 value,
                 ASFW::CMP::PCRBits::IsOnline(value) ? 1 : 0,
                 ASFW::CMP::PCRBits::GetP2P(value),
                 ASFW::CMP::PCRBits::GetChannel(value),
                 ASFW::CMP::PCRBits::GetSpeed(value),
                 value & 0x03FFu);
        return value;
    };

    auto readRemotePCR = [&](bool inputPlug, uint8_t plugNum, const char* stage) {
        (void)readRemotePCRValue(inputPlug, plugNum, stage);
    };

    auto snapshotRemotePCRs = [&](const char* stage) {
        constexpr uint32_t kMaxPCRSnapshotPlugs = 31;
        uint32_t inputCount = config.unitIsoInputPlugCount;
        uint32_t outputCount = config.unitIsoOutputPlugCount;
        if (inputCount > kMaxPCRSnapshotPlugs) {
            inputCount = kMaxPCRSnapshotPlugs;
        }
        if (outputCount > kMaxPCRSnapshotPlugs) {
            outputCount = kMaxPCRSnapshotPlugs;
        }

        ASFW_LOG(Audio,
                 "AVCAudioBackend: PCRSnapshot[%{public}s] unitIso input=%u output=%u",
                 stage,
                 config.unitIsoInputPlugCount,
                 config.unitIsoOutputPlugCount);

        for (uint32_t p = 0; p < inputCount; ++p) {
            readRemotePCR(true, static_cast<uint8_t>(p), stage);
        }
        for (uint32_t p = 0; p < outputCount; ++p) {
            readRemotePCR(false, static_cast<uint8_t>(p), stage);
        }
    };

    snapshotRemotePCRs("pre-cmd-a");

    auto* nub = publisher_.GetNub(guid);
    if (!nub) {
        (void)publisher_.EnsureNub(guid, config, "AVC-BringUp");
        nub = publisher_.GetNub(guid);
        if (!nub) {
            ASFW_LOG_ERROR(Audio,
                           "AVCAudioBackend: BringUpPipeline cannot publish nub "
                           "GUID=0x%016llx",
                           guid);
            return kIOReturnNotReady;
        }
    }

    // Allocate RX/TX queue memory up-front so the IR/IT contexts have
    // somewhere to stage data. ASFWAudioNub creates these on demand and
    // maps them for both sides; they are independent of CoreAudio engine
    // state.
    nub->EnsureRxQueueCreated();
    IOBufferMemoryDescriptor* rxMemRaw = nullptr;
    uint64_t rxBytes = 0;
    const kern_return_t rxCopy = nub->CopyRxQueueMemory(&rxMemRaw, &rxBytes);
    auto rxMem = Common::AdoptRetained(rxMemRaw);
    if (rxCopy != kIOReturnSuccess || !rxMem || rxBytes == 0) {
        return (rxCopy == kIOReturnSuccess) ? kIOReturnNoMemory : rxCopy;
    }

    IOBufferMemoryDescriptor* txMemRaw = nullptr;
    uint64_t txBytes = 0;
    const kern_return_t txCopy = nub->CopyTransmitQueueMemory(&txMemRaw, &txBytes);
    auto txMem = Common::AdoptRetained(txMemRaw);
    if (txCopy != kIOReturnSuccess || !txMem || txBytes == 0) {
        return (txCopy == kIOReturnSuccess) ? kIOReturnNoMemory : txCopy;
    }

    // --- Step 2: CMD A — standalone ExtStreamFormat CONTROL (iPCR) ---------
    // Apple's AppleFWAudioDevice::initHardware+0x4d0 fires a single
    // SetExtendedStreamFormat CONTROL on the iPCR unit plug ~33 ms before
    // any CMP or IRM work begins. Not chained to the CMP connect.
    if (record->protocol && !record->protocol->IsFormatDone()) {
        if (!transport) {
            ASFW_LOG_ERROR(Audio,
                           "AVCAudioBackend: BringUpPipeline missing FCP transport "
                           "GUID=0x%016llx node=0x%04x",
                           guid,
                           record->nodeId);
            return kIOReturnNotReady;
        }

        ASFW_LOG(Audio,
                 "AVCAudioBackend: CMD A — standalone ExtStreamFormat CONTROL (iPCR) "
                 "GUID=0x%016llx",
                 guid);
        const IOReturn formatKr = record->protocol->StartDuplex48k();
        if (formatKr != kIOReturnSuccess) {
            ASFW_LOG_ERROR(Audio,
                           "AVCAudioBackend: CMD A StartDuplex48k failed "
                           "GUID=0x%016llx kr=0x%x",
                           guid,
                           formatKr);
            return formatKr;
        }

        if (!WaitForFormatVerification(*record->protocol, kFormatVerificationTimeoutMs)) {
            ASFW_LOG_ERROR(Audio,
                           "AVCAudioBackend: CMD A verification timed out "
                           "GUID=0x%016llx",
                           guid);
            return kIOReturnTimeout;
        }

        ASFW_LOG(Audio,
                 "AVCAudioBackend: CMD A verified GUID=0x%016llx",
                 guid);
    }

    // Default: assume the device is on its internal clock (Apple externalSync=0;
    // transmit SYT free-runs). Overridden below from the protocol's clock-source
    // detection. Set explicitly every bring-up so a prior external session can't
    // leave the discipline engaged for an internal-clocked device.
    isoch_.SetExternalClockSource(false);

    if (record->protocol) {
        if (auto orderHint = record->protocol->GetAppleStartOrderHint();
            orderHint.has_value()) {
            const bool previousOrder = config.startInputBeforeOutput;
            config.startInputBeforeOutput = orderHint->inputBeforeOutput;
            // Apple's SetClockSource gates external-sync SYT slaving on a
            // non-internal clock source; mirror that into the isoch bridge.
            isoch_.SetExternalClockSource(orderHint->externalClock);
            if (lock_) {
                IOLockLock(lock_);
                configByGuid_[guid] = config;
                IOLockUnlock(lock_);
            }
            ASFW_LOG(Audio,
                     "AVCAudioBackend: Apple StartAllStreams live-clock hint "
                     "GUID=0x%016llx order=%{public}s reason=%{public}s "
                     "previous=%{public}s",
                     guid,
                     config.startInputBeforeOutput ? "input-then-output" : "output-then-input",
                     orderHint->reason ? orderHint->reason : "unknown",
                     previousOrder ? "input-then-output" : "output-then-input");
        }
    }

    // --- Shared IRM helpers (fix 97: Apple-faithful stateful allocation) ---
    //
    // Apple's IOFWIsochChannel::allocateChannelBegin scans channels 0→63,
    // intersects the requested mask with the LIVE IRM channels-available CSR,
    // and atomically claims the chosen channel (clearing its CSR bit). Because
    // the claim is stateful across calls, the first direction to allocate
    // takes the low channel and the second direction's lowest-first scan
    // naturally returns the next free channel — collision avoidance is
    // emergent from allocation ORDER + stateful IRM, never from rewriting a
    // device's PCR channel field.
    // (reports/apple_irm_channel_allocation_ida_pass_2026-05-19.md)
    //
    // claimSpecificChannel reserves exactly one channel (used to reserve the
    // device's already-advertised oPCR channel for IR, so the CMP connect is a
    // no-op channel write + p2p bump rather than a channel CHANGE).
    auto claimSpecificChannel = [&](uint8_t ch, const char* tag) -> bool {
        if (ch >= 64) return false;
        std::atomic<bool> done{false};
        std::atomic<ASFW::IRM::AllocationStatus> status{ASFW::IRM::AllocationStatus::Failed};
        irmClient_->AllocateChannel(ch,
            [&done, &status](ASFW::IRM::AllocationStatus s) {
                status.store(s, std::memory_order_release);
                done.store(true, std::memory_order_release);
            });
        if (WaitForIRM(done, status, 250)) {
            ASFW_LOG(Audio, "AVCAudioBackend: IRM reserved specific channel %u for %{public}s", ch, tag);
            return true;
        }
        ASFW_LOG(Audio, "AVCAudioBackend: IRM could not reserve channel %u for %{public}s", ch, tag);
        return false;
    };

    auto claimLowestFreeChannel = [&](uint8_t startFrom, const char* tag) -> uint8_t {
        for (uint8_t ch = startFrom; ch < 64; ++ch) {
            std::atomic<bool> done{false};
            std::atomic<ASFW::IRM::AllocationStatus> status{ASFW::IRM::AllocationStatus::Failed};
            irmClient_->AllocateChannel(ch,
                [&done, &status](ASFW::IRM::AllocationStatus s) {
                    status.store(s, std::memory_order_release);
                    done.store(true, std::memory_order_release);
                });
            if (WaitForIRM(done, status, 250)) {
                ASFW_LOG(Audio, "AVCAudioBackend: IRM claimed channel %u for %{public}s", ch, tag);
                return ch;
            }
        }
        ASFW_LOG_ERROR(Audio, "AVCAudioBackend: IRM could not find a free channel for %{public}s", tag);
        return kInvalidIsochChannel;
    };

    auto releaseChannel = [&](uint8_t ch) {
        if (ch == kInvalidIsochChannel) return;
        std::atomic<bool> done{false};
        std::atomic<ASFW::IRM::AllocationStatus> status{ASFW::IRM::AllocationStatus::Failed};
        irmClient_->ReleaseChannel(ch,
            [&done, &status](ASFW::IRM::AllocationStatus s) {
                status.store(s, std::memory_order_release);
                done.store(true, std::memory_order_release);
            });
        (void)WaitForIRM(done, status, 250);
    };

    uint8_t irChannel = kInvalidIsochChannel;
    uint8_t itChannel = kInvalidIsochChannel;
    bool opcrConnected = false;
    bool ipcrConnected = false;
    bool receiveStarted = false;
    bool transmitStarted = false;

    auto cleanupPartialBringup = [&]() {
        if (transmitStarted) {
            (void)isoch_.StopTransmit();
            transmitStarted = false;
        }
        if (ipcrConnected) {
            cmpClient_->DisconnectIPCR(0, [](ASFW::CMP::CMPStatus) {});
            ipcrConnected = false;
        }
        if (receiveStarted) {
            (void)isoch_.StopReceive();
            receiveStarted = false;
        }
        if (opcrConnected) {
            cmpClient_->DisconnectOPCR(0, [](ASFW::CMP::CMPStatus) {});
            opcrConnected = false;
        }
        ReleaseItBandwidth();
        releaseChannel(itChannel);
        releaseChannel(irChannel);
        itChannel = kInvalidIsochChannel;
        irChannel = kInvalidIsochChannel;
    };

    auto connectOPCR = [&]() -> IOReturn {
        std::atomic<bool> done{false};
        std::atomic<ASFW::CMP::CMPStatus> status{ASFW::CMP::CMPStatus::Failed};
        cmpClient_->ConnectOPCR(0, irChannel, kConnectionSpeed,
                                [&done, &status](ASFW::CMP::CMPStatus s) {
            status.store(s, std::memory_order_release);
            done.store(true, std::memory_order_release);
        });

        if (!WaitForCMP(done, status, kCmpCompletionTimeoutMs)) {
            ASFW_LOG_ERROR(Audio,
                           "AVCAudioBackend: CMP ConnectOPCR failed GUID=0x%016llx status=%d",
                           guid,
                           static_cast<int>(status.load(std::memory_order_acquire)));
            return kIOReturnError;
        }
        opcrConnected = true;
        return kIOReturnSuccess;
    };

    auto startReceive = [&]() -> IOReturn {
        const kern_return_t krRx = isoch_.StartReceive(irChannel,
                                                       hardware_,
                                                       rxMem,
                                                       rxBytes);
        if (krRx != kIOReturnSuccess) {
            ASFW_LOG_ERROR(Audio,
                           "AVCAudioBackend: StartReceive failed GUID=0x%016llx kr=0x%x",
                           guid,
                           krRx);
            return krRx;
        }
        receiveStarted = true;
        return kIOReturnSuccess;
    };

    auto preparePlaybackPath = [&]() {
        // Apple emits these inside SetUpOutputConnection -> SetSampleRate,
        // BEFORE the iPCR CMP lock and BEFORE the IT DMA goes live.
        if (record->protocol) {
            const IOReturn prepKr = record->protocol->PreparePlaybackPath();
            if (prepKr != kIOReturnSuccess && prepKr != kIOReturnUnsupported) {
                ASFW_LOG_ERROR(Audio,
                               "AVCAudioBackend: PreparePlaybackPath (CMDs B+C) failed "
                               "GUID=0x%016llx kr=0x%x — continuing",
                               guid,
                               prepKr);
            }
        }
    };

    auto connectIPCR = [&]() -> IOReturn {
        std::atomic<bool> done{false};
        std::atomic<ASFW::CMP::CMPStatus> status{ASFW::CMP::CMPStatus::Failed};
        cmpClient_->ConnectIPCR(0, itChannel, kConnectionSpeed,
                                [&done, &status](ASFW::CMP::CMPStatus s) {
            status.store(s, std::memory_order_release);
            done.store(true, std::memory_order_release);
        });

        if (!WaitForCMP(done, status, kCmpCompletionTimeoutMs)) {
            ASFW_LOG_ERROR(Audio,
                           "AVCAudioBackend: CMP ConnectIPCR failed GUID=0x%016llx status=%d",
                           guid,
                           static_cast<int>(status.load(std::memory_order_acquire)));
            return kIOReturnError;
        }
        ipcrConnected = true;
        return kIOReturnSuccess;
    };

    auto startTransmit = [&]() -> IOReturn {
        const uint8_t sid = ReadLocalSid(hardware_);
        const uint32_t streamModeRaw = static_cast<uint32_t>(config.streamMode);

        // Query protocol for wire-level AM824 slot count (includes MIDI
        // slots) and PCM channel count (audio-only, excludes MIDI/SPDIF).
        uint32_t am824Slots = config.outputChannelCount;
        uint32_t pcmChannels = config.outputChannelCount;
        if (record->protocol) {
            AudioStreamRuntimeCaps caps{};
            if (record->protocol->GetRuntimeAudioStreamCaps(caps)) {
                if (caps.hostToDeviceAm824Slots > 0) {
                    am824Slots = caps.hostToDeviceAm824Slots;
                }
                if (caps.hostOutputPcmChannels > 0) {
                    pcmChannels = caps.hostOutputPcmChannels;
                }
            }
        }

        const kern_return_t krTx = isoch_.StartTransmit(itChannel,
                                                        hardware_,
                                                        sid,
                                                        streamModeRaw,
                                                        pcmChannels,
                                                        am824Slots,
                                                        txMem,
                                                        txBytes,
                                                        nullptr,
                                                        0,
                                                        0,
                                                        config.hostOutputIsochChannelPositions.data(),
                                                        static_cast<uint32_t>(
                                                            config.hostOutputIsochChannelPositions.size()));
        if (krTx != kIOReturnSuccess) {
            ASFW_LOG_ERROR(Audio,
                           "AVCAudioBackend: StartTransmit failed GUID=0x%016llx kr=0x%x",
                           guid,
                           krTx);
            return krTx;
        }
        transmitStarted = true;
        return kIOReturnSuccess;
    };

    ASFW_LOG(Audio,
             "AVCAudioBackend: Apple StartAllStreams order GUID=0x%016llx order=%{public}s",
             guid,
             config.startInputBeforeOutput ? "input-then-output" : "output-then-input");

    auto bringUpInputSide = [&]() -> IOReturn {
        ASFW_LOG(Audio,
                 "AVCAudioBackend: bringUpInputSide BEGIN GUID=0x%016llx itChannel=%u",
                 guid,
                 itChannel);

        // Fix 97: IR channel is reserved up-front in the allocateChannels
        // phase (device's advertised oPCR channel reserved via stateful IRM,
        // or lowest-free if oPCR was offline). It is therefore already set by
        // the time we get here; nothing to claim. CMP connectOPCR then writes
        // the same channel back (a no-op channel field + p2p bump), never a
        // channel CHANGE — which is what avoided the old hardware_error(5).
        if (irChannel == kInvalidIsochChannel) {
            ASFW_LOG(Audio,
                     "AVCAudioBackend: bringUpInputSide END GUID=0x%016llx kr=0x%x step=claimIR",
                     guid,
                     kIOReturnNoResources);
            return kIOReturnNoResources;
        }
        if (const IOReturn kr = connectOPCR(); kr != kIOReturnSuccess) {
            ASFW_LOG(Audio,
                     "AVCAudioBackend: bringUpInputSide END GUID=0x%016llx kr=0x%x step=connectOPCR",
                     guid,
                     kr);
            return kr;
        }
        const IOReturn rxKr = startReceive();
        ASFW_LOG(Audio,
                 "AVCAudioBackend: bringUpInputSide END GUID=0x%016llx kr=0x%x step=startReceive irCh=%u",
                 guid,
                 rxKr,
                 irChannel);
        return rxKr;
    };

    auto bringUpOutputSide = [&]() -> IOReturn {
        ASFW_LOG(Audio,
                 "AVCAudioBackend: bringUpOutputSide BEGIN GUID=0x%016llx irChannel=%u",
                 guid,
                 irChannel);

        // Fix 97: IT claims the lowest free channel via stateful IRM. The IR
        // channel was already reserved in the allocateChannels phase, so the
        // IRM CSR bit for it is clear and this scan skips it automatically —
        // no explicit avoid, no device-PCR rewrite. This mirrors Apple's
        // allocateChannelBegin (lowest-first over the live IRM register).
        itChannel = claimLowestFreeChannel(0, "IT/iPCR");
        if (itChannel == kInvalidIsochChannel) {
            ASFW_LOG(Audio,
                     "AVCAudioBackend: bringUpOutputSide END GUID=0x%016llx kr=0x%x step=claimIT",
                     guid,
                     kIOReturnNoResources);
            return kIOReturnNoResources;
        }
        if (cmpClient_) {
            uint16_t payloadQuadlets = 0;
            if (record->protocol) {
                AudioStreamRuntimeCaps caps{};
                if (record->protocol->GetRuntimeAudioStreamCaps(caps) &&
                    caps.hostToDeviceAm824Slots > 0) {
                    // Apple's local oPCR advertises the host talker's packet capacity.
                    // Orpheus playback is blocking AM824: 8 data blocks plus 2 CIP quadlets.
                    payloadQuadlets = static_cast<uint16_t>((caps.hostToDeviceAm824Slots * 8u) + 2u);
                }
            }
            if (payloadQuadlets != 0) {
                cmpClient_->SetLocalOutputPayloadQuadlets(0, payloadQuadlets);
                // Apple parity: reserve isoch bus bandwidth for our talker (IT) channel,
                // not just the channel itself. Best-effort, non-fatal (see AllocateItBandwidth).
                AllocateItBandwidth(payloadQuadlets, itChannel);
            }
        }
        preparePlaybackPath();
        if (const IOReturn kr = connectIPCR(); kr != kIOReturnSuccess) {
            ASFW_LOG(Audio,
                     "AVCAudioBackend: bringUpOutputSide END GUID=0x%016llx kr=0x%x step=connectIPCR",
                     guid,
                     kr);
            return kr;
        }
        const IOReturn txKr = startTransmit();
        ASFW_LOG(Audio,
                 "AVCAudioBackend: bringUpOutputSide END GUID=0x%016llx kr=0x%x step=startTransmit itCh=%u",
                 guid,
                 txKr,
                 itChannel);
        return txKr;
    };

    // Fix 97: allocate BOTH isoch channels up-front via stateful IRM, BEFORE
    // either direction's stream is brought up. Allocation order is independent
    // of the stream-start order below.
    //
    //   1. Reserve the IR channel first. If the device's oPCR[0] is already
    //      online with a valid channel, reserve THAT specific channel — so the
    //      subsequent CMP connectOPCR writes the same channel back (no-op
    //      channel field + p2p bump) instead of a channel CHANGE (which is
    //      what produced hardware_error(5)). Apple's allocateChannelBegin with
    //      mask=-1 lands on the device's already-advertised low channel when
    //      it is free, so reserving it explicitly is behaviour-equivalent.
    //   2. IT then claims the lowest free channel; the IR reservation has
    //      already cleared that channel's IRM CSR bit, so the lowest-first
    //      scan skips it automatically.
    //
    // (reports/apple_irm_channel_allocation_ida_pass_2026-05-19.md)
    {
        const auto opcrValue = readRemotePCRValue(false, 0, "pre-IR-reserve");
        if (opcrValue.has_value() && ASFW::CMP::PCRBits::IsOnline(*opcrValue)) {
            const uint8_t advertisedCh = ASFW::CMP::PCRBits::GetChannel(*opcrValue);
            if (advertisedCh <= 30 && claimSpecificChannel(advertisedCh, "IR/oPCR")) {
                irChannel = advertisedCh;
                ASFW_LOG(Audio,
                         "AVCAudioBackend: reserved device-advertised oPCR[0] channel %u for IR",
                         irChannel);
            }
        }
        if (irChannel == kInvalidIsochChannel) {
            irChannel = claimLowestFreeChannel(0, "IR/oPCR");
            ASFW_LOG(Audio,
                     "AVCAudioBackend: oPCR[0] offline/unavailable — IR claimed lowest free channel %u",
                     irChannel);
        }
    }

    IOReturn orderKr = kIOReturnSuccess;
    if (config.startInputBeforeOutput) {
        orderKr = bringUpInputSide();
        if (orderKr == kIOReturnSuccess) {
            orderKr = bringUpOutputSide();
        }
    } else {
        orderKr = bringUpOutputSide();
        if (orderKr == kIOReturnSuccess) {
            orderKr = bringUpInputSide();
        }
    }

    if (orderKr != kIOReturnSuccess) {
        cleanupPartialBringup();
        ASFW_LOG(Audio,
                 "AVCAudioBackend: BringUpPipeline RETURN GUID=0x%016llx kr=0x%x reason=order-failed",
                 guid,
                 orderKr);
        return orderKr;
    }


    // AppleFWAudioDevice::StartAllStreams calls SyncInputStreams as part of the
    // attach start sequence. Keep the hook after both directions are live so the
    // DriverKit TX timing discipline can lock onto the IR side before user audio.
    isoch_.SyncOutputInputStreams();

    // --- Success: record liveness + log ------------------------------------
    if (lock_) {
        IOLockLock(lock_);
        pipelineGuid_ = guid;
        activeIrChannel_ = irChannel;
        activeItChannel_ = itChannel;
        IOLockUnlock(lock_);
    }

    ASFW_LOG(Audio,
             "AVCAudioBackend: Pipeline live at attach GUID=0x%016llx "
             "(in=%u out=%u mode=%{public}s irCh=%u itCh=%u)",
             guid,
             config.inputChannelCount,
             config.outputChannelCount,
             config.streamMode == Model::StreamMode::kBlocking ? "blocking" : "non-blocking",
             irChannel,
             itChannel);

    snapshotRemotePCRs("post-bringup");

    ASFW_LOG(Audio,
             "AVCAudioBackend: BringUpPipeline RETURN GUID=0x%016llx kr=0x%x reason=success",
             guid,
             kIOReturnSuccess);
    return kIOReturnSuccess;
}

// ---------------------------------------------------------------------------
// TearDownPipeline — detach-time reverse of BringUpPipeline.
// ---------------------------------------------------------------------------
void AVCAudioBackend::TearDownPipeline(uint64_t guid) noexcept {
    uint8_t irChannel = kInvalidIsochChannel;
    uint8_t itChannel = kInvalidIsochChannel;
    bool wasLive = false;
    bool startInputBeforeOutput = false;

    if (lock_) {
        IOLockLock(lock_);
        if (auto it = configByGuid_.find(guid); it != configByGuid_.end()) {
            startInputBeforeOutput = it->second.startInputBeforeOutput;
        }
        if (pipelineGuid_ == guid) {
            wasLive = true;
            irChannel = activeIrChannel_;
            itChannel = activeItChannel_;
            pipelineGuid_ = 0;
            activeIrChannel_ = kInvalidIsochChannel;
            activeItChannel_ = kInvalidIsochChannel;
        }
        IOLockUnlock(lock_);
    }

    if (!wasLive) {
        // Either bring-up never succeeded or teardown already ran. Still
        // kick isoch transport into stopped state as a safety net.
        (void)isoch_.StopTransmit();
        (void)isoch_.StopReceive();
        return;
    }

    if (!cmpClient_) {
        (void)isoch_.StopTransmit();
        (void)isoch_.StopReceive();
        return;
    }

    const auto* record = registry_.FindByGuid(guid);
    if (record) {
        cmpClient_->SetDeviceNode(static_cast<uint8_t>(record->nodeId),
                                  static_cast<ASFW::IRM::Generation>(record->gen));
    }

    auto disconnectIPCR = [&]() {
        std::atomic<bool> done{false};
        std::atomic<ASFW::CMP::CMPStatus> status{ASFW::CMP::CMPStatus::Failed};
        cmpClient_->DisconnectIPCR(0, [&done, &status](ASFW::CMP::CMPStatus s) {
            status.store(s, std::memory_order_release);
            done.store(true, std::memory_order_release);
        });
        (void)WaitForCMP(done, status, kCmpCompletionTimeoutMs);
    };

    auto disconnectOPCR = [&]() {
        std::atomic<bool> done{false};
        std::atomic<ASFW::CMP::CMPStatus> status{ASFW::CMP::CMPStatus::Failed};
        cmpClient_->DisconnectOPCR(0, [&done, &status](ASFW::CMP::CMPStatus s) {
            status.store(s, std::memory_order_release);
            done.store(true, std::memory_order_release);
        });
        (void)WaitForCMP(done, status, kCmpCompletionTimeoutMs);
    };

    auto stopInputSide = [&]() {
        (void)isoch_.StopReceive();
        disconnectOPCR();
    };

    auto stopOutputSide = [&]() {
        (void)isoch_.StopTransmit();
        disconnectIPCR();
    };

    ASFW_LOG(Audio,
             "AVCAudioBackend: Apple StopAllStreams order GUID=0x%016llx order=%{public}s",
             guid,
             startInputBeforeOutput ? "input-then-output" : "output-then-input");

    if (startInputBeforeOutput) {
        stopInputSide();
        stopOutputSide();
    } else {
        stopOutputSide();
        stopInputSide();
    }

    if (irmClient_ && record) {
        irmClient_->SetIRMNode(static_cast<uint8_t>(record->nodeId),
                               static_cast<ASFW::IRM::Generation>(record->gen));

        auto releaseChannel = [&](uint8_t ch) {
            if (ch == kInvalidIsochChannel) return;
            std::atomic<bool> done{false};
            std::atomic<ASFW::IRM::AllocationStatus> status{ASFW::IRM::AllocationStatus::Failed};
            irmClient_->ReleaseChannel(ch,
                [&done, &status](ASFW::IRM::AllocationStatus s) {
                    status.store(s, std::memory_order_release);
                    done.store(true, std::memory_order_release);
                });
            (void)WaitForIRM(done, status, 250);
        };

        ReleaseItBandwidth();
        releaseChannel(itChannel);
        releaseChannel(irChannel);
    }

    ASFW_LOG(Audio, "AVCAudioBackend: Pipeline torn down GUID=0x%016llx", guid);
}

} // namespace ASFW::Audio
