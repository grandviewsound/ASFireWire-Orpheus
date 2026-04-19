// SPDX-License-Identifier: LGPL-3.0-or-later
// Copyright (c) 2026 ASFireWire Project

#include "AVCAudioBackend.hpp"

#include "../../Common/DriverKitOwnership.hpp"
#include "../../Logging/Logging.hpp"
#include "../../Protocols/Audio/IDeviceProtocol.hpp"
#include "../../Protocols/AVC/AVCDefs.hpp"
#include "../../Protocols/AVC/IAVCDiscovery.hpp"

#include <DriverKit/IOLib.h>
#include <DriverKit/IOBufferMemoryDescriptor.h>
#include <DriverKit/OSSharedPtr.h>
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
    if (lock_) {
        IOLockLock(lock_);
        liveGuid = pipelineGuid_;
        IOLockUnlock(lock_);
    }

    if (liveGuid == guid) {
        ASFW_LOG(Audio,
                 "AVCAudioBackend: StartStreaming no-op, pipeline already live "
                 "GUID=0x%016llx (brought up at attach)",
                 guid);
        return kIOReturnSuccess;
    }

    // Fallback: attach-time bring-up never ran (or a different GUID is
    // live). Happens if OnAudioConfigurationReady fired before dependencies
    // were wired up. Bring it up now so playback still works.
    ASFW_LOG_WARNING(Audio,
                     "AVCAudioBackend: StartStreaming fallback — pipeline not live "
                     "for GUID=0x%016llx (live=0x%016llx), bringing up now",
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

    if (!cmpClient_ || !irmClient_) {
        ASFW_LOG_ERROR(Audio,
                       "AVCAudioBackend: BringUpPipeline missing CMP/IRM client "
                       "GUID=0x%016llx",
                       guid);
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
    }

    cmpClient_->SetDeviceNode(static_cast<uint8_t>(record->nodeId),
                              static_cast<ASFW::IRM::Generation>(record->gen));
    irmClient_->SetIRMNode(static_cast<uint8_t>(record->nodeId),
                           static_cast<ASFW::IRM::Generation>(record->gen));

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

    // --- Shared IRM helpers used by steps 3 and 6 --------------------------
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

    // --- Step 3: IRM alloc IR channel --------------------------------------
    const uint8_t irChannel = claimLowestFreeChannel(0, "IR/oPCR");
    if (irChannel == kInvalidIsochChannel) {
        return kIOReturnNoResources;
    }

    // --- Step 4: oPCR CMP connect (RMW×2 — see CMPClient::PerformConnect) --
    {
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
            releaseChannel(irChannel);
            return kIOReturnError;
        }
    }

    // --- Step 5: StartReceive — IR DMA goes live on the bus ----------------
    {
        const kern_return_t krRx = isoch_.StartReceive(irChannel,
                                                       hardware_,
                                                       rxMem,
                                                       rxBytes);
        if (krRx != kIOReturnSuccess) {
            ASFW_LOG_ERROR(Audio,
                           "AVCAudioBackend: StartReceive failed GUID=0x%016llx kr=0x%x",
                           guid,
                           krRx);
            cmpClient_->DisconnectOPCR(0, [](ASFW::CMP::CMPStatus) {});
            releaseChannel(irChannel);
            return krRx;
        }
    }

    // --- Step 6: IRM alloc IT channel --------------------------------------
    const uint8_t itChannel = claimLowestFreeChannel(static_cast<uint8_t>(irChannel + 1), "IT/iPCR");
    if (itChannel == kInvalidIsochChannel) {
        (void)isoch_.StopReceive();
        cmpClient_->DisconnectOPCR(0, [](ASFW::CMP::CMPStatus) {});
        releaseChannel(irChannel);
        return kIOReturnNoResources;
    }

    // --- Step 7: PreparePlaybackPath — CMD B + CMD C -----------------------
    // Apple emits these inside SetUpOutputConnection → SetSampleRate,
    // BEFORE the iPCR CMP lock and BEFORE the IT DMA goes live. Non-fatal
    // on failure: post-fix-53 tests show the device still ACKs the iPCR
    // lock and the downstream failure mode is device-silent regardless.
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

    // --- Step 8: iPCR CMP connect (RMW×2) ----------------------------------
    {
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
            (void)isoch_.StopReceive();
            cmpClient_->DisconnectOPCR(0, [](ASFW::CMP::CMPStatus) {});
            releaseChannel(itChannel);
            releaseChannel(irChannel);
            return kIOReturnError;
        }
    }

    // --- Step 9: StartTransmit — IT DMA goes live on the bus ---------------
    {
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
                                                        0);
        if (krTx != kIOReturnSuccess) {
            ASFW_LOG_ERROR(Audio,
                           "AVCAudioBackend: StartTransmit failed GUID=0x%016llx kr=0x%x",
                           guid,
                           krTx);
            cmpClient_->DisconnectIPCR(0, [](ASFW::CMP::CMPStatus) {});
            (void)isoch_.StopReceive();
            cmpClient_->DisconnectOPCR(0, [](ASFW::CMP::CMPStatus) {});
            releaseChannel(itChannel);
            releaseChannel(irChannel);
            return krTx;
        }
    }

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

    return kIOReturnSuccess;
}

// ---------------------------------------------------------------------------
// TearDownPipeline — detach-time reverse of BringUpPipeline.
// ---------------------------------------------------------------------------
void AVCAudioBackend::TearDownPipeline(uint64_t guid) noexcept {
    uint8_t irChannel = kInvalidIsochChannel;
    uint8_t itChannel = kInvalidIsochChannel;
    bool wasLive = false;

    if (lock_) {
        IOLockLock(lock_);
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

    // Reverse order: iPCR disconnect → IT stop → oPCR disconnect → IR stop
    {
        std::atomic<bool> done{false};
        std::atomic<ASFW::CMP::CMPStatus> status{ASFW::CMP::CMPStatus::Failed};
        cmpClient_->DisconnectIPCR(0, [&done, &status](ASFW::CMP::CMPStatus s) {
            status.store(s, std::memory_order_release);
            done.store(true, std::memory_order_release);
        });
        (void)WaitForCMP(done, status, kCmpCompletionTimeoutMs);
    }

    (void)isoch_.StopTransmit();

    {
        std::atomic<bool> done{false};
        std::atomic<ASFW::CMP::CMPStatus> status{ASFW::CMP::CMPStatus::Failed};
        cmpClient_->DisconnectOPCR(0, [&done, &status](ASFW::CMP::CMPStatus s) {
            status.store(s, std::memory_order_release);
            done.store(true, std::memory_order_release);
        });
        (void)WaitForCMP(done, status, kCmpCompletionTimeoutMs);
    }

    (void)isoch_.StopReceive();

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

        releaseChannel(itChannel);
        releaseChannel(irChannel);
    }

    ASFW_LOG(Audio, "AVCAudioBackend: Pipeline torn down GUID=0x%016llx", guid);
}

} // namespace ASFW::Audio
