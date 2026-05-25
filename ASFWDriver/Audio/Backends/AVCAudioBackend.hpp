// SPDX-License-Identifier: LGPL-3.0-or-later
// Copyright (c) 2026 ASFireWire Project
//
// AVCAudioBackend.hpp
// AV/C audio backend (Music subunit discovery) with CMP/PCR always for audio.

#pragma once

#include "IAudioBackend.hpp"

#include "../AudioNubPublisher.hpp"

#include "../../Discovery/DeviceRegistry.hpp"
#include "../../Hardware/HardwareInterface.hpp"
#include "../../Isoch/IsochService.hpp"
#include "../../IRM/IRMClient.hpp"
#include "../../Protocols/AVC/CMP/CMPClient.hpp"

#include <cstdint>
#include <unordered_map>

namespace ASFW::Protocols::AVC {
class IAVCDiscovery;
}

namespace ASFW::Audio {

class AVCAudioBackend final : public IAudioBackend {
public:
    AVCAudioBackend(AudioNubPublisher& publisher,
                    Discovery::DeviceRegistry& registry,
                    Driver::IsochService& isoch,
                    Driver::HardwareInterface& hardware) noexcept;
    ~AVCAudioBackend() noexcept override;

    AVCAudioBackend(const AVCAudioBackend&) = delete;
    AVCAudioBackend& operator=(const AVCAudioBackend&) = delete;

    [[nodiscard]] const char* Name() const noexcept override { return "AV/C"; }

    void SetCMPClient(ASFW::CMP::CMPClient* client) noexcept { cmpClient_ = client; }
    void SetIRMClient(ASFW::IRM::IRMClient* client) noexcept { irmClient_ = client; }
    void SetAVCDiscovery(ASFW::Protocols::AVC::IAVCDiscovery* discovery) noexcept {
        avcDiscovery_ = discovery;
    }

    void OnAudioConfigurationReady(uint64_t guid, const Model::ASFWAudioDevice& config) noexcept;
    void OnDeviceRemoved(uint64_t guid) noexcept;

    [[nodiscard]] IOReturn StartStreaming(uint64_t guid) noexcept override;
    [[nodiscard]] IOReturn StopStreaming(uint64_t guid) noexcept override;

private:
    [[nodiscard]] bool WaitForCMP(std::atomic<bool>& done,
                                  std::atomic<ASFW::CMP::CMPStatus>& status,
                                  uint32_t timeoutMs) noexcept;
    [[nodiscard]] bool WaitForIRM(std::atomic<bool>& done,
                                  std::atomic<ASFW::IRM::AllocationStatus>& status,
                                  uint32_t timeoutMs) noexcept;

    // IRM isochronous bandwidth alloc/release for the host-talker IT channel.
    // Apple reserves BANDWIDTH_AVAILABLE alongside the channel; we mirror that. Both are
    // best-effort + synchronous (WaitForIRM): a failure NEVER blocks streaming. Allocate
    // records activeItBandwidthUnits_; Release frees exactly that and zeroes it.
    void AllocateItBandwidth(uint16_t payloadQuadlets, uint8_t itChannel) noexcept;
    void ReleaseItBandwidth() noexcept;

    // Attach-time pipeline bring-up / detach-time teardown.
    //
    // Apr 13 2026: The full isoch pipeline (IRM + CMP + IT + IR + ExtFmt)
    // lives for the entire duration the device is on the bus, not for the
    // duration of a CoreAudio play session. Apple's AppleFWAudio brings
    // everything up inside initHardware and leaves it hot; Play/Stop in its
    // model is pure sample-gating on an already-running pipeline. We used
    // to do the bring-up at StartStreaming time which caused the DAC relay
    // to click on every spacebar press and the device to never settle into
    // its streaming state. See memory/dac-click-timing-insight.md.
    [[nodiscard]] IOReturn BringUpPipeline(uint64_t guid) noexcept;
    void TearDownPipeline(uint64_t guid) noexcept;

    AudioNubPublisher& publisher_;
    Discovery::DeviceRegistry& registry_;
    Driver::IsochService& isoch_;
    Driver::HardwareInterface& hardware_;

    ASFW::CMP::CMPClient* cmpClient_{nullptr};
    ASFW::IRM::IRMClient* irmClient_{nullptr};
    ASFW::Protocols::AVC::IAVCDiscovery* avcDiscovery_{nullptr};

    IOLock* lock_{nullptr};
    std::unordered_map<uint64_t, Model::ASFWAudioDevice> configByGuid_;

    // Non-zero when BringUpPipeline has succeeded and TearDownPipeline has
    // not yet run. Protected by lock_. Allows StartStreaming to short-circuit
    // as an idempotent no-op once the pipeline is live from the attach path.
    uint64_t pipelineGuid_{0};

    // Isoch channels IRM granted at BringUpPipeline time. TearDownPipeline
    // releases the same ones. 0xFF means "not allocated".
    uint8_t activeIrChannel_{0xFF};
    uint8_t activeItChannel_{0xFF};
    // IRM isochronous bandwidth allocated for the host-talker IT channel (host→device).
    // Apple allocates bandwidth (BANDWIDTH_AVAILABLE) alongside the channel; we mirror that.
    // Tracked so teardown releases exactly what was allocated. 0 = none allocated.
    uint32_t activeItBandwidthUnits_{0};
};

} // namespace ASFW::Audio
