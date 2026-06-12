// IsochAudioRxPipeline.cpp

#include "IsochAudioRxPipeline.hpp"

namespace ASFW::Isoch::Rx {

void IsochAudioRxPipeline::ConfigureFor48k() noexcept {
    cycleCorr_ = {};
    cycleCorr_.sampleRate = 48000.0;
    (void)ASFW::Timing::initializeHostTimebase();
}

void IsochAudioRxPipeline::OnStart() noexcept {
    streamProcessor_.Reset();
    packetParser_.Reset();
    ztsClock_.Reset();
    cycleCorr_ = CycleTimeCorrelation{};

    if (externalSyncBridge_) {
        externalSyncBridge_->Reset();
        externalSyncBridge_->active.store(true, std::memory_order_release);
    }
    externalSyncClockState_.Reset();
}

void IsochAudioRxPipeline::OnStop() noexcept {
    if (::ASFW::LogConfig::Shared().GetIsochVerbosity() >= 3) {
        streamProcessor_.LogStatistics();
    }

    if (externalSyncBridge_) {
        externalSyncBridge_->Reset();
    }
    externalSyncClockState_.Reset();
}

void IsochAudioRxPipeline::OnByteStream(const uint8_t* bytes, size_t length) noexcept {
    packetParser_.OnBytes(bytes, length, [this](const uint8_t* pkt, size_t pktLen) {
        OnPacket(pkt, pktLen);
    });
}

void IsochAudioRxPipeline::OnPacket(const uint8_t* payload, size_t length) noexcept {
    constexpr bool kNullProcessing = false;

    if constexpr (kNullProcessing) {
        streamProcessor_.RecordRawPacket(length);
        return;
    }

    const auto summary = streamProcessor_.ProcessPacket(payload, length);

    if (!externalSyncBridge_) {
        return;
    }

    if (!summary.hasValidCip) {
        externalSyncClockState_.Reset();
        return;
    }

    uint32_t updateSeq = 0;
    const uint64_t nowTicks = mach_absolute_time();
    const bool establishTransition = externalSyncClockState_.ObserveSample(*externalSyncBridge_,
                                                                          nowTicks,
                                                                          summary.syt,
                                                                          summary.fdf,
                                                                          summary.dbs,
                                                                          &updateSeq);
    if (establishTransition) {
        ASFW_LOG(Isoch, "IR SYT CLOCK ESTABLISHED syt=0x%04x fdf=0x%02x dbs=%u seq=%u",
                 summary.syt, summary.fdf, summary.dbs, updateSeq);
        externalSyncBridge_->clockEstablished.store(true, std::memory_order_release);
    }
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void IsochAudioRxPipeline::OnPollEnd(Driver::HardwareInterface& hw,
                                     uint32_t packetsProcessed, // NOLINT(bugprone-easily-swappable-parameters)
                                     uint64_t pollStartMachTicks) noexcept {
    if (packetsProcessed > 0) {
        const uint64_t end = mach_absolute_time();
        const uint64_t deltaTicks = end - pollStartMachTicks;
        const uint64_t deltaUs = ASFW::Diagnostics::MachTicksToMicroseconds(deltaTicks);
        streamProcessor_.RecordPollLatency(deltaUs, packetsProcessed);
    }

    // Read the (cycle-timer, host-uptime) pair once per poll, as atomically as the
    // hardware allows, and reuse it for both the zero-timestamp anchor (every poll)
    // and the rate estimate (every kCycleCorrPollInterval polls).
    auto [ct, up] = hw.ReadCycleTimeAndUpTime();

    // Hardware zero-timestamp anchor (Apple "use the time passed in the hardware
    // interrupt"): the device is bus cycle-master, so the FireWire cycle timer IS
    // its sample clock. Advance the monotonic device sample clock from the cycle
    // timer and publish (sampleTime, hostTicks) so the audio driver anchors
    // CoreAudio's zero timestamp to real hardware time rather than accumulating a
    // free-running software timer. Only advance while actually streaming.
    if (packetsProcessed > 0) {
        const uint64_t sampleTime = ztsClock_.Advance(ct, cycleCorr_.sampleRate);
        rxSharedQueue_.PublishHwZeroTimestampAnchor(sampleTime, up);
    }

    // Cycle-time rate estimation: slave the host audio clock to the device's
    // FireWire cycle clock so CoreAudio produces samples at *exactly* the device
    // rate (otherwise the host free-runs the nominal 48000 Hz software timer and
    // drifts against the device crystal → TX ring underruns → "funky" playback).
    // Capture the baseline on the very first poll, then re-estimate every
    // kCycleCorrPollInterval polls. Was 1000 with the baseline only taken on the
    // first interval boundary, so q8 stayed 0 (free-running fallback) for ~2
    // intervals after stream start; halved + baseline-on-first-poll so the lock
    // engages ~4x sooner.
    constexpr uint32_t kCycleCorrPollInterval = 500;
    cycleCorr_.pollsSinceLastUpdate++;
    if (!cycleCorr_.hasPrevious || cycleCorr_.pollsSinceLastUpdate >= kCycleCorrPollInterval) {
        if (cycleCorr_.hasPrevious) {
            const int64_t dFW = ASFW::Timing::deltaFWTimeNanos(ct, cycleCorr_.prevCycleTimer);
            const int64_t dHost = static_cast<int64_t>(ASFW::Timing::hostTicksToNanos(up))
                                - static_cast<int64_t>(ASFW::Timing::hostTicksToNanos(cycleCorr_.prevHostTicks));
            if (dFW > 0 && dHost > 0) {
                const double ratio = static_cast<double>(dHost) / static_cast<double>(dFW);
                const double nanosPerSample = ratio * (1e9 / cycleCorr_.sampleRate);
                const uint32_t q8 = static_cast<uint32_t>(nanosPerSample * 256.0 + 0.5);
                rxSharedQueue_.SetCorrHostNanosPerSampleQ8(q8);
                // Also hand the measured rate to the IT SYT generator (via the
                // shared bridge) so transmit SYT drifts at the REAL device rate
                // instead of nominal 48k. Applies on internal-clock devices too.
                if (externalSyncBridge_) {
                    externalSyncBridge_->hostNanosPerSampleQ8.store(q8, std::memory_order_release);
                }
                // Always-on (rate-limited to ~1s) so the host-clock lock is visible
                // at the default ASFWIsochVerbosity=1 — the 2026-06-06 first-audio
                // run could not confirm whether this lock ever engaged.
                ASFW_LOG_RL(Isoch, "cyclecorr/q8", 1000, OS_LOG_TYPE_DEFAULT,
                            "CycleCorr: ratio=%.6f nanosPerSample=%.1f q8=%u dFW=%lld dHost=%lld",
                            ratio, nanosPerSample, q8, dFW, dHost);
            }
        }
        cycleCorr_.prevCycleTimer = ct;
        cycleCorr_.prevHostTicks = up;
        cycleCorr_.hasPrevious = true;
        cycleCorr_.pollsSinceLastUpdate = 0;
    }

    if (externalSyncBridge_) {
        uint64_t staleTicks = ASFW::Timing::nanosToHostTicks(kExternalSyncStaleNanos);
        if (staleTicks == 0 && ASFW::Timing::initializeHostTimebase()) {
            staleTicks = ASFW::Timing::nanosToHostTicks(kExternalSyncStaleNanos);
        }
        (void)externalSyncClockState_.HandleStale(*externalSyncBridge_,
                                                  mach_absolute_time(),
                                                  staleTicks);
    }
}

void IsochAudioRxPipeline::SetSharedRxQueue(void* base, uint64_t bytes) noexcept {
    if (!base || bytes == 0) {
        (void)rxSharedQueue_.Attach(nullptr, 0);
        streamProcessor_.SetOutputSharedQueue(nullptr);
        ASFW_LOG(Isoch, "[Isoch] IR: Shared RX queue detached");
        return;
    }

    if (rxSharedQueue_.Attach(base, bytes)) {
        streamProcessor_.SetOutputSharedQueue(&rxSharedQueue_);
        ASFW_LOG(Isoch, "[Isoch] IR: Shared RX queue attached (%llu bytes)", bytes);
    } else {
        ASFW_LOG(Isoch, "[Isoch] IR: Failed to attach shared RX queue (base=%p bytes=%llu)", base, bytes);
        (void)rxSharedQueue_.Attach(nullptr, 0);
        streamProcessor_.SetOutputSharedQueue(nullptr);
    }
}

void IsochAudioRxPipeline::SetExternalSyncBridge(Core::ExternalSyncBridge* bridge) noexcept {
    externalSyncBridge_ = bridge;
    externalSyncClockState_.Reset();
    if (externalSyncBridge_) {
        externalSyncBridge_->Reset();
    }
}

} // namespace ASFW::Isoch::Rx
