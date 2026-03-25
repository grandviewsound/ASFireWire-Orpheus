// IsochAudioTxPipeline.cpp

#include "IsochAudioTxPipeline.hpp"

#include "../Encoding/TimingUtils.hpp"

namespace ASFW::Isoch {

void IsochAudioTxPipeline::SetSharedTxQueue(void* base, uint64_t bytes) noexcept {
    if (!base || bytes == 0) {
        ASFW_LOG(Isoch, "IT: SetSharedTxQueue - invalid parameters");
        return;
    }

    if (sharedTxQueue_.Attach(base, bytes)) {
        // Consumer-owned flush only: drop stale backlog on (re)attach.
        sharedTxQueue_.ConsumerDropQueuedFrames();
        ASFW_LOG(Isoch, "IT: Shared TX queue attached - capacity=%u frames",
                 sharedTxQueue_.CapacityFrames());
    } else {
        ASFW_LOG(Isoch, "IT: Failed to attach shared TX queue - invalid header?");
    }
}

uint32_t IsochAudioTxPipeline::SharedTxFillLevelFrames() const noexcept {
    if (!sharedTxQueue_.IsValid()) return 0;
    return sharedTxQueue_.FillLevelFrames();
}

uint32_t IsochAudioTxPipeline::SharedTxCapacityFrames() const noexcept {
    if (!sharedTxQueue_.IsValid()) return 0;
    return sharedTxQueue_.CapacityFrames();
}

void IsochAudioTxPipeline::SetExternalSyncBridge(Core::ExternalSyncBridge* bridge) noexcept {
    externalSyncBridge_ = bridge;
    externalSyncDiscipline_.Reset();
}

void IsochAudioTxPipeline::SetZeroCopyOutputBuffer(void* base, uint64_t bytes, uint32_t frameCapacity) noexcept {
    if (!base || bytes == 0 || frameCapacity == 0) {
        zeroCopyAudioBase_ = nullptr;
        zeroCopyAudioBytes_ = 0;
        zeroCopyFrameCapacity_ = 0;
        zeroCopyEnabled_ = false;
        assembler_.setZeroCopySource(nullptr, 0);

        if (base || bytes || frameCapacity) {
            ASFW_LOG(Isoch, "IT: SetZeroCopyOutputBuffer - invalid parameters");
        } else {
            ASFW_LOG(Isoch, "IT: ZERO-COPY disabled; using shared TX queue");
        }
        return;
    }

    zeroCopyAudioBase_ = base;
    zeroCopyAudioBytes_ = bytes;
    zeroCopyFrameCapacity_ = frameCapacity;
    zeroCopyEnabled_ = true;

    assembler_.setZeroCopySource(reinterpret_cast<const int32_t*>(base), frameCapacity);

    ASFW_LOG(Isoch, "IT: ✅ ZERO-COPY enabled! AudioBuffer base=%p bytes=%llu frames=%u assembler=%s",
             base, bytes, frameCapacity,
             assembler_.isZeroCopyEnabled() ? "ENABLED" : "fallback");
}

kern_return_t IsochAudioTxPipeline::Configure(uint8_t sid, uint32_t streamModeRaw, uint32_t requestedChannels) noexcept {
    if (!sharedTxQueue_.IsValid()) {
        ASFW_LOG(Isoch, "IT: Configure failed - shared TX queue missing");
        return kIOReturnNotReady;
    }

    const uint32_t queueChannels = sharedTxQueue_.Channels();
    if (queueChannels == 0 || queueChannels > Encoding::kMaxSupportedChannels) {
        ASFW_LOG(Isoch, "IT: Configure failed - invalid queueChannels=%u", queueChannels);
        return kIOReturnBadArgument;
    }
    if (requestedChannels != 0 && requestedChannels != queueChannels) {
        ASFW_LOG(Isoch, "IT: Configure failed - requestedChannels=%u queueChannels=%u mismatch",
                 requestedChannels, queueChannels);
        return kIOReturnBadArgument;
    }

    // Fix 29: BeBoB convention — 1 MIDI channel per data block on the wire.
    // Audio channels (queueChannels) drive CoreAudio/ring buffer sizing.
    // Wire DBS = audio + MIDI, used for CIP header and packet sizing.
    constexpr uint32_t kMidiChannels = 1;
    assembler_.reconfigure(queueChannels, sid, kMidiChannels);

    requestedStreamMode_ = (streamModeRaw == 1u)
        ? Encoding::StreamMode::kBlocking
        : Encoding::StreamMode::kNonBlocking;
    effectiveStreamMode_ = requestedStreamMode_;
    assembler_.setStreamMode(effectiveStreamMode_);

    ASFW_LOG(Isoch, "IT: Stream mode resolved requested=%{public}s effective=%{public}s",
             requestedStreamMode_ == Encoding::StreamMode::kBlocking ? "blocking" : "non-blocking",
             effectiveStreamMode_ == Encoding::StreamMode::kBlocking ? "blocking" : "non-blocking");

    const uint32_t framesPerDataPacket = assembler_.samplesPerDataPacket();
    const uint32_t wDbs = assembler_.wireDbs();
    const uint32_t payloadBytes = framesPerDataPacket * wDbs * sizeof(uint32_t);
    const uint32_t packetBytes = Encoding::kCIPHeaderSize + payloadBytes;
    ASFW_LOG(Isoch,
             "IT: Channel geometry resolved audioCh=%u midi=%u wireDBS=%u framesPerData=%u payloadBytes=%u packetBytes=%u",
             queueChannels, kMidiChannels, wDbs, framesPerDataPacket, payloadBytes, packetBytes);

    return kIOReturnSuccess;
}

void IsochAudioTxPipeline::ResetForStart() noexcept {
    assembler_.reset();
    externalSyncDiscipline_.Reset();

    counters_.resyncApplied.store(0, std::memory_order_relaxed);
    counters_.staleFramesDropped.store(0, std::memory_order_relaxed);
    counters_.legacyPumpMovedFrames.store(0, std::memory_order_relaxed);
    counters_.legacyPumpSkipped.store(0, std::memory_order_relaxed);
    counters_.exitZeroRefill.store(0, std::memory_order_relaxed);
    counters_.underrunSilencedPackets.store(0, std::memory_order_relaxed);
    counters_.audioInjectCursorResets.store(0, std::memory_order_relaxed);
    counters_.audioInjectMissedPackets.store(0, std::memory_order_relaxed);
    counters_.rbLowEvents.store(0, std::memory_order_relaxed);
    counters_.txqLowEvents.store(0, std::memory_order_relaxed);

    fillLevelAlert_ = {};

    adaptiveFill_.baseTarget = Config::kTxBufferProfile.legacyRbTargetFrames;
    adaptiveFill_.currentTarget = adaptiveFill_.baseTarget;
    adaptiveFill_.maxTarget = adaptiveFill_.baseTarget * 4;
    adaptiveFill_.underrunsInWindow = 0;
    adaptiveFill_.windowTickCount = 0;
    adaptiveFill_.cleanWindows = 0;
    adaptiveFill_.lastCombinedUnderruns = 0;

    audioWriteIndex_ = 0;

    dbcTracker_.lastDbc = 0;
    dbcTracker_.lastDataBlockCount = 0;
    dbcTracker_.firstPacket = true;
    dbcTracker_.discontinuityCount.store(0, std::memory_order_relaxed);

    // SYT generator (cycle-based, Linux approach). TODO: derive rate from stream formats.
    sytGenerator_.initialize(48000.0, assembler_.samplesPerDataPacket());
    sytGenerator_.reset();
    cycleTrackingValid_ = false;
}

void IsochAudioTxPipeline::PrePrimeFromSharedQueue() noexcept {
    if (!sharedTxQueue_.IsValid() || zeroCopyEnabled_) {
        if (zeroCopyEnabled_) {
            ASFW_LOG(Isoch, "IT: Pre-prime skipped (ZERO-COPY mode)");
        }
        return;
    }

    const uint32_t fillBefore = sharedTxQueue_.FillLevelFrames();
    const uint32_t startupPrimeLimitFrames = Config::kTxBufferProfile.startupPrimeLimitFrames;
    uint32_t remainingPrimeFrames = startupPrimeLimitFrames;
    ASFW_LOG(Isoch, "IT: Pre-prime transfer - shared queue has %u frames (limit=%u)",
             fillBefore, startupPrimeLimitFrames);

    constexpr uint32_t kTransferChunk = Config::kTransferChunkFrames;
    int32_t transferBuf[kTransferChunk * Encoding::kMaxSupportedChannels];
    uint32_t totalTransferred = 0;
    uint32_t chunkCount = 0;
    bool primeLimitHit = false;

    while (sharedTxQueue_.FillLevelFrames() > 0) {
        if (startupPrimeLimitFrames != 0 && remainingPrimeFrames == 0) {
            primeLimitHit = true;
            break;
        }

        uint32_t toRead = sharedTxQueue_.FillLevelFrames();
        if (toRead > kTransferChunk) toRead = kTransferChunk;
        if (startupPrimeLimitFrames != 0 && toRead > remainingPrimeFrames) {
            toRead = remainingPrimeFrames;
        }

        const uint32_t read = sharedTxQueue_.Read(transferBuf, toRead);
        if (read == 0) break;

        if (chunkCount < 3) {
            ASFW_LOG(Isoch, "IT: SharedQ chunk[%u] read=%u samples=[%08x,%08x,%08x,%08x]",
                     chunkCount, read,
                     static_cast<uint32_t>(transferBuf[0]),
                     static_cast<uint32_t>(transferBuf[1]),
                     static_cast<uint32_t>(transferBuf[2]),
                     static_cast<uint32_t>(transferBuf[3]));
        }
        chunkCount++;

        const uint32_t written = assembler_.ringBuffer().write(transferBuf, read);
        totalTransferred += written;
        if (startupPrimeLimitFrames != 0) {
            remainingPrimeFrames = (written >= remainingPrimeFrames) ? 0 : (remainingPrimeFrames - written);
        }

        if (written < read) break;
    }

    ASFW_LOG(Isoch, "IT: Pre-prime transferred %u frames to assembler (fill=%u limit=%u hit=%s)",
             totalTransferred,
             assembler_.bufferFillLevel(),
             startupPrimeLimitFrames,
             primeLimitHit ? "YES" : "NO");
}

// Fix #24: Diagnostic counter for OnRefillTickPreHW
static uint64_t sRefillTickCount = 0;

void IsochAudioTxPipeline::OnRefillTickPreHW() noexcept {
    if (sharedTxQueue_.IsValid() && sharedTxQueue_.ConsumerApplyPendingResync()) {
        counters_.resyncApplied.fetch_add(1, std::memory_order_relaxed);
    }

    // Legacy (non-zero-copy) path: keep assembler ring near a target fill.
    if (!zeroCopyEnabled_ && sharedTxQueue_.IsValid()) {
        ++sRefillTickCount;
        const uint32_t targetRbFillFrames = adaptiveFill_.currentTarget;
        constexpr uint32_t kMaxRbFillFrames = Config::kTxBufferProfile.legacyRbMaxFrames;
        constexpr uint32_t kTransferChunkFrames = Config::kTransferChunkFrames;
        constexpr uint32_t kMaxChunksPerRefill = Config::kTxBufferProfile.legacyMaxChunksPerRefill;

        const uint32_t rbFill = assembler_.bufferFillLevel();
        uint32_t pumpedFrames = 0;
        bool skipped = true;

        if (rbFill < targetRbFillFrames) {
            skipped = false;
            uint32_t want = targetRbFillFrames - rbFill;
            int32_t transferBuf[kTransferChunkFrames * Encoding::kMaxSupportedChannels];
            uint32_t chunks = 0;

            while (want > 0 && chunks < kMaxChunksPerRefill) {
                const uint32_t qFill = sharedTxQueue_.FillLevelFrames();
                if (qFill == 0) break;

                const uint32_t rbSpace = assembler_.ringBuffer().availableSpace();
                if (rbSpace == 0) break;

                uint32_t toRead = want;
                if (toRead > qFill) toRead = qFill;
                if (toRead > rbSpace) toRead = rbSpace;
                if (toRead > kTransferChunkFrames) toRead = kTransferChunkFrames;

                const uint32_t read = sharedTxQueue_.Read(transferBuf, toRead);
                if (read == 0) break;

                const uint32_t written = assembler_.ringBuffer().write(transferBuf, read);
                pumpedFrames += written;
                if (written < read) break;

                want -= written;
                ++chunks;

                if (assembler_.bufferFillLevel() >= kMaxRbFillFrames) break;
            }
        }

        if (skipped) {
            counters_.legacyPumpSkipped.fetch_add(1, std::memory_order_relaxed);
        } else {
            counters_.legacyPumpMovedFrames.fetch_add(pumpedFrames, std::memory_order_relaxed);
        }

        // Fix #24: Periodic pump diagnostic (every ~8000 ticks ≈ every 1s)
        if (sRefillTickCount % 8000 == 1) {
            ASFW_LOG(Isoch,
                     "Pump[%llu]: rbFill=%u target=%u txFill=%u pumped=%u skipped=%{public}s",
                     sRefillTickCount,
                     assembler_.bufferFillLevel(),
                     targetRbFillFrames,
                     sharedTxQueue_.FillLevelFrames(),
                     pumpedFrames,
                     skipped ? "YES" : "NO");
        }

        // Fill level threshold alerts (with hysteresis) - non-zero-copy only
        {
            const uint32_t rbCap = assembler_.ringBuffer().capacity();
            const uint32_t rbFillNow = assembler_.bufferFillLevel();
            const uint32_t rbLowThresh = rbCap / 20;     // 5%
            const uint32_t rbRecoverThresh = rbCap / 10;  // 10%

            if (!fillLevelAlert_.rbLow && rbFillNow < rbLowThresh) {
                fillLevelAlert_.rbLow = true;
                counters_.rbLowEvents.fetch_add(1, std::memory_order_relaxed);
            } else if (fillLevelAlert_.rbLow && rbFillNow >= rbRecoverThresh) {
                fillLevelAlert_.rbLow = false;
            }

            const uint32_t txqFill = sharedTxQueue_.FillLevelFrames();
            const uint32_t txqCap = sharedTxQueue_.CapacityFrames();
            const uint32_t txqLowThresh = txqCap / 20;     // 5%
            const uint32_t txqRecoverThresh = txqCap / 10;  // 10%

            if (!fillLevelAlert_.txqLow && txqFill < txqLowThresh) {
                fillLevelAlert_.txqLow = true;
                counters_.txqLowEvents.fetch_add(1, std::memory_order_relaxed);
            } else if (fillLevelAlert_.txqLow && txqFill >= txqRecoverThresh) {
                fillLevelAlert_.txqLow = false;
            }
        }
    }
}

void IsochAudioTxPipeline::OnPollTick1ms() noexcept {
    // Adaptive fill (1-second windows, non-zero-copy only)
    if (!zeroCopyEnabled_ && sharedTxQueue_.IsValid()) {
        ++adaptiveFill_.windowTickCount;

        const uint64_t curZeroRefills = counters_.exitZeroRefill.load(std::memory_order_relaxed);
        const uint64_t curAssemblerUnderruns = assembler_.underrunDiag().underrunCount.load(std::memory_order_relaxed);
        const uint64_t combinedUnderruns = curZeroRefills + curAssemblerUnderruns;
        if (combinedUnderruns > adaptiveFill_.lastCombinedUnderruns) {
            adaptiveFill_.underrunsInWindow += static_cast<uint32_t>(combinedUnderruns - adaptiveFill_.lastCombinedUnderruns);
            adaptiveFill_.lastCombinedUnderruns = combinedUnderruns;
        }

        if (adaptiveFill_.windowTickCount >= 1000) {
            if (adaptiveFill_.underrunsInWindow >= 3) {
                uint32_t newTarget = adaptiveFill_.currentTarget + 128;
                if (newTarget > adaptiveFill_.maxTarget) newTarget = adaptiveFill_.maxTarget;
                if (newTarget != adaptiveFill_.currentTarget) {
                    ASFW_LOG(Isoch, "IT: ADAPTIVE FILL ESCALATE %u -> %u (underruns=%u in window)",
                             adaptiveFill_.currentTarget, newTarget, adaptiveFill_.underrunsInWindow);
                    adaptiveFill_.currentTarget = newTarget;
                }
                adaptiveFill_.cleanWindows = 0;
            } else if (adaptiveFill_.underrunsInWindow == 0) {
                ++adaptiveFill_.cleanWindows;
                if (adaptiveFill_.cleanWindows >= 10 && adaptiveFill_.currentTarget > adaptiveFill_.baseTarget) {
                    uint32_t newTarget = adaptiveFill_.currentTarget;
                    newTarget = (newTarget > adaptiveFill_.baseTarget + 64)
                        ? newTarget - 64
                        : adaptiveFill_.baseTarget;
                    if (newTarget != adaptiveFill_.currentTarget) {
                        ASFW_LOG(Isoch, "IT: ADAPTIVE FILL DECAY %u -> %u (cleanWindows=%u)",
                                 adaptiveFill_.currentTarget, newTarget, adaptiveFill_.cleanWindows);
                        adaptiveFill_.currentTarget = newTarget;
                    }
                }
            } else {
                adaptiveFill_.cleanWindows = 0;
            }

            adaptiveFill_.windowTickCount = 0;
            adaptiveFill_.underrunsInWindow = 0;
        }
    }
}

uint16_t IsochAudioTxPipeline::ComputeDataSyt(uint32_t transmitCycle) noexcept {
    if (!sytGenerator_.isValid() || !cycleTrackingValid_) {
        return Encoding::SYTGenerator::kNoInfo;
    }

    const uint16_t txSyt = sytGenerator_.computeDataSYT(transmitCycle);
    MaybeApplyExternalSyncDiscipline(txSyt);
    return txSyt;
}

void IsochAudioTxPipeline::MaybeApplyExternalSyncDiscipline(uint16_t txSyt) noexcept {
    bool enabled = false;
    uint16_t rxSyt = Core::ExternalSyncBridge::kNoInfoSyt;

    if (externalSyncBridge_) {
        const bool active = externalSyncBridge_->active.load(std::memory_order_acquire);
        const bool established = externalSyncBridge_->clockEstablished.load(std::memory_order_acquire);
        const uint64_t lastUpdateTicks =
            externalSyncBridge_->lastUpdateHostTicks.load(std::memory_order_acquire);

        uint64_t staleThresholdTicks = ASFW::Timing::nanosToHostTicks(100'000'000ULL);
        if (staleThresholdTicks == 0 && ASFW::Timing::initializeHostTimebase()) {
            staleThresholdTicks = ASFW::Timing::nanosToHostTicks(100'000'000ULL);
        }

        if (active && established && staleThresholdTicks != 0 && lastUpdateTicks != 0) {
            const uint64_t nowTicks = mach_absolute_time();
            if (nowTicks >= lastUpdateTicks &&
                (nowTicks - lastUpdateTicks) <= staleThresholdTicks) {
                const uint32_t packed = externalSyncBridge_->lastPackedRx.load(std::memory_order_acquire);
                const uint16_t candidateSyt = Core::ExternalSyncBridge::UnpackSYT(packed);
                const uint8_t candidateFdf = Core::ExternalSyncBridge::UnpackFDF(packed);
                if (candidateSyt != Core::ExternalSyncBridge::kNoInfoSyt &&
                    candidateFdf == Core::ExternalSyncBridge::kFdf48k) {
                    enabled = true;
                    rxSyt = candidateSyt;
                }
            }
        }
    }

    const auto disciplineResult = externalSyncDiscipline_.Update(enabled, txSyt, rxSyt);
    if (enabled && disciplineResult.correctionTicks != 0) {
        sytGenerator_.nudgeOffsetTicks(disciplineResult.correctionTicks);
    }
}

Tx::IsochTxPacket IsochAudioTxPipeline::NextSilentPacket(uint32_t transmitCycle) noexcept {
    uint16_t syt = Encoding::SYTGenerator::kNoInfo;
    const bool willBeData = assembler_.nextIsData();
    if (willBeData) {
        syt = ComputeDataSyt(transmitCycle);
    }

    // silent=true: cadence/DBC/CIP advance, audio payload is valid AM824 silence.
    auto pkt = assembler_.assembleNext(syt, /*silent=*/true);

    // Producer-side DBC continuity validation (ignore NO-DATA).
    if (pkt.isData) {
        const uint8_t samplesInPkt = static_cast<uint8_t>(assembler_.samplesPerDataPacket());
        if (!dbcTracker_.firstPacket) {
            const uint8_t expectedDbc = static_cast<uint8_t>(dbcTracker_.lastDbc + dbcTracker_.lastDataBlockCount);
            if (pkt.dbc != expectedDbc) {
                dbcTracker_.discontinuityCount.fetch_add(1, std::memory_order_relaxed);
            }
        }
        dbcTracker_.lastDbc = pkt.dbc;
        dbcTracker_.lastDataBlockCount = samplesInPkt;
        dbcTracker_.firstPacket = false;
    }

    Tx::IsochTxPacket out{};
    out.words = reinterpret_cast<const uint32_t*>(pkt.data);
    out.sizeBytes = pkt.size;
    out.isData = pkt.isData;
    out.dbc = pkt.dbc;
    return out;
}

// Fix #24: Diagnostic counters for InjectNearHw
static uint64_t sInjectCallCount = 0;
static uint64_t sInjectPacketsWritten = 0;
static uint64_t sInjectNonZeroPackets = 0;
static int32_t  sInjectPeakSample = 0;

void IsochAudioTxPipeline::InjectNearHw(uint32_t hwPacketIndex, Tx::IsochTxDescriptorSlab& slab) noexcept {
    constexpr uint32_t numPackets = Tx::Layout::kNumPackets;

    const bool zeroCopySync = zeroCopyEnabled_ && sharedTxQueue_.IsValid() && zeroCopyFrameCapacity_ > 0;

    // Target: write real audio up to kAudioWriteAhead packets ahead of HW
    const uint32_t audioTarget = (hwPacketIndex + Tx::Layout::kAudioWriteAhead) % numPackets;

    // If audio cursor fell behind HW (scheduling stall), reset to HW position.
    const uint32_t distBehind = (hwPacketIndex + numPackets - audioWriteIndex_) % numPackets;
    if (distBehind > 0 && distBehind < numPackets / 2) {
        counters_.audioInjectCursorResets.fetch_add(1, std::memory_order_relaxed);
        counters_.audioInjectMissedPackets.fetch_add(distBehind, std::memory_order_relaxed);
        audioWriteIndex_ = hwPacketIndex;
    }

    uint32_t toInject = (audioTarget + numPackets - audioWriteIndex_) % numPackets;
    if (toInject > Tx::Layout::kAudioWriteAhead) toInject = Tx::Layout::kAudioWriteAhead;

    if (toInject == 0) {
        return;
    }

    ++sInjectCallCount;

    const uint32_t framesPerPacket = assembler_.samplesPerDataPacket();
    const uint32_t channels = assembler_.channelCount();

    // Fix #24: Capture first packet's samples for diagnostic
    int32_t diagSample0 = 0, diagSample1 = 0;
    uint32_t diagAM824_0 = 0, diagAM824_1 = 0;
    uint32_t diagFramesRead = 0;
    uint32_t dataPacketsThisCall = 0;
    bool hasNonZeroThisCall = false;

    for (uint32_t i = 0; i < toInject; ++i) {
        const uint32_t idx = (audioWriteIndex_ + i) % numPackets;

        const uint32_t descBase = idx * Tx::Layout::kBlocksPerPacket;
        auto* lastDesc = slab.GetDescriptorPtr(descBase + 2);
        const uint16_t reqCount = static_cast<uint16_t>(lastDesc->control & 0xFFFF);
        const bool isData = (reqCount > Encoding::kCIPHeaderSize);
        if (!isData) continue;

        int32_t samples[Encoding::kSamplesPerDataPacket * Encoding::kMaxSupportedChannels];
        uint32_t framesRead = 0;

        if (zeroCopySync) {
            uint32_t zeroCopyFillBefore = sharedTxQueue_.FillLevelFrames();

            // Drop stale backlog if queue lag exceeds buffer capacity
            if (zeroCopyFillBefore > zeroCopyFrameCapacity_) {
                const uint32_t drop = zeroCopyFillBefore - zeroCopyFrameCapacity_;
                const uint32_t dropped = sharedTxQueue_.ConsumeFrames(drop);
                counters_.staleFramesDropped.fetch_add(dropped, std::memory_order_relaxed);
                zeroCopyFillBefore -= dropped;
            }

            const uint32_t readAbs = sharedTxQueue_.ReadIndexFrames();
            const uint32_t phase = sharedTxQueue_.ZeroCopyPhaseFrames() % zeroCopyFrameCapacity_;
            assembler_.setZeroCopyReadPosition((readAbs + phase) % zeroCopyFrameCapacity_);

            if (assembler_.isZeroCopyEnabled() && zeroCopyAudioBase_) {
                const int32_t* zcBase = reinterpret_cast<const int32_t*>(zeroCopyAudioBase_);
                const uint32_t zcPos = assembler_.zeroCopyReadPosition();
                for (uint32_t f = 0; f < framesPerPacket; ++f) {
                    const uint32_t frameIdx = (zcPos + f) % zeroCopyFrameCapacity_;
                    const uint32_t sampleIdx = frameIdx * channels;
                    for (uint32_t ch = 0; ch < channels; ++ch) {
                        samples[f * channels + ch] = zcBase[sampleIdx + ch];
                    }
                }
                assembler_.setZeroCopyReadPosition((zcPos + framesPerPacket) % zeroCopyFrameCapacity_);
                framesRead = framesPerPacket;
            } else {
                framesRead = assembler_.ringBuffer().read(samples, framesPerPacket);
            }

            const uint32_t consumed = sharedTxQueue_.ConsumeFrames(framesPerPacket);
            if (consumed < framesPerPacket || zeroCopyFillBefore < framesPerPacket) {
                counters_.exitZeroRefill.fetch_add(1, std::memory_order_relaxed);
                counters_.underrunSilencedPackets.fetch_add(1, std::memory_order_relaxed);
                assembler_.recordUnderrun(zeroCopyFillBefore, framesPerPacket,
                                          consumed, 0, 0);
                continue; // leave silence in place
            }
        } else {
            framesRead = assembler_.ringBuffer().read(samples, framesPerPacket);
        }

        if (framesRead < framesPerPacket) {
            const size_t samplesRead = framesRead * channels;
            const size_t totalSamples = framesPerPacket * channels;
            std::memset(&samples[samplesRead], 0,
                        (totalSamples - samplesRead) * sizeof(int32_t));
        }

        uint8_t* payloadVirt = slab.PayloadPtr(idx);
        if (!payloadVirt) {
            continue;
        }
        uint32_t* quadlets = reinterpret_cast<uint32_t*>(payloadVirt + Encoding::kCIPHeaderSize);

        // Fix 29: Encode per data block — audio quadlets then MIDI no-data.
        // Data block layout: [ch0][ch1]...[ch(N-1)][MIDI]
        const uint32_t dbs = assembler_.wireDbs();
        const uint32_t midi = assembler_.midiChannels();
        const uint32_t midiNoData = Encoding::AM824Encoder::encodeMidiNoData();

        for (uint32_t f = 0; f < framesPerPacket; ++f) {
            for (uint32_t ch = 0; ch < channels; ++ch) {
                quadlets[f * dbs + ch] = Encoding::AM824Encoder::encode(samples[f * channels + ch]);
            }
            for (uint32_t m = 0; m < midi; ++m) {
                quadlets[f * dbs + channels + m] = midiNoData;
            }
        }

        // Fix #24: Track diagnostics
        ++dataPacketsThisCall;
        ++sInjectPacketsWritten;
        if (dataPacketsThisCall == 1) {
            diagSample0 = samples[0];
            diagSample1 = (channels > 1) ? samples[1] : 0;
            diagAM824_0 = quadlets[0];
            diagAM824_1 = (channels > 1) ? quadlets[1] : 0;
            diagFramesRead = framesRead;
        }
        for (uint32_t s = 0; s < framesPerPacket * channels; ++s) {
            if (samples[s] != 0) {
                hasNonZeroThisCall = true;
                int32_t abs_s = (samples[s] < 0) ? -samples[s] : samples[s];
                if (abs_s > sInjectPeakSample) sInjectPeakSample = abs_s;
            }
        }
    }

    if (hasNonZeroThisCall) sInjectNonZeroPackets += dataPacketsThisCall;

    audioWriteIndex_ = audioTarget;

    // Fix #27: Use IoBarrier (dsb sy) instead of WriteBarrier (dmb ish) to ensure
    // audio payload writes are visible to the PCIe FireWire DMA engine.
    // dmb ish only orders within CPU cores; dsb sy commits to the full system domain.
    std::atomic_thread_fence(std::memory_order_release);
    ASFW::Driver::IoBarrier();

    // Fix #24: Periodic diagnostic log (every ~4000 calls ≈ every 0.5s)
    if (sInjectCallCount % 4000 == 1) {
        const uint32_t rbFill = assembler_.bufferFillLevel();
        const uint32_t txFill = sharedTxQueue_.IsValid() ? sharedTxQueue_.FillLevelFrames() : 0;
        ASFW_LOG(Isoch,
                 "InjectNearHw[%llu]: toInject=%u data=%u framesRead=%u rbFill=%u txFill=%u "
                 "nonZeroPkts=%llu peak=0x%08x | pcm=[%08x,%08x] am824=[%08x,%08x]",
                 sInjectCallCount,
                 toInject,
                 dataPacketsThisCall,
                 diagFramesRead,
                 rbFill,
                 txFill,
                 sInjectNonZeroPackets,
                 static_cast<uint32_t>(sInjectPeakSample),
                 static_cast<uint32_t>(diagSample0),
                 static_cast<uint32_t>(diagSample1),
                 diagAM824_0,
                 diagAM824_1);

        // Dump CIP headers + first AM824 quadlet from one DATA packet's DMA buffer
        // to verify actual wire-level payload.
        for (uint32_t i = 0; i < toInject; ++i) {
            const uint32_t idx = (audioWriteIndex_ + numPackets - toInject + i) % numPackets;
            const uint32_t descBase = idx * Tx::Layout::kBlocksPerPacket;
            auto* olDesc = slab.GetDescriptorPtr(descBase + 2);
            const uint16_t olReq = static_cast<uint16_t>(olDesc->control & 0xFFFF);
            if (olReq <= Encoding::kCIPHeaderSize) continue; // skip NO-DATA
            uint8_t* pv = slab.PayloadPtr(idx);
            if (!pv) continue;
            const uint32_t* q = reinterpret_cast<const uint32_t*>(pv);
            // Also read OMI descriptor
            auto* omiDesc = reinterpret_cast<ASFW::Async::HW::OHCIDescriptorImmediate*>(slab.GetDescriptorPtr(descBase));
            ASFW_LOG(Isoch,
                     "DMA-DUMP pkt=%u: OMI ctrl=0x%08x imm0=0x%08x imm1=0x%08x | "
                     "OL ctrl=0x%08x req=%u | CIP Q0=0x%08x Q1=0x%08x AM824[0]=0x%08x AM824[1]=0x%08x",
                     idx,
                     omiDesc->common.control,
                     omiDesc->immediateData[0],
                     omiDesc->immediateData[1],
                     olDesc->control,
                     olReq,
                     q[0], q[1], q[2], q[3]);
            break; // one DATA packet is enough
        }
    }
}

} // namespace ASFW::Isoch
