#include "AudioClockEngine.hpp"

#include "../../Logging/LogConfig.hpp"
#include "../../Logging/Logging.hpp"

#include <DriverKit/DriverKit.h>

#include <cmath>

namespace ASFW::Isoch::Audio {
namespace detail {

void ResetZeroCopyTimeline(ZeroCopyTimelineState& timeline) {
    timeline.valid = false;
    timeline.lastSampleTime = 0;
    timeline.publishedSampleTime = 0;
    timeline.discontinuities = 0;
    timeline.phaseFrames = 0;
}

void ResetClockSync(ClockSyncState& clockSync) {
    clockSync.fillErrorIntegral = 0;
    clockSync.lastFillError = 0;
    clockSync.fractionalTicks = 0.0;
    clockSync.adjustmentCount = 0;
    clockSync.maxCorrectionPpm = 0.0;
    clockSync.saturationCount = 0;
    clockSync.wasSaturated = false;
    clockSync.driftDirection = 0;
    clockSync.monotoneDriftTicks = 0;
    clockSync.ztsAnchorPll.Reset();
    clockSync.ztsAnchorFit.Reset();
}

uint64_t RoundWithFraction(double& fractionalTicks, double currentTicksPerBuffer) {
    const double exactTicks = currentTicksPerBuffer + fractionalTicks;
    const auto roundedTicks = static_cast<uint64_t>(exactTicks);
    fractionalTicks = exactTicks - static_cast<double>(roundedTicks);
    return roundedTicks;
}

uint64_t ApplyCycleTimeClock(AudioClockEngineState& state, uint32_t q8) {
    const double nanosPerSample = q8 / 256.0;
    struct mach_timebase_info tb;
    mach_timebase_info(&tb);
    const double hostTicksPerSample = nanosPerSample * static_cast<double>(tb.denom)
                                    / static_cast<double>(tb.numer);
    state.clockSync->currentTicksPerBuffer = hostTicksPerSample * state.ioBufferPeriodFrames;
    return RoundWithFraction(state.clockSync->fractionalTicks,
                             state.clockSync->currentTicksPerBuffer);
}

uint64_t ApplyZeroCopyPllClock(AudioClockEngineState& state) {
    const uint32_t fillLevel = state.txQueueWriter->FillLevelFrames();
    const int32_t fillError = static_cast<int32_t>(fillLevel)
                            - static_cast<int32_t>(state.clockSync->targetFillLevel);

    constexpr double kMaxPpm = 100.0;
    constexpr int32_t kDeadbandFrames = 8;
    constexpr double kPpmPerFrame = 0.45;
    constexpr double kIppmPerFrameTick = 0.0008;
    constexpr int64_t kIntegralClamp = 200000;

    int32_t controlError = fillError;
    if (std::abs(controlError) <= kDeadbandFrames) {
        controlError = 0;
    }

    double ppmUnclamped = (kPpmPerFrame * controlError)
                        + (kIppmPerFrameTick * static_cast<double>(state.clockSync->fillErrorIntegral));
    if (const bool satHigh = (ppmUnclamped > kMaxPpm) && (controlError > 0),
                  satLow = (ppmUnclamped < -kMaxPpm) && (controlError < 0);
        !(satHigh || satLow)) {
        state.clockSync->fillErrorIntegral += controlError;
        if (state.clockSync->fillErrorIntegral > kIntegralClamp) {
            state.clockSync->fillErrorIntegral = kIntegralClamp;
        }
        if (state.clockSync->fillErrorIntegral < -kIntegralClamp) {
            state.clockSync->fillErrorIntegral = -kIntegralClamp;
        }
    }

    ppmUnclamped = (kPpmPerFrame * controlError)
                 + (kIppmPerFrameTick * static_cast<double>(state.clockSync->fillErrorIntegral));
    double corrPpm = ppmUnclamped;
    if (corrPpm > kMaxPpm) {
        corrPpm = kMaxPpm;
    }
    if (corrPpm < -kMaxPpm) {
        corrPpm = -kMaxPpm;
    }

    const double correction = state.clockSync->nominalTicksPerBuffer * (corrPpm / 1e6);
    state.clockSync->currentTicksPerBuffer = state.clockSync->nominalTicksPerBuffer + correction;
    state.clockSync->lastFillError = fillError;
    state.clockSync->adjustmentCount++;

    if (std::fabs(corrPpm) > state.clockSync->maxCorrectionPpm) {
        state.clockSync->maxCorrectionPpm = std::fabs(corrPpm);
    }

    const bool saturated = (std::fabs(corrPpm) >= kMaxPpm - 0.1);
    if (saturated && !state.clockSync->wasSaturated) {
        state.clockSync->saturationCount++;
        ASFW_LOG_RL(Audio,
                    "pll/sat",
                    500,
                    OS_LOG_TYPE_DEFAULT,
                    "PLL SATURATED corr=%.1f ppm fill=%u target=%u err=%d sat#=%llu",
                    corrPpm,
                    fillLevel,
                    state.clockSync->targetFillLevel,
                    fillError,
                    state.clockSync->saturationCount);
    }
    state.clockSync->wasSaturated = saturated;

    int32_t curDir = 0;
    if (controlError > 0) {
        curDir = 1;
    } else if (controlError < 0) {
        curDir = -1;
    }
    if (curDir != 0 && curDir == state.clockSync->driftDirection) {
        state.clockSync->monotoneDriftTicks++;
        if (state.clockSync->monotoneDriftTicks == 200) {
            ASFW_LOG_RL(Audio,
                        "pll/drift",
                        2000,
                        OS_LOG_TYPE_DEFAULT,
                        "PLL MONOTONE DRIFT dir=%{public}s 200+ ticks fill=%u target=%u",
                        curDir > 0 ? "fast" : "slow",
                        fillLevel,
                        state.clockSync->targetFillLevel);
        }
    } else {
        state.clockSync->driftDirection = curDir;
        state.clockSync->monotoneDriftTicks = (curDir != 0) ? 1 : 0;
    }

    return RoundWithFraction(state.clockSync->fractionalTicks,
                             state.clockSync->currentTicksPerBuffer);
}

uint64_t ApplyNominalClock(AudioClockEngineState& state, bool withLegacyTxUpdate) {
    if (withLegacyTxUpdate) {
        const uint32_t fillLevel = state.txQueueWriter->FillLevelFrames();
        const int32_t fillError = static_cast<int32_t>(fillLevel)
                                - static_cast<int32_t>(state.clockSync->targetFillLevel);
        state.clockSync->lastFillError = fillError;
        state.clockSync->fillErrorIntegral = 0;
        state.clockSync->currentTicksPerBuffer = state.clockSync->nominalTicksPerBuffer;
        state.clockSync->fractionalTicks = 0.0;
        state.clockSync->maxCorrectionPpm = 0.0;
        return static_cast<uint64_t>(state.clockSync->nominalTicksPerBuffer);
    }

    return RoundWithFraction(state.clockSync->fractionalTicks,
                             state.clockSync->currentTicksPerBuffer);
}

uint64_t ComputeHostTicksPerBuffer(AudioClockEngineState& state,
                                   uint32_t q8,
                                   bool rxPllReady) {
    if (q8 > 0) {
        return ApplyCycleTimeClock(state, q8);
    }
    if (state.zeroCopyEnabled && state.txQueueValid) {
        return ApplyZeroCopyPllClock(state);
    }
    if (rxPllReady) {
        return ApplyNominalClock(state, false);
    }
    if (state.txQueueValid && !state.zeroCopyEnabled) {
        return ApplyNominalClock(state, true);
    }
    return static_cast<uint64_t>(state.clockSync->currentTicksPerBuffer);
}

void LogPeriodicMetrics(AudioClockEngineState& state,
                        uint64_t time,
                        bool localEncodingActive,
                        uint32_t rxFill,
                        bool rxPllReady,
                        uint32_t q8) {
    // ~1s cadence at the 512-frame/48k tick rate (~93.75 Hz). Kept short so the
    // CLK:/IO: clock-servo metrics surface within a second and survive the brief
    // CoreAudio Start/Stop churn (which resets this counter on every StartDevice).
    if (++(*state.metricsLogCounter) % 96 != 0) {
        return;
    }

    const uint64_t framesReceived = state.ioMetrics->totalFramesReceived.load(std::memory_order_relaxed);
    const uint64_t framesSent = state.ioMetrics->totalFramesSent.load(std::memory_order_relaxed);
    const uint64_t callbacks = state.ioMetrics->callbackCount.load(std::memory_order_relaxed);
    const uint64_t underruns = state.ioMetrics->underruns.load(std::memory_order_relaxed);

    uint32_t ringFillLevel = 0;
    uint64_t ringUnderruns = 0;
    if (localEncodingActive) {
        ringFillLevel = state.packetAssembler->bufferFillLevel();
        ringUnderruns = state.packetAssembler->underrunCount();
    }

    const uint64_t elapsed = time - state.ioMetrics->startTime;
    struct mach_timebase_info timebase;
    mach_timebase_info(&timebase);
    const double elapsedSec = static_cast<double>(elapsed) * static_cast<double>(timebase.numer)
                            / static_cast<double>(timebase.denom)
                            / 1e9;

    if (elapsedSec <= 0.0) {
        return;
    }

    const double framesPerSec = static_cast<double>(framesReceived) / elapsedSec;
    const double dt = elapsedSec - state.encodingMetrics->lastLogElapsedSec;
    const uint64_t dp = state.encodingMetrics->packetsGenerated - state.encodingMetrics->lastLogPackets;
    const double packetsPerSec = (dt > 0.0) ? static_cast<double>(dp) / dt : 0.0;

    // Fix #24: Always print IO metrics (was gated by verbosity >= 3)
    {
        ASFW_LOG(Audio,
                 "IO: %.1fs recv=%llu sent=%llu (%.0f/s) cb=%llu ring=%u rxFill=%u overruns=%llu underruns=%llu/%llu | LocalEnc:%{public}s %llu pkts (%.0f/s, D:%llu N:%llu)",
                 elapsedSec,
                 framesReceived,
                 framesSent,
                 framesPerSec,
                 callbacks,
                 ringFillLevel,
                 rxFill,
                 state.encodingMetrics->overruns,
                 underruns,
                 ringUnderruns,
                 localEncodingActive ? "ON" : "OFF",
                 state.encodingMetrics->packetsGenerated,
                 packetsPerSec,
                 state.encodingMetrics->dataPackets,
                 state.encodingMetrics->noDataPackets);

        const double corrPpm = ((state.clockSync->currentTicksPerBuffer
                               - state.clockSync->nominalTicksPerBuffer)
                               / state.clockSync->nominalTicksPerBuffer) * 1e6;
        if (q8 > 0) {
            const uint32_t txFill = state.txQueueValid ? state.txQueueWriter->FillLevelFrames() : 0;
            ASFW_LOG(Audio,
                     "CLK: q8=%u corr=%.1f ppm rxFill=%u txFill=%u (cycle-time, unified)",
                     q8,
                     corrPpm,
                     rxFill,
                     txFill);
        } else if (state.zeroCopyEnabled && state.txQueueValid) {
            const uint32_t fill = state.txQueueWriter->FillLevelFrames();
            ASFW_LOG(Audio,
                     "CLK-TX: fill=%u target=%u err=%d integral=%lld corr=%.1f ppm (max=%.1f) zcDisc=%llu",
                     fill,
                     state.clockSync->targetFillLevel,
                     state.clockSync->lastFillError,
                     state.clockSync->fillErrorIntegral,
                     corrPpm,
                     state.clockSync->maxCorrectionPpm,
                     state.zeroCopyTimeline->discontinuities);
        } else if (rxPllReady) {
            ASFW_LOG(Audio,
                     "CLK-RX: fill=%u corr=0.0 ppm q8=0 (awaiting cycle-time)",
                     rxFill);
        } else if (state.txQueueValid) {
            const uint32_t fill = state.txQueueWriter->FillLevelFrames();
            ASFW_LOG(Audio,
                     "CLK: fill=%u target=%u err=%d nominal (legacy TX path)",
                     fill,
                     state.clockSync->targetFillLevel,
                     state.clockSync->lastFillError);
        }
    }

    state.encodingMetrics->lastLogPackets = state.encodingMetrics->packetsGenerated;
    state.encodingMetrics->lastLogElapsedSec = elapsedSec;
}

void DrainLocalEncoding(AudioClockEngineState& state) {
    while (state.packetAssembler->bufferFillLevel() >= state.packetAssembler->samplesPerDataPacket()) {
        const auto packet = state.packetAssembler->assembleNext(0xFFFF);
        state.encodingMetrics->packetsGenerated++;
        if (packet.isData) {
            state.encodingMetrics->dataPackets++;
        } else {
            state.encodingMetrics->noDataPackets++;
        }
    }
}

} // namespace detail

void PrepareClockEngineForStart(AudioClockEngineState& state) {
    if (!state.audioDevice || !state.timestampTimer || !state.clockSync ||
        !state.hostTicksPerBuffer || !state.ioMetrics || !state.metricsLogCounter ||
        !state.packetAssembler || !state.zeroCopyTimeline || !state.rxStartupDrained) {
        return;
    }

    state.ioMetrics->totalFramesReceived.store(0, std::memory_order_relaxed);
    state.ioMetrics->totalFramesSent.store(0, std::memory_order_relaxed);
    state.ioMetrics->callbackCount.store(0, std::memory_order_relaxed);
    state.ioMetrics->underruns.store(0, std::memory_order_relaxed);
    state.ioMetrics->startTime = mach_absolute_time();
    *state.metricsLogCounter = 0;

    state.packetAssembler->reset();
    *state.rxStartupDrained = false;
    detail::ResetZeroCopyTimeline(*state.zeroCopyTimeline);

    struct mach_timebase_info timebaseInfo;
    mach_timebase_info(&timebaseInfo);

    const double sampleRate = state.currentSampleRate;
    double hostTicksPerBuffer = static_cast<double>(state.ioBufferPeriodFrames * NSEC_PER_SEC) / sampleRate;
    hostTicksPerBuffer = (hostTicksPerBuffer * static_cast<double>(timebaseInfo.denom))
                       / static_cast<double>(timebaseInfo.numer);
    *state.hostTicksPerBuffer = static_cast<uint64_t>(hostTicksPerBuffer);

    state.clockSync->nominalTicksPerBuffer = hostTicksPerBuffer;
    state.clockSync->currentTicksPerBuffer = hostTicksPerBuffer;
    detail::ResetClockSync(*state.clockSync);

    // Seed the AppleUSBAudio-style fit's jitter floor at ~10us in host ticks
    // (ns -> ticks is *denom/numer, same conversion used for the period above).
    state.clockSync->ztsAnchorFit.Reset();
    state.clockSync->ztsAnchorFit.jitterFloorTicks =
        10000.0 * static_cast<double>(timebaseInfo.denom)
                / static_cast<double>(timebaseInfo.numer);

    if (state.txQueueValid && state.txQueueWriter) {
        state.txQueueWriter->ProducerSetZeroCopyPhaseFrames(0);
        state.txQueueWriter->ProducerRequestConsumerResync();
    }

    if (state.txQueueValid) {
        if (state.zeroCopyEnabled && state.zeroCopyFrameCapacity > 0) {
            uint32_t target = (state.zeroCopyFrameCapacity * 5) / 8;
            if (target < 8) {
                target = 8;
            }
            state.clockSync->targetFillLevel = target;
        } else {
            state.clockSync->targetFillLevel = 64;
        }
    } else {
        state.clockSync->targetFillLevel = 2048;
    }

    ASFW_LOG(Audio,
             "ASFWAudioDriver: Clock sync target fill=%u (zeroCopy=%{public}s)",
             state.clockSync->targetFillLevel,
             state.zeroCopyEnabled ? "YES" : "NO");

    ASFW_LOG(Audio,
             "ASFWAudioDriver: Timer interval = %llu ticks (%.0f Hz, period=%u frames)",
             *state.hostTicksPerBuffer,
             sampleRate,
             state.ioBufferPeriodFrames);

    state.audioDevice->UpdateCurrentZeroTimestamp(0, 0);

    // NOTE: the timestamp timer is NOT armed here. It is created and armed inline in
    // ASFWAudioDriver::Start() (the dext's own work-queue context) and free-runs for
    // the driver's lifetime — mirroring the sibling ASFWMIDIDriver's RX-poll timer,
    // which fires reliably. Four prior builds armed the timer from THIS path (reached
    // on the CoreAudio HAL thread) via DispatchAsync onto a work/dedicated queue, and
    // the arm block never executed → the timer never fired → the zero-timestamp anchor
    // stayed frozen → garbled audio. StartDevice now only flips isRunning; the already-
    // running timer's ZtsTimerOccurred callback gates real work on that flag.
    // (state.workQueue is retained in the struct for compatibility but is unused here.)
}

void PrepareClockEngineForStop(AudioClockEngineState& state) {
    if (!state.clockSync || !state.zeroCopyTimeline) {
        return;
    }

    detail::ResetClockSync(*state.clockSync);
    state.zeroCopyTimeline->valid = false;

    // Do NOT disable the timer here. It free-runs for the driver's lifetime (armed in
    // Start, like ASFWMIDIDriver's poll); StopDevice just clears isRunning so the
    // ZtsTimerOccurred callback re-arms but skips the work. Disabling on every stop
    // re-introduces the "needs to be re-armed on the delivery queue" problem on the
    // next start. The timer is torn down (SetEnable(false)) only at dext Stop/free.
    ASFW_LOG(Audio, "ASFWAudioDriver: Clock engine stopped (timer keeps free-running)");
}

void HandleClockTimerTick(AudioClockEngineState& state, uint64_t time) {
    if (!state.audioDevice || !state.timestampTimer || !state.clockSync ||
        !state.ioMetrics || !state.metricsLogCounter || !state.packetAssembler ||
        !state.encodingMetrics || !state.rxQueueReader || !state.zeroCopyTimeline ||
        !state.txQueueWriter) {
        return;
    }

    const bool localEncodingActive = !state.txQueueValid;

    uint32_t rxFill = 0;
    bool rxPllReady = false;
    if (state.rxQueueValid) {
        rxFill = state.rxQueueReader->FillLevelFrames();
        rxPllReady = true;
    }

    uint64_t currentSampleTime = 0;
    uint64_t currentHostTime = 0;
    state.audioDevice->GetCurrentZeroTimestamp(&currentSampleTime, &currentHostTime);

    const uint32_t q8 = state.rxQueueValid ? state.rxQueueReader->CorrHostNanosPerSampleQ8() : 0;
    const uint64_t hostTicksPerBuffer = detail::ComputeHostTicksPerBuffer(state, q8, rxPllReady);

    // Apple-faithful path, PLL-smoothed (2026-06-10, HW log 21-22-25): the raw
    // RX-poll (cycle-timer sample-time, host-uptime) anchor pair is correct in
    // RATE (slope = 48001.5 fps, the real device clock) but carries up to ~2.7ms
    // of capture-instant phase noise per publish. Republishing it raw made the
    // HAL's anchor-scheduled IO wakes fire at ~161/s instead of 250/s — a ~3.5%
    // feed deficit that drained the TX cushion every ~400ms (the residual Local-
    // mode stutter). The ZtsAnchorPll publishes a smooth grid timeline (sample
    // += period, host += q8-disciplined period) and only SLEWS phase toward the
    // raw anchor, bounded per tick. Falls back to the plain accumulator until
    // the first anchor arrives; a frozen anchor (RX stopped) simply stops the
    // slew, which IS the accumulator behavior that paced a perfect 250 cb/s.
    uint64_t hwSampleTime = 0;
    uint64_t hwHostTicks = 0;
    const bool haveHwAnchor = state.rxQueueValid &&
        state.rxQueueReader->ReadHwZeroTimestampAnchor(hwSampleTime, hwHostTicks);
    bool usedHwAnchor = false;

    const double ticksPerBuffer = state.clockSync->currentTicksPerBuffer > 0.0
        ? state.clockSync->currentTicksPerBuffer
        : static_cast<double>(hostTicksPerBuffer);

    // A/B: ZtsAnchorFit (AppleUSBAudio-faithful history-fit + EMA jitter bound,
    // 2026-06-14 RE) vs the older single-target ZtsAnchorPll. Flip this one line
    // to compare both anchor strategies on the same hardware/log.
    constexpr bool kUseAnchorFit = true;

    const bool anchorFresh = haveHwAnchor &&
        hwHostTicks != (kUseAnchorFit
                            ? state.clockSync->ztsAnchorFit.lastHwHostTicks
                            : state.clockSync->ztsAnchorPll.lastHwHostTicks);

    // Advance one grid period per elapsed timer beat (HW log 2026-06-10_22-22-09:
    // crediting exactly ONE period per delivered callback while deliveries lagged
    // the grid made the published timeline run ~15% slow in wall terms — still
    // rate-consistent, so audio played, but the anchor receded from "now" at
    // ~0.15 s/s). The raw anchor is offered only on the last beat so the phase
    // slew applies once, against the fully advanced grid.
    const uint32_t elapsedPeriods = state.elapsedPeriods > 0 ? state.elapsedPeriods : 1;

    // Advance the selected anchor: seed on the first tick, otherwise advance one
    // grid period per elapsed beat and offer the raw anchor only on the last beat
    // (so the slew/fit-publish applies once against the fully advanced grid).
    // Both ZtsAnchorPll and ZtsAnchorFit share the same Tick() signature and the
    // {valid, sampleTime, hostTicks} surface, so the loop body is identical.
    auto advanceAnchor = [&](auto& anchor) -> bool {
        bool ok = false;
        if (!anchor.valid) {
            ok = anchor.Tick(state.ioBufferPeriodFrames, ticksPerBuffer,
                             haveHwAnchor, hwSampleTime, hwHostTicks);
        } else {
            for (uint32_t beat = 0; beat < elapsedPeriods; ++beat) {
                const bool lastBeat = (beat + 1 == elapsedPeriods);
                ok = anchor.Tick(state.ioBufferPeriodFrames, ticksPerBuffer,
                                 lastBeat && haveHwAnchor, hwSampleTime, hwHostTicks);
            }
        }
        if (ok) {
            currentSampleTime = anchor.sampleTime;
            currentHostTime = anchor.hostTicks;
            usedHwAnchor = true;
        }
        return ok;
    };

    const bool anchorValid = kUseAnchorFit
        ? advanceAnchor(state.clockSync->ztsAnchorFit)
        : advanceAnchor(state.clockSync->ztsAnchorPll);

    if (!anchorValid) {
        if (currentHostTime != 0) {
            currentSampleTime += static_cast<uint64_t>(elapsedPeriods) * state.ioBufferPeriodFrames;
            currentHostTime += static_cast<uint64_t>(elapsedPeriods) * hostTicksPerBuffer;
        } else {
            currentSampleTime = 0;
            currentHostTime = time;
        }
    }

    state.audioDevice->UpdateCurrentZeroTimestamp(currentSampleTime, currentHostTime);

    // The timer free-runs and re-arms itself in ZtsTimerOccurred (unconditionally, even
    // when not streaming). Publish the freshly corrected period so that re-arm tracks
    // the device clock instead of the nominal seed. (No WakeAtTime here — the callback
    // owns re-arm, so a stop/start can't leave the timer un-armed.)
    *state.hostTicksPerBuffer = hostTicksPerBuffer;

    // Anchor-freshness probe (2026-06-09 HAL cadence analysis):
    // the CoreAudio HAL derives the device rate from deltas (robust to publish cadence),
    // but schedules each IO wake as `cycle_anchor - 1 cycle` in *host* time. So the
    // zero-timestamp we publish must read as a RECENT PAST anchor: `host - mach_now`
    // should be <= 0 (a small negative, ~one cycle behind). A large POSITIVE lead means
    // our published host runs AHEAD of real mach time -> the HAL's wake lands in the
    // future -> the IO thread sleeps -> the ~130x feed throttle seen in log 20-39-12.
    // Also report timer lateness (mach_now - scheduled `time`) to catch the accumulator
    // path over-advancing host when the timer fires off-cadence.
    const uint64_t machNow = mach_absolute_time();
    const int64_t hostLeadTicks = static_cast<int64_t>(currentHostTime) - static_cast<int64_t>(machNow);
    const int64_t timerLateTicks = static_cast<int64_t>(machNow) - static_cast<int64_t>(time);

    ASFW_LOG_RL(Audio, "zts/anchor", 1000, OS_LOG_TYPE_DEFAULT,
                "zts/anchor: src=%{public}s sample=%llu host=%llu q8=%u "
                "lead=%lld cyc=%llu late=%lld n=%u",
                usedHwAnchor
                    ? (anchorFresh ? (kUseAnchorFit ? "hw-fit" : "hw-pll") : "hw-coast")
                    : "accum",
                currentSampleTime,
                currentHostTime,
                q8,
                static_cast<long long>(hostLeadTicks),
                static_cast<unsigned long long>(hostTicksPerBuffer),
                static_cast<long long>(timerLateTicks),
                elapsedPeriods);

    detail::LogPeriodicMetrics(state, time, localEncodingActive, rxFill, rxPllReady, q8);

    if (localEncodingActive) {
        detail::DrainLocalEncoding(state);
    }
}

} // namespace ASFW::Isoch::Audio
