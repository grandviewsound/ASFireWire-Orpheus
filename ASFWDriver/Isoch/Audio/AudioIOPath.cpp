#include "AudioIOPath.hpp"

#include "../../Logging/Logging.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace ASFW::Isoch::Audio {
namespace detail {

constexpr uint32_t kRxTargetFillFrames = 2048;

struct PcmScanSummary {
    bool hasNonZero{false};
    uint32_t nonZeroFrames{0};
    uint32_t firstFrame{0};
    uint32_t firstChannel{0};
    uint32_t channelMask{0};
    int32_t firstSample{0};
    int32_t peakSample{0};
};

void ScanPcmSpan(const int32_t* pcm,
                 uint32_t frames,
                 uint32_t channels,
                 uint32_t baseFrame,
                 PcmScanSummary& summary) {
    if (!pcm || frames == 0 || channels == 0) {
        return;
    }

    for (uint32_t frame = 0; frame < frames; ++frame) {
        bool frameHasNonZero = false;
        for (uint32_t ch = 0; ch < channels; ++ch) {
            const int32_t sample = pcm[(static_cast<size_t>(frame) * channels) + ch];
            if (sample == 0) {
                continue;
            }

            if (!summary.hasNonZero) {
                summary.hasNonZero = true;
                summary.firstFrame = baseFrame + frame;
                summary.firstChannel = ch;
                summary.firstSample = sample;
            }
            if (ch < 32) {
                summary.channelMask |= (1u << ch);
            }
            int32_t absSample = (sample < 0) ? -sample : sample;
            if (absSample > summary.peakSample) {
                summary.peakSample = absSample;
            }
            frameHasNonZero = true;
        }
        if (frameHasNonZero) {
            ++summary.nonZeroFrames;
        }
    }
}

void LogFirstPcmFrameChannels(const int32_t* frame, uint32_t channels) {
    if (!frame || channels == 0) {
        return;
    }

    char buffer[512];
    int written = std::snprintf(buffer,
                                sizeof(buffer),
                                "WriteEnd frame channels=%u",
                                channels);
    if (written < 0) {
        return;
    }

    size_t offset = static_cast<size_t>(written);
    for (uint32_t ch = 0; ch < channels && offset < sizeof(buffer); ++ch) {
        written = std::snprintf(buffer + offset,
                                sizeof(buffer) - offset,
                                " ch%u=0x%08x",
                                ch,
                                static_cast<uint32_t>(frame[ch]));
        if (written < 0) {
            return;
        }
        offset += static_cast<size_t>(written);
    }

    ASFW_LOG(Audio, "%{public}s", buffer);
}

void MaybeDrainRxStartup(AudioIOPathState& state) {
    if (!state.rxQueueValid || !state.rxQueueReader || !state.rxStartupDrained || *state.rxStartupDrained) {
        return;
    }

    const uint32_t fill = state.rxQueueReader->FillLevelFrames();
    if (fill > kRxTargetFillFrames + 256U) {
        const uint32_t excess = fill - kRxTargetFillFrames;
        state.rxQueueReader->ConsumeFrames(excess);
    }
    *state.rxStartupDrained = true;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
kern_return_t HandleBeginRead(AudioIOPathState& state,
                              uint32_t ioBufferFrameSize, // NOLINT(bugprone-easily-swappable-parameters)
                              uint64_t sampleTime) {
    if (!state.inputBuffer) {
        return kIOReturnNotReady;
    }

    IOAddressSegment segment{};
    const kern_return_t status = state.inputBuffer->GetAddressRange(&segment);
    if (status != kIOReturnSuccess || segment.address == 0) {
        return kIOReturnSuccess;
    }

    const uint32_t bufferFrames = state.ioBufferPeriodFrames;
    const uint32_t offsetFrames = static_cast<uint32_t>(sampleTime % bufferFrames);
    uint32_t firstFrames = ioBufferFrameSize;
    uint32_t secondFrames = 0;
    if ((offsetFrames + ioBufferFrameSize) > bufferFrames) {
        firstFrames = bufferFrames - offsetFrames;
        secondFrames = ioBufferFrameSize - firstFrames;
    }

    const uint32_t ch = (state.inputChannelCount > 0) ? state.inputChannelCount : state.channelCount;
    if (ch == 0) {
        return kIOReturnBadArgument;
    }

    const uint64_t offsetBytes = uint64_t(offsetFrames) * sizeof(int32_t) * ch;
    const size_t firstBytes = size_t(firstFrames) * sizeof(int32_t) * ch;
    const size_t secondBytes = size_t(secondFrames) * sizeof(int32_t) * ch;

    MaybeDrainRxStartup(state);

    if (state.rxQueueValid && state.rxQueueReader) {
        auto* pcmFirst = reinterpret_cast<int32_t*>(segment.address + offsetBytes);
        const uint32_t read1 = state.rxQueueReader->Read(pcmFirst, firstFrames);
        if (read1 < firstFrames) {
            memset(pcmFirst + (static_cast<size_t>(read1) * ch),
                   0,
                   size_t(firstFrames - read1) * sizeof(int32_t) * ch);
        }

        if (secondFrames > 0) {
            auto* pcmSecond = reinterpret_cast<int32_t*>(segment.address);
            if (read1 == firstFrames) {
                const uint32_t read2 = state.rxQueueReader->Read(pcmSecond, secondFrames);
                if (read2 < secondFrames) {
                    memset(pcmSecond + (static_cast<size_t>(read2) * ch),
                           0,
                           size_t(secondFrames - read2) * sizeof(int32_t) * ch);
                }
            } else {
                memset(pcmSecond, 0, secondBytes);
            }
        }
    } else {
        memset(reinterpret_cast<void*>(segment.address + offsetBytes), 0, firstBytes);
        if (secondFrames > 0) {
            memset(reinterpret_cast<void*>(segment.address), 0, secondBytes);
        }
    }

    return kIOReturnSuccess;
}

void RebaseZeroCopyTimeline(AudioIOPathState& state,
                            uint64_t sampleTime,
                            ZeroCopyTimelineState& timeline) {
    const uint32_t bufferFrames = (state.zeroCopyFrameCapacity > 0)
                                    ? state.zeroCopyFrameCapacity
                                    : state.ioBufferPeriodFrames;
    const uint32_t writeIdx = state.txQueueWriter->WriteIndexFrames();
    const uint32_t samplePos = static_cast<uint32_t>(sampleTime % bufferFrames);
    const uint32_t phase = (samplePos + bufferFrames - (writeIdx % bufferFrames)) % bufferFrames;

    timeline.phaseFrames = phase;
    state.txQueueWriter->ProducerSetZeroCopyPhaseFrames(phase);
    state.txQueueWriter->ProducerRequestConsumerResync();
}

uint32_t WriteEndZeroCopyPublish(AudioIOPathState& state,
                                 uint32_t ioBufferFrameSize,
                                 uint64_t sampleTime,
                                 uint32_t& outFramesRequested) {
    auto& timeline = *state.zeroCopyTimeline;
    bool rebased = false;

    if (!timeline.valid) {
        timeline.valid = true;
        timeline.lastSampleTime = sampleTime;
        timeline.publishedSampleTime = sampleTime;
        rebased = true;
    } else if (sampleTime < timeline.lastSampleTime) {
        timeline.discontinuities++;
        ASFW_LOG_RL(Audio,
                    "zc/disc",
                    500,
                    OS_LOG_TYPE_DEFAULT,
                    "ZERO-COPY DISCONTINUITY (rebase) sampleTime=%llu lastSampleTime=%llu gap=%lld disc=%llu",
                    sampleTime,
                    timeline.lastSampleTime,
                    static_cast<int64_t>(sampleTime) - static_cast<int64_t>(timeline.lastSampleTime),
                    timeline.discontinuities);
        timeline.lastSampleTime = sampleTime;
        timeline.publishedSampleTime = sampleTime;
        rebased = true;
    } else {
        timeline.lastSampleTime = sampleTime;
    }

    if (rebased) {
        RebaseZeroCopyTimeline(state, sampleTime, timeline);
    }

    uint64_t desiredPublishedSample = sampleTime + ioBufferFrameSize;
    if (desiredPublishedSample < timeline.publishedSampleTime) {
        timeline.discontinuities++;
        ASFW_LOG_RL(Audio,
                    "zc/disc",
                    500,
                    OS_LOG_TYPE_DEFAULT,
                    "ZERO-COPY DISCONTINUITY (publish) sampleTime=%llu published=%llu desired=%llu disc=%llu",
                    sampleTime,
                    timeline.publishedSampleTime,
                    desiredPublishedSample,
                    timeline.discontinuities);

        timeline.publishedSampleTime = sampleTime;
        RebaseZeroCopyTimeline(state, sampleTime, timeline);
        desiredPublishedSample = sampleTime + ioBufferFrameSize;
    }

    const uint64_t toPublish64 = desiredPublishedSample - timeline.publishedSampleTime;
    const uint32_t toPublish = (toPublish64 > 0xFFFFFFFFULL)
                                 ? 0xFFFFFFFFU
                                 : static_cast<uint32_t>(toPublish64);

    outFramesRequested = toPublish;
    const uint32_t framesWritten = state.txQueueWriter->PublishFrames(toPublish);
    timeline.publishedSampleTime += framesWritten;
    return framesWritten;
}

// Diagnostic counters for HandleWriteEnd (Fix #24)
static uint64_t sWriteEndCallCount = 0;
static uint64_t sWriteEndNonZeroFrames = 0;
static int32_t  sWriteEndPeakSample = 0;
static bool     sWriteEndFirstNonZeroLogged = false;

kern_return_t HandleWriteEnd(AudioIOPathState& state,
                             uint32_t ioBufferFrameSize,
                             uint64_t sampleTime) {
    if (!state.outputBuffer) {
        return kIOReturnNotReady;
    }

    IOAddressSegment segment{};
    const kern_return_t status = state.outputBuffer->GetAddressRange(&segment);
    if (status != kIOReturnSuccess || segment.address == 0) {
        return kIOReturnSuccess;
    }

    const uint32_t bufferFrames = state.ioBufferPeriodFrames;
    const uint32_t offsetFrames = static_cast<uint32_t>(sampleTime % bufferFrames);
    const uint32_t ch = state.outputChannelCount;
    if (ch == 0) {
        return kIOReturnBadArgument;
    }

    const uint64_t offsetBytes = uint64_t(offsetFrames) * sizeof(int32_t) * ch;
    uint32_t firstFrames = ioBufferFrameSize;
    uint32_t secondFrames = 0;
    if ((offsetFrames + ioBufferFrameSize) > bufferFrames) {
        firstFrames = bufferFrames - offsetFrames;
        secondFrames = ioBufferFrameSize - firstFrames;
    }

    const auto* pcmDataFirst = reinterpret_cast<const int32_t*>(segment.address + offsetBytes);
    const auto* pcmDataSecond = reinterpret_cast<const int32_t*>(segment.address);

    PcmScanSummary scan{};
    ScanPcmSpan(pcmDataFirst, firstFrames, ch, offsetFrames, scan);
    if (secondFrames > 0) {
        ScanPcmSpan(pcmDataSecond, secondFrames, ch, 0, scan);
    }
    sWriteEndNonZeroFrames += scan.nonZeroFrames;
    if (scan.peakSample > sWriteEndPeakSample) {
        sWriteEndPeakSample = scan.peakSample;
    }

    uint32_t framesWritten = 0;
    uint32_t framesRequested = ioBufferFrameSize;
    const uint32_t txFillBefore =
        (state.txQueueValid && state.txQueueWriter) ? state.txQueueWriter->FillLevelFrames() : 0;

    if (state.txQueueValid && state.txQueueWriter) {
        if (state.zeroCopyEnabled && state.zeroCopyTimeline) {
            framesWritten = WriteEndZeroCopyPublish(state,
                                                    ioBufferFrameSize,
                                                    sampleTime,
                                                    framesRequested);
        } else {
            const uint32_t firstWrite = state.txQueueWriter->Write(pcmDataFirst, firstFrames);
            framesWritten = firstWrite;
            if (firstWrite == firstFrames && secondFrames > 0) {
                framesWritten += state.txQueueWriter->Write(pcmDataSecond, secondFrames);
            }
        }
    } else if (state.packetAssembler) {
        const uint32_t firstWrite = state.packetAssembler->ringBuffer().write(pcmDataFirst, firstFrames);
        framesWritten = firstWrite;
        if (firstWrite == firstFrames && secondFrames > 0) {
            framesWritten += state.packetAssembler->ringBuffer().write(pcmDataSecond, secondFrames);
        }
    }
    const uint32_t txFillAfter =
        (state.txQueueValid && state.txQueueWriter) ? state.txQueueWriter->FillLevelFrames() : 0;

    if (framesWritten < framesRequested && state.encodingOverruns) {
        (*state.encodingOverruns)++;
    }

    if (scan.hasNonZero && !sWriteEndFirstNonZeroLogged) {
        sWriteEndFirstNonZeroLogged = true;
        ASFW_LOG(Audio,
                 "WriteEnd FIRST NONZERO: call=%llu sampleTime=%llu frame=%u ch=%u "
                 "sample=0x%08x peak=0x%08x chMask=0x%08x written=%u requested=%u txFill=%u->%u",
                 sWriteEndCallCount + 1,
                 sampleTime,
                 scan.firstFrame,
                 scan.firstChannel,
                 static_cast<uint32_t>(scan.firstSample),
                 static_cast<uint32_t>(scan.peakSample),
                 scan.channelMask,
                 framesWritten,
                 framesRequested,
                 txFillBefore,
                 txFillAfter);
    }

    // Fix #24: Periodic diagnostic log (every ~500 calls ≈ every 5 seconds at 48kHz/512)
    ++sWriteEndCallCount;
    if (sWriteEndCallCount % 500 == 1) {
        ASFW_LOG(Audio,
                 "WriteEnd[%llu]: frames=%u written=%u txQ=%{public}s zc=%{public}s txFill=%u->%u "
                 "nonZeroFrames=%llu peak=0x%08x firstNZ=f%u/ch%u sample=0x%08x chMask=0x%08x "
                 "samples=[%08x,%08x,%08x,%08x]",
                 sWriteEndCallCount,
                 ioBufferFrameSize,
                 framesWritten,
                 (state.txQueueValid && state.txQueueWriter) ? "YES" : "NO",
                 state.zeroCopyEnabled ? "YES" : "NO",
                 txFillBefore,
                 txFillAfter,
                 sWriteEndNonZeroFrames,
                 static_cast<uint32_t>(sWriteEndPeakSample),
                 scan.firstFrame,
                 scan.firstChannel,
                 static_cast<uint32_t>(scan.firstSample),
                 scan.channelMask,
                 static_cast<uint32_t>(pcmDataFirst[0]),
                 static_cast<uint32_t>(ch > 1 ? pcmDataFirst[1] : 0),
                 static_cast<uint32_t>(ch > 0 ? pcmDataFirst[ch] : 0),
                 static_cast<uint32_t>(ch > 1 ? pcmDataFirst[ch + 1] : 0));
        LogFirstPcmFrameChannels(pcmDataFirst, ch);
    }

    return kIOReturnSuccess;
}

} // namespace detail

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
kern_return_t HandleIOOperation(AudioIOPathState& state,
                                IOUserAudioIOOperation operation, // NOLINT(bugprone-easily-swappable-parameters)
                                uint32_t ioBufferFrameSize,
                                uint64_t sampleTime) {
    switch (operation) {
        case IOUserAudioIOOperationBeginRead:
            return detail::HandleBeginRead(state, ioBufferFrameSize, sampleTime);
        case IOUserAudioIOOperationWriteEnd:
            return detail::HandleWriteEnd(state, ioBufferFrameSize, sampleTime);
        default:
            return kIOReturnSuccess;
    }
}

} // namespace ASFW::Isoch::Audio
