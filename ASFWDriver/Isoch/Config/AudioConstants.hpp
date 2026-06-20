#pragma once

#include <cstdint>

namespace ASFW::Isoch::Config {

/// Maximum AM824 slots per isochronous data block (CIP DBS).
/// Wire-level container size: PCM audio + MIDI + control slots combined.
inline constexpr uint32_t kMaxAmdtpDbs = 32;

/// Maximum host-facing PCM channel count the driver can handle.
/// Must be <= kMaxAmdtpDbs since PCM channels occupy a subset of DBS slots.
inline constexpr uint32_t kMaxPcmChannels = kMaxAmdtpDbs;

static_assert(kMaxPcmChannels <= kMaxAmdtpDbs,
              "PCM channel cap cannot exceed AMDTP DBS — PCM slots are a subset of DBS");

// Shared queue / buffer sizing.
//
// BASE depth for the 44.1k/48k rate family. 4096 frames ≈ 85 ms @ 48 kHz —
// the jitter headroom in the CoreAudio↔dext shared SPSC queue. The shared
// queues are sized at allocation with QueueCapacityFramesForRate() below so the
// time-domain headroom stays ~constant across rate families (higher rates pack
// more frames into the same wall-clock window). These remain the documented
// BASE / floor and are the operands the TX-profile static_asserts validate
// against (runtime capacity is always >= base, so the asserts stay conservative).
inline constexpr uint32_t kTxQueueCapacityFrames = 4096;
inline constexpr uint32_t kRxQueueCapacityFrames = 4096;
inline constexpr uint32_t kAudioRingBufferFrames = 4096;
inline constexpr uint32_t kAudioIoPeriodFrames = 512;

/// Highest PCM sample rate the driver provisions buffers for.
inline constexpr uint32_t kMaxAudioSampleRateHz = 192000;

/// Shared-queue depth (frames) for a given sample rate, scaled by rate family
/// so the time-domain jitter headroom stays ~constant (~85 ms) across rates.
///
/// Mirrors Apple's AppleFWAudioIsochStream::SetSampleRate, which sizes the audio
/// sample buffer 6400 / 12800 / 25600 frames for the 48k / 96k / 192k families
/// (a doubling per family). We use the same doubling law, rounded to the
/// power-of-two the SPSC ring requires for its mask-based modulo.
[[nodiscard]] constexpr uint32_t QueueCapacityFramesForRate(uint32_t rateHz) noexcept {
    if (rateHz >= 96001) {
        return kTxQueueCapacityFrames * 4;  // 176.4k / 192k family → 16384
    }
    if (rateHz >= 48001) {
        return kTxQueueCapacityFrames * 2;  // 88.2k / 96k family   → 8192
    }
    return kTxQueueCapacityFrames;          // ≤ 48k family (base, HW-proven) → 4096
}

/// Direct-mapped ("zero-copy") output path master gate. SHARED contract between
/// the two halves that must agree or audio goes silent:
///   • engine side (ASFWAudioDriver): hands CoreAudio the nub's shared output
///     buffer instead of a private buffer (MapZeroCopyOutputFromNub), and
///   • transport side (AVCAudioBackend → IsochService → IsochAudioTxPipeline):
///     points the PacketAssembler at that SAME buffer (setZeroCopySource), and
///   • clock side (AudioClockEngine): publishes the zero-timestamp anchor from
///     the assembler's real read position (the AppleUSBAudio
///     getCurrentSampleFrame/takeTimeStamp coupling) instead of the RX anchor.
/// When false, every half falls back to the legacy CoreAudio→shared-TX-queue
/// copy path (the jun14 last-known-good). See
/// research/ida/usbaudio_output_datapath_2026-06-17.md.
inline constexpr bool kEnableZeroCopyOutputPath = true;

/// Worst-case shared-queue depth (192 kHz family) — the provisioning ceiling.
inline constexpr uint32_t kMaxQueueCapacityFrames = QueueCapacityFramesForRate(kMaxAudioSampleRateHz);

static_assert(kTxQueueCapacityFrames != 0 && ((kTxQueueCapacityFrames & (kTxQueueCapacityFrames - 1)) == 0),
              "TX queue capacity must be power-of-two");
static_assert(kRxQueueCapacityFrames != 0 && ((kRxQueueCapacityFrames & (kRxQueueCapacityFrames - 1)) == 0),
              "RX queue capacity must be power-of-two");
static_assert(kAudioRingBufferFrames != 0 && ((kAudioRingBufferFrames & (kAudioRingBufferFrames - 1)) == 0),
              "Audio ring buffer frame count must be power-of-two");
static_assert(kMaxQueueCapacityFrames != 0 && ((kMaxQueueCapacityFrames & (kMaxQueueCapacityFrames - 1)) == 0),
              "Max queue capacity must be power-of-two (SPSC ring uses mask modulo)");

} // namespace ASFW::Isoch::Config

