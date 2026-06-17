//
// TimingUtils.hpp
// ASFWDriver
//
// FireWire ↔ Host time conversion utilities per IEC 61883-6
// Adapted from FWAIsoch/include/utils/TimingUtils.hpp
//

#pragma once

#include <cstdint>
#include <DriverKit/IOLib.h>  // Provides mach_absolute_time, mach_timebase_info in DriverKit

namespace ASFW::Timing {

//-----------------------------------------------------------------------------
// Host timebase (macOS only)
//-----------------------------------------------------------------------------

/// Cached mach timebase info for host ↔ nanoseconds conversion
inline mach_timebase_info_data_t gHostTimebaseInfo = {0, 0}; // NOSONAR(cpp:S5421): intentionally mutable — populated once by initializeHostTimebase()

/// Initialize host timebase (call once at driver start)
[[nodiscard]] inline bool initializeHostTimebase() noexcept {
    if (gHostTimebaseInfo.denom == 0) {
        kern_return_t kr = mach_timebase_info(&gHostTimebaseInfo);
        return (kr == KERN_SUCCESS && gHostTimebaseInfo.denom != 0);
    }
    return true;
}

//-----------------------------------------------------------------------------
// FireWire timing constants (IEC 61883-6)
//-----------------------------------------------------------------------------

constexpr uint32_t kTicksPerCycle = 3072;         // 24.576 MHz / 8000 Hz
constexpr uint32_t kCyclesPerSecond = 8000;
constexpr uint64_t kTicksPerSecond = 24'576'000ULL;
constexpr uint64_t kNanosPerSecond = 1'000'000'000ULL;
constexpr uint64_t kNanosPerCycle = 125'000ULL;   // 125 µs per cycle

/// 128-second wrap period for FireWire cycle timer
constexpr uint32_t kFWTimeWrapSeconds = 128;
constexpr int64_t kFWTimeWrapNanos = int64_t(kFWTimeWrapSeconds) * int64_t(kNanosPerSecond);

/// Transfer delay per IEC 61883-6 §7.3 (matches Linux TRANSFER_DELAY_TICKS)
constexpr uint32_t kTransferDelayTicks = 0x2E00;  // ~479 µs
constexpr uint64_t kTransferDelayNanos = 
    (uint64_t(kTransferDelayTicks) * kNanosPerCycle) / kTicksPerCycle;

//-----------------------------------------------------------------------------
// Cycle timer register field extraction
//-----------------------------------------------------------------------------

/// Masks for 32-bit OHCI cycle timer register
constexpr uint32_t kCycleTimerSecondsMask = 0xFE000000;  // bits 31:25
constexpr uint32_t kCycleTimerSecondsShift = 25;
constexpr uint32_t kCycleTimerCyclesMask = 0x01FFF000;   // bits 24:12
constexpr uint32_t kCycleTimerCyclesShift = 12;
constexpr uint32_t kCycleTimerOffsetMask = 0x00000FFF;   // bits 11:0

//-----------------------------------------------------------------------------
// Conversion functions
//-----------------------------------------------------------------------------

/// Convert 32-bit FireWire cycle timer to nanoseconds
[[nodiscard]] inline uint64_t encodedFWTimeToNanos(uint32_t cycleTimer) noexcept {
    uint32_t sec = (cycleTimer & kCycleTimerSecondsMask) >> kCycleTimerSecondsShift;
    uint32_t cyc = (cycleTimer & kCycleTimerCyclesMask) >> kCycleTimerCyclesShift;
    uint32_t off = cycleTimer & kCycleTimerOffsetMask;
    
    // Total time in nanoseconds
    uint64_t ns = uint64_t(sec) * kNanosPerSecond;
    ns += uint64_t(cyc) * kNanosPerCycle;
    ns += (uint64_t(off) * kNanosPerCycle) / kTicksPerCycle;
    
    return ns;
}

/// Convert nanoseconds to 32-bit FireWire cycle timer format
[[nodiscard]] inline uint32_t nanosToEncodedFWTime(uint64_t nanos) noexcept {
    // Wrap to [0, 128s)
    nanos %= (kFWTimeWrapSeconds * kNanosPerSecond);
    
    uint32_t sec = static_cast<uint32_t>(nanos / kNanosPerSecond) & 0x7F;
    uint64_t remNs = nanos % kNanosPerSecond;
    
    uint32_t cyc = static_cast<uint32_t>(remNs / kNanosPerCycle);
    uint64_t offsetNs = remNs % kNanosPerCycle;
    uint32_t off = static_cast<uint32_t>((offsetNs * kTicksPerCycle) / kNanosPerCycle);
    
    return (sec << kCycleTimerSecondsShift) 
         | (cyc << kCycleTimerCyclesShift) 
         | off;
}

/// Convert mach_absolute_time ticks to nanoseconds
[[nodiscard]] inline uint64_t hostTicksToNanos(uint64_t ticks) noexcept {
    if (gHostTimebaseInfo.denom == 0) return 0;
    
    // ticks * numer / denom
    // Use 128-bit multiplication to avoid overflow on long uptimes
    __uint128_t tmp = __uint128_t(ticks) * gHostTimebaseInfo.numer;
    return static_cast<uint64_t>(tmp / gHostTimebaseInfo.denom);
}

/// Convert nanoseconds to mach_absolute_time ticks
[[nodiscard]] inline uint64_t nanosToHostTicks(uint64_t nanos) noexcept {
    if (gHostTimebaseInfo.numer == 0) return 0;
    
    __uint128_t tmp = __uint128_t(nanos) * gHostTimebaseInfo.denom;
    return static_cast<uint64_t>(tmp / gHostTimebaseInfo.numer);
}

/// Signed delta between two FireWire times (handles 128s wrap)
[[nodiscard]] inline int64_t deltaFWTimeNanos(uint32_t a, uint32_t b) noexcept {
    int64_t na = static_cast<int64_t>(encodedFWTimeToNanos(a));
    int64_t nb = static_cast<int64_t>(encodedFWTimeToNanos(b));
    int64_t d = na - nb;
    
    // If delta > 64s, wrap around (shortest path)
    constexpr int64_t halfWrap = kFWTimeWrapNanos / 2;
    if (d > halfWrap) d -= kFWTimeWrapNanos;
    if (d < -halfWrap) d += kFWTimeWrapNanos;
    
    return d;
}

/// Normalize nanoseconds to [0, 128s) handling negative values
[[nodiscard]] inline uint64_t normalizeToFWTimeRange(int64_t nanos) noexcept {
    // Handle negative values with proper modulo
    int64_t normalized = ((nanos % kFWTimeWrapNanos) + kFWTimeWrapNanos) % kFWTimeWrapNanos;
    return static_cast<uint64_t>(normalized);
}

//-----------------------------------------------------------------------------
// FireWire cycle-timer → monotonic device sample clock
//-----------------------------------------------------------------------------

/// Accumulates wrap-corrected FireWire cycle-timer deltas into a monotonic
/// device-nanosecond clock and converts it to a sample-frame count. When the
/// device is bus cycle-master (e.g. Orpheus), the OHCI cycle timer *is* the
/// device's sample clock, so this yields CoreAudio's zero-timestamp sample-time
/// straight from hardware — Apple's "use the time passed in the hardware
/// interrupt" contract. Pure/header-only so it is unit-testable off-hardware.
struct FwSampleClock {
    uint64_t deviceNanos{0};  // monotonic device time in ns since Reset()
    uint32_t prevCt{0};       // last cycle-timer reading
    bool     hasPrev{false};

    /// Reject absurd inter-poll gaps (anomalous reads / long stalls) so a single
    /// bad delta cannot corrupt the monotonic clock. Polls are sub-ms..few-ms.
    static constexpr int64_t kMaxPollDeltaNanos = 100'000'000;  // 100 ms

    void Reset() noexcept {
        deviceNanos = 0;
        prevCt = 0;
        hasPrev = false;
    }

    /// Advance by the wrap-corrected delta from the previous reading and return
    /// the device sample-time (frames) for the given sample rate.
    [[nodiscard]] uint64_t Advance(uint32_t cycleTimer, double sampleRate) noexcept {
        if (hasPrev) {
            const int64_t dNs = deltaFWTimeNanos(cycleTimer, prevCt);
            if (dNs > 0 && dNs < kMaxPollDeltaNanos) {
                deviceNanos += static_cast<uint64_t>(dNs);
            }
        }
        prevCt = cycleTimer;
        hasPrev = true;
        return static_cast<uint64_t>(
            (static_cast<double>(deviceNanos) * sampleRate) / 1e9 + 0.5);
    }
};

//-----------------------------------------------------------------------------
// Zero-timestamp anchor PLL — smooth published timeline, hw-disciplined phase
//-----------------------------------------------------------------------------

/// Smooths the raw hardware zero-timestamp anchor into a low-jitter published
/// (sampleTime, hostTicks) timeline for UpdateCurrentZeroTimestamp.
///
/// Why (HW log 2026-06-10_21-22-25): the raw anchor pair is captured at the RX
/// poll instant, so successive publishes carry up to ~2.7 ms of phase noise.
/// The CoreAudio HAL schedules each IO wake directly off the published anchor
/// host time (per the 2026-06-09 HAL zero-timestamp cadence analysis),
/// so that noise wrecked the wake cadence: IO ran at ~161 cb/s instead of 250,
/// a chronic ~3.5% feed deficit that drained the TX cushion every ~400 ms. The
/// smooth software-accumulator path paced a perfect 250 cb/s in the same log.
///
/// The PLL therefore publishes a grid timeline — sampleTime advances by exactly
/// one buffer period per tick, hostTicks by the rate-disciplined (q8) period —
/// and uses the raw hw anchor only to slew phase, bounded per tick, the way
/// Apple's audio families filter takeTimeStamp jitter rather than republishing
/// raw interrupt times.
struct ZtsAnchorPll {
    bool     valid{false};
    uint64_t sampleTime{0};      // published sample-time (grid)
    uint64_t hostTicks{0};       // published host time (smooth, slewed)
    double   hostFrac{0.0};      // sub-tick accumulator for the period advance
    uint64_t lastHwHostTicks{0}; // freshness: slew only when the anchor moved

    /// Per-tick phase gain: correct 1/32 of the measured error each tick. The
    /// real disciplining need is tiny (~tens of ppm), so a low gain keeps the
    /// residual publish jitter at ~1/32 of the raw anchor noise.
    static constexpr double kPhaseGain = 1.0 / 32.0;
    /// Hard slew bound per tick as a fraction of the period — keeps hostTicks
    /// strictly monotonic (advance is always >= 99% of a period).
    static constexpr double kMaxSlewFractionOfPeriod = 0.01;
    /// If the anchor disagrees by more than this many periods, it restarted
    /// (e.g. stream re-prime reset the device sample clock): re-seed instead
    /// of slewing toward a bogus target for minutes.
    static constexpr double kReseedThresholdPeriods = 50.0;

    void Reset() noexcept {
        valid = false;
        sampleTime = 0;
        hostTicks = 0;
        hostFrac = 0.0;
        lastHwHostTicks = 0;
    }

    /// Advance the published timeline by one buffer period and, when a fresh
    /// hw anchor is available, slew phase toward it. Returns true when the
    /// published pair is valid (seeded); false means the caller must fall back
    /// (no hw anchor seen yet).
    /// @param periodFrames    zero-timestamp buffer period in frames
    /// @param ticksPerBuffer  rate-corrected host ticks per period (q8 path)
    /// @param haveAnchor      raw hw anchor readable this tick
    /// @param hwSample        raw anchor device sample-time
    /// @param hwHost          raw anchor host ticks (capture instant)
    [[nodiscard]] bool Tick(uint32_t periodFrames, double ticksPerBuffer,
                            bool haveAnchor, uint64_t hwSample,
                            uint64_t hwHost) noexcept {
        if (!valid) {
            if (!haveAnchor) {
                return false;
            }
            sampleTime = hwSample;
            hostTicks = hwHost;
            hostFrac = 0.0;
            lastHwHostTicks = hwHost;
            valid = true;
            return true;
        }

        const double exact = ticksPerBuffer + hostFrac;
        const auto step = static_cast<uint64_t>(exact);
        hostFrac = exact - static_cast<double>(step);
        sampleTime += periodFrames;
        hostTicks += step;

        if (haveAnchor && hwHost != lastHwHostTicks && periodFrames > 0) {
            lastHwHostTicks = hwHost;
            // Project the raw anchor onto the published sample-time and compare
            // host times there: err > 0 means we publish behind the hardware.
            const double ticksPerFrame = ticksPerBuffer / periodFrames;
            const double hwHostAtSample = static_cast<double>(hwHost)
                + (static_cast<double>(sampleTime) - static_cast<double>(hwSample))
                  * ticksPerFrame;
            const double err = hwHostAtSample - static_cast<double>(hostTicks);

            // Sample-axis staleness guard (HW log 2026-06-10_22-22-09): if ticks
            // arrive slower than one grid period each, the published pair stays
            // ON the device line — the projected host error above cannot see it —
            // while the pair recedes into the past. The raw anchor is captured
            // "now", so the grid sample must stay within the same threshold of it.
            const double sampleGap = static_cast<double>(sampleTime)
                - static_cast<double>(hwSample);
            const double maxSampleGap = kReseedThresholdPeriods
                * static_cast<double>(periodFrames);

            if (const double reseed = kReseedThresholdPeriods * ticksPerBuffer;
                err > reseed || err < -reseed
                || sampleGap > maxSampleGap || sampleGap < -maxSampleGap) {
                sampleTime = hwSample;
                hostTicks = hwHost;
                hostFrac = 0.0;
                return true;
            }

            double corr = err * kPhaseGain;
            const double maxSlew = ticksPerBuffer * kMaxSlewFractionOfPeriod;
            if (corr > maxSlew) corr = maxSlew;
            if (corr < -maxSlew) corr = -maxSlew;
            if (corr >= 0.0) {
                hostTicks += static_cast<uint64_t>(corr);
            } else {
                hostTicks -= static_cast<uint64_t>(-corr);
            }
        }
        return true;
    }
};

/// AppleUSBAudio-faithful anchor smoother (RE 2026-06-14, AppleUSBAudioEngine:
/// (frame,host) history regression + applyOffsetAmountToFilter + EMA jitter
/// bound). Where ZtsAnchorPll slews toward the *single latest* raw anchor — so
/// each tick re-injects that sample's full capture jitter — ZtsAnchorFit
/// publishes the anchor from a least-squares line fitted over a ring of recent
/// raw (sample,host) pairs. One fitted line yields BOTH the rate (slope) and the
/// phase (intercept), self-consistently, and averages the per-sample capture
/// noise away. An EMA jitter bound (alpha = 1/256, matching Apple's
/// updateMaxTimestampJitter) rejects outliers and detects true discontinuities.
struct ZtsAnchorFit {
    static constexpr int    kCapacity    = 128;        // Apple uses up to 1024; 128 ~= 1.4s @ ~512 fr/tick
    static constexpr int    kWarmup      = 16;         // min samples before the fit is trusted
    static constexpr double kJitterAlpha = 1.0 / 256.0;// EMA gain == AppleUSBAudio updateMaxTimestampJitter
    static constexpr double kOutlierK    = 6.0;        // |residual| > K*bound  => drop (don't pollute the fit)
    static constexpr double kReseedK     = 24.0;       // sustained |residual| > K*bound => clock restarted
    static constexpr int    kReseedRun   = 4;          // consecutive reseed-level residuals before reseeding

    bool     valid{false};
    uint64_t sampleTime{0};        // published grid sample axis (authoritative, advanced by caller)
    uint64_t hostTicks{0};         // published host time (from the fitted line)
    double   hostFrac{0.0};        // sub-tick accumulator for the warmup fallback advance
    uint64_t lastHwHostTicks{0};   // freshness: only act on a moved raw anchor

    // Jitter bound in HOST TICKS. Floor is timebase-dependent (~10us) so the
    // engine sets it once at Start; tests set it directly. EMA-tracked above it.
    double   jitterFloorTicks{0.0};
    double   jitterBound{0.0};
    int      outlierRun{0};

    // Raw anchor ring (absolute values; the fit is computed centered on the ring
    // mean, so magnitudes stay within double precision regardless of run length).
    uint64_t ringSample[kCapacity]{};
    uint64_t ringHost[kCapacity]{};
    int      count{0};
    int      head{0};
    double   slope{0.0};           // last fitted ticks-per-frame (the device rate)

    void Reset() noexcept {
        valid = false; sampleTime = 0; hostTicks = 0; hostFrac = 0.0;
        lastHwHostTicks = 0; jitterBound = 0.0; outlierRun = 0;
        count = 0; head = 0; slope = 0.0;
        // jitterFloorTicks intentionally preserved across Reset (timebase constant).
    }

    void Push(uint64_t s, uint64_t h) noexcept {
        ringSample[head] = s; ringHost[head] = h;
        head = (head + 1) % kCapacity;
        if (count < kCapacity) ++count;
    }

    /// Centered ordinary least squares over the ring. Returns slope b and the
    /// ring means (mf, mh); the published line is host = mh + b*(sample - mf).
    [[nodiscard]] bool FitLine(double& b, double& mf, double& mh) const noexcept {
        if (count < kWarmup) return false;
        const double n = static_cast<double>(count);
        double sf = 0.0, sh = 0.0;
        for (int k = 0; k < count; ++k) {
            sf += static_cast<double>(ringSample[k]);
            sh += static_cast<double>(ringHost[k]);
        }
        mf = sf / n; mh = sh / n;
        double sff = 0.0, sfh = 0.0;
        for (int k = 0; k < count; ++k) {
            const double df = static_cast<double>(ringSample[k]) - mf;
            const double dh = static_cast<double>(ringHost[k]) - mh;
            sff += df * df; sfh += df * dh;
        }
        if (sff <= 0.0) return false;
        b = sfh / sff;
        return true;
    }

    /// @param periodFrames           zero-timestamp buffer period in frames
    /// @param fallbackTicksPerBuffer q8-corrected period, used only until warm
    /// @param haveAnchor             raw hw anchor readable this tick
    /// @param hwSample / hwHost      raw anchor pair (device sample, capture host)
    [[nodiscard]] bool Tick(uint32_t periodFrames, double fallbackTicksPerBuffer,
                            bool haveAnchor, uint64_t hwSample,
                            uint64_t hwHost) noexcept {
        if (!valid) {
            if (!haveAnchor) return false;
            Push(hwSample, hwHost);
            sampleTime = hwSample; hostTicks = hwHost; hostFrac = 0.0;
            lastHwHostTicks = hwHost; jitterBound = jitterFloorTicks;
            valid = true;
            return true;
        }

        sampleTime += periodFrames;

        const bool fresh = haveAnchor && hwHost != lastHwHostTicks && periodFrames > 0;
        if (fresh) {
            lastHwHostTicks = hwHost;
            double b = 0.0, mf = 0.0, mh = 0.0;
            if (FitLine(b, mf, mh)) {
                const double predict = mh + b * (static_cast<double>(hwSample) - mf);
                const double resid = static_cast<double>(hwHost) - predict;
                const double aresid = resid < 0.0 ? -resid : resid;
                const double bound = jitterBound > jitterFloorTicks ? jitterBound : jitterFloorTicks;
                if (aresid > kReseedK * bound) {
                    if (++outlierRun > kReseedRun) {
                        // True discontinuity (e.g. re-prime reset the device sample
                        // clock): drop stale history and re-seed on this anchor.
                        count = 0; head = 0; outlierRun = 0;
                        Push(hwSample, hwHost);
                        sampleTime = hwSample; hostTicks = hwHost; hostFrac = 0.0;
                        jitterBound = jitterFloorTicks;
                        return true;
                    }
                } else {
                    outlierRun = 0;
                    if (aresid <= kOutlierK * bound) {
                        Push(hwSample, hwHost);
                        jitterBound += (aresid - jitterBound) * kJitterAlpha;
                        if (jitterBound < jitterFloorTicks) jitterBound = jitterFloorTicks;
                    }
                    // between kOutlierK and kReseedK: transient noise — reject, hold bound.
                }
            } else {
                Push(hwSample, hwHost); // still warming up
            }
        }

        double b = 0.0, mf = 0.0, mh = 0.0;
        if (FitLine(b, mf, mh)) {
            slope = b;
            const double host = mh + b * (static_cast<double>(sampleTime) - mf);
            hostTicks = host < 0.0 ? 0u : static_cast<uint64_t>(host);
        } else {
            const double exact = fallbackTicksPerBuffer + hostFrac;
            const auto step = static_cast<uint64_t>(exact);
            hostFrac = exact - static_cast<double>(step);
            hostTicks += step;
        }
        return true;
    }
};

} // namespace ASFW::Timing
