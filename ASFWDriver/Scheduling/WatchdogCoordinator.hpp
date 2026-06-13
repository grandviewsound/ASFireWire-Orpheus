#pragma once

#include <cstdint>

#ifdef ASFW_HOST_TEST
#include "../Testing/HostDriverKitStubs.hpp"
#else
#include <DriverKit/IODispatchQueue.h>
#include <DriverKit/IOTimerDispatchSource.h>
#include <DriverKit/OSAction.h>
#include <DriverKit/OSSharedPtr.h>
#endif

namespace ASFW {
namespace Async {
class IAsyncSubsystemPort;
}
namespace Isoch {
class IsochReceiveContext;
class IsochTransmitContext;
} // namespace Isoch
} // namespace ASFW

class ASFWDriver;

namespace ASFW::Driver {
class ControllerCore;
class StatusPublisher;

class WatchdogCoordinator {
  public:
    WatchdogCoordinator() = default;
    ~WatchdogCoordinator() = default;

    kern_return_t Prepare(::ASFWDriver& service, OSSharedPtr<IODispatchQueue> workQueue);
    void Stop();
    void Reset();

    void Schedule(uint64_t delayUsec);

    void HandleTick(ControllerCore* controller, ASFW::Async::IAsyncSubsystemPort* asyncSubsystem,
                    ASFW::Isoch::IsochReceiveContext* isochReceiveContext,
                    ASFW::Isoch::IsochTransmitContext* isochTransmitContext,
                    StatusPublisher& statusPublisher);

    // Deadline-anchored re-arm math, header-inline for host testing. Given the
    // previously armed deadline, the current time, and the period (all in mach
    // ticks), returns the next deadline anchored on the prior one rather than on
    // `now`, so handler execution + delivery lag become a constant phase offset
    // instead of compounding into the effective period. Re-anchors at `now` on
    // the first arm (prevDeadline == 0) or after a long stall (> 64 periods).
    [[nodiscard]] static uint64_t ComputeNextDeadline(uint64_t prevDeadline, uint64_t now,
                                                      uint64_t period) noexcept {
        if (period == 0) {
            return now;  // defensive: never spin the catch-up loop forever
        }
        uint64_t deadline = prevDeadline;
        // First arm, or stalled longer than 64 periods (e.g. across system
        // sleep): re-anchor at `now` rather than stepping across the whole gap.
        if (deadline == 0 || deadline + period * 64 < now) {
            deadline = now;
        }
        // Extend the prior deadline by whole periods until it lands in the
        // future. A handler slower than one period skips beats here (no
        // double-poll) while keeping the average cadence equal to `period`.
        do {
            deadline += period;
        } while (deadline <= now);
        return deadline;
    }

  private:
    OSSharedPtr<IOTimerDispatchSource> timer_;
    OSSharedPtr<OSAction> action_;
    uint64_t nextDeadline_{0};
    uint32_t isochLogDivider_{0};
    uint32_t itLogDivider_{0};
};

} // namespace ASFW::Driver
