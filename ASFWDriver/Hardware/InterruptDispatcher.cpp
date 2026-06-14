#include "InterruptDispatcher.hpp"

#include "../Async/Interfaces/IAsyncSubsystemPort.hpp"
#include "../Controller/ControllerCore.hpp"
#include "../Diagnostics/StatusPublisher.hpp"
#include "../Isoch/IsochService.hpp"
#include "../Logging/Logging.hpp"
#include "HardwareInterface.hpp"
#include "OHCIConstants.hpp"
#include "RegisterMap.hpp"

namespace ASFW::Driver {

void InterruptDispatcher::HandleSnapshot(const InterruptSnapshot& snap, ControllerCore& controller,
                                         HardwareInterface& hardware, IODispatchQueue& workQueue,
                                         IsochService& isoch, StatusPublisher& statusPublisher,
                                         ASFW::Async::IAsyncSubsystemPort* asyncSubsystem) {
    controller.HandleInterrupt(snap);

    // ===== ISOCHRONOUS RECEIVE INTERRUPT =====
    // Per OHCI §9.1: kIsochRx (bit 7) indicates one or more IR contexts have completed descriptors.
    // We read isoRecvEvent to determine which contexts, clear it, then dispatch processing.
    if ((snap.intEvent & IntEventBits::kIsochRx) && snap.isoRecvEvent != 0) {
        // Clear the per-context event bits to acknowledge
        hardware.Write(Register32::kIsoRecvIntEventClear, snap.isoRecvEvent);

        // Context 0 is our single IR context for now
        if ((snap.isoRecvEvent & 0x01) && isoch.ReceiveContext()) {
            // Dispatch descriptor processing to workqueue (deferred from ISR)
            workQueue.DispatchAsync(^{
              if (isoch.ReceiveContext()) {
                  isoch.ReceiveContext()->Poll();
              }
            });
        }
    }

    // ===== ISOCHRONOUS TRANSMIT INTERRUPT =====
    // Per OHCI §9.2: kIsochTx (bit 6) indicates IT context completion.
    //
    // Apple-faithful drain (AppleFWOHCI_DCLProgram::handleInterrupt, FWOHCI_IR.md:338):
    // re-read IsoXmitEvent FRESH, clear it, process, then re-check in a loop. We must NOT
    // clear the stale wide-read `snap.isoXmitEvent` captured back in
    // CaptureInterruptSnapshot: a completion landing in that read→clear window is lost by the
    // write-1-to-clear, which latches IsoXmitEvent bit0 = 1, keeps the IntEvent.isochTx summary
    // asserted, and the MSI edge never re-fires — the IT IRQ freezes permanently while DMA runs
    // (reproduced HW soak 2026-06-14: IRQ stuck at 58818, IsoXmitEvent=0x1 for the whole run).
    // The main IntEvent (incl. isochTx) is already acked ahead of us in ControllerCore::
    // HandleInterrupt(), so this matches Linux ohci.c ack-first ordering.
    if (snap.intEvent & IntEventBits::kIsochTx) {
        // Single IT context; completions coalesce into bit0, so a small bound suffices while
        // still letting us drain anything HW sets while we were processing.
        unsigned int safety = 8;
        uint32_t xmitEvent = hardware.Read(Register32::kIsoXmitEvent); // fresh read
        while (xmitEvent != 0 && safety-- > 0) {
            // Ack the bits we just observed (flushed, so the clear lands before the re-read).
            hardware.ClearIsoXmitEvents(xmitEvent);

            // DEBUG: Sample interrupt rate
            static uint32_t txIrqCtr = 0;
            if ((++txIrqCtr % 100) == 0) {
                ASFW_LOG_V3(Controller, "[IRQ] IsoTx Fired! Count=%u IsoTxEvent=0x%08x", txIrqCtr,
                            xmitEvent);
            }

            // Context 0 is our single IT context. Process directly in ISR context for lowest
            // latency (IT RefillRing is a fast atomic assemble + mem writes; DispatchAsync would
            // add latency that risks underruns with small buffers).
            if ((xmitEvent & 0x01) && isoch.TransmitContext()) {
                isoch.TransmitContext()->HandleInterrupt();
            }

            // Re-check: catch completions HW set during processing before we leave the ISR.
            xmitEvent = hardware.Read(Register32::kIsoXmitEvent);
        }
    }

    if (snap.intEvent != 0) {
        const uint32_t asyncMask = IntEventBits::kReqTxComplete | IntEventBits::kRespTxComplete |
                                   IntEventBits::kARRQ | IntEventBits::kARRS |
                                   IntEventBits::kRQPkt | IntEventBits::kRSPkt;
        if (snap.intEvent & asyncMask) {
            statusPublisher.SetLastAsyncCompletion(mach_absolute_time());
        }

        SharedStatusReason reason = SharedStatusReason::Interrupt;
        if (snap.intEvent & IntEventBits::kBusReset) {
            reason = SharedStatusReason::BusReset;
        } else if (snap.intEvent & asyncMask) {
            reason = SharedStatusReason::AsyncActivity;
        } else if (snap.intEvent & IntEventBits::kUnrecoverableError) {
            reason = SharedStatusReason::Interrupt;
        }

        statusPublisher.Publish(&controller, asyncSubsystem, reason, snap.intEvent);
    }
}

} // namespace ASFW::Driver
