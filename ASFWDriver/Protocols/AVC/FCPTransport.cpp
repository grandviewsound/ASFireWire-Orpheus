//
// FCPTransport.cpp
// ASFWDriver - AV/C Protocol Layer
//
// FCP (Function Control Protocol) transport layer implementation
//

#include "FCPTransport.hpp"
#include "../../Logging/Logging.hpp"
#ifndef ASFW_HOST_TEST
#include <DriverKit/OSCollections.h>
#include <DriverKit/IOLib.h>
#endif
#include <time.h>

using namespace ASFW::Protocols::AVC;

//==============================================================================
// Constructor / Destructor
//==============================================================================

// OSDefineMetaClassAndStructors(FCPTransport, OSObject);

//==============================================================================
// Init / Free
//==============================================================================

bool FCPTransport::init(Protocols::Ports::FireWireBusOps* busOps,
                        Protocols::Ports::FireWireBusInfo* busInfo,
                        Discovery::FWDevice* device,
                        const FCPTransportConfig& config) {
    if (!OSObject::init()) {
        return false;
    }

    busOps_ = busOps;
    busInfo_ = busInfo;
    device_ = device;
    config_ = config;

    if (!busOps_ || !busInfo_) {
        ASFW_LOG_V1(FCP, "FCPTransport: Missing FireWire bus ports");
        return false;
    }

    // Allocate lock (DriverKit IOLock)
    lock_ = IOLockAlloc();
    if (!lock_) {
        ASFW_LOG_V1(FCP, "FCPTransport: Failed to allocate lock");
        return false;
    }

    // Create dedicated DriverKit dispatch queue for timeout handling
    IODispatchQueue* queue = nullptr;
    IODispatchQueueName queueName = "com.asfw.fcp.timeout";
    auto kr = IODispatchQueue::Create(queueName, 0, 0, &queue);
    if (kr != kIOReturnSuccess || !queue) {
        ASFW_LOG_V1(FCP, "FCPTransport: Failed to create timeout queue (kr=0x%08x)", kr);
    } else {
        timeoutQueue_ = OSSharedPtr(queue, OSNoRetain);
    }

    ASFW_LOG_V1(FCP,
                "FCPTransport: Initialized for device nodeID=%u, "
                "cmdAddr=0x%llx, rspAddr=0x%llx",
                device_->GetNodeID(), config_.commandAddress,
                config_.responseAddress);
    
    return true;
}

void FCPTransport::free() {
    shuttingDown_ = true;

    // Cancel any pending command
    if (pending_) {
        CompleteCommand(FCPStatus::kTransportError, {});
    }

    // Release queue
    timeoutQueue_.reset();

    // Free lock (cannot throw in DriverKit)
    if (lock_) {
        IOLockFree(lock_);
        lock_ = nullptr;
    }

    ASFW_LOG_V1(FCP, "FCPTransport: Destroyed");
    
    OSObject::free();
}

//==============================================================================
// Command Submission
//==============================================================================

FCPHandle FCPTransport::SubmitCommand(const FCPFrame& command,
                                      FCPCompletion completion) {
    if (!command.IsValid()) {
        ASFW_LOG_V1(FCP,
                     "FCPTransport: Invalid command size %zu (must be 3-512)",
                     command.length);
        completion(FCPStatus::kInvalidPayload, {});
        return {};
    }

    if (shuttingDown_) {
        completion(FCPStatus::kTransportError, {});
        return {};
    }

    IOLockLock(lock_);

    if (pending_) {
        IOLockUnlock(lock_);

        ASFW_LOG_V1(FCP,
                     "FCPTransport: Command already pending");
        completion(FCPStatus::kBusy, {});
        return {};
    }

    auto cmd = std::make_unique<OutstandingCommand>();
    cmd->command = command;
    cmd->completion = std::move(completion);
    // Keep generation source consistent with RX routing: always query bus generation.
    cmd->generation = static_cast<uint32_t>(busInfo_->GetGeneration().value);
    cmd->retriesLeft = config_.maxRetries;
    cmd->allowBusResetRetry = config_.allowBusResetRetry;
    cmd->gotInterim = false;

    {
        char hexbuf[64] = {0};
        size_t hexlen = std::min(command.length, static_cast<size_t>(16));
        for (size_t i = 0; i < hexlen; i++) {
            snprintf(hexbuf + i*3, 4, "%02x ", command.data[i]);
        }
        ASFW_LOG_HEX(FCP,
                    "FCPTransport: Submitting command: opcode=0x%02x, length=%zu, "
                    "generation=%u, retries=%u, data=[%{public}s]",
                    command.data[2], command.length,
                    cmd->generation, cmd->retriesLeft, hexbuf);
    }

    pending_ = std::move(cmd);

    FCPFrame commandCopy = pending_->command;

    IOLockUnlock(lock_);

    auto handle = SubmitWriteCommand(commandCopy);
    if (!handle.value) {
        IOLockLock(lock_);
        if (!pending_) {
            IOLockUnlock(lock_);
            return {};
        }

        auto completionCallback = std::move(pending_->completion);
        pending_.reset();

        IOLockUnlock(lock_);
        ASFW_LOG_V1(FCP,
                     "FCPTransport: Failed to submit async write");
        completionCallback(FCPStatus::kTransportError, {});
        return {};
    }

    IOLockLock(lock_);
    if (!pending_) {
        IOLockUnlock(lock_);
        busOps_->Cancel(handle);
        return {};
    }

    pending_->asyncHandle = handle;

    ScheduleTimeout(config_.timeoutMs);

    IOLockUnlock(lock_);

    return FCPHandle{kTransactionID};
}

ASFW::Async::AsyncHandle FCPTransport::SubmitWriteCommand(const FCPFrame& frame) {
    if (!pending_) {
        return Async::AsyncHandle{0};
    }

    const FW::Generation gen{pending_->generation};
    const FW::NodeId node{static_cast<uint8_t>(device_->GetNodeID() & 0x3Fu)};
    const Async::FWAddress addr{Async::FWAddress::AddressParts{
        .addressHi = static_cast<uint16_t>((config_.commandAddress >> 32U) & 0xFFFFU),
        .addressLo = static_cast<uint32_t>(config_.commandAddress & 0xFFFFFFFFU),
    }};

    // FCP diagnostic: log exact bytes being sent to device's FCP_COMMAND register
    {
        auto payload = frame.Payload();
        if (payload.size() >= 4) {
            ASFW_LOG(FCP,
                     "FCP TX → node=%u gen=%u addr=0x%04x%08x len=%zu bytes=[%02X %02X %02X %02X%s]",
                     node.value, gen.value,
                     addr.addressHi, addr.addressLo,
                     payload.size(),
                     payload[0], payload[1], payload[2], payload[3],
                     payload.size() > 4 ? " ..." : "");
        } else {
            ASFW_LOG(FCP, "FCP TX → node=%u gen=%u len=%zu (too short!)",
                     node.value, gen.value, payload.size());
        }
    }

    return busOps_->WriteBlock(gen,
                               node,
                               addr,
                               frame.Payload(),
                               FW::FwSpeed::S100,
                               [this](Async::AsyncStatus status, std::span<const uint8_t> response) {
                                   this->OnAsyncWriteComplete(status, response);
                               });
}

//==============================================================================
// Command Cancellation
//==============================================================================

bool FCPTransport::CancelCommand(FCPHandle handle) {
    if (!handle.IsValid() || handle.transactionID != kTransactionID) {
        return false;
    }

    IOLockLock(lock_);

    if (!pending_) {
        IOLockUnlock(lock_);
        return false;
    }

    ASFW_LOG_V2(FCP,
                "FCPTransport: Cancelling command");

    // Cancel async operation
    busOps_->Cancel(pending_->asyncHandle);

    IOLockUnlock(lock_);

    CompleteCommand(FCPStatus::kTransportError, {});

    return true;
}

//==============================================================================
// Response Reception
//==============================================================================

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void FCPTransport::OnFCPResponse(uint16_t srcNodeID,
                                 uint32_t generation,
                                 std::span<const uint8_t> payload) {
    // Diag: log every FCP response arrival with first bytes so we can tell
    // "response arrived but got dropped" from "response never arrived".
    const size_t pn = payload.size();
    auto pb = [&](size_t i) -> uint8_t { return i < pn ? payload[i] : 0xFFu; };
    ASFW_LOG_V1(FCP,
                 "FCPTransport: OnFCPResponse ENTER src=0x%04x gen=%u len=%zu "
                 "bytes=%02x %02x %02x %02x %02x %02x %02x %02x",
                 srcNodeID, generation, pn,
                 pb(0), pb(1), pb(2), pb(3), pb(4), pb(5), pb(6), pb(7));

    IOLockLock(lock_);

    if (!pending_) {
        IOLockUnlock(lock_);
        ASFW_LOG_V1(FCP,
                     "FCPTransport: Spurious response (no pending command) src=0x%04x",
                     srcNodeID);
        return;
    }

    const uint16_t expectedNodeID = device_->GetNodeID();
    const bool exactMatch = srcNodeID == expectedNodeID;
    const bool nodeNumberMatch = (srcNodeID & 0x3F) == (expectedNodeID & 0x3F);
    if (!exactMatch && !nodeNumberMatch) {
        IOLockUnlock(lock_);
        ASFW_LOG_V1(FCP,
                     "FCPTransport: Response from wrong node: 0x%04x (expected node 0x%02x)",
                     srcNodeID, expectedNodeID & 0x3F);
        return;
    }
    if (!exactMatch && nodeNumberMatch) {
        ASFW_LOG_V3(FCP,
                     "FCPTransport: Accepting response with matching node number but different bus ID "
                     "(src=0x%04x expected=0x%04x)",
                     srcNodeID, expectedNodeID);
    }

    // Generation value can be unknown (0) in some receive paths while the bus is still
    // converging; accept unknown generation as wildcard to avoid dropping valid responses.
    if (!pending_->allowBusResetRetry &&
        generation != 0 &&
        pending_->generation != 0 &&
        generation != pending_->generation) {
        const uint32_t pendingGen = pending_->generation;
        IOLockUnlock(lock_);
        ASFW_LOG_V1(FCP,
                     "FCPTransport: Response generation mismatch: %u (expected %u)",
                     generation, pendingGen);
        return;
    }
    if (!pending_->allowBusResetRetry &&
        (generation == 0 || pending_->generation == 0)) {
        ASFW_LOG_V3(FCP,
                    "FCPTransport: Accepting response with unknown generation "
                    "(rx=%u pending=%u)",
                    generation,
                    pending_->generation);
    }

    if (!ValidateResponse(payload)) {
        IOLockUnlock(lock_);
        ASFW_LOG_V1(FCP,
                     "FCPTransport: Response validation failed (likely stale/duplicate response)");
        return;
    }

    FCPFrame response;
    response.length = std::min(payload.size(), response.data.size());
    std::copy_n(payload.begin(), response.length, response.data.begin());

    ASFW_LOG_V2(FCP,
                "FCPTransport: Received response: ctype=0x%02x, length=%zu",
                response.data[0], response.length);

    if (response.data[0] == static_cast<uint8_t>(AVCResponseType::kInterim)) {
        pending_->gotInterim = true;

        ASFW_LOG_V2(FCP,
                    "FCPTransport: Got INTERIM response, extending timeout to %u ms",
                    config_.interimTimeoutMs);

        ScheduleTimeout(config_.interimTimeoutMs);

        IOLockUnlock(lock_);
        return;
    }

    IOLockUnlock(lock_);

    CompleteCommand(FCPStatus::kOk, response);
}

//==============================================================================
// Async Write Completion
//==============================================================================

void FCPTransport::OnAsyncWriteComplete(Async::AsyncStatus status,
                                        std::span<const uint8_t> response) {
    (void)response;
    IOLockLock(lock_);

    if (!pending_) {
        IOLockUnlock(lock_);
        return;
    }

    if (status != Async::AsyncStatus::kSuccess) {
        ASFW_LOG_V1(FCP,
                     "FCPTransport: Async write failed: %d",
                     static_cast<int>(status));

        if (pending_->retriesLeft > 0) {
            pending_->retriesLeft--;
            ASFW_LOG_V2(FCP,
                        "FCPTransport: Retrying command (%u retries left)",
                        pending_->retriesLeft);

            IOLockUnlock(lock_);
            RetryCommand();
            return;
        }

        IOLockUnlock(lock_);

        CompleteCommand(FCPStatus::kTransportError, {});
        return;
    }

    IOLockUnlock(lock_);
}

//==============================================================================
// Timeout Handling
//==============================================================================

void FCPTransport::OnCommandTimeout() {
    IOLockLock(lock_);

    if (!pending_) {
        IOLockUnlock(lock_);
        return;
    }

    const auto& f = pending_->command;
    const size_t n = f.length;
    auto byteAt = [&](size_t i) -> uint8_t { return i < n ? f.data[i] : 0xFFu; };
    ASFW_LOG_V1(FCP,
                 "FCPTransport: Command timeout (interim=%d len=%zu gen=%u "
                 "handle=0x%08x tLabelRaw=%u "
                 "bytes=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x)",
                 pending_->gotInterim, n, pending_->generation,
                 pending_->asyncHandle.value,
                 static_cast<uint32_t>(pending_->asyncHandle.value ? pending_->asyncHandle.value - 1 : 0),
                 byteAt(0), byteAt(1), byteAt(2), byteAt(3),
                 byteAt(4), byteAt(5), byteAt(6), byteAt(7),
                 byteAt(8), byteAt(9), byteAt(10), byteAt(11));

    // Do not retry on timeout. AV/C CONTROL commands are not idempotent — the
    // target already received and may be processing the original write, and a
    // delayed response landing after a retry races the second transaction.
    // Let the caller decide whether retrying this specific command is safe.
    IOLockUnlock(lock_);
    CompleteCommand(FCPStatus::kTimeout, {});
}

void FCPTransport::ScheduleTimeout(uint32_t timeoutMs) {
    if (!pending_) {
        return;
    }

    pending_->timeoutToken = ++nextTimeoutToken_;

    auto queue = timeoutQueue_;
    if (!queue) {
        ASFW_LOG_V1(FCP,
                     "FCPTransport: Timeout queue unavailable (timeoutMs=%u)",
                     timeoutMs);
        return;
    }

    const uint64_t token = pending_->timeoutToken;

    queue->DispatchAsync(^{
        IOLockLock(lock_);
        bool stillPending = (pending_ && pending_->timeoutToken == token);
        IOLockUnlock(lock_);

        if (!stillPending) {
            return;
        }

        IOSleep(static_cast<uint64_t>(timeoutMs));

        IOLockLock(lock_);
        bool shouldFire = (pending_ && pending_->timeoutToken == token);
        if (shouldFire) {
            pending_->timeoutToken = 0;
        }
        IOLockUnlock(lock_);

        if (shouldFire) {
            OnCommandTimeout();
        }
    });
}

void FCPTransport::CancelTimeout() {
    if (pending_) {
        pending_->timeoutToken = 0;
    }
}

//==============================================================================
// Retry Logic
//==============================================================================

void FCPTransport::RetryCommand() {
    IOLockLock(lock_);

    if (!pending_) {
        IOLockUnlock(lock_);
        return;
    }

    // Keep retries aligned with Async/RX generation source.
    pending_->generation = static_cast<uint32_t>(busInfo_->GetGeneration().value);
    pending_->gotInterim = false;

    ASFW_LOG_V2(FCP,
                "FCPTransport: Retrying command with generation=%u",
                pending_->generation);

    FCPFrame commandCopy = pending_->command;
    IOLockUnlock(lock_);

    auto handle = SubmitWriteCommand(commandCopy);
    if (!handle.value) {
        ASFW_LOG_V1(FCP,
                     "FCPTransport: Async write submission failed during retry");
        CompleteCommand(FCPStatus::kTransportError, {});
        return;
    }

    IOLockLock(lock_);
    if (!pending_) {
        IOLockUnlock(lock_);
        busOps_->Cancel(handle);
        return;
    }

    pending_->asyncHandle = handle;

    ScheduleTimeout(config_.timeoutMs);

    IOLockUnlock(lock_);
}

//==============================================================================
// Bus Reset Handling
//==============================================================================

void FCPTransport::OnBusReset(uint32_t newGeneration) {
    IOLockLock(lock_);

    if (!pending_) {
        IOLockUnlock(lock_);
        return;
    }

    ASFW_LOG_V2(FCP,
                "FCPTransport: Bus reset during command (gen %u → %u, "
                "allowRetry=%d, retriesLeft=%u)",
                pending_->generation, newGeneration,
                pending_->allowBusResetRetry, pending_->retriesLeft);

    if (pending_->allowBusResetRetry && pending_->retriesLeft > 0) {
        pending_->retriesLeft--;
        pending_->generation = newGeneration;
        pending_->gotInterim = false;

        ASFW_LOG_V2(FCP,
                    "FCPTransport: Retrying command after bus reset");

        IOLockUnlock(lock_);
        RetryCommand();
        return;
    } else {
        IOLockUnlock(lock_);

        CompleteCommand(FCPStatus::kBusReset, {});
        return;
    }
}

bool FCPTransport::ValidateResponse(std::span<const uint8_t> response) const {
    if (response.size() < kAVCFrameMinSize) {
        ASFW_LOG_V1(FCP,
                     "FCPTransport: Response too small: %zu bytes",
                     response.size());
        return false;
    }

    if (response.size() > kAVCFrameMaxSize) {
        ASFW_LOG_V1(FCP,
                     "FCPTransport: Response too large: %zu bytes",
                     response.size());
        return false;
    }

    uint8_t cmdAddress = pending_->command.data[1];
    uint8_t rspAddress = response[1];

    if (cmdAddress != rspAddress) {
        ASFW_LOG_V1(FCP,
                     "FCPTransport: Response address mismatch: 0x%02x (expected 0x%02x)",
                     rspAddress, cmdAddress);
        return false;
    }

    uint8_t cmdOpcode = pending_->command.data[2];
    uint8_t rspOpcode = response[2];

    bool opcodeMatches = false;

    if (((cmdAddress & 0xF8) == 0x20) && (cmdOpcode == 0xD0)) {
        opcodeMatches = (rspOpcode == 0xD0 || rspOpcode == 0xC1 ||
                        rspOpcode == 0xC2 || rspOpcode == 0xC3 ||
                        rspOpcode == 0xC4);

        if (!opcodeMatches) {
            ASFW_LOG_V1(FCP,
                         "FCPTransport: Tape transport-state response opcode invalid: 0x%02x",
                         rspOpcode);
        }
    } else {
        opcodeMatches = ((rspOpcode & 0x7F) == (cmdOpcode & 0x7F));

        if (!opcodeMatches) {
            ASFW_LOG_V1(FCP,
                         "FCPTransport: Response opcode mismatch: 0x%02x (expected 0x%02x)",
                         rspOpcode, cmdOpcode);
        }
    }

    if (!opcodeMatches) {
        return false;
    }

    // Tighten matching for the command families we flood during Orpheus
    // discovery/startup so late responses cannot satisfy the wrong pending
    // request purely because the subunit/opcode pair matches.
    auto matchByte = [&](size_t index, const char* label) -> bool {
        if (pending_->command.length <= index || response.size() <= index) {
            return true;
        }
        if (pending_->command.data[index] != response[index]) {
            ASFW_LOG_V1(FCP,
                         "FCPTransport: Response %s mismatch at byte %zu: 0x%02x (expected 0x%02x)",
                         label,
                         index,
                         response[index],
                         pending_->command.data[index]);
            return false;
        }
        return true;
    };

    if (cmdOpcode == 0xBF) {
        // Match subfunction + addressing tuple. This disambiguates the heavy
        // mix of C0/C1, input/output, and unit-vs-subunit plug traffic seen
        // during Orpheus attach without over-constraining response-specific
        // status/format bytes.
        return matchByte(3, "subfunction") &&
               matchByte(4, "direction") &&
               matchByte(5, "address/type") &&
               matchByte(6, "plug");
    }

    if (cmdOpcode == 0x1A) {
        // SIGNAL SOURCE has response layouts that do NOT mirror the request's
        // first operand byte.
        //
        // Working Apple Orpheus captures show two important cases:
        //  - QuerySyncPlugReconnect (SPECIFIC INQUIRY, operand[0]=0x0F):
        //    the response changes byte 3 to 0x30 / 0x08, but preserves bytes
        //    4..7 as the queried target tuple.
        //  - STATUS signal-source topology queries:
        //    the response rewrites bytes 3..5 with source information and only
        //    echoes the destination tuple in bytes 6..7.
        //
        // Matching byte 3 as if it were echoed causes valid Apple-style
        // responses to be rejected, which leaves discovery wedged behind a
        // "pending" command until timeout.
        if (pending_->command.length > 3 && pending_->command.data[3] == 0x0F) {
            return matchByte(4, "target-subunit") &&
                   matchByte(5, "target-plug") &&
                   matchByte(6, "signal-source-arg0") &&
                   matchByte(7, "signal-source-arg1");
        }

        return matchByte(6, "dest-plug-type") &&
               matchByte(7, "dest-plug");
    }

    return opcodeMatches;
}

//==============================================================================
// Command Completion
//==============================================================================

void FCPTransport::CompleteCommand(FCPStatus status, const FCPFrame& response) {
    // Must NOT be called with lock held

    IOLockLock(lock_);

    if (!pending_) {
        IOLockUnlock(lock_);
        return;
    }

    auto completion = std::move(pending_->completion);

    // Cancel timeout
    CancelTimeout();

    // Clear pending
    pending_.reset();

    IOLockUnlock(lock_);

    // Invoke completion OUTSIDE lock
    completion(status, response);
}
