//
// CMPClient.cpp
// ASFWDriver - CMP (Connection Management Procedures)
//
// CMP client implementation for connecting to device's PCR registers.
//

#include "CMPClient.hpp"
#include "../../../Common/CallbackUtils.hpp"
#include "../../../Logging/Logging.hpp"
#include <DriverKit/IOLib.h>
#include <os/log.h>

namespace ASFW::CMP {

// ============================================================================
// Constructor / Destructor
// ============================================================================

CMPClient::CMPClient(Async::IFireWireBusOps& busOps)
    : busOps_(busOps)
{
}

CMPClient::~CMPClient() = default;

// ============================================================================
// Configuration
// ============================================================================

void CMPClient::SetDeviceNode(uint8_t nodeId, IRM::Generation generation) {
    deviceNodeId_ = nodeId;
    generation_ = generation;
    
    ASFW_LOG(CMP, "CMPClient: Set device node=%u generation=%u",
             nodeId, generation.value);
}

// ============================================================================
// Internal Helpers
// ============================================================================

void CMPClient::ReadPCRQuadlet(uint32_t addressLo, PCRReadCallback callback) {
    auto callbackState = Common::ShareCallback(std::move(callback));
    Async::FWAddress addr{Async::FWAddress::AddressParts{
        .addressHi = PCRRegisters::kAddressHi,
        .addressLo = addressLo,
    }};
    
    // PCR operations use device's max speed (typically S400)
    // Note: CMP to device PCRs can use full speed (unlike IRM which requires S100)
    FW::FwSpeed speed{2};  // S400
    FW::NodeId node{deviceNodeId_};
    FW::Generation gen{generation_};
    
    ASFW_LOG(CMP, "CMPClient: Reading PCR at 0x%08X (node=%u gen=%u)",
             addressLo, deviceNodeId_, generation_.value);
    
    busOps_.ReadQuad(gen, node, addr, speed,
        [callbackState, addressLo](Async::AsyncStatus status, std::span<const uint8_t> payload) {
            if (status == Async::AsyncStatus::kSuccess && payload.size() == 4) {
                uint32_t raw = 0;
                std::memcpy(&raw, payload.data(), sizeof(raw));
                uint32_t hostValue = OSSwapBigToHostInt32(raw);
                
                ASFW_LOG(CMP, "CMPClient: Read PCR 0x%08X = 0x%08X (online=%d p2p=%u ch=%u)",
                         addressLo, hostValue,
                         PCRBits::IsOnline(hostValue),
                         PCRBits::GetP2P(hostValue),
                         PCRBits::GetChannel(hostValue));
                
                Common::InvokeSharedCallback(callbackState, true, hostValue);
            } else {
                ASFW_LOG(CMP,
                         "CMPClient: Read PCR 0x%08X failed: status=%{public}s(%u)",
                         addressLo,
                         ASFW::Async::ToString(status),
                         static_cast<unsigned>(status));
                Common::InvokeSharedCallback(callbackState, false, 0u);
            }
        });
}

void CMPClient::CompareSwapPCR(uint32_t addressLo, uint32_t expected, uint32_t desired,
                                CMPCallback callback) {
    auto callbackState = Common::ShareCallback(std::move(callback));
    Async::FWAddress addr{Async::FWAddress::AddressParts{
        .addressHi = PCRRegisters::kAddressHi,
        .addressLo = addressLo,
    }};
    
    FW::FwSpeed speed{2};  // S400
    FW::NodeId node{deviceNodeId_};
    FW::Generation gen{generation_};
    
    // Build CAS operand: [compare_value][swap_value] in big-endian
    std::array<uint8_t, 8> operand;
    uint32_t expectedBE = OSSwapHostToBigInt32(expected);
    uint32_t desiredBE = OSSwapHostToBigInt32(desired);
    std::memcpy(&operand[0], &expectedBE, 4);
    std::memcpy(&operand[4], &desiredBE, 4);
    
    ASFW_LOG(CMP, "CMPClient: Lock PCR 0x%08X: 0x%08X → 0x%08X",
             addressLo, expected, desired);
    
    busOps_.Lock(gen, node, addr, FW::LockOp::kCompareSwap,
        std::span{operand}, 4, speed,
        [callbackState, expected, desired, addressLo](Async::AsyncStatus status, std::span<const uint8_t> payload) {
            if (status == Async::AsyncStatus::kSuccess && payload.size() == 4) {
                uint32_t raw = 0;
                std::memcpy(&raw, payload.data(), sizeof(raw));
                uint32_t oldValue = OSSwapBigToHostInt32(raw);
                
                bool succeeded = (oldValue == expected);
                if (succeeded) {
                    ASFW_LOG(CMP, "CMPClient: Lock PCR 0x%08X succeeded (0x%08X → 0x%08X)",
                             addressLo, expected, desired);
                    Common::InvokeSharedCallback(callbackState, CMPStatus::Success);
                } else {
                    ASFW_LOG(CMP, "CMPClient: Lock PCR 0x%08X contention (expected=0x%08X actual=0x%08X)",
                             addressLo, expected, oldValue);
                    Common::InvokeSharedCallback(callbackState, CMPStatus::Failed);
                }
            } else {
                ASFW_LOG(CMP,
                         "CMPClient: Lock PCR 0x%08X failed: status=%{public}s(%u)",
                         addressLo,
                         ASFW::Async::ToString(status),
                         static_cast<unsigned>(status));
                Common::InvokeSharedCallback(callbackState, CMPStatus::Failed);
            }
        });
}

// ============================================================================
// oPCR Operations (device→host stream)
// ============================================================================

void CMPClient::ReadOPCR(uint8_t plugNum, PCRReadCallback callback) {
    if (plugNum > 30) {
        ASFW_LOG(CMP, "CMPClient: Invalid oPCR plug number %u", plugNum);
        callback(false, 0);
        return;
    }
    
    ReadPCRQuadlet(PCRRegisters::GetOPCRAddress(plugNum), callback);
}

void CMPClient::ConnectOPCR(uint8_t plugNum, uint8_t channel, uint8_t speed, CMPCallback callback) {
    if (plugNum > 30) {
        ASFW_LOG(CMP, "CMPClient: Invalid oPCR plug number %u", plugNum);
        callback(CMPStatus::Failed);
        return;
    }
    if (channel > 63) {
        ASFW_LOG(CMP, "CMPClient: Invalid channel %u for oPCR", channel);
        callback(CMPStatus::Failed);
        return;
    }

    ASFW_LOG(CMP, "CMPClient: Connecting oPCR[%u] on channel %u speed %u", plugNum, channel, speed);
    PerformConnect(PCRRegisters::GetOPCRAddress(plugNum),
                   plugNum,
                   channel,
                   speed,
                   callback);
}

void CMPClient::DisconnectOPCR(uint8_t plugNum, CMPCallback callback) {
    if (plugNum > 30) {
        ASFW_LOG(CMP, "CMPClient: Invalid oPCR plug number %u", plugNum);
        callback(CMPStatus::Failed);
        return;
    }
    
    ASFW_LOG(CMP, "CMPClient: Disconnecting oPCR[%u]", plugNum);
    PerformDisconnect(PCRRegisters::GetOPCRAddress(plugNum), plugNum, callback);
}

// ============================================================================
// iPCR Operations (host→device stream)
// ============================================================================

void CMPClient::ReadIPCR(uint8_t plugNum, PCRReadCallback callback) {
    if (plugNum > 30) {
        ASFW_LOG(CMP, "CMPClient: Invalid iPCR plug number %u", plugNum);
        callback(false, 0);
        return;
    }
    
    ReadPCRQuadlet(PCRRegisters::GetIPCRAddress(plugNum), callback);
}

void CMPClient::ConnectIPCR(uint8_t plugNum, uint8_t channel, uint8_t speed, CMPCallback callback) {
    if (plugNum > 30) {
        ASFW_LOG(CMP, "CMPClient: Invalid iPCR plug number %u", plugNum);
        callback(CMPStatus::Failed);
        return;
    }
    if (channel > 63) {
        ASFW_LOG(CMP, "CMPClient: Invalid channel %u", channel);
        callback(CMPStatus::Failed);
        return;
    }

    ASFW_LOG(CMP,
             "CMPClient: Connecting iPCR[%u] on channel %u (speed %u ignored; iPCR speed bits are device-owned)",
             plugNum,
             channel,
             speed);
    (void)speed;
    PerformConnect(PCRRegisters::GetIPCRAddress(plugNum),
                   plugNum,
                   channel,
                   std::nullopt,
                   callback);
}

void CMPClient::DisconnectIPCR(uint8_t plugNum, CMPCallback callback) {
    if (plugNum > 30) {
        ASFW_LOG(CMP, "CMPClient: Invalid iPCR plug number %u", plugNum);
        callback(CMPStatus::Failed);
        return;
    }
    
    ASFW_LOG(CMP, "CMPClient: Disconnecting iPCR[%u]", plugNum);
    PerformDisconnect(PCRRegisters::GetIPCRAddress(plugNum), plugNum, callback);
}

// ============================================================================
// Private Implementation
// ============================================================================

// Apr 13 2026: Split into TWO compare-swap operations per direction to match
// Apple's dtrace: AppleFWAudio/AM824AVC fires `cmpNewPointToPointConnection`
// twice per plug bring-up. Stage A programs channel + speed with p2p UNCHANGED;
// stage B bumps p2p N→N+1. This sequencing gives the device firmware a
// quiescent window to latch channel/speed before the p2p edge that activates
// the connection — doing both in one CAS was wire-valid but didn't match the
// ordering the Orpheus firmware expects.
void CMPClient::PerformConnect(uint32_t pcrAddress, uint8_t plugNum,
                                std::optional<uint8_t> setChannel,
                                std::optional<uint8_t> setSpeed,
                                CMPCallback callback) {
    // Stage A: Read → set channel/speed (p2p unchanged) → CAS
    ReadPCRQuadlet(pcrAddress, [this, pcrAddress, plugNum, setChannel, setSpeed, callback](bool success, uint32_t current) {
        if (!success) {
            ASFW_LOG(CMP, "CMPClient: Connect failed - cannot read PCR 0x%08X (stage A)", pcrAddress);
            callback(CMPStatus::Failed);
            return;
        }

        if (!PCRBits::IsOnline(current)) {
            ASFW_LOG(CMP, "CMPClient: Connect failed - plug %u not online (PCR=0x%08X)",
                     plugNum, current);
            callback(CMPStatus::Failed);
            return;
        }

        uint8_t p2p = PCRBits::GetP2P(current);
        if (p2p >= 3) {
            ASFW_LOG(CMP, "CMPClient: Connect failed - p2p count already max (%u)", p2p);
            callback(CMPStatus::NoResources);
            return;
        }

        // Stage A new value: preserve p2p, set channel/speed.
        // Per Linux kernel sound/firewire/cmp.c: speed is set in oPCR but NOT
        // in iPCR (iPCR speed is device-determined).
        uint32_t stageAVal = current;
        if (setSpeed.has_value()) {
            stageAVal = PCRBits::SetSpeed(stageAVal, *setSpeed);
        }
        if (setChannel.has_value()) {
            stageAVal = PCRBits::SetChannel(stageAVal, *setChannel);
        }

        if (stageAVal == current) {
            // Nothing to program in stage A (no channel/speed change requested).
            // Skip straight to stage B.
            ASFW_LOG(CMP, "CMPClient: Connect PCR 0x%08X stage A skipped (no change), proceeding to stage B",
                     pcrAddress);
            PerformConnectStageB(pcrAddress, plugNum, callback);
            return;
        }

        const char* stageALabel = setSpeed.has_value() ? "program channel/speed"
                                                       : "program channel";
        ASFW_LOG(CMP, "CMPClient: Connect PCR 0x%08X stage A: %s (0x%08X → 0x%08X)",
                 pcrAddress, stageALabel, current, stageAVal);

        CompareSwapPCR(pcrAddress, current, stageAVal,
                       [this, pcrAddress, plugNum, callback](CMPStatus status) {
            if (status != CMPStatus::Success) {
                ASFW_LOG(CMP, "CMPClient: Connect PCR 0x%08X stage A CAS failed (%d)",
                         pcrAddress, static_cast<int>(status));
                callback(status);
                return;
            }
            // Stage B: bump p2p N→N+1
            PerformConnectStageB(pcrAddress, plugNum, callback);
        });
    });
}

void CMPClient::PerformConnectStageB(uint32_t pcrAddress, uint8_t plugNum,
                                      CMPCallback callback) {
    ReadPCRQuadlet(pcrAddress, [this, pcrAddress, plugNum, callback](bool success, uint32_t current) {
        if (!success) {
            ASFW_LOG(CMP, "CMPClient: Connect failed - cannot read PCR 0x%08X (stage B)", pcrAddress);
            callback(CMPStatus::Failed);
            return;
        }

        if (!PCRBits::IsOnline(current)) {
            ASFW_LOG(CMP, "CMPClient: Connect failed - plug %u offline between stages (PCR=0x%08X)",
                     plugNum, current);
            callback(CMPStatus::Failed);
            return;
        }

        uint8_t p2p = PCRBits::GetP2P(current);
        if (p2p >= 3) {
            ASFW_LOG(CMP, "CMPClient: Connect failed - p2p hit max between stages (%u)", p2p);
            callback(CMPStatus::NoResources);
            return;
        }

        uint32_t stageBVal = PCRBits::SetP2P(current, p2p + 1);

        ASFW_LOG(CMP, "CMPClient: Connect PCR 0x%08X stage B: p2p %u→%u (0x%08X → 0x%08X)",
                 pcrAddress, p2p, p2p + 1, current, stageBVal);

        CompareSwapPCR(pcrAddress, current, stageBVal, callback);
    });
}

void CMPClient::PerformDisconnect(uint32_t pcrAddress, uint8_t plugNum, CMPCallback callback) {
    // Step 1: Read current PCR value
    ReadPCRQuadlet(pcrAddress, [this, pcrAddress, plugNum, callback](bool success, uint32_t current) {
        if (!success) {
            ASFW_LOG(CMP, "CMPClient: Disconnect failed - cannot read PCR 0x%08X", pcrAddress);
            callback(CMPStatus::Failed);
            return;
        }

        // Step 2: Check p2p count
        uint8_t p2p = PCRBits::GetP2P(current);
        if (p2p == 0) {
            ASFW_LOG(CMP, "CMPClient: Disconnect - p2p already 0, nothing to do");
            callback(CMPStatus::Success);  // Already disconnected
            return;
        }

        // Step 3: Force p2p to 0 (reset all connections) so subsequent ConnectOPCR/IPCR
        // starts from a clean state. This handles stale connections from unclean shutdowns.
        uint32_t newVal = PCRBits::SetP2P(current, 0);

        ASFW_LOG(CMP, "CMPClient: Disconnect PCR 0x%08X: p2p %u→0 (0x%08X → 0x%08X) [full reset]",
                 pcrAddress, p2p, current, newVal);

        // Step 4: Lock-compare-swap
        CompareSwapPCR(pcrAddress, current, newVal, callback);
    });
}

} // namespace ASFW::CMP
