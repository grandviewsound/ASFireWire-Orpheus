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
    // Seed the local PCR register file (host as CMP target, gaps 3.3/3.4) so an
    // incoming peer read/lock before any connection sees present-but-idle plugs.
    // Master plug registers: online + plug count (matches the prior responder).
    const uint32_t masterValue =
        PCRBits::kOnlineMask | (static_cast<uint32_t>(1) << PCRBits::kP2PShift);
    localPcr_.Set(LocalPcrRegisterFile::Reg::kOutputMaster, 0, masterValue);
    localPcr_.Set(LocalPcrRegisterFile::Reg::kInputMaster, 0, masterValue);
    for (uint8_t i = 0; i < kMaxLocalPlugs; ++i) {
        SyncLocalPcrRegister(/*outputPlug=*/true, i);
        SyncLocalPcrRegister(/*outputPlug=*/false, i);
    }
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
    // Host is acting as LISTENER for this connection (device transmits on its
    // oPCR, host receives), so the host's matching local-side register is the
    // iPCR. Update software state on success — mirrors Apple's
    // UpdateLocalInputPlug call after SetUpOutputConnection.
    PerformConnect(PCRRegisters::GetOPCRAddress(plugNum),
                   plugNum,
                   channel,
                   speed,
                   [this, plugNum, channel, speed, callback](CMPStatus status) {
                       if (status == CMPStatus::Success) {
                           UpdateLocalInputPlug(plugNum, channel, speed, /*establish=*/true);
                       }
                       callback(status);
                   });
}

void CMPClient::DisconnectOPCR(uint8_t plugNum, CMPCallback callback) {
    if (plugNum > 30) {
        ASFW_LOG(CMP, "CMPClient: Invalid oPCR plug number %u", plugNum);
        callback(CMPStatus::Failed);
        return;
    }

    ASFW_LOG(CMP, "CMPClient: Disconnecting oPCR[%u]", plugNum);
    PerformDisconnect(PCRRegisters::GetOPCRAddress(plugNum),
                      plugNum,
                      [this, plugNum, callback](CMPStatus status) {
                          if (status == CMPStatus::Success) {
                              UpdateLocalInputPlug(plugNum, 0x3F, 0, /*establish=*/false);
                          }
                          callback(status);
                      });
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
    // Host is acting as TALKER for this connection (host transmits on its
    // local oPCR, device receives on its own iPCR). Update software state on
    // success — mirrors Apple's UpdateLocalOutputPlug call after
    // SetUpInputConnection.
    PerformConnect(PCRRegisters::GetIPCRAddress(plugNum),
                   plugNum,
                   channel,
                   std::nullopt,
                   [this, plugNum, channel, speed, callback](CMPStatus status) {
                       if (status == CMPStatus::Success) {
                           UpdateLocalOutputPlug(plugNum, channel, speed, /*establish=*/true);
                       }
                       callback(status);
                   });
}

void CMPClient::DisconnectIPCR(uint8_t plugNum, CMPCallback callback) {
    if (plugNum > 30) {
        ASFW_LOG(CMP, "CMPClient: Invalid iPCR plug number %u", plugNum);
        callback(CMPStatus::Failed);
        return;
    }

    ASFW_LOG(CMP, "CMPClient: Disconnecting iPCR[%u]", plugNum);
    PerformDisconnect(PCRRegisters::GetIPCRAddress(plugNum),
                      plugNum,
                      [this, plugNum, callback](CMPStatus status) {
                          if (status == CMPStatus::Success) {
                              UpdateLocalOutputPlug(plugNum, 0x3F, 0, /*establish=*/false);
                          }
                          callback(status);
                      });
}

// ============================================================================
// Private Implementation
// ============================================================================

// Fix 98: SINGLE combined compare-swap per connect, byte-for-byte matching
// Apple's AM824AVC::cmpNewPointToPointConnection (IOFireWireAVC, analysis
// 2026-05-19). Apple computes one new value — p2p counter +1, plus speed and
// (ONLY on the first connection, i.e. p2p 0→1) the channel — and writes it in
// ONE CAS. The earlier two-stage form wrote the channel with p2p unchanged
// first; that standalone channel write (ch!=0, p2p=0) is an operation Apple
// never performs and the Orpheus rejected it with bus-level hardware_error(5).
// Reference: reports/apple_irm_channel_allocation_ida_pass_2026-05-19.md
//   newVal  = current + (1<<24)                 ; p2p++
//   newVal |= speed<<14                          ; if oPCR (setSpeed present)
//   if ((newVal p2p field) == 1): set channel    ; first connection only
void CMPClient::PerformConnect(uint32_t pcrAddress, uint8_t plugNum,
                                std::optional<uint8_t> setChannel,
                                std::optional<uint8_t> setSpeed,
                                CMPCallback callback,
                                uint8_t attempt,
                                int targetP2P) {
    ReadPCRQuadlet(pcrAddress,
                   [this, pcrAddress, plugNum, setChannel, setSpeed, callback, attempt, targetP2P]
                   (bool success, uint32_t current) {
        if (!success) {
            ASFW_LOG(CMP, "CMPClient: Connect failed - cannot read PCR 0x%08X (attempt %u)",
                     pcrAddress, attempt + 1);
            callback(CMPStatus::Failed);
            return;
        }

        // Gap 2.8 (reports/apple_irm_channel_allocation_ida_pass_2026-05-19.md):
        // a successful read of all-ones is a failed/uninitialized-plug read —
        // 0xFFFFFFFF can never be a valid PCR (it implies p2p=63 and every
        // reserved bit set). Apple resets it to 0 and PROCEEDS with the connect
        // rather than bailing, so the p2p 0→1 edge initializes the plug. Match
        // that: collapse to 0 here so the guards below see a clean zero (online=0,
        // p2p=0 → target 1) and the connection establishes instead of failing the
        // target>3 / not-online checks on garbage. (Genuine no-response is the
        // !success bail above; a real 0x00000000 still falls through to the
        // not-online bail — the analysis pass only flags the all-ones case.)
        const bool garbageRead = (current == 0xFFFFFFFFU);
        if (garbageRead) {
            ASFW_LOG(CMP,
                     "CMPClient: PCR 0x%08X read 0xFFFFFFFF (uninitialized/garbage) — "
                     "resetting to 0 and proceeding (Apple parity, gap 2.8)",
                     pcrAddress);
            current = 0;
        }

        if (!garbageRead && !PCRBits::IsOnline(current)) {
            ASFW_LOG(CMP, "CMPClient: Connect failed - plug %u not online (PCR=0x%08X)",
                     plugNum, current);
            callback(CMPStatus::Failed);
            return;
        }

        const uint8_t p2p = PCRBits::GetP2P(current);

        // First attempt latches the desired final p2p (= current + 1). On retry
        // we keep the same target so a quiet prior-attempt landing reads as
        // success rather than re-incrementing.
        const int target = (targetP2P < 0) ? static_cast<int>(p2p) + 1 : targetP2P;
        if (target > 3) {
            ASFW_LOG(CMP, "CMPClient: Connect failed - p2p target %d exceeds 3 (PCR=0x%08X)",
                     target, current);
            callback(CMPStatus::NoResources);
            return;
        }

        // Early-accept: a prior attempt's CAS may have landed even if its
        // lockResponse was lost/malformed. If the register already shows our
        // target p2p, the connection is established.
        if (static_cast<int>(p2p) >= target) {
            ASFW_LOG(CMP,
                     "CMPClient: Connect PCR 0x%08X p2p=%u already at target %d on attempt %u — accepting",
                     pcrAddress, p2p, target, attempt + 1);
            callback(CMPStatus::Success);
            return;
        }

        // Apple's single combined value: bump p2p, set speed (oPCR only), and
        // set the channel ONLY when establishing the first connection (the
        // p2p 0→1 edge) AND no broadcast connection owns the channel (bit 30
        // clear). On a refresh (p2p already >0) the channel is left exactly as
        // the device advertises it. Speed/overhead: Apple applies them only to
        // oPCR-region plugs (offset ≤0x7f); for iPCR-region plugs neither is
        // written. Callers reflect this — ConnectOPCR passes a speed, ConnectIPCR
        // passes none. Overhead is 0 in both of Apple's SetUp*Connection paths
        // (byte-confirmed: xorl %r9d,%r9d), so it is intentionally not written.
        uint32_t desired = PCRBits::SetP2P(current, p2p + 1);
        if (setSpeed.has_value()) {
            desired = PCRBits::SetSpeed(desired, *setSpeed);
        }
        const bool firstConnection = (p2p == 0);
        const bool broadcastActive = (current & PCRBits::kBcastMask) != 0;
        const bool writeChannel = firstConnection && !broadcastActive && setChannel.has_value();
        if (writeChannel) {
            desired = PCRBits::SetChannel(desired, *setChannel);
        }

        ASFW_LOG(CMP,
                 "CMPClient: Connect PCR 0x%08X [attempt %u/%u]: p2p %u→%u%s (0x%08X → 0x%08X)",
                 pcrAddress, attempt + 1, kConnectMaxAttempts, p2p, p2p + 1,
                 writeChannel ? " +channel" : "",
                 current, desired);

        CompareSwapPCR(pcrAddress, current, desired,
                       [this, pcrAddress, plugNum, setChannel, setSpeed, callback, attempt, target]
                       (CMPStatus status) {
            if (status == CMPStatus::Success) {
                callback(CMPStatus::Success);
                return;
            }
            if (attempt + 1 >= kConnectMaxAttempts) {
                ASFW_LOG(CMP,
                         "CMPClient: Connect PCR 0x%08X exhausted %u attempts — giving up (last status=%d)",
                         pcrAddress, kConnectMaxAttempts, static_cast<int>(status));
                callback(status);
                return;
            }
            ASFW_LOG(CMP,
                     "CMPClient: Connect PCR 0x%08X CAS failed (%d) — sleeping %u ms then retry %u/%u",
                     pcrAddress, static_cast<int>(status), kConnectRetrySleepMs,
                     attempt + 2, kConnectMaxAttempts);
            IOSleep(kConnectRetrySleepMs);
            // Re-read inside the retry recomputes the value against the current
            // register, and the target p2p is held steady so an already-landed
            // increment is accepted on the next pass.
            PerformConnect(pcrAddress, plugNum, setChannel, setSpeed, callback,
                           attempt + 1, target);
        });
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

// ============================================================================
// Local Plug State (Apr 26 gap 1.1; now paired with the CSR read responder)
//
// Mirrors AppleFWAudio's AM824AVC::UpdateLocalInputPlug /
// UpdateLocalOutputPlug bookkeeping so the host has matching local-side state
// for peer reads of iMPR/oMPR/iPCR/oPCR.
// ============================================================================

void CMPClient::UpdateLocalInputPlug(uint8_t plugNum, uint8_t channel,
                                     uint8_t speed, bool establish) {
    if (plugNum >= kMaxLocalPlugs) {
        return;
    }
    LocalPlugState& state = localInputPlugs_[plugNum];
    if (establish) {
        if (state.p2pCount < 0x3F) {
            state.p2pCount++;
        }
        state.inUse = true;
        state.channel = channel;
        state.speed = speed;
    } else {
        if (state.p2pCount > 0) {
            state.p2pCount--;
        }
        if (state.p2pCount == 0) {
            state.inUse = false;
            state.channel = 0x3F;
        }
    }
    ASFW_LOG(CMP,
             "CMPClient: local iPCR[%u] %{public}s p2p=%u ch=%u speed=%u",
             plugNum,
             establish ? "ESTABLISH" : "BREAK",
             state.p2pCount,
             state.channel,
             state.speed);
    SyncLocalPcrRegister(/*outputPlug=*/false, plugNum);
}

void CMPClient::UpdateLocalOutputPlug(uint8_t plugNum, uint8_t channel,
                                      uint8_t speed, bool establish) {
    if (plugNum >= kMaxLocalPlugs) {
        return;
    }
    LocalPlugState& state = localOutputPlugs_[plugNum];
    if (establish) {
        if (state.p2pCount < 0x3F) {
            state.p2pCount++;
        }
        state.inUse = true;
        state.channel = channel;
        state.speed = speed;
    } else {
        if (state.p2pCount > 0) {
            state.p2pCount--;
        }
        if (state.p2pCount == 0) {
            state.inUse = false;
            state.channel = 0x3F;
            state.payloadQuadlets = 0;
        }
    }
    ASFW_LOG(CMP,
             "CMPClient: local oPCR[%u] %{public}s p2p=%u ch=%u speed=%u payloadQ=%u",
             plugNum,
             establish ? "ESTABLISH" : "BREAK",
             state.p2pCount,
             state.channel,
             state.speed,
             state.payloadQuadlets);
    SyncLocalPcrRegister(/*outputPlug=*/true, plugNum);
}

// --- Host-as-CMP-target local PCR register file (gaps 3.3/3.4) --------------

void CMPClient::SyncLocalPcrRegister(bool outputPlug, uint8_t plugNum) noexcept {
    if (plugNum >= kMaxLocalPlugs) {
        return;
    }
    // Build the raw quadlet exactly as the read responder did (BuildLocal*PCRValue):
    // both plug kinds advertise online; oPCR (host talker) also carries speed +
    // payload; channel/p2p only appear once a connection is established.
    uint32_t value = PCRBits::kOnlineMask;
    if (outputPlug) {
        const LocalPlugState& s = localOutputPlugs_[plugNum];
        value = PCRBits::SetSpeed(value, s.speed);
        value |= static_cast<uint32_t>(s.payloadQuadlets & 0x03FFU);
        if (s.p2pCount != 0) {
            value = PCRBits::SetP2P(value, s.p2pCount);
            value = PCRBits::SetChannel(value, s.channel);
        }
        localPcr_.Set(LocalPcrRegisterFile::Reg::kOutputPlug, plugNum, value);
    } else {
        const LocalPlugState& s = localInputPlugs_[plugNum];
        if (s.p2pCount != 0) {
            value = PCRBits::SetP2P(value, s.p2pCount);
            value = PCRBits::SetChannel(value, s.channel);
        }
        localPcr_.Set(LocalPcrRegisterFile::Reg::kInputPlug, plugNum, value);
    }
}

bool CMPClient::MapAddressToReg(uint32_t addressLo,
                                LocalPcrRegisterFile::Reg& reg,
                                uint8_t& index) noexcept {
    using Reg = LocalPcrRegisterFile::Reg;
    if (addressLo == PCRRegisters::kOMPR) { reg = Reg::kOutputMaster; index = 0; return true; }
    if (addressLo == PCRRegisters::kIMPR) { reg = Reg::kInputMaster;  index = 0; return true; }

    constexpr uint32_t kStride = PCRRegisters::kPCRStride;
    const uint32_t oEnd = PCRRegisters::kOPCRBase + (kMaxLocalPlugs * kStride);
    if (addressLo >= PCRRegisters::kOPCRBase && addressLo < oEnd &&
        ((addressLo - PCRRegisters::kOPCRBase) % kStride) == 0) {
        reg = Reg::kOutputPlug;
        index = static_cast<uint8_t>((addressLo - PCRRegisters::kOPCRBase) / kStride);
        return true;
    }
    const uint32_t iEnd = PCRRegisters::kIPCRBase + (kMaxLocalPlugs * kStride);
    if (addressLo >= PCRRegisters::kIPCRBase && addressLo < iEnd &&
        ((addressLo - PCRRegisters::kIPCRBase) % kStride) == 0) {
        reg = Reg::kInputPlug;
        index = static_cast<uint8_t>((addressLo - PCRRegisters::kIPCRBase) / kStride);
        return true;
    }
    return false;
}

std::optional<uint32_t> CMPClient::ReadLocalPcr(uint32_t addressLo) const noexcept {
    LocalPcrRegisterFile::Reg reg{};
    uint8_t index = 0;
    if (!MapAddressToReg(addressLo, reg, index)) {
        return std::nullopt;
    }
    return localPcr_.Read(reg, index);
}

std::optional<uint32_t> CMPClient::CompareSwapLocalPcr(uint32_t addressLo,
                                                       uint32_t expected,
                                                       uint32_t desired,
                                                       bool* swapped) noexcept {
    LocalPcrRegisterFile::Reg reg{};
    uint8_t index = 0;
    if (!MapAddressToReg(addressLo, reg, index)) {
        if (swapped != nullptr) {
            *swapped = false;
        }
        return std::nullopt;
    }
    return localPcr_.CompareSwap(reg, index, expected, desired, swapped);
}

LocalPlugState CMPClient::GetLocalOutputPlug(uint8_t plugNum) const {
    if (plugNum >= kMaxLocalPlugs) {
        return LocalPlugState{};
    }
    return localOutputPlugs_[plugNum];
}

LocalPlugState CMPClient::GetLocalInputPlug(uint8_t plugNum) const {
    if (plugNum >= kMaxLocalPlugs) {
        return LocalPlugState{};
    }
    return localInputPlugs_[plugNum];
}

void CMPClient::SetLocalOutputPayloadQuadlets(uint8_t plugNum,
                                              uint16_t payloadQuadlets) noexcept {
    if (plugNum >= kMaxLocalPlugs) {
        return;
    }
    auto& state = localOutputPlugs_[plugNum];
    state.payloadQuadlets = static_cast<uint16_t>(payloadQuadlets & 0x03FFu);
    ASFW_LOG(CMP,
             "CMPClient: local oPCR[%u] payload hint payloadQ=%u",
             plugNum,
             state.payloadQuadlets);
}

} // namespace ASFW::CMP
