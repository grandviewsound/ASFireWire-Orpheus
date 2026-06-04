#pragma once

#include "../../../IRM/IRMTypes.hpp"
#include "../../../Async/Interfaces/IFireWireBusOps.hpp"
#include "LocalPcrRegisterFile.hpp"
#include <functional>
#include <cstdint>
#include <optional>

namespace ASFW::CMP {

// ============================================================================
// PCR Constants (IEC 61883-1)
// ============================================================================

/// PCR register addresses on device (CSR space)
namespace PCRRegisters {
    constexpr uint16_t kAddressHi = 0xFFFF;  ///< CSR address high word
    
    constexpr uint32_t kOMPR = 0xF0000900;   ///< Output Master Plug Register
    constexpr uint32_t kOPCRBase = 0xF0000904; ///< oPCR[0] base
    constexpr uint32_t kIMPR = 0xF0000980;   ///< Input Master Plug Register
    constexpr uint32_t kIPCRBase = 0xF0000984; ///< iPCR[0] base
    constexpr uint32_t kPCRStride = 4;        ///< 4 bytes per plug
    
    inline uint32_t GetOPCRAddress(uint8_t plug) { return kOPCRBase + plug * kPCRStride; }
    inline uint32_t GetIPCRAddress(uint8_t plug) { return kIPCRBase + plug * kPCRStride; }
}

// ============================================================================
// PCR Bit Fields (IEC 61883-1 §10.7)
// ============================================================================

/// PCR bit masks and shifts (IEC 61883-1:2003 §10.7)
///
/// PCR layout (oPCR and iPCR share upper fields):
///   [31]    online (read-only)
///   [30]    broadcast connection counter (1 bit)
///   [29:24] point-to-point connection counter (6 bits)
///   [23:22] extended speed (oPCR only; reserved on iPCR)
///   [21:16] channel number (6 bits)
///   [15:14] speed (2 bits)
///   [13:10] overhead_ID (oPCR) or reserved (iPCR)
///   [9:0]   payload in quadlets
///
/// Reference: Linux kernel sound/firewire/cmp.c (authoritative)
namespace PCRBits {
    // Common to oPCR and iPCR
    constexpr uint32_t kOnlineMask      = 0x80000000;  ///< Bit 31: online
    constexpr uint32_t kBcastMask       = 0x40000000;  ///< Bit 30: broadcast connection counter
    constexpr uint32_t kP2PMask         = 0x3F000000;  ///< Bits 29-24: p2p count (6 bits)
    constexpr uint8_t  kP2PShift        = 24;
    constexpr uint32_t kChannelMask     = 0x003F0000;  ///< Bits 21-16: channel
    constexpr uint8_t  kChannelShift    = 16;
    constexpr uint32_t kSpeedMask       = 0x0000C000;  ///< Bits 15-14: speed
    constexpr uint8_t  kSpeedShift      = 14;

    // Extract p2p count from PCR value
    inline uint8_t GetP2P(uint32_t pcr) { return (pcr & kP2PMask) >> kP2PShift; }

    // Set p2p count in PCR value
    inline uint32_t SetP2P(uint32_t pcr, uint8_t p2p) {
        return (pcr & ~kP2PMask) | ((static_cast<uint32_t>(p2p) & 0x3F) << kP2PShift);
    }

    // Extract channel from PCR value
    inline uint8_t GetChannel(uint32_t pcr) { return (pcr & kChannelMask) >> kChannelShift; }

    // Set channel in PCR value
    inline uint32_t SetChannel(uint32_t pcr, uint8_t ch) {
        return (pcr & ~kChannelMask) | ((static_cast<uint32_t>(ch) & 0x3F) << kChannelShift);
    }

    // Extract speed from PCR value (bits 15-14)
    inline uint8_t GetSpeed(uint32_t pcr) { return (pcr & kSpeedMask) >> kSpeedShift; }

    // Set speed in PCR value (bits 15-14: 0=S100, 1=S200, 2=S400, 3=S800)
    inline uint32_t SetSpeed(uint32_t pcr, uint8_t speed) {
        return (pcr & ~kSpeedMask) | ((static_cast<uint32_t>(speed) & 0x03) << kSpeedShift);
    }

    // Check if online
    inline bool IsOnline(uint32_t pcr) { return (pcr & kOnlineMask) != 0; }
}

// ============================================================================
// CMP Status Codes
// ============================================================================

/// CMP operation status (compatible with IRM::AllocationStatus)
using CMPStatus = IRM::AllocationStatus;

/// CMP operation callback
using CMPCallback = std::function<void(CMPStatus status)>;

/// PCR read callback
using PCRReadCallback = std::function<void(bool success, uint32_t value)>;

// ============================================================================
// Local Plug State (Apr 26 — replicates AppleFWAudio's UpdateLocalInputPlug /
// UpdateLocalOutputPlug; AM824AVC tracks the host's own plug state as a CMP
// listener/talker counterpart to the remote device's PCR.)
// ============================================================================

/// Local-side PCR state, mirroring what Apple maintains via
/// AM824AVC::UpdateLocalInputPlug / UpdateLocalOutputPlug.
///
/// Per IEC 61883 CMP, when the host establishes a P2P connection both
/// endpoints must update their respective plug registers:
///  - Host as TALKER (we transmit, device receives): host's LOCAL oPCR
///  - Host as LISTENER (device transmits, we receive): host's LOCAL iPCR
struct LocalPlugState {
    bool inUse{false};       ///< Has at least one active P2P connection
    uint8_t p2pCount{0};     ///< Active p2p connections on this plug (Apple retries up to 4-5x)
    uint8_t channel{0x3F};   ///< Channel selected (0x3F = none)
    uint8_t speed{2};        ///< Speed code (2 = S400)
    uint16_t payloadQuadlets{0}; ///< oPCR payload capacity when host is TALKER
};

// ============================================================================
// CMPClient - Connection Management Procedures Client
// ============================================================================

/**
 * CMPClient - Manages CMP connections to remote device's plugs.
 *
 * This is a CMP **client** that connects TO a device's PCR registers.
 * It performs:
 * - Read of oPCR/iPCR registers
 * - Lock-compare-swap to increment/decrement p2p connection count
 *
 * Per IEC 61883-1 §10.8:
 * - CMP ESTABLISH: Increment p2p count (create connection)
 * - CMP BREAK: Decrement p2p count (destroy connection)
 *
 * Usage:
 *   CMPClient cmpClient(busOps);
 *   cmpClient.SetDeviceNode(deviceNodeId, generation);
 *   cmpClient.ConnectOPCR(0, irChannel, 2, [](CMPStatus status) { ... }); // speed=2 (S400)
 *
 * Reference: Apple's LockRq to 0xF000.0904 in FireBug logs
 */
class CMPClient {
public:
    /**
     * Construct CMP client with bus operations interface.
     * @param busOps Canonical async bus operations (same as IRMClient)
     */
    explicit CMPClient(Async::IFireWireBusOps& busOps);
    ~CMPClient();
    
    // =========================================================================
    // Configuration
    // =========================================================================
    
    /**
     * Set target device node and generation.
     * Call after topology scan when device node ID is known.
     * @param nodeId Device node ID
     * @param generation Current bus generation
     */
    void SetDeviceNode(uint8_t nodeId, IRM::Generation generation);
    
    /**
     * Get current device node ID.
     * @return Device node ID (0xFF = not set)
     */
    [[nodiscard]] uint8_t GetDeviceNodeID() const { return deviceNodeId_; }
    
    /**
     * Get current generation.
     * @return Bus generation
     */
    [[nodiscard]] IRM::Generation GetGeneration() const { return generation_; }
    
    // =========================================================================
    // oPCR Operations (device→host stream, device transmits)
    // =========================================================================
    
    /**
     * Read oPCR[plugNum] from device.
     * @param plugNum Output plug number (0-30)
     * @param callback Completion with (success, rawValue)
     */
    void ReadOPCR(uint8_t plugNum, PCRReadCallback callback);
    
    /**
     * CMP ESTABLISH on oPCR - connect to device's output plug.
     * Increments p2p connection count and sets the channel via lock-compare-swap.
     * The device will transmit on the specified channel after connect.
     *
     * Per IEC 61883-1: host MUST set the channel in oPCR so the device
     * knows which isochronous channel to transmit on.
     *
     * @param plugNum Output plug number (usually 0)
     * @param channel Isochronous channel the IR context listens on
     * @param speed Speed code (0=S100, 1=S200, 2=S400, 3=S800)
     * @param callback Completion callback
     */
    void ConnectOPCR(uint8_t plugNum, uint8_t channel, uint8_t speed, CMPCallback callback);
    
    /**
     * CMP BREAK on oPCR - disconnect from device's output plug.
     * Decrements p2p connection count via lock-compare-swap.
     * 
     * @param plugNum Output plug number (usually 0)
     * @param callback Completion callback
     */
    void DisconnectOPCR(uint8_t plugNum, CMPCallback callback);
    
    // =========================================================================
    // iPCR Operations (host→device stream, device receives)
    // =========================================================================
    
    /**
     * Read iPCR[plugNum] from device.
     * @param plugNum Input plug number (0-30)
     * @param callback Completion with (success, rawValue)
     */
    void ReadIPCR(uint8_t plugNum, PCRReadCallback callback);
    
    /**
     * CMP ESTABLISH on iPCR - connect to device's input plug.
     * Increments p2p connection count via lock-compare-swap.
     * 
     * After success, device should accept isochronous data we send.
     * 
     * @param plugNum Input plug number (usually 0)
     * @param channel Channel number to set in iPCR
     * @param speed Speed code (0=S100, 1=S200, 2=S400, 3=S800)
     * @param callback Completion callback
     */
    void ConnectIPCR(uint8_t plugNum, uint8_t channel, uint8_t speed, CMPCallback callback);
    
    /**
     * CMP BREAK on iPCR - disconnect from device's input plug.
     * Decrements p2p connection count via lock-compare-swap.
     *
     * @param plugNum Input plug number (usually 0)
     * @param callback Completion callback
     */
    void DisconnectIPCR(uint8_t plugNum, CMPCallback callback);

    // =========================================================================
    // Local Plug State — replicates Apple's UpdateLocalInputPlug /
    // UpdateLocalOutputPlug software bookkeeping.
    // =========================================================================

    /// Host's local output plug — populated when we ESTABLISH on device's iPCR
    /// (host transmits, peer would query our oPCR to verify the talker exists).
    [[nodiscard]] LocalPlugState GetLocalOutputPlug(uint8_t plugNum) const;

    /// Payload advertised through the host's local oPCR for the active TX stream.
    void SetLocalOutputPayloadQuadlets(uint8_t plugNum, uint16_t payloadQuadlets) noexcept;

    /// Host's local input plug — populated when we ESTABLISH on device's oPCR
    /// (host receives, peer would query our iPCR to verify the listener exists).
    [[nodiscard]] LocalPlugState GetLocalInputPlug(uint8_t plugNum) const;

    // =========================================================================
    // Host-as-CMP-target: local PCR register file (gaps 3.3/3.4).
    //
    // The raw lockable backing store a PEER reads/locks when it connects TO this
    // node. Kept as the single source of truth for incoming PCR transactions:
    // our own connect bookkeeping (UpdateLocal*Plug) seeds it, the read responder
    // serves from it, and an incoming compare_swap mutates it. Mirrors Apple's
    // IOFireWirePCRSpace quadlet array. addressLo is the CSR offset (0xF00009xx).
    // =========================================================================

    /// Read a host PCR quadlet for an incoming peer read. nullopt if the address
    /// is outside our PCR range / not a valid plug.
    [[nodiscard]] std::optional<uint32_t> ReadLocalPcr(uint32_t addressLo) const noexcept;

    /// 1394 compare_swap from an incoming peer lock. Returns the PRIOR value
    /// (always, when the address maps); stores `desired` iff prior==`expected`.
    /// nullopt if the address is outside our PCR range / not a valid plug.
    [[nodiscard]] std::optional<uint32_t> CompareSwapLocalPcr(uint32_t addressLo,
                                                             uint32_t expected,
                                                             uint32_t desired,
                                                             bool* swapped = nullptr) noexcept;

    /// Gap 3.5 — local-PCR change notify (Apple
    /// `IOFireWireAVCTargetSpace::pcrModified(plugType, plugNum, newValue)` parity).
    /// Invoked when an inbound peer lock successfully writes one of our local PCRs
    /// (e.g. a device CMP-connecting to our plug bumps p2p / sets the channel).
    /// `plugType`/`plugNum`/`newValue` mirror Apple's relay arguments. The DriverKit
    /// equivalent of Apple's AVCProtocolUserClient relay (async delivery to a
    /// user-space CMP client) layers on top of this seam; not yet wired.
    using PcrChangeListener =
        std::function<void(LocalPcrRegisterFile::Reg plugType, uint8_t plugNum, uint32_t newValue)>;
    void SetPcrChangeListener(PcrChangeListener listener) noexcept {
        pcrChangeListener_ = std::move(listener);
    }

private:
    Async::IFireWireBusOps& busOps_;
    uint8_t deviceNodeId_{0xFF};
    IRM::Generation generation_{0};

    static constexpr uint8_t kMaxLocalPlugs = 31;  // PCR plug numbers 0..30
    LocalPlugState localOutputPlugs_[kMaxLocalPlugs];
    LocalPlugState localInputPlugs_[kMaxLocalPlugs];

    // Raw lockable backing store for incoming peer reads/locks (gaps 3.3/3.4),
    // seeded from LocalPlugState by UpdateLocal*Plug.
    LocalPcrRegisterFile localPcr_;

    // Gap 3.5 change-notify sink (optional; no-op until a listener is registered).
    PcrChangeListener pcrChangeListener_{};

    // Serialize a plug's structured state into its raw PCR quadlet (matches the
    // BuildLocal*PCRValue layout used by the read responder) and store it in the
    // register file so incoming reads/locks see a consistent value.
    void SyncLocalPcrRegister(bool outputPlug, uint8_t plugNum) noexcept;

    // Map a CSR addressLo (0xF00009xx) to the register file's (Reg, index).
    // Returns false if the address is outside our local PCR range.
    [[nodiscard]] static bool MapAddressToReg(uint32_t addressLo,
                                              LocalPcrRegisterFile::Reg& reg,
                                              uint8_t& index) noexcept;

    void UpdateLocalOutputPlug(uint8_t plugNum, uint8_t channel, uint8_t speed, bool establish);
    void UpdateLocalInputPlug(uint8_t plugNum, uint8_t channel, uint8_t speed, bool establish);
    
    // Internal helpers
    void ReadPCRQuadlet(uint32_t addressLo, PCRReadCallback callback);
    void CompareSwapPCR(uint32_t addressLo, uint32_t expected, uint32_t desired, 
                        CMPCallback callback);
    
    // Connect/disconnect implementation (shared logic).
    //
    // Fix 98: PerformConnect is a SINGLE combined compare-swap that matches
    // Apple's AM824AVC::cmpNewPointToPointConnection byte-for-byte: it
    // increments the p2p counter AND (only on the first connection, p2p 0→1)
    // writes the channel/speed, all in ONE swap. The previous two-stage form
    // (channel with p2p unchanged, then p2p bump) issued a standalone channel
    // write the Orpheus rejected with hardware_error(5) — Apple never writes a
    // channel without the simultaneous p2p edge.
    // (reports/apple_irm_channel_allocation_ida_pass_2026-05-19.md)
    //
    // The CAS is retried up to kConnectMaxAttempts with a 1 ms sleep, mirroring
    // AppleFWAudio's UpdateLocal*Plug 4-5× retry loop and handling two observed
    // Orpheus failure modes: (a) lockResponse never arrives; (b) lockResponse
    // is lost/malformed but the register already shows the target — early-accept
    // on re-read. `attempt` counts retries; `targetP2P` is the final p2p value
    // we want (-1 on first call = derive from the initial read).
    static constexpr uint8_t kConnectMaxAttempts = 5;
    static constexpr uint32_t kConnectRetrySleepMs = 1;
    void PerformConnect(uint32_t pcrAddress, uint8_t plugNum,
                        std::optional<uint8_t> setChannel,
                        std::optional<uint8_t> setSpeed,
                        CMPCallback callback,
                        uint8_t attempt = 0,
                        int targetP2P = -1);
    void PerformDisconnect(uint32_t pcrAddress, uint8_t plugNum, CMPCallback callback);
};

} // namespace ASFW::CMP
