#include "IsochService.hpp"

#include <DriverKit/IOLib.h>
#include <atomic>

#include "IsochReceiveContext.hpp"
#include "Transmit/IsochTransmitContext.hpp"
#include "Memory/IsochDMAMemoryManager.hpp"
#include "Config/TxBufferProfiles.hpp"
#include "Encoding/TimingUtils.hpp"
#include "../Discovery/DeviceManager.hpp"
#include "../Discovery/FWDevice.hpp"
#include "../Hardware/HardwareInterface.hpp"
#include "../Protocols/AVC/AVCDiscovery.hpp"
#include "../Protocols/AVC/CMP/CMPClient.hpp"
#include "../Logging/Logging.hpp"
#include <com.kevinpeters.ASFW.ASFWDriver/ASFWAudioNub.h>

namespace ASFW::Driver {

kern_return_t IsochService::StartReceive(uint8_t channel,
                                         HardwareInterface& hardware,
                                         ASFW::Discovery::DeviceManager* deviceManager,
                                         ASFW::CMP::CMPClient* cmpClient,
                                         ASFW::Protocols::AVC::AVCDiscovery* avcDiscovery) {
    // Idempotent: if IR is already running, do not re-configure (would overflow the DMA slab).
    if (isochReceiveContext_ &&
        isochReceiveContext_->GetState() == ASFW::Isoch::IRPolicy::State::Running) {
        ASFW_LOG(Controller, "[Isoch] IR already running; StartReceive is idempotent");
        return kIOReturnSuccess;
    }

    if (!isochReceiveContext_) {
        ASFW::Isoch::Memory::IsochMemoryConfig config;
        config.numDescriptors = ASFW::Isoch::IsochReceiveContext::kNumDescriptors;
        config.packetSizeBytes = ASFW::Isoch::IsochReceiveContext::kMaxPacketSize;
        config.descriptorAlignment = 16;
        config.payloadPageAlignment = 16384;

        auto isochMem = ASFW::Isoch::Memory::IsochDMAMemoryManager::Create(config);
        if (!isochMem) {
            ASFW_LOG(Controller, "[Isoch] ❌ StartIsochReceive: Failed to create Memory Manager");
            return kIOReturnNoMemory;
        }

        if (!isochMem->Initialize(hardware)) {
            ASFW_LOG(Controller, "[Isoch] ❌ StartIsochReceive: Failed to initialize DMA slabs");
            return kIOReturnNoMemory;
        }

        isochReceiveContext_ = ASFW::Isoch::IsochReceiveContext::Create(&hardware, isochMem);

        if (!isochReceiveContext_) {
            ASFW_LOG(Controller, "[Isoch] ❌ StartIsochReceive: Context creation failed");
            return kIOReturnNoMemory;
        }
        ASFW_LOG(Controller, "[Isoch] ✅ provisioned Isoch Context with Dedicated Memory");
    }

    isochReceiveContext_->SetExternalSyncBridge(&externalSyncBridge_);

    auto result = isochReceiveContext_->Configure(channel, 0);
    if (result != kIOReturnSuccess) {
        ASFW_LOG(Controller, "[Isoch] ❌ Failed to Configure IR Context: 0x%x", result);
        return result;
    }

    // Wire shared RX queue to IR context (mirrors TX queue wiring in StartTransmit)
    if (avcDiscovery) {
        if (auto* audioNub = avcDiscovery->GetFirstAudioNub()) {
            audioNub->EnsureRxQueueCreated();
            void* rxBase = audioNub->GetRxQueueLocalMapping();
            uint64_t rxBytes = audioNub->GetRxQueueBytes();
            if (rxBase && rxBytes > 0) {
                isochReceiveContext_->SetSharedRxQueue(rxBase, rxBytes);
            }
        }
    }

    result = isochReceiveContext_->Start();
    if (result != kIOReturnSuccess) {
        ASFW_LOG(Controller, "[Isoch] ❌ Failed to Start IR Context: 0x%x", result);
        return result;
    }

    ASFW_LOG(Controller, "[Isoch] ✅ Started IR Context 0 for Channel %u!", channel);

    irChannel_ = channel;

    if (deviceManager && cmpClient) {
        auto devices = deviceManager->GetReadyDevices();
        if (!devices.empty()) {
            auto target = devices.front();
            uint16_t nodeId = target->GetNodeID();
            ASFW::IRM::Generation gen = target->GetGeneration();

            cmpClient->SetDeviceNode(static_cast<uint8_t>(nodeId & 0x3F), gen);

            // Disconnect first to clear any stale p2p connection from a previous
            // driver load that didn't cleanly disconnect. Then connect on our IR channel.
            const uint8_t irChannel = channel;
            cmpClient->DisconnectOPCR(0, [cmpClient, irChannel](ASFW::CMP::CMPStatus) {
                // Ignore disconnect result — p2p may already be 0, that's fine.
                cmpClient->ConnectOPCR(0, irChannel, [](ASFW::CMP::CMPStatus status) {
                    if (status == ASFW::CMP::CMPStatus::Success) {
                        ASFW_LOG(Controller, "[Isoch] ✅ CMP ConnectOPCR Success!");
                    } else {
                        ASFW_LOG(Controller, "[Isoch] ❌ CMP ConnectOPCR Failed: %d", static_cast<int>(status));
                    }
                });
            });
        } else {
            ASFW_LOG(Controller, "[Isoch] ⚠️ No devices ready for CMP oPCR connection");
        }
    }

    return kIOReturnSuccess;
}

kern_return_t IsochService::StopReceive(ASFW::CMP::CMPClient* cmpClient) {
    if (!isochReceiveContext_) {
        return kIOReturnNotReady;
    }

    isochReceiveContext_->Stop();
    // Release the context so the next StartReceive() allocates a fresh DMA slab.
    // The bump allocator inside IsochDMAMemoryManager cannot be reset in-place;
    // releasing here is the simplest way to avoid slab exhaustion on restart.
    isochReceiveContext_.reset();
    externalSyncBridge_.Reset();
    ASFW_LOG(Controller, "[Isoch] Stopped IR Context 0");

    if (cmpClient) {
        cmpClient->DisconnectOPCR(0, [](ASFW::CMP::CMPStatus status) {
            if (status == ASFW::CMP::CMPStatus::Success) {
                ASFW_LOG(Controller, "[Isoch] ✅ CMP DisconnectOPCR Success!");
            } else {
                ASFW_LOG(Controller, "[Isoch] ❌ CMP DisconnectOPCR Failed: %d", static_cast<int>(status));
            }
        });
    }

    return kIOReturnSuccess;
}

kern_return_t IsochService::StartTransmit(uint8_t channel,
                                          HardwareInterface& hardware,
                                          ASFW::Discovery::DeviceManager* deviceManager,
                                          ASFW::CMP::CMPClient* cmpClient,
                                          ASFW::Protocols::AVC::AVCDiscovery* avcDiscovery) {
    constexpr bool kEnableIsochZeroCopyPath = false;  // temporary A/B gate

    if (isochTransmitContext_ &&
        isochTransmitContext_->GetState() == ASFW::Isoch::ITState::Running) {
        ASFW_LOG(Controller, "[Isoch] IT already running; StartTransmit is idempotent");
        return kIOReturnSuccess;
    }

    if (!isochTransmitContext_) {
        ASFW::Isoch::Memory::IsochMemoryConfig config;
        config.numDescriptors = ASFW::Isoch::IsochTransmitContext::kRingBlocks;
        config.packetSizeBytes = ASFW::Isoch::IsochTransmitContext::kMaxPacketSize;
        config.descriptorAlignment = ASFW::Isoch::IsochTransmitContext::kOHCIPageSize;
        config.payloadPageAlignment = 16384;

        auto isochMem = ASFW::Isoch::Memory::IsochDMAMemoryManager::Create(config);
        if (!isochMem) {
            ASFW_LOG(Controller, "[Isoch] ❌ StartIsochTransmit: Failed to create Memory Manager");
            return kIOReturnNoMemory;
        }

        if (!isochMem->Initialize(hardware)) {
            ASFW_LOG(Controller, "[Isoch] ❌ StartIsochTransmit: Failed to initialize DMA slabs");
            return kIOReturnNoMemory;
        }

        isochTransmitContext_ = ASFW::Isoch::IsochTransmitContext::Create(&hardware, isochMem);

        if (!isochTransmitContext_) {
            ASFW_LOG(Controller, "[Isoch] ❌ StartIsochTransmit: Context creation failed");
            return kIOReturnNoMemory;
        }
        ASFW_LOG(Controller, "[Isoch] ✅ provisioned IT Context with Dedicated Memory");
    }

    uint32_t startTargetFill = ASFW::Isoch::Config::kTxBufferProfile.startWaitTargetFrames;
    uint32_t requestedStreamModeRaw = 0;
    uint32_t requestedChannels = 0;
    uint8_t localNodeId = 0;

    if (avcDiscovery) {
        if (auto* audioNub = avcDiscovery->GetFirstAudioNub()) {
            requestedStreamModeRaw = audioNub->GetStreamMode();
            requestedChannels = audioNub->GetChannelCount();
            ASFW_LOG(Controller, "[Isoch] Requested stream mode from nub: %{public}s",
                     requestedStreamModeRaw == 1u ? "blocking" : "non-blocking");
            ASFW_LOG(Controller, "[Isoch] Requested channels from nub: %u", requestedChannels);

            void* txQueueBase = audioNub->GetTxQueueLocalMapping();
            uint64_t txQueueBytes = audioNub->GetTxQueueBytes();

            if (txQueueBase && txQueueBytes > 0) {
                isochTransmitContext_->SetSharedTxQueue(txQueueBase, txQueueBytes);
                ASFW_LOG(Controller, "[Isoch] Wired shared TX queue to IT context (bytes=%llu)",
                         txQueueBytes);
            }

            isochTransmitContext_->SetZeroCopyOutputBuffer(nullptr, 0, 0);

            if (kEnableIsochZeroCopyPath) {
                void* zeroCopyBase = audioNub->GetOutputAudioLocalMapping();
                uint64_t zeroCopyBytes = audioNub->GetOutputAudioBytes();
                uint32_t zeroCopyFrames = audioNub->GetOutputAudioFrameCapacity();

                if (zeroCopyBase && zeroCopyBytes > 0 && zeroCopyFrames > 0) {
                    isochTransmitContext_->SetZeroCopyOutputBuffer(zeroCopyBase, zeroCopyBytes, zeroCopyFrames);
                    uint32_t target = (zeroCopyFrames * 5) / 8;
                    if (target < 8) target = 8;
                    startTargetFill = target;
                    ASFW_LOG(Controller, "[Isoch] ✅ ZERO-COPY wired! AudioBuffer base=%p bytes=%llu frames=%u",
                             zeroCopyBase, zeroCopyBytes, zeroCopyFrames);
                } else {
                    ASFW_LOG(Controller, "[Isoch] ZERO-COPY not available, using SharedTxQueue fallback");
                }
            } else {
                ASFW_LOG(Controller, "[Isoch] ZERO-COPY disabled by build flag; using SharedTxQueue");
            }
        }
    }

    if (isochTransmitContext_->SharedTxCapacityFrames() == 0) {
        ASFW_LOG(Controller, "[Isoch] ❌ StartTransmit blocked: shared TX queue metadata missing");
        return kIOReturnNotReady;
    }

    if (!isochReceiveContext_ ||
        isochReceiveContext_->GetState() != ASFW::Isoch::IRPolicy::State::Running) {
        ASFW_LOG(Controller, "[Isoch] ❌ StartTransmit blocked: IR context is not running");
        return kIOReturnNotReady;
    }

    // SYT gate: IT pipeline handles "no SYT" gracefully by sending silence until
    // IR SYT clock is established (ExternalSyncBridge discipline activates automatically).
    // We start IT immediately rather than blocking — this avoids timing out during the
    // BeBoB bus reset window (which delays SYT establishment by 2-8 seconds).
    if (externalSyncBridge_.clockEstablished.load(std::memory_order_acquire)) {
        ASFW_LOG(Controller, "[Isoch] IT starting with IR SYT already established");
    } else {
        ASFW_LOG(Controller, "[Isoch] IT starting before IR SYT established — will sync once IR clock ready");
    }

    isochTransmitContext_->SetExternalSyncBridge(&externalSyncBridge_);

    auto result = isochTransmitContext_->Configure(channel,
                                                   localNodeId,
                                                   requestedStreamModeRaw,
                                                   requestedChannels);
    if (result != kIOReturnSuccess) {
        ASFW_LOG(Controller, "[Isoch] ❌ Failed to Configure IT Context: 0x%x", result);
        return result;
    }

    const auto& txProfile = ASFW::Isoch::Config::kTxBufferProfile;
    ASFW_LOG(Controller,
             "[Isoch] IT TX profile=%{public}s startWait=%u startupPrimeLimit=%u legacy(target=%u max=%u chunks=%u)",
             txProfile.name,
             txProfile.startWaitTargetFrames,
             txProfile.startupPrimeLimitFrames,
             txProfile.legacyRbTargetFrames,
             txProfile.legacyRbMaxFrames,
             txProfile.legacyMaxChunksPerRefill);

    uint32_t fillLevel = 0;
    uint32_t targetFill = startTargetFill;
    const uint32_t queueCapacity = isochTransmitContext_->SharedTxCapacityFrames();
    if (queueCapacity > 0 && targetFill > queueCapacity) {
        ASFW_LOG(Controller, "[Isoch] IT start wait target clamped %u -> %u (queueCapacity)",
                 targetFill, queueCapacity);
        targetFill = queueCapacity;
    }
    const int maxWaitMs = 100;

    ASFW_LOG(Controller, "[Isoch] IT start wait targetFill=%u (zeroCopy=%{public}s)",
             targetFill, isochTransmitContext_->IsZeroCopyEnabled() ? "YES" : "NO");

    for (int waitMs = 0; waitMs < maxWaitMs; waitMs += 5) {
        fillLevel = isochTransmitContext_->SharedTxFillLevelFrames();
        if (fillLevel >= targetFill) {
            break;
        }
        IOSleep(5);
    }

    result = isochTransmitContext_->Start();
    if (result != kIOReturnSuccess) {
        ASFW_LOG(Controller, "[Isoch] Failed to Start IT Context: 0x%x", result);
        return result;
    }

    ASFW_LOG(Controller, "[Isoch] ✅ Started IT Context for Channel %u!", channel);

    itChannel_ = channel;

    if (deviceManager && cmpClient) {
        auto devices = deviceManager->GetReadyDevices();
        if (!devices.empty()) {
            auto target = devices.front();
            uint16_t nodeId = target->GetNodeID();
            ASFW::IRM::Generation gen = target->GetGeneration();

            cmpClient->SetDeviceNode(static_cast<uint8_t>(nodeId & 0x3F), gen);

            // Disconnect first to clear any stale p2p connection, then connect on IT channel.
            const uint8_t itChannel = channel;
            cmpClient->DisconnectIPCR(0, [cmpClient, itChannel](ASFW::CMP::CMPStatus) {
                // Ignore disconnect result — p2p may already be 0, that's fine.
                cmpClient->ConnectIPCR(0, itChannel, [](ASFW::CMP::CMPStatus status) {
                    if (status == ASFW::CMP::CMPStatus::Success) {
                        ASFW_LOG(Controller, "[Isoch] ✅ CMP ConnectIPCR Success!");
                    } else {
                        ASFW_LOG(Controller, "[Isoch] ❌ CMP ConnectIPCR Failed: %d", static_cast<int>(status));
                    }
                });
            });
        }
    }

    return kIOReturnSuccess;
}

kern_return_t IsochService::StopTransmit(ASFW::CMP::CMPClient* cmpClient) {
    if (!isochTransmitContext_) {
        return kIOReturnNotReady;
    }

    isochTransmitContext_->Stop();
    ASFW_LOG(Controller, "[Isoch] Stopped IT Context");

    if (cmpClient) {
        cmpClient->DisconnectIPCR(0, [](ASFW::CMP::CMPStatus status){
            if (status == ASFW::CMP::CMPStatus::Success) {
                ASFW_LOG(Controller, "[Isoch] ✅ CMP DisconnectIPCR Success!");
            } else {
                ASFW_LOG(Controller, "[Isoch] ❌ CMP DisconnectIPCR Failed: %d", static_cast<int>(status));
            }
        });
    }

    return kIOReturnSuccess;
}

void IsochService::ReconnectOPCR(ASFW::CMP::CMPClient* cmpClient) {
    if (!cmpClient || irChannel_ == 0xFF || !isochReceiveContext_) {
        return;
    }
    const uint8_t ch = irChannel_;
    ASFW_LOG(Controller, "[Isoch] Reconnecting CMP oPCR on channel %u after device resume", ch);
    cmpClient->DisconnectOPCR(0, [cmpClient, ch](ASFW::CMP::CMPStatus) {
        cmpClient->ConnectOPCR(0, ch, [](ASFW::CMP::CMPStatus status) {
            if (status == ASFW::CMP::CMPStatus::Success) {
                ASFW_LOG(Controller, "[Isoch] ✅ CMP oPCR reconnected after device resume");
            } else {
                ASFW_LOG(Controller, "[Isoch] ❌ CMP oPCR reconnect failed: %d",
                         static_cast<int>(status));
            }
        });
    });
}

void IsochService::ReconnectIPCR(ASFW::CMP::CMPClient* cmpClient) {
    if (!cmpClient || itChannel_ == 0xFF || !isochTransmitContext_) {
        return;
    }
    const uint8_t ch = itChannel_;
    ASFW_LOG(Controller, "[Isoch] Reconnecting CMP iPCR on channel %u after device resume", ch);
    cmpClient->DisconnectIPCR(0, [cmpClient, ch](ASFW::CMP::CMPStatus) {
        cmpClient->ConnectIPCR(0, ch, [](ASFW::CMP::CMPStatus status) {
            if (status == ASFW::CMP::CMPStatus::Success) {
                ASFW_LOG(Controller, "[Isoch] ✅ CMP iPCR reconnected after device resume");
            } else {
                ASFW_LOG(Controller, "[Isoch] ❌ CMP iPCR reconnect failed: %d",
                         static_cast<int>(status));
            }
        });
    });
}

void IsochService::StopAll() {
    if (isochReceiveContext_) {
        isochReceiveContext_->Stop();
        isochReceiveContext_.reset();
    }
    if (isochTransmitContext_) {
        isochTransmitContext_->Stop();
        isochTransmitContext_.reset();
    }
    externalSyncBridge_.Reset();
}

} // namespace ASFW::Driver
