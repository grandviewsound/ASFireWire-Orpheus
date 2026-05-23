// SPDX-License-Identifier: LGPL-3.0-or-later
// Copyright (c) 2026 ASFireWire Project
//
// MidiNubPublisher.hpp
// Centralized creation/lookup/termination of ASFWMIDINub instances (per GUID).

#pragma once

#include "../Audio/Model/ASFWAudioDevice.hpp"

#include <DriverKit/IOLib.h>
#include <cstdint>
#include <unordered_map>

class IOService;
class IOLock;
class ASFWMIDINub;

namespace ASFW::MIDI {

void RegisterIsochServiceForGuid(uint64_t guid, void* isochService) noexcept;
void UnregisterIsochServiceForGuid(uint64_t guid) noexcept;
[[nodiscard]] void* ResolveIsochServiceForGuid(uint64_t guid) noexcept;

class MidiNubPublisher {
public:
    MidiNubPublisher(IOService* driver, void* isochService) noexcept;
    ~MidiNubPublisher() noexcept;

    MidiNubPublisher(const MidiNubPublisher&) = delete;
    MidiNubPublisher& operator=(const MidiNubPublisher&) = delete;

    [[nodiscard]] bool EnsureNub(uint64_t guid,
                                 const ASFW::Audio::Model::ASFWAudioDevice& config,
                                 const char* sourceTag) noexcept;

    [[nodiscard]] ASFWMIDINub* GetNub(uint64_t guid) const noexcept;

    void TerminateNub(uint64_t guid, const char* reasonTag) noexcept;

private:
    [[nodiscard]] bool ReserveGuidLocked(uint64_t guid) noexcept;

    IOService* driver_{nullptr};
    void* isochService_{nullptr};
    IOLock* lock_{nullptr};
    std::unordered_map<uint64_t, ASFWMIDINub*> nubsByGuid_{};
};

} // namespace ASFW::MIDI
