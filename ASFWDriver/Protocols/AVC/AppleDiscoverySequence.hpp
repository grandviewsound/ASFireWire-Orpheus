// SPDX-License-Identifier: LGPL-3.0-or-later
// Copyright (c) 2026 ASFireWire Project
//
// AppleDiscoverySequence — replays Apple's ~174-command AVC discovery in exact
// order captured by dtrace on macOS 11 during Orpheus cold-attach (Apr 12 2026).
//
// The phase count and per-plug iteration adapt to whatever the device reports
// in Phase 2 (GetSubUnitInfo, GetPlugInfo), so this is general-purpose for any
// AV/C device, not Orpheus-specific.
//
// Phases 1-11 are STATUS / SPECIFIC INQUIRY commands (read-only discovery).
// Phase 12 sends mixer-state reads (GetChannelVolumeInfo / GetChannelMute).
// CMD A/B/C (ExtStreamFormat CONTROL) are NOT included here — they belong in
// the streaming bring-up pipeline after audio-config publication.

#pragma once

#include "FCPTransport.hpp"
#include <vector>
#include <cstdint>

namespace ASFW::Protocols::AVC {

class AppleDiscoverySequence {
public:
    // ── Result types ─────────────────────────────────────────────────────

    struct SubunitEntry {
        uint8_t type;    // raw subunit type byte (1=Audio, 12=Music, …)
        uint8_t maxId;
    };

    struct PlugFormatResult {
        bool valid = false;
        uint8_t direction = 0;   // 0=input(dest), 1=output(source)
        uint8_t plugType  = 0;   // 0=isoch, 1=subunit
        uint8_t plugNum   = 0;
        std::vector<uint8_t> rawResponse;
    };

    struct Result {
        bool success = false;

        // Phase 1
        bool supportsExtFormatList = false;

        // Phase 2 — unit topology
        uint8_t isoInputPlugs  = 0;
        uint8_t isoOutputPlugs = 0;
        uint8_t extInputPlugs  = 0;
        uint8_t extOutputPlugs = 0;
        std::vector<SubunitEntry> subunits;
        bool hasAudioSubunit = false;
        bool hasMusicSubunit = false;
        uint8_t audioSubunitAddr = 0;   // e.g. 0x08
        uint8_t musicSubunitAddr = 0;   // e.g. 0x60

        // Phase 3 — audio subunit
        uint8_t audioDestPlugs = 0;     // input from audio-subunit POV
        uint8_t audioSrcPlugs  = 0;     // output from audio-subunit POV
        std::vector<uint8_t> audioDescriptorData;

        // Phase 4 — music subunit
        uint8_t musicDestPlugs = 0;
        uint8_t musicSrcPlugs  = 0;
        std::vector<uint8_t> musicDescriptorData;

        // Phase 6 — GetExtStreamFormat at audio subunit
        std::vector<PlugFormatResult> audioSubunitFormats;

        // Phase 8 — GetExtStreamFormat at music subunit
        std::vector<PlugFormatResult> musicSubunitFormats;

        // Phase 9 — Format LIST at UNIT (isoch plugs)
        uint32_t formatListCommandsSent = 0;
        std::vector<PlugFormatResult> unitIsochFormats;

        // Phase 10 — QuerySyncPlugReconnect
        uint8_t syncPlugsAccepted = 0;
        uint8_t syncPlugsTotal    = 0;

        // Phase 11
        uint32_t secondPassCommandsSent = 0;

        // Phase 12 — mixer reads (raw responses kept for future use)
        uint32_t mixerCommandsSent = 0;
    };

    // ── Public API ───────────────────────────────────────────────────────

    explicit AppleDiscoverySequence(FCPTransport& transport);

    /// Run the full Apple discovery sequence synchronously.
    /// Blocks the calling thread. Must NOT be called from the FCP timeout queue.
    Result RunSync();

private:
    FCPTransport& transport_;
    Result result_;

    // ── FCP send helper ──────────────────────────────────────────────────

    static constexpr uint32_t kMaxWaitMs   = 15000; // safety net; FCP own timeout fires first
    static constexpr uint32_t kPollMs      = 5;

    struct RawResult {
        bool     ok           = false;
        uint8_t  responseType = 0;  // ctype byte of response
        FCPFrame response;
    };

    RawResult SendRaw(const uint8_t* data, size_t length);
    RawResult SendRaw(std::initializer_list<uint8_t> bytes);

    // ── Phase implementations ────────────────────────────────────────────

    void Phase1_CheckExtFormatListSupport();
    void Phase2_UnitTopology();
    void Phase3_AudioSubunit();
    void Phase4_MusicSubunit();
    void Phase5_SignalSourceExternalAndAudio();
    void Phase6_AudioSubunitFormats();
    void Phase7_SignalSourceMusic();
    void Phase8_MusicSubunitFormats();
    void Phase9_FormatListAtUnit();
    void Phase10_SyncPlugReconnect();
    void Phase11_SecondPassFormats();
    void Phase12_MixerReads();

    // ── Descriptor read helper ───────────────────────────────────────────

    /// Read a descriptor in Apple's chunked pattern (OPEN → READ × N).
    /// |descriptorId| selects status (0x80) vs identifier (0x00).
    void ReadDescriptorChunked(uint8_t subunitAddr, uint8_t descriptorId,
                               std::vector<uint8_t>& outData);

    // ── Response helpers ─────────────────────────────────────────────────

    static bool IsAccepted(uint8_t ctype);
};

} // namespace ASFW::Protocols::AVC
