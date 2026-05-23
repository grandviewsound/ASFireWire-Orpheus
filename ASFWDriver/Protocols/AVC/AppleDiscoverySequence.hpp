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

    struct SignalSourceResult {
        bool valid = false;
        uint8_t targetSubunit = 0xFF;
        uint8_t targetPlug = 0xFF;
        uint8_t sourceSubunit = 0xFF;
        uint8_t sourcePlug = 0xFF;
        uint8_t signalStatus = 0xFF;
        uint8_t streamStatus = 0xFF;
        bool hasFeedback = false;
        std::vector<uint8_t> rawResponse;
    };

    struct MixerReadResult {
        bool valid = false;
        uint8_t functionBlockId = 0xFF;
        uint8_t infoType = 0xFF;
        uint8_t channel = 0xFF;
        uint8_t controlSelector = 0xFF;
        uint8_t selectorAttribute = 0xFF;
        bool isMute = false;
        bool isVolume = false;
        bool mute = false;
        int16_t volume = -1;
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

        // SIGNAL SOURCE reads from phases 5, 7, 10, and 12.
        std::vector<SignalSourceResult> signalSources;

        // Phase 10 — QuerySyncPlugReconnect
        uint8_t syncPlugsAccepted = 0;
        uint8_t syncPlugsTotal    = 0;

        // Phase 11
        uint32_t secondPassCommandsSent = 0;

        // Phase 12 — mixer reads (raw responses kept for future use)
        uint32_t mixerCommandsSent = 0;
        std::vector<MixerReadResult> mixerReads;
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

    /// Send a SIGNAL SOURCE STATUS query (`01 ff 1a ff ff fe <subunit> <plug>`)
    /// and log the full response bytes plus a parsed interpretation. Used by
    /// Phase 5/7/10/12 to surface what sync source the device reports for each
    /// plug — the OS-log redaction strips operand bytes from `[FCP] FCP TX`
    /// lines, so we re-emit the bytes through the unfiltered Discovery
    /// channel here. Tag is a short label like "P5/ext" / "P5/audio".
    RawResult QueryAndLogSignalSource(const char* tag,
                                      uint8_t targetSubunit,
                                      uint8_t targetPlug);

    /// Send one Apple Phase-12 AUDIO FUNCTION BLOCK read and keep the parsed
    /// result. analysis evidence:
    ///   AM824AVC::GetChannelMute reads response[10] == 0x70 when ctype == 0x0c.
    ///   AM824AVC::GetChannelVolumeInfo reads big-endian response[10..11] when ctype == 0x0c.
    RawResult QueryAndLogMixerRead(uint8_t subunitAddr,
                                   uint8_t functionBlockId,
                                   uint8_t infoType,
                                   uint8_t channel,
                                   uint8_t controlSelector,
                                   uint8_t selectorAttribute,
                                   uint8_t valueLength);
};

} // namespace ASFW::Protocols::AVC
