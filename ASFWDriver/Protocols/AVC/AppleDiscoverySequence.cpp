// SPDX-License-Identifier: LGPL-3.0-or-later
// Copyright (c) 2026 ASFireWire Project
//
// AppleDiscoverySequence — replays Apple's ~174-command AVC discovery.
// Byte patterns verified against Apr 12 2026 dtrace capture on macOS 11.
// See memory: apple-discovery-sequence-full-174.md for the definitive reference.

#include "AppleDiscoverySequence.hpp"
#include "../../Logging/Logging.hpp"
#ifndef ASFW_HOST_TEST
#include <DriverKit/IOLib.h>
#endif
#include <atomic>
#include <algorithm>

using namespace ASFW::Protocols::AVC;

// ── Construction ─────────────────────────────────────────────────────────────

AppleDiscoverySequence::AppleDiscoverySequence(FCPTransport& transport)
    : transport_(transport) {}

// ── Helpers ──────────────────────────────────────────────────────────────────

bool AppleDiscoverySequence::IsAccepted(uint8_t ctype) {
    // 0x09 = ACCEPTED (CONTROL success)
    // 0x0C = IMPLEMENTED/STABLE (STATUS success)
    return ctype == 0x09 || ctype == 0x0C;
}

// ── SignalSource diagnostic ──────────────────────────────────────────────────
//
// Sends a SIGNAL SOURCE STATUS query and logs the full response payload at V1
// so the next hardware capture surfaces what sync source the device currently
// reports per plug. The `[FCP] FCP TX` line in os_log redacts operand bytes
// past the first quadlet; this helper re-emits the response through the
// Discovery channel which is not redacted.
//
// analysis evidence: AM824AVC::GetSignalSourceInfo decodes the stable response as:
//   byte [3] status bits: low nibble=signal status, bit4=feedback, bits7:5=stream status
//   byte [4] source subunit
//   byte [5] source plug
//   bytes[6..7] destination echo
//
// On Orpheus the "signal source" field tells us which plug / subunit is
// currently driving this destination, so for sync-source queries it reveals
// whether the device is on Local, Wordclock, S/PDIF, ADAT, etc.

AppleDiscoverySequence::RawResult
AppleDiscoverySequence::QueryAndLogSignalSource(const char* tag,
                                                uint8_t targetSubunit,
                                                uint8_t targetPlug) {
    auto raw = SendRaw({0x01, 0xFF, 0x1A, 0xFF, 0xFF, 0xFE,
                         targetSubunit, targetPlug});

    if (!raw.ok) {
        ASFW_LOG_V1(Discovery,
                    "AppleDiscovery: %{public}s SSI dst=%02x:%02x — no response",
                    tag, targetSubunit, targetPlug);
        return raw;
    }

    const auto& bytes = raw.response.data;
    const size_t len  = raw.response.length;

    // Compact 12-byte hex dump (typical SSI response is 8-16 bytes).
    char hex[64] = {0};
    size_t hexLen = std::min<size_t>(len, 12);
    for (size_t i = 0; i < hexLen; i++) {
        snprintf(hex + (i * 3), sizeof(hex) - (i * 3),
                 (i + 1 == hexLen) ? "%02x" : "%02x ", bytes[i]);
    }

    uint8_t statusByte = (len >= 4) ? bytes[3] : 0xFF;
    uint8_t srcSubunit = (len >= 5) ? bytes[4] : 0xFF;
    uint8_t srcPlug    = (len >= 6) ? bytes[5] : 0xFF;
    uint8_t echoSubunit = (len >= 7) ? bytes[6] : 0xFF;
    uint8_t echoPlug = (len >= 8) ? bytes[7] : 0xFF;
    uint8_t signalStatus = statusByte & 0x0F;
    uint8_t streamStatus = statusByte >> 5;
    bool hasFeedback = (statusByte & 0x10) != 0;

    SignalSourceResult parsed{};
    parsed.valid = raw.responseType == 0x0C && len >= 6;
    parsed.targetSubunit = targetSubunit;
    parsed.targetPlug = targetPlug;
    parsed.sourceSubunit = srcSubunit;
    parsed.sourcePlug = srcPlug;
    parsed.signalStatus = signalStatus;
    parsed.streamStatus = streamStatus;
    parsed.hasFeedback = hasFeedback;
    parsed.rawResponse.assign(bytes.begin(), bytes.begin() + len);
    result_.signalSources.push_back(std::move(parsed));

    ASFW_LOG_V1(Discovery,
                "AppleDiscovery: %{public}s SSI dst=%02x:%02x → ctype=%02x "
                "status=0x%02x sig=%u stream=%u fb=%u src=%02x:%02x echo=%02x:%02x [%{public}s]",
                tag, targetSubunit, targetPlug,
                raw.responseType,
                statusByte,
                signalStatus,
                streamStatus,
                hasFeedback ? 1 : 0,
                srcSubunit,
                srcPlug,
                echoSubunit,
                echoPlug,
                hex);

    return raw;
}

AppleDiscoverySequence::RawResult
AppleDiscoverySequence::QueryAndLogMixerRead(uint8_t subunitAddr,
                                             uint8_t functionBlockId,
                                             uint8_t infoType,
                                             uint8_t channel,
                                             uint8_t controlSelector,
                                             uint8_t selectorAttribute,
                                             uint8_t valueLength) {
    uint8_t cmd[] = {
        0x01, subunitAddr, 0xB8, 0x81,
        functionBlockId, infoType,
        0x02, channel,
        controlSelector, selectorAttribute,
        0xFF, 0xFF
    };
    auto raw = SendRaw(cmd, valueLength == 2 ? 12 : 11);

    MixerReadResult parsed{};
    parsed.functionBlockId = functionBlockId;
    parsed.infoType = infoType;
    parsed.channel = channel;
    parsed.controlSelector = controlSelector;
    parsed.selectorAttribute = selectorAttribute;
    parsed.isMute = (controlSelector == 0x01);
    parsed.isVolume = (controlSelector == 0x02);

    if (raw.ok) {
        const auto& bytes = raw.response.data;
        const size_t len = raw.response.length;
        parsed.rawResponse.assign(bytes.begin(), bytes.begin() + len);
        parsed.valid = raw.responseType == 0x0C;

        if (parsed.valid && parsed.isMute && len >= 11) {
            parsed.mute = bytes[10] == 0x70;
        }

        if (parsed.valid && parsed.isVolume && len >= 12) {
            uint16_t rawVolume = (static_cast<uint16_t>(bytes[10]) << 8) | bytes[11];
            parsed.volume = static_cast<int16_t>(rawVolume);
        }
    }

    const char* selectorName = parsed.isMute ? "mute" : (parsed.isVolume ? "volume" : "control");
    if (!raw.ok) {
        ASFW_LOG_V1(Discovery,
                    "AppleDiscovery: Phase12 %{public}s fb=%02x info=%02x ch=%02x attr=%02x — no response",
                    selectorName, functionBlockId, infoType, channel, selectorAttribute);
    } else if (parsed.isMute) {
        ASFW_LOG_V1(Discovery,
                    "AppleDiscovery: Phase12 mute fb=%02x info=%02x ch=%02x ctype=%02x valid=%u value=0x%02x mute=%u",
                    functionBlockId, infoType, channel, raw.responseType, parsed.valid ? 1 : 0,
                    raw.response.length >= 11 ? raw.response.data[10] : 0xFF,
                    parsed.mute ? 1 : 0);
    } else if (parsed.isVolume) {
        ASFW_LOG_V1(Discovery,
                    "AppleDiscovery: Phase12 volume fb=%02x info=%02x ch=%02x attr=%02x ctype=%02x valid=%u volume=%d",
                    functionBlockId, infoType, channel, selectorAttribute, raw.responseType, parsed.valid ? 1 : 0,
                    static_cast<int>(parsed.volume));
    } else {
        ASFW_LOG_V1(Discovery,
                    "AppleDiscovery: Phase12 control fb=%02x info=%02x ch=%02x sel=%02x attr=%02x ctype=%02x valid=%u",
                    functionBlockId, infoType, channel, controlSelector, selectorAttribute,
                    raw.responseType, parsed.valid ? 1 : 0);
    }

    result_.mixerReads.push_back(std::move(parsed));
    return raw;
}

// ── SendRaw — synchronous FCP bridge ─────────────────────────────────────────
//
// Submits a raw FCP frame via the async FCPTransport and polls with IOSleep
// until the callback fires.  Must NOT be called from the FCP timeout queue
// (deadlock).  The FCP transport's own timeout (6 s initial, 10 s after
// interim) fires before kMaxWaitMs, so the polling loop is purely a safety
// net.

AppleDiscoverySequence::RawResult
AppleDiscoverySequence::SendRaw(const uint8_t* data, size_t length) {
    RawResult raw{};

    FCPFrame frame{};
    frame.length = std::min(length, frame.data.size());
    std::copy_n(data, frame.length, frame.data.begin());

    // Quadlet-align (IEC 61883-1 §9.3.1)
    size_t padded = (frame.length + 3) & ~size_t(3);
    if (padded > frame.length) {
        std::fill(frame.data.begin() + frame.length,
                  frame.data.begin() + padded, uint8_t(0));
    }
    frame.length = padded;

    std::atomic<bool> done{false};
    FCPStatus fcpStatus = FCPStatus::kTimeout;
    FCPFrame responseFrame{};

    (void)transport_.SubmitCommand(frame,
        [&done, &fcpStatus, &responseFrame](FCPStatus status, const FCPFrame& rsp) {
            fcpStatus = status;
            responseFrame = rsp;
            done.store(true, std::memory_order_release);
        });

    uint32_t elapsed = 0;
    while (!done.load(std::memory_order_acquire) && elapsed < kMaxWaitMs) {
        IOSleep(kPollMs);
        elapsed += kPollMs;
    }

    if (!done.load(std::memory_order_acquire)) {
        ASFW_LOG_V1(Discovery,
                    "AppleDiscovery: SendRaw safety-net timeout (%u ms)", kMaxWaitMs);
        return raw;
    }

    if (fcpStatus == FCPStatus::kOk && responseFrame.length >= 3) {
        raw.ok = true;
        raw.responseType = responseFrame.data[0];
        raw.response = responseFrame;
    } else {
        ASFW_LOG_V2(Discovery,
                    "AppleDiscovery: SendRaw failed (fcpStatus=%d, rspLen=%zu)",
                    static_cast<int>(fcpStatus), responseFrame.length);
    }

    return raw;
}

AppleDiscoverySequence::RawResult
AppleDiscoverySequence::SendRaw(std::initializer_list<uint8_t> bytes) {
    return SendRaw(bytes.begin(), bytes.size());
}

// ── ReadDescriptorChunked ────────────────────────────────────────────────────
//
// Sends READ DESCRIPTOR commands in Apple's chunked pattern:
//   1. First read at offset=0, length=0 (header query) — device returns the
//      first 0x84 bytes and reports its per-chunk data_length in the response.
//   2. Extract the descriptor's declared total length from payload bytes [0..1]
//      (AV/C Descriptor Mechanism TA 2002010 §10.1: every descriptor begins with
//      a 16-bit big-endian length field; the value counts all bytes of the
//      descriptor including the length field itself).
//   3. Continue chunked reads until we have the full declared length OR the
//      device reports read_result_status = kComplete (0x10).
//
// Response layout (single-byte specifier = descriptorId):
//   [0] ctype  [1] subunit  [2] opcode(0x09)  [3] descriptor_id
//   [4] read_result_status  [5] reserved
//   [6..7] data_length (this chunk only)   [8..9] address (offset of this chunk)
//   [10..] descriptor data
//
// Caller must OPEN the descriptor before calling this.

void AppleDiscoverySequence::ReadDescriptorChunked(uint8_t subunitAddr,
                                                    uint8_t descriptorId,
                                                    std::vector<uint8_t>& outData) {
    static constexpr uint16_t kChunkSize       = 0x84;  // Apple's chunk size (132 bytes)
    static constexpr uint32_t kMaxChunks       = 64;    // safety cap (64 * 132 ≈ 8 KiB)
    static constexpr uint16_t kMaxDescriptorLen = 8192; // sanity ceiling
    static constexpr uint16_t kSafetyMargin    = 64;    // for devices that under-report length
    static constexpr size_t   kStatusIdx       = 4;
    static constexpr size_t   kLenIdx          = 6;
    static constexpr size_t   kDataStart       = 10;
    static constexpr uint8_t  kStatusComplete  = 0x10;

    outData.clear();

    auto appendChunk = [&outData](const FCPFrame& frame,
                                   uint8_t& outStatus,
                                   uint16_t& outChunkLen) -> bool {
        if (frame.length < kDataStart) { return false; }
        outStatus   = frame.data[kStatusIdx];
        outChunkLen = (static_cast<uint16_t>(frame.data[kLenIdx]) << 8) | frame.data[kLenIdx + 1];
        size_t available = frame.length - kDataStart;
        size_t take = std::min<size_t>(outChunkLen, available);
        outData.insert(outData.end(),
                       frame.data.begin() + kDataStart,
                       frame.data.begin() + kDataStart + take);
        return true;
    };

    // First read: offset=0, length=0 (header query)
    // Wire: [00 <subunit> 09 <descId> ff ff 00 00 00 00]
    auto r0 = SendRaw({0x00, subunitAddr, 0x09, descriptorId,
                        0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00});
    if (!r0.ok) {
        ASFW_LOG_V1(Discovery,
                    "AppleDiscovery: ReadDescriptor header read failed "
                    "(subunit=0x%02x desc=0x%02x)", subunitAddr, descriptorId);
        return;
    }

    uint8_t status0 = 0;
    uint16_t chunkLen0 = 0;
    if (!appendChunk(r0.response, status0, chunkLen0)) {
        ASFW_LOG_V1(Discovery,
                    "AppleDiscovery: ReadDescriptor header response too short (%zu)",
                    r0.response.length);
        return;
    }

    // Extract declared total descriptor length from payload bytes [0..1].
    if (outData.size() < 2) {
        ASFW_LOG_V1(Discovery,
                    "AppleDiscovery: ReadDescriptor first chunk lacks length prefix "
                    "(subunit=0x%02x desc=0x%02x, got %zu bytes)",
                    subunitAddr, descriptorId, outData.size());
        return;
    }
    uint16_t declaredLen = (static_cast<uint16_t>(outData[0]) << 8) | outData[1];
    if (declaredLen < 2 || declaredLen > kMaxDescriptorLen) {
        ASFW_LOG_V1(Discovery,
                    "AppleDiscovery: ReadDescriptor suspicious declared length %u "
                    "(subunit=0x%02x desc=0x%02x)",
                    declaredLen, subunitAddr, descriptorId);
        return;
    }

    ASFW_LOG_V2(Discovery,
                "AppleDiscovery: ReadDescriptor subunit=0x%02x desc=0x%02x "
                "declaredLen=%u, first chunk=%u bytes, status=0x%02x",
                subunitAddr, descriptorId, declaredLen, chunkLen0, status0);

    // Dual termination: stop when we've read the declared length OR when device
    // reports kComplete (0x10).  Apogee-style safety margin handles devices that
    // under-report the declared length (seen on Duet/Ensemble per DescriptorAccessor).
    uint32_t targetBytes = static_cast<uint32_t>(declaredLen) + kSafetyMargin;
    bool deviceSaidDone  = (status0 == kStatusComplete);

    uint32_t chunks = 1;
    while (!deviceSaidDone
           && outData.size() < declaredLen
           && chunks < kMaxChunks) {
        uint32_t remainingToTarget = (outData.size() < targetBytes)
                                   ? (targetBytes - static_cast<uint32_t>(outData.size()))
                                   : 0;
        uint16_t readLen = static_cast<uint16_t>(std::min<uint32_t>(remainingToTarget, kChunkSize));
        if (readLen == 0) { break; }

        uint16_t offset = static_cast<uint16_t>(outData.size());

        // Wire: [00 <subunit> 09 <descId> ff ff <lenHi> <lenLo> <offHi> <offLo>]
        uint8_t cmd[] = {
            0x00, subunitAddr, 0x09, descriptorId,
            0xFF, 0xFF,
            static_cast<uint8_t>((readLen >> 8) & 0xFF),
            static_cast<uint8_t>(readLen & 0xFF),
            static_cast<uint8_t>((offset >> 8) & 0xFF),
            static_cast<uint8_t>(offset & 0xFF)
        };

        auto rN = SendRaw(cmd, sizeof(cmd));
        if (!rN.ok) {
            ASFW_LOG_V1(Discovery,
                        "AppleDiscovery: ReadDescriptor chunk %u failed at offset 0x%04x "
                        "(subunit=0x%02x desc=0x%02x, got %zu/%u)",
                        chunks, offset, subunitAddr, descriptorId,
                        outData.size(), declaredLen);
            break;
        }

        size_t before = outData.size();
        uint8_t statusN = 0;
        uint16_t chunkLenN = 0;
        if (!appendChunk(rN.response, statusN, chunkLenN)) {
            ASFW_LOG_V1(Discovery,
                        "AppleDiscovery: ReadDescriptor chunk %u response too short (%zu)",
                        chunks, rN.response.length);
            break;
        }
        if (outData.size() == before) {
            ASFW_LOG_V2(Discovery,
                        "AppleDiscovery: ReadDescriptor chunk %u empty — stopping at %zu/%u",
                        chunks, outData.size(), declaredLen);
            break;
        }

        chunks++;
        if (statusN == kStatusComplete) {
            deviceSaidDone = true;
        }
    }

    ASFW_LOG_V1(Discovery,
                "AppleDiscovery: ReadDescriptor complete — %zu/%u bytes in %u chunks "
                "(subunit=0x%02x desc=0x%02x, term=%s)",
                outData.size(), declaredLen, chunks, subunitAddr, descriptorId,
                deviceSaidDone ? "COMPLETE" : "LEN");
}

// ── RunSync ──────────────────────────────────────────────────────────────────

AppleDiscoverySequence::Result AppleDiscoverySequence::RunSync() {
    result_ = {};

    ASFW_LOG_V1(Discovery, "AppleDiscovery: === Starting Apple discovery sequence ===");

    Phase1_CheckExtFormatListSupport();
    Phase2_UnitTopology();

    // Phases 3-4 depend on Phase 2 discovering subunits
    if (result_.hasAudioSubunit) {
        Phase3_AudioSubunit();
    }
    if (result_.hasMusicSubunit) {
        Phase4_MusicSubunit();
    }

    Phase5_SignalSourceExternalAndAudio();

    if (result_.hasAudioSubunit) {
        Phase6_AudioSubunitFormats();
    }

    Phase7_SignalSourceMusic();

    if (result_.hasMusicSubunit) {
        Phase8_MusicSubunitFormats();
    }

    Phase9_FormatListAtUnit();
    Phase10_SyncPlugReconnect();
    Phase11_SecondPassFormats();
    Phase12_MixerReads();

    result_.success = true;

    ASFW_LOG_V1(Discovery,
                "AppleDiscovery: === Sequence complete — "
                "iso=%u/%u ext=%u/%u subunits=%zu ===",
                result_.isoInputPlugs, result_.isoOutputPlugs,
                result_.extInputPlugs, result_.extOutputPlugs,
                result_.subunits.size());

    return result_;
}

// ═════════════════════════════════════════════════════════════════════════════
// Phase 1 — CheckForExtendedStreamFormatListSupport
// Apple: InitializeDeviceInfo+0x490
// 2 commands: LIST entry 0 for isoch output plug 0, then isoch input plug 0
// ═════════════════════════════════════════════════════════════════════════════

void AppleDiscoverySequence::Phase1_CheckExtFormatListSupport() {
    ASFW_LOG_V2(Discovery, "AppleDiscovery: Phase 1 — CheckExtFormatListSupport");

    // 01 ff bf c1 01 00 00 00 ff ff 00   — LIST, output, isoch plug 0, entry 0
    auto r1 = SendRaw({0x01, 0xFF, 0xBF, 0xC1,
                        0x01, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00});
    if (r1.ok && IsAccepted(r1.responseType)) {
        result_.supportsExtFormatList = true;
    }

    // 01 ff bf c1 00 00 00 00 ff ff 00   — LIST, input, isoch plug 0, entry 0
    auto r2 = SendRaw({0x01, 0xFF, 0xBF, 0xC1,
                        0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00});
    if (r2.ok && IsAccepted(r2.responseType)) {
        result_.supportsExtFormatList = true;
    }

    ASFW_LOG_V2(Discovery,
                "AppleDiscovery: Phase 1 done — formatListSupport=%d",
                result_.supportsExtFormatList);
}

// ═════════════════════════════════════════════════════════════════════════════
// Phase 2 — Unit-level topology (GetPlugInfo + GetSubUnitInfo)
// Apple: initHardware+0x2e4
// ═════════════════════════════════════════════════════════════════════════════

void AppleDiscoverySequence::Phase2_UnitTopology() {
    ASFW_LOG_V2(Discovery, "AppleDiscovery: Phase 2 — UnitTopology");

    // GetPlugInfo at UNIT: 01 ff 02 00 ff ff ff ff
    auto r1 = SendRaw({0x01, 0xFF, 0x02, 0x00, 0xFF, 0xFF, 0xFF, 0xFF});
    if (r1.ok && IsAccepted(r1.responseType) && r1.response.length >= 8) {
        // Response: [0C FF 02 00 <isoIn> <isoOut> <extIn> <extOut>]
        //   operands at byte[3..6]
        // Actually: byte[0]=ctype, [1]=subunit, [2]=opcode, [3..6]=operands
        // GetPlugInfo response: byte[3]=subfunction(00), then:
        //   byte[4]=isoInputPlugs, byte[5]=isoOutputPlugs,
        //   byte[6]=extInputPlugs, byte[7]=extOutputPlugs
        // Wait, let me re-check. The command is:
        //   [01 ff 02 00 ff ff ff ff]
        //   ctype=01, subunit=ff, opcode=02, operands=[00 ff ff ff ff]
        // Response:
        //   [0c ff 02 00 02 02 09 00]  (Orpheus example)
        //   byte[3]=00 (subfunc), byte[4]=isoIn=2, byte[5]=isoOut=2,
        //   byte[6]=extIn=9, byte[7]=extOut=0
        result_.isoInputPlugs  = r1.response.data[4];
        result_.isoOutputPlugs = r1.response.data[5];
        result_.extInputPlugs  = r1.response.data[6];
        result_.extOutputPlugs = r1.response.data[7];

        ASFW_LOG_V1(Discovery,
                    "AppleDiscovery: UNIT plugs — iso in=%u out=%u, ext in=%u out=%u",
                    result_.isoInputPlugs, result_.isoOutputPlugs,
                    result_.extInputPlugs, result_.extOutputPlugs);
    } else {
        ASFW_LOG_V1(Discovery, "AppleDiscovery: GetPlugInfo at UNIT failed");
    }

    // GetSubUnitInfo at UNIT: 01 ff 31 07 ff ff ff ff
    auto r2 = SendRaw({0x01, 0xFF, 0x31, 0x07, 0xFF, 0xFF, 0xFF, 0xFF});
    if (r2.ok && IsAccepted(r2.responseType) && r2.response.length >= 8) {
        // Response: [0C FF 31 07 <sub0> <sub1> <sub2> <sub3>]
        // Each non-FF byte encodes: type[7:3] | maxID[2:0]
        // Orpheus returns: 08 60 ff ff
        //   0x08 = type=1(Audio) id=0 → address byte 0x08
        //   0x60 = type=12(Music) id=0 → address byte 0x60
        for (int i = 0; i < 4; i++) {
            uint8_t b = r2.response.data[4 + i];
            if (b == 0xFF) continue;

            uint8_t type  = (b >> 3) & 0x1F;
            uint8_t maxId = b & 0x07;

            SubunitEntry entry;
            entry.type  = type;
            entry.maxId = maxId;
            result_.subunits.push_back(entry);

            ASFW_LOG_V2(Discovery,
                        "AppleDiscovery: Subunit type=%u maxId=%u (addr byte=0x%02x)",
                        type, maxId, b);

            if (type == 1) {  // Audio
                result_.hasAudioSubunit = true;
                result_.audioSubunitAddr = b;  // 0x08
            }
            if (type == 12) {  // Music (0x0C)
                result_.hasMusicSubunit = true;
                result_.musicSubunitAddr = b;  // 0x60
            }
        }
    } else {
        ASFW_LOG_V1(Discovery, "AppleDiscovery: GetSubUnitInfo at UNIT failed");
    }

    ASFW_LOG_V2(Discovery,
                "AppleDiscovery: Phase 2 done — audio=%d(0x%02x) music=%d(0x%02x)",
                result_.hasAudioSubunit, result_.audioSubunitAddr,
                result_.hasMusicSubunit, result_.musicSubunitAddr);
}

// ═════════════════════════════════════════════════════════════════════════════
// Phase 3 — Audio subunit enumeration
// AppleFWAudioDevice::InitializeDeviceInfo probes descriptor 0x80 for each
// subunit table entry, but for AV/C Audio subunit type 0x01 it consumes
// descriptor 0x00 and wraps those bytes in synthetic root 0x0C0C0000.
// ═════════════════════════════════════════════════════════════════════════════

void AppleDiscoverySequence::Phase3_AudioSubunit() {
    const uint8_t sub = result_.audioSubunitAddr;  // 0x08
    ASFW_LOG_V2(Discovery, "AppleDiscovery: Phase 3 — AudioSubunit (0x%02x)", sub);

    // GetPlugInfo at AUDIO subunit: 01 08 02 00 ff ff ff ff
    auto r1 = SendRaw({0x01, sub, 0x02, 0x00, 0xFF, 0xFF, 0xFF, 0xFF});
    if (r1.ok && IsAccepted(r1.responseType) && r1.response.length >= 8) {
        // Response operands: [00 <destPlugs> <srcPlugs> ff ff]
        // For subunit GetPlugInfo: byte[4]=dest(input), byte[5]=src(output)
        result_.audioDestPlugs = r1.response.data[4];
        result_.audioSrcPlugs  = r1.response.data[5];
        ASFW_LOG_V1(Discovery,
                    "AppleDiscovery: Audio subunit plugs — dest=%u src=%u",
                    result_.audioDestPlugs, result_.audioSrcPlugs);
    }

    // OpenDescriptor: status/probe (0x80). analysis shows GetSubunitDescriptor(..., 0x80)
    // is attempted before the type-specific descriptor parse.
    // 00 08 08 80 01 ff
    auto open80 = SendRaw({0x00, sub, 0x08, 0x80, 0x01, 0xFF});
    std::vector<uint8_t> audioStatusProbe;
    if (open80.ok && IsAccepted(open80.responseType)) {
        ReadDescriptorChunked(sub, 0x80, audioStatusProbe);
    }

    // OpenDescriptor: identifier (0x00), which Apple stores for Audio type 0x01.
    // 00 08 08 00 01 ff
    auto open00 = SendRaw({0x00, sub, 0x08, 0x00, 0x01, 0xFF});

    // ReadDescriptor from identifier (0x00). analysis evidence:
    // AppleFWAudioDevice::InitializeDeviceInfo -> AM824AVC::GetSubunitDescriptor(sub, 0, ...)
    // for subunit type 1, followed by AVCInfoBlock::init(..., 0x0C0C0000, len).
    if (open00.ok && IsAccepted(open00.responseType)) {
        ReadDescriptorChunked(sub, 0x00, result_.audioDescriptorData);
    }

    ASFW_LOG_V2(Discovery,
                "AppleDiscovery: Phase 3 done — descriptor %zu bytes",
                result_.audioDescriptorData.size());
}

// ═════════════════════════════════════════════════════════════════════════════
// Phase 4 — Music subunit enumeration
// GetPlugInfo + OpenDescriptor(0x80) + ReadDescriptor(0x80) in ~20 chunks
// ═════════════════════════════════════════════════════════════════════════════

void AppleDiscoverySequence::Phase4_MusicSubunit() {
    const uint8_t sub = result_.musicSubunitAddr;  // 0x60
    ASFW_LOG_V2(Discovery, "AppleDiscovery: Phase 4 — MusicSubunit (0x%02x)", sub);

    // GetPlugInfo at MUSIC subunit: 01 60 02 00 ff ff ff ff
    auto r1 = SendRaw({0x01, sub, 0x02, 0x00, 0xFF, 0xFF, 0xFF, 0xFF});
    if (r1.ok && IsAccepted(r1.responseType) && r1.response.length >= 8) {
        result_.musicDestPlugs = r1.response.data[4];
        result_.musicSrcPlugs  = r1.response.data[5];
        ASFW_LOG_V1(Discovery,
                    "AppleDiscovery: Music subunit plugs — dest=%u src=%u",
                    result_.musicDestPlugs, result_.musicSrcPlugs);
    }

    // OpenDescriptor: status (0x80)
    // 00 60 08 80 01 ff
    SendRaw({0x00, sub, 0x08, 0x80, 0x01, 0xFF});

    // ReadDescriptor from status (0x80) — Apple reads ~20 chunks
    ReadDescriptorChunked(sub, 0x80, result_.musicDescriptorData);

    ASFW_LOG_V2(Discovery,
                "AppleDiscovery: Phase 4 done — descriptor %zu bytes",
                result_.musicDescriptorData.size());
}

// ═════════════════════════════════════════════════════════════════════════════
// Phase 5 — Signal source enumeration at UNIT (external + audio plugs)
// Apple: 20 calls total — 9 for ext plugs + 11 for audio subunit plugs
// ═════════════════════════════════════════════════════════════════════════════

void AppleDiscoverySequence::Phase5_SignalSourceExternalAndAudio() {
    ASFW_LOG_V2(Discovery, "AppleDiscovery: Phase 5 — SignalSource (ext + audio)");

    // External plugs: ff:00, ff:01, ff:80..ff:86
    // Apple captures 9 external plug queries.
    // Plug numbers: 0x00, 0x01, 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86
    // These correspond to iso input 0, iso input 1, then ext plugs 0..6
    // But we derive the list from Phase 2 plug counts for generality.

    // Iso input plugs (0x00..)
    for (uint8_t p = 0; p < result_.isoInputPlugs; p++) {
        QueryAndLogSignalSource("P5/iso", 0xFF, p);
    }

    // External input plugs (0x80..)
    for (uint8_t p = 0; p < result_.extInputPlugs; p++) {
        QueryAndLogSignalSource("P5/ext", 0xFF, uint8_t(0x80 + p));
    }

    // Audio subunit plugs: 08:00..08:0a (11 plugs for Orpheus)
    // Derived from audioDestPlugs + audioSrcPlugs
    if (result_.hasAudioSubunit) {
        uint8_t totalAudioPlugs = result_.audioDestPlugs + result_.audioSrcPlugs;
        for (uint8_t p = 0; p < totalAudioPlugs; p++) {
            QueryAndLogSignalSource("P5/audio", result_.audioSubunitAddr, p);
        }
    }

    ASFW_LOG_V2(Discovery, "AppleDiscovery: Phase 5 done");
}

// ═════════════════════════════════════════════════════════════════════════════
// Phase 6 — GetExtendedStreamFormat at AUDIO subunit (per plug)
// Apple: output plugs 0..5, then input plugs 0..10
// ═════════════════════════════════════════════════════════════════════════════

void AppleDiscoverySequence::Phase6_AudioSubunitFormats() {
    const uint8_t sub = result_.audioSubunitAddr;  // 0x08
    ASFW_LOG_V2(Discovery, "AppleDiscovery: Phase 6 — AudioSubunit formats (0x%02x)", sub);

    // Output (source) plugs: 01 08 bf c0 01 01 <plug> ff ff ff
    for (uint8_t p = 0; p < result_.audioSrcPlugs; p++) {
        auto r = SendRaw({0x01, sub, 0xBF, 0xC0,
                           0x01, 0x01, p, 0xFF, 0xFF, 0xFF});
        PlugFormatResult pfr;
        pfr.direction = 1;  // output
        pfr.plugType = 1;   // subunit
        pfr.plugNum = p;
        if (r.ok && IsAccepted(r.responseType)) {
            pfr.valid = true;
            pfr.rawResponse.assign(r.response.data.begin(),
                                   r.response.data.begin() + r.response.length);
        }
        result_.audioSubunitFormats.push_back(pfr);
    }

    // Input (dest) plugs: 01 08 bf c0 00 01 <plug> ff ff ff
    for (uint8_t p = 0; p < result_.audioDestPlugs; p++) {
        auto r = SendRaw({0x01, sub, 0xBF, 0xC0,
                           0x00, 0x01, p, 0xFF, 0xFF, 0xFF});
        PlugFormatResult pfr;
        pfr.direction = 0;  // input
        pfr.plugType = 1;   // subunit
        pfr.plugNum = p;
        if (r.ok && IsAccepted(r.responseType)) {
            pfr.valid = true;
            pfr.rawResponse.assign(r.response.data.begin(),
                                   r.response.data.begin() + r.response.length);
        }
        result_.audioSubunitFormats.push_back(pfr);
    }

    ASFW_LOG_V2(Discovery,
                "AppleDiscovery: Phase 6 done — %zu audio format results",
                result_.audioSubunitFormats.size());
}

// ═════════════════════════════════════════════════════════════════════════════
// Phase 7 — Signal source at UNIT for music subunit plugs
// Apple: 8 calls for 60:00..60:07
// ═════════════════════════════════════════════════════════════════════════════

void AppleDiscoverySequence::Phase7_SignalSourceMusic() {
    ASFW_LOG_V2(Discovery, "AppleDiscovery: Phase 7 — SignalSource (music)");

    if (!result_.hasMusicSubunit) {
        ASFW_LOG_V2(Discovery, "AppleDiscovery: Phase 7 skipped — no music subunit");
        return;
    }

    // Music subunit plugs: for Orpheus, Apple queries 60:00..60:07
    // Derived from musicDestPlugs (input plugs at music subunit)
    uint8_t totalMusicPlugs = result_.musicDestPlugs;
    for (uint8_t p = 0; p < totalMusicPlugs; p++) {
        QueryAndLogSignalSource("P7/music", result_.musicSubunitAddr, p);
    }

    ASFW_LOG_V2(Discovery, "AppleDiscovery: Phase 7 done (%u plugs)", totalMusicPlugs);
}

// ═════════════════════════════════════════════════════════════════════════════
// Phase 8 — GetExtendedStreamFormat at MUSIC subunit (per plug)
// Apple: output plugs 0..8, then input plugs 0..7
// ═════════════════════════════════════════════════════════════════════════════

void AppleDiscoverySequence::Phase8_MusicSubunitFormats() {
    const uint8_t sub = result_.musicSubunitAddr;  // 0x60
    ASFW_LOG_V2(Discovery, "AppleDiscovery: Phase 8 — MusicSubunit formats (0x%02x)", sub);

    // Output (source) plugs: 01 60 bf c0 01 01 <plug> ff ff ff
    for (uint8_t p = 0; p < result_.musicSrcPlugs; p++) {
        auto r = SendRaw({0x01, sub, 0xBF, 0xC0,
                           0x01, 0x01, p, 0xFF, 0xFF, 0xFF});
        PlugFormatResult pfr;
        pfr.direction = 1;  // output
        pfr.plugType = 1;   // subunit
        pfr.plugNum = p;
        if (r.ok && IsAccepted(r.responseType)) {
            pfr.valid = true;
            pfr.rawResponse.assign(r.response.data.begin(),
                                   r.response.data.begin() + r.response.length);
        }
        result_.musicSubunitFormats.push_back(pfr);
    }

    // Input (dest) plugs: 01 60 bf c0 00 01 <plug> ff ff ff
    for (uint8_t p = 0; p < result_.musicDestPlugs; p++) {
        auto r = SendRaw({0x01, sub, 0xBF, 0xC0,
                           0x00, 0x01, p, 0xFF, 0xFF, 0xFF});
        PlugFormatResult pfr;
        pfr.direction = 0;  // input
        pfr.plugType = 1;   // subunit
        pfr.plugNum = p;
        if (r.ok && IsAccepted(r.responseType)) {
            pfr.valid = true;
            pfr.rawResponse.assign(r.response.data.begin(),
                                   r.response.data.begin() + r.response.length);
        }
        result_.musicSubunitFormats.push_back(pfr);
    }

    ASFW_LOG_V2(Discovery,
                "AppleDiscovery: Phase 8 done — %zu music format results",
                result_.musicSubunitFormats.size());
}

// ═════════════════════════════════════════════════════════════════════════════
// Phase 9 — Format LIST enumeration at UNIT (isoch plugs)
// For each isoch plug: 7× LIST entries 0..6 + 1× current format read
// Apple order: output 0, output 1, input 0, input 1
// ═════════════════════════════════════════════════════════════════════════════

void AppleDiscoverySequence::Phase9_FormatListAtUnit() {
    ASFW_LOG_V2(Discovery, "AppleDiscovery: Phase 9 — FormatList at UNIT");

    uint32_t cmdCount = 0;

    auto queryPlug = [&](uint8_t direction, uint8_t plugNum) {
        // LIST entries 0..6:
        // 01 ff bf c1 <dir> 00 00 <plug> ff ff <entry>
        for (uint8_t entry = 0; entry < 7; entry++) {
            auto r = SendRaw({0x01, 0xFF, 0xBF, 0xC1,
                               direction, 0x00, 0x00, plugNum,
                               0xFF, 0xFF, entry});
            cmdCount++;

            PlugFormatResult pfr;
            pfr.direction = direction;
            pfr.plugType = 0;  // isoch
            pfr.plugNum = plugNum;
            if (r.ok && IsAccepted(r.responseType)) {
                pfr.valid = true;
                pfr.rawResponse.assign(r.response.data.begin(),
                                       r.response.data.begin() + r.response.length);
            }
            result_.unitIsochFormats.push_back(pfr);

            // If device returns NOT_IMPLEMENTED, stop querying this plug
            if (r.ok && r.responseType == 0x08) break;
        }

        // Current format (terminator/verify):
        // 01 ff bf c0 <dir> 00 00 <plug> ff ff
        auto rCur = SendRaw({0x01, 0xFF, 0xBF, 0xC0,
                              direction, 0x00, 0x00, plugNum, 0xFF, 0xFF});
        cmdCount++;

        PlugFormatResult pfr;
        pfr.direction = direction;
        pfr.plugType = 0;  // isoch
        pfr.plugNum = plugNum;
        if (rCur.ok && IsAccepted(rCur.responseType)) {
            pfr.valid = true;
            pfr.rawResponse.assign(rCur.response.data.begin(),
                                   rCur.response.data.begin() + rCur.response.length);
        }
        result_.unitIsochFormats.push_back(pfr);
    };

    // Apple order: output plugs first, then input plugs
    for (uint8_t p = 0; p < result_.isoOutputPlugs; p++) {
        queryPlug(0x01, p);  // output
    }
    for (uint8_t p = 0; p < result_.isoInputPlugs; p++) {
        queryPlug(0x00, p);  // input
    }

    result_.formatListCommandsSent = cmdCount;
    ASFW_LOG_V2(Discovery,
                "AppleDiscovery: Phase 9 done — %u commands sent",
                cmdCount);
}

// ═════════════════════════════════════════════════════════════════════════════
// Phase 10 — QuerySyncPlugReconnect + interleaved SignalSource
// Apple: 11 QSPR + 4 SignalSource, interleaved
// QSPR uses CTYPE=0x02 (SPECIFIC INQUIRY), opcode=0x1A, subfunc=0x0F
// ═════════════════════════════════════════════════════════════════════════════

void AppleDiscoverySequence::Phase10_SyncPlugReconnect() {
    ASFW_LOG_V2(Discovery, "AppleDiscovery: Phase 10 — SyncPlugReconnect");

    // Apple's exact interleaved sequence from the capture.
    // The interleaving pattern is specific — replicate it exactly.
    //
    // QSPR: 02 ff 1a 0f ff <plug> 60 07
    // SSI:  01 ff 1a ff ff fe ff <plug>

    struct Step {
        enum Type { QSPR, SSI } type;
        uint8_t targetSubunit;  // for QSPR
        uint8_t targetPlug;     // for QSPR or SSI
    };

    // Build the interleaved sequence matching Apple exactly.
    // From apple-discovery-sequence-full-174.md Phase 10:
    std::vector<Step> steps;

    // QSPR plug 00
    steps.push_back({Step::QSPR, 0xFF, 0x00});
    // SSI ff:00
    steps.push_back({Step::SSI, 0xFF, 0x00});
    // QSPR plug 01
    steps.push_back({Step::QSPR, 0xFF, 0x01});
    // QSPR plugs 80..85
    for (uint8_t p = 0x80; p <= 0x85; p++) {
        steps.push_back({Step::QSPR, 0xFF, p});
    }
    // SSI ff:85
    steps.push_back({Step::SSI, 0xFF, 0x85});
    // QSPR plug 86
    steps.push_back({Step::QSPR, 0xFF, 0x86});
    // SSI ff:86
    steps.push_back({Step::SSI, 0xFF, 0x86});
    // QSPR plug 87
    steps.push_back({Step::QSPR, 0xFF, 0x87});
    // SSI ff:87
    steps.push_back({Step::SSI, 0xFF, 0x87});
    // QSPR final: music subunit plug 08
    steps.push_back({Step::QSPR, 0x60, 0x08});

    uint8_t accepted = 0;
    uint8_t total = 0;

    for (const auto& step : steps) {
        if (step.type == Step::QSPR) {
            // 02 ff 1a 0f <targetSubunit> <targetPlug> 60 07
            auto r = SendRaw({0x02, 0xFF, 0x1A, 0x0F,
                               step.targetSubunit, step.targetPlug, 0x60, 0x07});
            total++;
            if (r.ok && IsAccepted(r.responseType)) {
                accepted++;
            }
        } else {
            QueryAndLogSignalSource("P10/qspr", 0xFF, step.targetPlug);
        }
    }

    result_.syncPlugsAccepted = accepted;
    result_.syncPlugsTotal = total;

    ASFW_LOG_V1(Discovery,
                "AppleDiscovery: Phase 10 done — QSPR %u/%u accepted",
                accepted, total);
}

// ═════════════════════════════════════════════════════════════════════════════
// Phase 11 — Second pass GetExtendedStreamFormat (output plugs only)
// Apple re-reads formats at AUDIO subunit (6 output) + MUSIC subunit (9 output)
// ═════════════════════════════════════════════════════════════════════════════

void AppleDiscoverySequence::Phase11_SecondPassFormats() {
    ASFW_LOG_V2(Discovery, "AppleDiscovery: Phase 11 — SecondPass formats");

    uint32_t cmdCount = 0;

    // Audio subunit output plugs: 01 08 bf c0 01 01 <plug> ff ff ff
    if (result_.hasAudioSubunit) {
        for (uint8_t p = 0; p < result_.audioSrcPlugs; p++) {
            SendRaw({0x01, result_.audioSubunitAddr, 0xBF, 0xC0,
                      0x01, 0x01, p, 0xFF, 0xFF, 0xFF});
            cmdCount++;
        }
    }

    // Music subunit output plugs: 01 60 bf c0 01 01 <plug> ff ff ff
    if (result_.hasMusicSubunit) {
        for (uint8_t p = 0; p < result_.musicSrcPlugs; p++) {
            SendRaw({0x01, result_.musicSubunitAddr, 0xBF, 0xC0,
                      0x01, 0x01, p, 0xFF, 0xFF, 0xFF});
            cmdCount++;
        }
    }

    result_.secondPassCommandsSent = cmdCount;
    ASFW_LOG_V2(Discovery,
                "AppleDiscovery: Phase 11 done — %u re-reads", cmdCount);
}

// ═════════════════════════════════════════════════════════════════════════════
// Phase 12 — Mixer state reads (GetChannelVolumeInfo + GetChannelMute)
// Apple: addressed to AUDIO subunit (0x08), opcode 0xB8
//
// NOTE: Apple's phase 12 also includes CMD A/B/C (ExtStreamFormat CONTROL).
// Those are NOT sent here — they belong in the streaming bring-up pipeline
// (BringUpPipeline in AVCAudioBackend) because they require IRM/CMP context.
// We only send the STATUS/read commands from this phase.
// ═════════════════════════════════════════════════════════════════════════════

void AppleDiscoverySequence::Phase12_MixerReads() {
    ASFW_LOG_V2(Discovery, "AppleDiscovery: Phase 12 — MixerReads");

    if (!result_.hasAudioSubunit) {
        ASFW_LOG_V2(Discovery, "AppleDiscovery: Phase 12 skipped — no audio subunit");
        return;
    }

    const uint8_t sub = result_.audioSubunitAddr;  // 0x08
    uint32_t cmdCount = 0;

    // analysis: AM824AVC::GetChannelVolumeInfo/GetChannelMute build:
    //
    // GetChannelVolumeInfo: 01 08 b8 81 <blockID> <infoType> 02 <channel> 02 02 ff ff
    // GetChannelMute:       01 08 b8 81 <blockID> 10         02 <channel> 01 01 ff
    //
    // infoType 0x10 is current. The extra infoType probes are:
    // 0x01 = resolution, 0x02 = minimum, 0x03 = maximum.
    //
    // Apple stores output and input function block IDs on AppleFWAudioDevice and
    // passes that byte as blockID when creating/servicing HAL controls. We probe
    // candidate block IDs 1 and 2 with channel 0, matching the captured sequence.
    {
        const uint8_t blockId = 0x01;

        // Current volume and mute for channel 0.
        QueryAndLogMixerRead(sub, blockId, 0x10, 0x00, 0x02, 0x02, 2);
        cmdCount++;

        QueryAndLogMixerRead(sub, blockId, 0x10, 0x00, 0x01, 0x01, 1);
        cmdCount++;

        // Volume resolution/min/max.
        for (uint8_t infoType = 0x01; infoType <= 0x03; infoType++) {
            QueryAndLogMixerRead(sub, blockId, infoType, 0x00, 0x02, 0x02, 2);
            cmdCount++;
        }
    }

    {
        const uint8_t blockId = 0x02;

        QueryAndLogMixerRead(sub, blockId, 0x10, 0x00, 0x02, 0x02, 2);
        cmdCount++;

        QueryAndLogMixerRead(sub, blockId, 0x10, 0x00, 0x01, 0x01, 1);
        cmdCount++;

        for (uint8_t infoType = 0x01; infoType <= 0x03; infoType++) {
            QueryAndLogMixerRead(sub, blockId, infoType, 0x00, 0x02, 0x02, 2);
            cmdCount++;
        }
    }

    // Final SignalSource query: 01 ff 1a ff ff fe 60 07
    QueryAndLogSignalSource("P12/final", 0x60, 0x07);
    cmdCount++;

    result_.mixerCommandsSent = cmdCount;
    ASFW_LOG_V1(Discovery,
                "AppleDiscovery: Phase 12 done — %u mixer/SSI commands", cmdCount);
}
