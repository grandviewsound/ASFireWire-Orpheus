// SPDX-License-Identifier: LGPL-3.0-or-later
// Copyright (c) 2026 ASFireWire Project
//
// AM824MidiCodec.hpp
// IEC 61883-6 AM824 MIDI conformant data helpers.

#pragma once

#include <cstdint>

namespace ASFW::MIDI {

constexpr uint8_t kAM824MidiNoData = 0x80;
constexpr uint8_t kAM824MidiMaxDataBytes = 3;

struct AM824MidiBytes {
    uint8_t count{0};
    uint8_t bytes[kAM824MidiMaxDataBytes]{};
};

class AM824MidiCodec {
public:
    [[nodiscard]] static constexpr uint32_t EncodeNoData() noexcept {
        return ByteSwap32(static_cast<uint32_t>(kAM824MidiNoData) << 24);
    }

    [[nodiscard]] static constexpr uint32_t EncodeBytes(const uint8_t* bytes, uint8_t count) noexcept {
        const uint8_t clamped = (count > kAM824MidiMaxDataBytes) ? kAM824MidiMaxDataBytes : count;
        uint32_t hostWord = static_cast<uint32_t>(kAM824MidiNoData + clamped) << 24;
        if (bytes && clamped > 0) {
            hostWord |= static_cast<uint32_t>(bytes[0]) << 16;
        }
        if (bytes && clamped > 1) {
            hostWord |= static_cast<uint32_t>(bytes[1]) << 8;
        }
        if (bytes && clamped > 2) {
            hostWord |= static_cast<uint32_t>(bytes[2]);
        }
        return ByteSwap32(hostWord);
    }

    [[nodiscard]] static constexpr bool IsMidiQuadlet(uint32_t wireQuadlet) noexcept {
        const uint8_t label = Label(wireQuadlet);
        return label >= kAM824MidiNoData && label <= (kAM824MidiNoData + kAM824MidiMaxDataBytes);
    }

    [[nodiscard]] static constexpr uint8_t Label(uint32_t wireQuadlet) noexcept {
        return static_cast<uint8_t>((ByteSwap32(wireQuadlet) >> 24) & 0xffu);
    }

    [[nodiscard]] static constexpr AM824MidiBytes DecodeBytes(uint32_t wireQuadlet) noexcept {
        const uint32_t hostWord = ByteSwap32(wireQuadlet);
        const uint8_t label = static_cast<uint8_t>((hostWord >> 24) & 0xffu);
        AM824MidiBytes out{};
        if (label < kAM824MidiNoData || label > (kAM824MidiNoData + kAM824MidiMaxDataBytes)) {
            return out;
        }

        out.count = static_cast<uint8_t>(label - kAM824MidiNoData);
        if (out.count > 0) {
            out.bytes[0] = static_cast<uint8_t>((hostWord >> 16) & 0xffu);
        }
        if (out.count > 1) {
            out.bytes[1] = static_cast<uint8_t>((hostWord >> 8) & 0xffu);
        }
        if (out.count > 2) {
            out.bytes[2] = static_cast<uint8_t>(hostWord & 0xffu);
        }
        return out;
    }

private:
    [[nodiscard]] static constexpr uint32_t ByteSwap32(uint32_t x) noexcept {
        return ((x & 0xff000000u) >> 24) |
               ((x & 0x00ff0000u) >> 8)  |
               ((x & 0x0000ff00u) << 8)  |
               ((x & 0x000000ffu) << 24);
    }
};

} // namespace ASFW::MIDI
