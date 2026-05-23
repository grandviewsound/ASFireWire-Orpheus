// SPDX-License-Identifier: LGPL-3.0-or-later
// Copyright (c) 2026 ASFireWire Project
//
// UMPToMidi1.hpp
// Small MIDI 1.0 UMP decoder for MIDIDriverKit destination input.

#pragma once

#include "AM824MidiCodec.hpp"

#include <cstdint>

namespace ASFW::MIDI {

class UMPToMidi1 {
public:
    struct DecodedBytes {
        uint8_t count{0};
        uint8_t bytes[kAM824MidiMaxDataBytes]{};
    };

    [[nodiscard]] static constexpr DecodedBytes DecodeWord(uint32_t word) noexcept {
        const uint8_t messageType = static_cast<uint8_t>((word >> 28) & 0x0fu);
        const uint8_t status = static_cast<uint8_t>((word >> 16) & 0xffu);
        const uint8_t data1 = static_cast<uint8_t>((word >> 8) & 0xffu);
        const uint8_t data2 = static_cast<uint8_t>(word & 0xffu);

        if (messageType == 0x2u) {
            return DecodeMidi10ChannelVoice(status, data1, data2);
        }
        if (messageType == 0x1u) {
            return DecodeSystemMessage(status, data1, data2);
        }
        return {};
    }

private:
    [[nodiscard]] static constexpr DecodedBytes DecodeMidi10ChannelVoice(uint8_t status,
                                                                         uint8_t data1,
                                                                         uint8_t data2) noexcept {
        DecodedBytes out{};
        if ((status & 0x80u) == 0 || status >= 0xf0u) {
            return out;
        }

        out.bytes[0] = status;
        out.bytes[1] = data1;
        if ((status & 0xf0u) == 0xc0u || (status & 0xf0u) == 0xd0u) {
            out.count = 2;
        } else {
            out.bytes[2] = data2;
            out.count = 3;
        }
        return out;
    }

    [[nodiscard]] static constexpr DecodedBytes DecodeSystemMessage(uint8_t status,
                                                                    uint8_t data1,
                                                                    uint8_t data2) noexcept {
        DecodedBytes out{};
        if (status < 0xf0u) {
            return out;
        }

        out.bytes[0] = status;
        switch (status) {
            case 0xf1u:
            case 0xf3u:
                out.bytes[1] = data1;
                out.count = 2;
                break;
            case 0xf2u:
                out.bytes[1] = data1;
                out.bytes[2] = data2;
                out.count = 3;
                break;
            case 0xf6u:
            case 0xf8u:
            case 0xf9u:
            case 0xfau:
            case 0xfbu:
            case 0xfcu:
            case 0xfdu:
            case 0xfeu:
            case 0xffu:
                out.count = 1;
                break;
            default:
                break;
        }
        return out;
    }
};

} // namespace ASFW::MIDI
