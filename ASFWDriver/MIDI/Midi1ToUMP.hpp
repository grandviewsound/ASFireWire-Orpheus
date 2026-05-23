// SPDX-License-Identifier: LGPL-3.0-or-later
// Copyright (c) 2026 ASFireWire Project
//
// Midi1ToUMP.hpp
// MIDI 1.0 byte-message encoder for MIDIDriverKit source output.

#pragma once

#include <cstdint>

namespace ASFW::MIDI {

class Midi1ToUMP {
public:
    struct EncodedWords {
        uint8_t count{0};
        uint32_t words[1]{};
    };

    [[nodiscard]] static constexpr EncodedWords EncodeBytes(const uint8_t* bytes,
                                                            uint8_t count) noexcept {
        EncodedWords out{};
        if (!bytes || count == 0 || count > 3) {
            return out;
        }

        const uint8_t status = bytes[0];
        if (status >= 0x80u && status < 0xf0u) {
            return EncodeChannelVoice(bytes, count);
        }
        if (status >= 0xf0u) {
            return EncodeSystemMessage(bytes, count);
        }
        return out;
    }

private:
    [[nodiscard]] static constexpr EncodedWords EncodeChannelVoice(const uint8_t* bytes,
                                                                   uint8_t count) noexcept {
        EncodedWords out{};
        const uint8_t status = bytes[0];
        const uint8_t high = static_cast<uint8_t>(status & 0xf0u);
        const uint8_t needed = (high == 0xc0u || high == 0xd0u) ? 2 : 3;
        if (count != needed) {
            return out;
        }

        out.words[0] = (0x2u << 28) |
                       (static_cast<uint32_t>(status) << 16) |
                       (static_cast<uint32_t>(bytes[1]) << 8);
        if (needed == 3) {
            out.words[0] |= bytes[2];
        }
        out.count = 1;
        return out;
    }

    [[nodiscard]] static constexpr EncodedWords EncodeSystemMessage(const uint8_t* bytes,
                                                                    uint8_t count) noexcept {
        EncodedWords out{};
        const uint8_t status = bytes[0];
        uint8_t needed = 0;

        switch (status) {
            case 0xf1u:
            case 0xf3u:
                needed = 2;
                break;
            case 0xf2u:
                needed = 3;
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
                needed = 1;
                break;
            default:
                return out;
        }

        if (count != needed) {
            return out;
        }

        out.words[0] = (0x1u << 28) |
                       (static_cast<uint32_t>(status) << 16);
        if (needed > 1) {
            out.words[0] |= static_cast<uint32_t>(bytes[1]) << 8;
        }
        if (needed > 2) {
            out.words[0] |= bytes[2];
        }
        out.count = 1;
        return out;
    }
};

} // namespace ASFW::MIDI
