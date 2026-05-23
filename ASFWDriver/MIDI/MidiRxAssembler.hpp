// SPDX-License-Identifier: LGPL-3.0-or-later
// Copyright (c) 2026 ASFireWire Project
//
// MidiRxAssembler.hpp
// AM824 MIDI receive helper. It extracts MIDI 1.0 byte-stream chunks from
// AM824 MIDI quadlets without allocation.

#pragma once

#include "AM824MidiCodec.hpp"

#include <cstdint>

namespace ASFW::MIDI {

class MidiRxAssembler {
public:
    [[nodiscard]] AM824MidiBytes PushAM824Quadlet(uint32_t wireQuadlet) noexcept {
        if (!AM824MidiCodec::IsMidiQuadlet(wireQuadlet)) {
            ++nonMidiQuadlets_;
            return {};
        }

        const auto bytes = AM824MidiCodec::DecodeBytes(wireQuadlet);
        if (bytes.count == 0) {
            ++emptyQuadlets_;
        } else {
            byteCount_ += bytes.count;
        }
        return bytes;
    }

    [[nodiscard]] uint32_t ByteCount() const noexcept { return byteCount_; }
    [[nodiscard]] uint32_t EmptyQuadlets() const noexcept { return emptyQuadlets_; }
    [[nodiscard]] uint32_t NonMidiQuadlets() const noexcept { return nonMidiQuadlets_; }

    void Reset() noexcept {
        byteCount_ = 0;
        emptyQuadlets_ = 0;
        nonMidiQuadlets_ = 0;
    }

private:
    uint32_t byteCount_{0};
    uint32_t emptyQuadlets_{0};
    uint32_t nonMidiQuadlets_{0};
};

} // namespace ASFW::MIDI
