// SPDX-License-Identifier: LGPL-3.0-or-later
// Copyright (c) 2026 ASFireWire Project
//
// MidiTxQueue.hpp
// Single-producer/single-consumer byte queue for AM824 MIDI transmit.

#pragma once

#include "AM824MidiCodec.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace ASFW::MIDI {

template <size_t CapacityBytes>
class MidiTxQueue {
    static_assert(CapacityBytes >= 4, "MidiTxQueue needs room for at least one short MIDI message");

public:
    [[nodiscard]] bool PushByte(uint8_t byte) noexcept {
        const uint32_t head = head_.load(std::memory_order_relaxed);
        const uint32_t next = Advance(head);
        if (next == tail_.load(std::memory_order_acquire)) {
            droppedBytes_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        buffer_[head] = byte;
        head_.store(next, std::memory_order_release);
        return true;
    }

    [[nodiscard]] uint32_t PushBytes(const uint8_t* bytes, uint32_t count) noexcept {
        if (!bytes || count == 0) {
            return 0;
        }
        uint32_t pushed = 0;
        for (; pushed < count; ++pushed) {
            if (!PushByte(bytes[pushed])) {
                break;
            }
        }
        return pushed;
    }

    [[nodiscard]] AM824MidiBytes PopForAM824() noexcept {
        AM824MidiBytes out{};
        while (out.count < kAM824MidiMaxDataBytes && PopByte(out.bytes[out.count])) {
            ++out.count;
        }
        return out;
    }

    [[nodiscard]] uint32_t PopAM824Quadlet() noexcept {
        const auto bytes = PopForAM824();
        return (bytes.count == 0)
            ? AM824MidiCodec::EncodeNoData()
            : AM824MidiCodec::EncodeBytes(bytes.bytes, bytes.count);
    }

    [[nodiscard]] bool Empty() const noexcept {
        return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
    }

    [[nodiscard]] uint32_t DroppedBytes() const noexcept {
        return droppedBytes_.load(std::memory_order_relaxed);
    }

    void Clear() noexcept {
        tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release);
    }

private:
    [[nodiscard]] static constexpr uint32_t Advance(uint32_t index) noexcept {
        return (index + 1u) % static_cast<uint32_t>(CapacityBytes);
    }

    [[nodiscard]] bool PopByte(uint8_t& outByte) noexcept {
        const uint32_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;
        }
        outByte = buffer_[tail];
        tail_.store(Advance(tail), std::memory_order_release);
        return true;
    }

    std::array<uint8_t, CapacityBytes> buffer_{};
    std::atomic<uint32_t> head_{0};
    std::atomic<uint32_t> tail_{0};
    std::atomic<uint32_t> droppedBytes_{0};
};

} // namespace ASFW::MIDI
