// AM824MidiCodecTests.cpp

#include "MIDI/AM824MidiCodec.hpp"
#include "MIDI/MidiRxAssembler.hpp"
#include "MIDI/MidiTxQueue.hpp"

#include <gtest/gtest.h>

using ASFW::MIDI::AM824MidiCodec;
using ASFW::MIDI::MidiRxAssembler;
using ASFW::MIDI::MidiTxQueue;

namespace {

uint8_t WireByte(uint32_t wireQuadlet, uint32_t byteIndex) {
    return static_cast<uint8_t>((wireQuadlet >> (byteIndex * 8u)) & 0xffu);
}

} // namespace

TEST(AM824MidiCodecTests, EncodesNoDataPlaceholder) {
    const uint32_t q = AM824MidiCodec::EncodeNoData();
    EXPECT_EQ(WireByte(q, 0), 0x80);
    EXPECT_EQ(WireByte(q, 1), 0x00);
    EXPECT_EQ(WireByte(q, 2), 0x00);
    EXPECT_EQ(WireByte(q, 3), 0x00);
}

TEST(AM824MidiCodecTests, EncodesOneTwoAndThreeBytePayloads) {
    const uint8_t one[] = {0xf8};
    const uint8_t two[] = {0xc0, 0x05};
    const uint8_t three[] = {0x90, 0x40, 0x7f};

    const uint32_t q1 = AM824MidiCodec::EncodeBytes(one, 1);
    EXPECT_EQ(WireByte(q1, 0), 0x81);
    EXPECT_EQ(WireByte(q1, 1), 0xf8);

    const uint32_t q2 = AM824MidiCodec::EncodeBytes(two, 2);
    EXPECT_EQ(WireByte(q2, 0), 0x82);
    EXPECT_EQ(WireByte(q2, 1), 0xc0);
    EXPECT_EQ(WireByte(q2, 2), 0x05);

    const uint32_t q3 = AM824MidiCodec::EncodeBytes(three, 3);
    EXPECT_EQ(WireByte(q3, 0), 0x83);
    EXPECT_EQ(WireByte(q3, 1), 0x90);
    EXPECT_EQ(WireByte(q3, 2), 0x40);
    EXPECT_EQ(WireByte(q3, 3), 0x7f);
}

TEST(AM824MidiCodecTests, DecodesPayloads) {
    const uint8_t bytes[] = {0x80, 0x3c, 0x00};
    const auto decoded = AM824MidiCodec::DecodeBytes(AM824MidiCodec::EncodeBytes(bytes, 3));

    ASSERT_EQ(decoded.count, 3);
    EXPECT_EQ(decoded.bytes[0], 0x80);
    EXPECT_EQ(decoded.bytes[1], 0x3c);
    EXPECT_EQ(decoded.bytes[2], 0x00);
}

TEST(MidiTxQueueTests, EmitsNoDataWhenEmpty) {
    MidiTxQueue<8> queue;
    EXPECT_EQ(queue.PopAM824Quadlet(), AM824MidiCodec::EncodeNoData());
}

TEST(MidiTxQueueTests, ChunksByteStreamIntoAm824Triples) {
    MidiTxQueue<16> queue;
    const uint8_t bytes[] = {0x90, 0x40, 0x7f, 0x80, 0x40, 0x00, 0xf8};
    EXPECT_EQ(queue.PushBytes(bytes, sizeof(bytes)), sizeof(bytes));

    auto first = AM824MidiCodec::DecodeBytes(queue.PopAM824Quadlet());
    ASSERT_EQ(first.count, 3);
    EXPECT_EQ(first.bytes[0], 0x90);
    EXPECT_EQ(first.bytes[1], 0x40);
    EXPECT_EQ(first.bytes[2], 0x7f);

    auto second = AM824MidiCodec::DecodeBytes(queue.PopAM824Quadlet());
    ASSERT_EQ(second.count, 3);
    EXPECT_EQ(second.bytes[0], 0x80);
    EXPECT_EQ(second.bytes[1], 0x40);
    EXPECT_EQ(second.bytes[2], 0x00);

    auto third = AM824MidiCodec::DecodeBytes(queue.PopAM824Quadlet());
    ASSERT_EQ(third.count, 1);
    EXPECT_EQ(third.bytes[0], 0xf8);
}

TEST(MidiTxQueueTests, ReportsOverflowWithoutAllocating) {
    MidiTxQueue<4> queue;
    const uint8_t bytes[] = {1, 2, 3, 4};

    EXPECT_EQ(queue.PushBytes(bytes, sizeof(bytes)), 3u);
    EXPECT_EQ(queue.DroppedBytes(), 1u);
}

TEST(MidiRxAssemblerTests, ExtractsBytesAndCountsEmptyQuadlets) {
    MidiRxAssembler rx;
    const uint8_t bytes[] = {0xf0, 0x7d, 0x01};

    auto empty = rx.PushAM824Quadlet(AM824MidiCodec::EncodeNoData());
    EXPECT_EQ(empty.count, 0);
    EXPECT_EQ(rx.EmptyQuadlets(), 1u);

    auto payload = rx.PushAM824Quadlet(AM824MidiCodec::EncodeBytes(bytes, 3));
    ASSERT_EQ(payload.count, 3);
    EXPECT_EQ(payload.bytes[0], 0xf0);
    EXPECT_EQ(payload.bytes[1], 0x7d);
    EXPECT_EQ(payload.bytes[2], 0x01);
    EXPECT_EQ(rx.ByteCount(), 3u);
}

TEST(MidiRxAssemblerTests, IgnoresNonMidiQuadlets) {
    MidiRxAssembler rx;
    const auto decoded = rx.PushAM824Quadlet(0x00000040u);

    EXPECT_EQ(decoded.count, 0);
    EXPECT_EQ(rx.NonMidiQuadlets(), 1u);
}
