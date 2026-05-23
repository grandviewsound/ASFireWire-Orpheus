#include <gtest/gtest.h>

#include "Isoch/Receive/PacketStreamParser.hpp"

#include <array>
#include <cstring>
#include <vector>

using ASFW::Isoch::Rx::PacketStreamParser;

namespace {

// Build a synthetic IR-bufferFill packet matching Apple's processReceivedPackets
// expectations: [4B cycleTimestamp][4B isoch header (data_length:16 high)]
// followed by `payload` bytes, padded up to a quadlet.
std::vector<uint8_t> MakePacket(uint16_t dataLength, uint8_t fill = 0xAB) {
    const size_t padded = (dataLength + 3u) & ~size_t{3u};
    std::vector<uint8_t> out(8 + padded, fill);
    // cycleTimestamp at [0..3] — value doesn't matter for the parser.
    out[0] = 0x10;
    out[1] = 0x00;
    out[2] = 0x00;
    out[3] = 0x00;
    // isoch header at [4..7] — host-endian uint32_t with dataLength in bits 31-16.
    // OHCI byte-swaps wire BE → host LE; on LE host, low byte = bits 0-7,
    // high byte = bits 24-31. So dataLength (bits 31-16) lives at bytes 6-7.
    out[4] = 0x00;             // bits 7-0 = sy(4) | tcode(4)
    out[5] = 0x00;             // bits 15-8 = chan(6) | tag(2)
    out[6] = static_cast<uint8_t>(dataLength & 0xFFu);
    out[7] = static_cast<uint8_t>((dataLength >> 8) & 0xFFu);
    return out;
}

} // namespace

TEST(PacketStreamParserTests, EmptyChunk_NoEmission) {
    PacketStreamParser parser;
    int calls = 0;
    parser.OnBytes(nullptr, 0, [&](const uint8_t*, size_t) { ++calls; });
    EXPECT_EQ(calls, 0);
}

TEST(PacketStreamParserTests, SingleWholePacketInOneChunk) {
    PacketStreamParser parser;
    auto pkt = MakePacket(360);
    int calls = 0;
    size_t observedLen = 0;
    parser.OnBytes(pkt.data(), pkt.size(), [&](const uint8_t*, size_t len) {
        ++calls;
        observedLen = len;
    });
    EXPECT_EQ(calls, 1);
    EXPECT_EQ(observedLen, pkt.size());
    EXPECT_EQ(parser.EmittedPackets(), 1u);
    EXPECT_EQ(parser.PendingScratchBytes(), 0u);
}

TEST(PacketStreamParserTests, BackToBackPacketsInOneChunk) {
    PacketStreamParser parser;
    auto a = MakePacket(360, 0x11);
    auto b = MakePacket(120, 0x22);
    std::vector<uint8_t> chunk;
    chunk.insert(chunk.end(), a.begin(), a.end());
    chunk.insert(chunk.end(), b.begin(), b.end());
    int calls = 0;
    std::vector<size_t> lens;
    parser.OnBytes(chunk.data(), chunk.size(), [&](const uint8_t*, size_t len) {
        ++calls;
        lens.push_back(len);
    });
    ASSERT_EQ(calls, 2);
    EXPECT_EQ(lens[0], a.size());
    EXPECT_EQ(lens[1], b.size());
}

TEST(PacketStreamParserTests, PacketStraddlesTwoChunks) {
    // Apple's parser must reassemble a packet that spans two descriptor
    // boundaries. The straddle copy lands in the parser's scratch, not the
    // caller's buffer.
    PacketStreamParser parser;
    auto pkt = MakePacket(360, 0x55);
    const size_t splitAt = 100;
    const size_t firstLen = splitAt;
    const size_t secondLen = pkt.size() - splitAt;

    int calls = 0;
    size_t observedLen = 0;
    bool observedScratchPath = false;

    parser.OnBytes(pkt.data(), firstLen, [&](const uint8_t*, size_t) {
        ++calls; // should not fire yet
    });
    EXPECT_EQ(calls, 0);
    EXPECT_EQ(parser.PendingScratchBytes(), firstLen);

    parser.OnBytes(pkt.data() + firstLen, secondLen,
                   [&](const uint8_t* delivered, size_t len) {
        ++calls;
        observedLen = len;
        // Bytes 0-99 of the delivered packet must equal first chunk —
        // proves scratch reassembly worked.
        for (size_t i = 0; i < firstLen; ++i) {
            if (delivered[i] != pkt[i]) {
                observedScratchPath = false;
                return;
            }
        }
        for (size_t i = firstLen; i < pkt.size(); ++i) {
            if (delivered[i] != pkt[i]) {
                observedScratchPath = false;
                return;
            }
        }
        observedScratchPath = true;
    });
    EXPECT_EQ(calls, 1);
    EXPECT_EQ(observedLen, pkt.size());
    EXPECT_TRUE(observedScratchPath);
    EXPECT_EQ(parser.PendingScratchBytes(), 0u);
}

TEST(PacketStreamParserTests, PartialHeaderAcrossChunks) {
    PacketStreamParser parser;
    auto pkt = MakePacket(40);
    int calls = 0;
    size_t observedLen = 0;

    // Only 5 bytes of the 8-byte prefix — parser must wait.
    parser.OnBytes(pkt.data(), 5, [&](const uint8_t*, size_t) { ++calls; });
    EXPECT_EQ(calls, 0);
    EXPECT_EQ(parser.PendingScratchBytes(), 5u);

    // Remaining bytes complete the packet.
    parser.OnBytes(pkt.data() + 5, pkt.size() - 5,
                   [&](const uint8_t*, size_t len) {
        ++calls;
        observedLen = len;
    });
    EXPECT_EQ(calls, 1);
    EXPECT_EQ(observedLen, pkt.size());
}

TEST(PacketStreamParserTests, BogusOversizeHeaderDoesNotHang) {
    PacketStreamParser parser;
    // dataLength = 0xFFFF would make pktSize 8 + 0xFFFC = 65540 > kMaxPacketBytes.
    auto pkt = MakePacket(0xFFFF);
    int calls = 0;
    parser.OnBytes(pkt.data(), pkt.size(), [&](const uint8_t*, size_t) { ++calls; });
    EXPECT_EQ(calls, 0);
    EXPECT_GT(parser.DroppedPackets(), 0u);
}

TEST(PacketStreamParserTests, ResetClearsScratch) {
    PacketStreamParser parser;
    auto pkt = MakePacket(360);
    parser.OnBytes(pkt.data(), 50, [](const uint8_t*, size_t) {});
    EXPECT_GT(parser.PendingScratchBytes(), 0u);
    parser.Reset();
    EXPECT_EQ(parser.PendingScratchBytes(), 0u);
    EXPECT_EQ(parser.EmittedPackets(), 0u);
    EXPECT_EQ(parser.DroppedPackets(), 0u);
}
