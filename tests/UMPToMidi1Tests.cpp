// UMPToMidi1Tests.cpp

#include "MIDI/UMPToMidi1.hpp"

#include <gtest/gtest.h>

using ASFW::MIDI::UMPToMidi1;

TEST(UMPToMidi1Tests, DecodesMidi10ChannelVoiceThreeByteMessage) {
    const auto decoded = UMPToMidi1::DecodeWord(0x2090407fu);

    ASSERT_EQ(decoded.count, 3);
    EXPECT_EQ(decoded.bytes[0], 0x90);
    EXPECT_EQ(decoded.bytes[1], 0x40);
    EXPECT_EQ(decoded.bytes[2], 0x7f);
}

TEST(UMPToMidi1Tests, DecodesMidi10ChannelVoiceTwoByteMessage) {
    const auto decoded = UMPToMidi1::DecodeWord(0x20c00500u);

    ASSERT_EQ(decoded.count, 2);
    EXPECT_EQ(decoded.bytes[0], 0xc0);
    EXPECT_EQ(decoded.bytes[1], 0x05);
}

TEST(UMPToMidi1Tests, DecodesSystemRealtimeMessage) {
    const auto decoded = UMPToMidi1::DecodeWord(0x10f80000u);

    ASSERT_EQ(decoded.count, 1);
    EXPECT_EQ(decoded.bytes[0], 0xf8);
}

TEST(UMPToMidi1Tests, DecodesSystemCommonMessage) {
    const auto decoded = UMPToMidi1::DecodeWord(0x10f23412u);

    ASSERT_EQ(decoded.count, 3);
    EXPECT_EQ(decoded.bytes[0], 0xf2);
    EXPECT_EQ(decoded.bytes[1], 0x34);
    EXPECT_EQ(decoded.bytes[2], 0x12);
}

TEST(UMPToMidi1Tests, IgnoresUnsupportedPacketTypes) {
    const auto decoded = UMPToMidi1::DecodeWord(0x40f80000u);

    EXPECT_EQ(decoded.count, 0);
}
