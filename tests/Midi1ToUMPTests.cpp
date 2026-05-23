// Midi1ToUMPTests.cpp

#include "MIDI/Midi1ToUMP.hpp"

#include <gtest/gtest.h>

using ASFW::MIDI::Midi1ToUMP;

TEST(Midi1ToUMPTests, EncodesChannelVoiceThreeByteMessage) {
    const uint8_t bytes[] = {0x90, 0x40, 0x7f};
    const auto encoded = Midi1ToUMP::EncodeBytes(bytes, sizeof(bytes));

    ASSERT_EQ(encoded.count, 1);
    EXPECT_EQ(encoded.words[0], 0x2090407fu);
}

TEST(Midi1ToUMPTests, EncodesChannelVoiceTwoByteMessage) {
    const uint8_t bytes[] = {0xc0, 0x05};
    const auto encoded = Midi1ToUMP::EncodeBytes(bytes, sizeof(bytes));

    ASSERT_EQ(encoded.count, 1);
    EXPECT_EQ(encoded.words[0], 0x20c00500u);
}

TEST(Midi1ToUMPTests, EncodesSystemRealtimeMessage) {
    const uint8_t bytes[] = {0xf8};
    const auto encoded = Midi1ToUMP::EncodeBytes(bytes, sizeof(bytes));

    ASSERT_EQ(encoded.count, 1);
    EXPECT_EQ(encoded.words[0], 0x10f80000u);
}

TEST(Midi1ToUMPTests, EncodesSystemCommonMessage) {
    const uint8_t bytes[] = {0xf2, 0x34, 0x12};
    const auto encoded = Midi1ToUMP::EncodeBytes(bytes, sizeof(bytes));

    ASSERT_EQ(encoded.count, 1);
    EXPECT_EQ(encoded.words[0], 0x10f23412u);
}

TEST(Midi1ToUMPTests, RejectsIncompleteMessage) {
    const uint8_t bytes[] = {0x90, 0x40};
    const auto encoded = Midi1ToUMP::EncodeBytes(bytes, sizeof(bytes));

    EXPECT_EQ(encoded.count, 0);
}
