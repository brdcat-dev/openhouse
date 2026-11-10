#include <gtest/gtest.h>

#include <array>
#include <cstdint>

#include "radio_mode.hpp"

using lh::usb::IsBluetoothReceiver;
using lh::usb::ParseRadioMode;
using lh::usb::RadioMode;
using lh::usb::RadioModeQuery;
using lh::usb::RadioModeReport;

TEST(RadioMode, OnlyTheReceiverWithBluetoothIsPicked) {
  // A Bigscreen Beyond's two built-in receivers: the serials decide, as in the stock driver.
  EXPECT_TRUE(IsBluetoothReceiver(0x2102, "081B7D738F-RYB"));
  EXPECT_FALSE(IsBluetoothReceiver(0x2102, "07D2B425E3-LYX"));
  // A plain Watchman dongle: wrong product, and no role suffix.
  EXPECT_FALSE(IsBluetoothReceiver(0x2101, "93920AB945"));
  // The suffix alone isn't enough.
  EXPECT_FALSE(IsBluetoothReceiver(0x2101, "081B7D738F-RYB"));
  // Serials that are too short, or end in B without the role suffix.
  EXPECT_FALSE(IsBluetoothReceiver(0x2102, "-RYB"));
  EXPECT_FALSE(IsBluetoothReceiver(0x2102, ""));
  EXPECT_FALSE(IsBluetoothReceiver(0x2102, "1234567B"));
}

TEST(RadioMode, SetReports) {
  const auto bluetooth = RadioModeReport(RadioMode::Bluetooth);
  EXPECT_EQ(bluetooth[0], 0xff);
  EXPECT_EQ(bluetooth[1], 0xb5);
  EXPECT_EQ(bluetooth[2], 0x01);
  EXPECT_EQ(bluetooth[3], 0x01);
  EXPECT_EQ(RadioModeReport(RadioMode::Valve)[3], 0x00);
  for (size_t i = 4; i < bluetooth.size(); ++i) EXPECT_EQ(bluetooth[i], 0x00);
}

TEST(RadioMode, QueryReport) {
  const auto query = RadioModeQuery();
  EXPECT_EQ(query[0], 0xff);
  EXPECT_EQ(query[1], 0xb6);
  for (size_t i = 2; i < query.size(); ++i) EXPECT_EQ(query[i], 0x00);
}

TEST(RadioMode, ParsesReplies) {
  const std::array<uint8_t, 4> valve{0xff, 0xb6, 0x01, 0x00}, bluetooth{0xff, 0xb6, 0x01, 0x01};
  EXPECT_EQ(ParseRadioMode(valve.data(), 4), RadioMode::Valve);
  EXPECT_EQ(ParseRadioMode(bluetooth.data(), 4), RadioMode::Bluetooth);
}

TEST(RadioMode, RejectsOtherReplies) {
  // What a 0x2101 dongle answers: no mode.
  const std::array<uint8_t, 4> unsupported{0xff, 0xb6, 0x00, 0x00};
  EXPECT_EQ(ParseRadioMode(unsupported.data(), 4), std::nullopt);
  // A reply to some other command.
  const std::array<uint8_t, 4> other{0xff, 0x83, 0x01, 0x01};
  EXPECT_EQ(ParseRadioMode(other.data(), 4), std::nullopt);
  // A mode this code doesn't know.
  const std::array<uint8_t, 4> unknown{0xff, 0xb6, 0x01, 0x07};
  EXPECT_EQ(ParseRadioMode(unknown.data(), 4), std::nullopt);
  // Too short, failed read (hidapi returns -1), or no buffer.
  const std::array<uint8_t, 4> fine{0xff, 0xb6, 0x01, 0x01};
  EXPECT_EQ(ParseRadioMode(fine.data(), 3), std::nullopt);
  EXPECT_EQ(ParseRadioMode(fine.data(), -1), std::nullopt);
  EXPECT_EQ(ParseRadioMode(nullptr, 4), std::nullopt);
}
