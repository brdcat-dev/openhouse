#include "bluetooth/batch.hpp"
#include "bluetooth/protocol.hpp"
#include <gtest/gtest.h>
#include <array>
#include <atomic>
#include <condition_variable>
#include <mutex>

using namespace std::chrono_literals;
using namespace lh::bluetooth;

TEST(BluetoothBatch, UsesOnlyEnabledManagedStationsAndDeduplicatesThem) {
  const auto names = ParseManagedStationNames("LHB-87660BC4:BS2:1,LHB-2E59B706:BS2:0,broken,"
      "LHB-87660BC4:BS2:1,LHB-GGGGGGGG:BS2:1,LHB-1055D06E:BS2:1");
  EXPECT_EQ(names, (std::vector<std::string>{"LHB-1055D06E", "LHB-87660BC4"}));
  EXPECT_TRUE(ParseManagedStationNames("").empty());
}

TEST(BluetoothBatch, ParsesDeviceVersionComponentsAndRejectsMalformedValues) {
  const auto version = ParseFirmwareVersions("R: 2.9, M: 1.8, B: 3.4");
  EXPECT_EQ(version.radio, 521);
  EXPECT_EQ(version.mcu, 3008);
  EXPECT_EQ(version.bootloader, 772);
  EXPECT_THROW(ParseFirmwareVersions("R: 2.9, M: 1.8"), std::runtime_error);
  EXPECT_THROW(ParseFirmwareVersions("R: 2.9, M: 1., B: 3.4"), std::runtime_error);
  EXPECT_THROW(ParseFirmwareVersions("R: 2.256, M: 1.8, B: 3.4"), std::runtime_error);
}

TEST(BluetoothBatch, RecognizesReportedModesWithoutAcceptingUnknownValues) {
  EXPECT_EQ(DecodePowerState(0), 0);
  EXPECT_EQ(DecodePowerState(2), 2);
  for (const auto awake : {1, 9, 11}) EXPECT_EQ(DecodePowerState(uint8_t(awake)), 1);
  for (const auto unknown : {3, 8, 10, 255}) EXPECT_EQ(DecodePowerState(uint8_t(unknown)), -1);
}

TEST(BluetoothBatch, SendsAllRequestsBeforePollingAndWaitsForEveryState) {
  const std::array<uint64_t, 4> addresses{1, 2, 3, 4};
  std::mutex mutex;
  std::condition_variable changed;
  unsigned entered = 0, completed = 0;
  std::array<std::atomic<unsigned>, 4> reads{};
  const auto results = SetPower(addresses, 0, Clock::now() + 2s,
      [&](uint64_t, uint8_t mode, Clock::time_point) {
        EXPECT_EQ(mode, 0);
        std::unique_lock lock(mutex);
        ++entered;
        changed.notify_all();
        const bool concurrent = changed.wait_for(lock, 1s, [&] { return entered == 4; });
        ++completed;
        return concurrent;
      },
      [&](uint64_t address, Clock::time_point) {
        { std::lock_guard lock(mutex); EXPECT_EQ(completed, 4); }
        return ++reads[address - 1] >= address ? 0 : 1;
      });
  ASSERT_EQ(results.size(), 4);
  for (size_t i = 0; i < results.size(); ++i) {
    EXPECT_EQ(results[i].address, addresses[i]);
    EXPECT_TRUE(results[i].acknowledged);
    EXPECT_TRUE(results[i].confirmed);
    EXPECT_GE(reads[i].load(), addresses[i]);
  }
}

TEST(BluetoothBatch, MissingStationDoesNotPreventConfirmingOthers) {
  const std::array<uint64_t, 3> addresses{1, 2, 3};
  const auto results = SetPower(addresses, 2, Clock::now() + 500ms,
      [](uint64_t address, uint8_t, Clock::time_point limit) {
        if (address == 2) std::this_thread::sleep_until(limit);
        return address != 2;
      },
      [](uint64_t address, Clock::time_point) { return address == 3 ? -1 : 2; });
  ASSERT_EQ(results.size(), 3);
  EXPECT_TRUE(results[0].confirmed);
  EXPECT_FALSE(results[1].acknowledged);
  EXPECT_FALSE(results[1].confirmed);
  EXPECT_TRUE(results[2].acknowledged);
  EXPECT_FALSE(results[2].confirmed);
}

TEST(BluetoothBatch, RetriesOnlyFailedWritesWithinTheSharedBudget) {
  const std::array<uint64_t, 2> addresses{1, 2};
  std::array<std::atomic<unsigned>, 2> attempts{};
  const auto results = SetPower(addresses, 0, Clock::now() + 1s,
      [&](uint64_t address, uint8_t, Clock::time_point) {
        const auto attempt = ++attempts[address - 1];
        return address == 1 || attempt > 1;
      }, [](uint64_t, Clock::time_point) { return 0; });
  EXPECT_TRUE(results[0].confirmed);
  EXPECT_TRUE(results[1].confirmed);
  EXPECT_EQ(attempts[0], 1);
  EXPECT_EQ(attempts[1], 2);
}
