#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <future>
#include <span>
#include <thread>
#include <vector>
#include <string>
#include "protocol.hpp"

namespace lh::bluetooth {
using Clock = std::chrono::steady_clock;
struct PowerResult {
  uint64_t address;
  bool acknowledged = false;
  bool confirmed = false;
  unsigned attempts = 0;
  int reported = -1;
  std::string error;
};

// Finish sending the whole batch before polling any station's reported mode.
template<class Write, class Read>
std::vector<PowerResult> SetPower(std::span<const uint64_t> addresses, uint8_t mode,
                                  Clock::time_point deadline, Write write, Read read) {
  std::vector<std::future<PowerResult>> pending;
  const auto start = Clock::now();
  // A missing write must leave time to verify the responsive stations.
  const auto writeDeadline = start + (deadline - start) * 2 / 3;
  for (const auto address : addresses) {
    pending.push_back(std::async(std::launch::async, [=] {
      PowerResult result{address};
      for (unsigned attempt = 0; attempt < 3 && Clock::now() < writeDeadline; ++attempt) {
        ++result.attempts;
        try { result.acknowledged = write(address, mode, writeDeadline); }
        catch (const std::exception& error) { result.error = error.what(); }
        catch (...) { result.error = "Bluetooth write failed"; }
        if (result.acknowledged) break;
        if (attempt < 2) std::this_thread::sleep_until(std::min(writeDeadline, Clock::now() + std::chrono::milliseconds(100)));
      }
      return result;
    }));
  }
  std::vector<PowerResult> results;
  for (auto& operation : pending) results.push_back(operation.get());
  pending.clear();
  for (auto result : results) {
    pending.push_back(std::async(std::launch::async, [=]() mutable {
      while (result.acknowledged && Clock::now() < deadline) {
        try {
          result.reported = read(result.address, deadline);
          if (result.reported >= 0 && result.reported <= 255 && DecodePowerState(uint8_t(result.reported)) == mode) {
            result.confirmed = true;
            result.error.clear();
            break;
          }
        } catch (const std::exception& error) { result.error = error.what(); }
        catch (...) { result.error = "Bluetooth read failed"; }
        std::this_thread::sleep_until(std::min(deadline, Clock::now() + std::chrono::milliseconds(100)));
      }
      return result;
    }));
  }
  results.clear();
  for (auto& operation : pending) results.push_back(operation.get());
  return results;
}
}  // namespace lh::bluetooth
