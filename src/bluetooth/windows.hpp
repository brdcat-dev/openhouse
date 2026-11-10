#pragma once
#include "batch.hpp"
#include "protocol.hpp"
#include <string>

namespace lh::bluetooth {
struct Station {
  std::string name;
  uint64_t address;
  int rssi;
};
// Call from an initialized COM multithreaded apartment.
bool PcAdapterAvailable();
struct DeviceInfo {
  FirmwareVersions versions{};
  std::string firmware, hardware, manufacturer, model, serial;
};
DeviceInfo ReadDeviceInfo(uint64_t address, std::chrono::milliseconds timeout);
std::vector<Station> Scan(std::chrono::milliseconds duration, std::span<const std::string> candidates);
std::vector<PowerResult> SetPowerBatch(std::span<const uint64_t> addresses, uint8_t mode,
                                      std::chrono::milliseconds timeout);
}  // namespace lh::bluetooth
