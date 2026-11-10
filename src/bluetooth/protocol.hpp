#pragma once
#include <charconv>
#include <cstdint>
#include <stdexcept>
#include <string_view>
#include <string>
#include <set>
#include <vector>

namespace lh::bluetooth {
inline std::vector<std::string> ParseManagedStationNames(std::string_view list) {
  std::set<std::string> names;
  for (size_t start = 0; start < list.size();) {
    const auto end = list.find(',', start);
    const auto entry = list.substr(start, end == std::string_view::npos ? end : end - start);
    if (entry.size() == 18 && entry.starts_with("LHB-") && entry.ends_with(":BS2:1")) {
      const auto serial = entry.substr(4, 8);
      if (serial.find_first_not_of("0123456789ABCDEFabcdef") == std::string_view::npos) names.emplace(entry.substr(0, 12));
    }
    if (names.size() > 32) throw std::runtime_error("Too many managed Bluetooth stations");
    if (end == std::string_view::npos) break;
    start = end + 1;
  }
  return {names.begin(), names.end()};
}
inline int DecodePowerState(uint8_t value) {
  switch (value) {
    case 0: return 0;
    case 2: return 2;
    case 1: case 9: case 11: return 1;
    default: return -1;
  }
}
struct FirmwareVersions { uint32_t radio; uint32_t mcu; uint32_t bootloader; };
inline FirmwareVersions ParseFirmwareVersions(std::string_view text) {
  const auto version = [&](const char* label) {
    auto position = text.find(label);
    if (position == std::string_view::npos) throw std::runtime_error("Missing Bluetooth firmware component");
    position += 2;
    while (position < text.size() && text[position] == ' ') ++position;
    unsigned major = 0, minor = 0;
    const auto first = std::from_chars(text.data() + position, text.data() + text.size(), major);
    if (first.ec != std::errc{} || first.ptr == text.data() + text.size() || *first.ptr != '.')
      throw std::runtime_error("Invalid Bluetooth firmware version");
    const auto second = std::from_chars(first.ptr + 1, text.data() + text.size(), minor);
    if (second.ec != std::errc{} || major > 255 || minor > 255) throw std::runtime_error("Invalid Bluetooth firmware version");
    return major * 256 + minor;
  };
  const auto mcu = version("M:");
  return {version("R:"), ((mcu >> 8) + 2) * 1000 + (mcu & 255), version("B:")};
}
}  // namespace lh::bluetooth
