// The Bluetooth side of a headset's built-in receiver.
//
// A 0x2102 receiver's radio runs either Valve's wireless protocol or Bluetooth LE. SteamVR's
// base station power management (vrmonitor) drives Bluetooth through the receiver's serial
// port, but asks the headset's driver to switch the mode first (IVRDebug, device 0):
// "set_radio_mode_ble", "set_radio_mode_valve", "get_radio_mode". Those requests reach the
// stock driver through the headset's proxy, and it can't serve them here: it never sees the
// receivers (stock_hmd.hpp). So ours answers, or the stations neither wake nor sleep.
//
// Addresses are the stock Windows DLL (SteamVR 2.19.2), and the reports were checked on a
// Bigscreen Beyond's receivers. 0x2101 dongles answer the query with no mode.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace lh::usb {

enum class RadioMode : uint8_t { Valve = 0, Bluetooth = 1 };

// Stock 0x4a7a0: a 0x2102 receiver's serial ends in "-<side><?><kind>" (side L/R, kind M/B),
// and kind B is the one with Bluetooth, e.g. "081B7D738F-RYB" (not "07D2B425E3-LYX").
inline bool IsBluetoothReceiver(uint16_t pid, const std::string& serial) {
  return pid == 0x2102 && serial.size() > 4 && serial[serial.size() - 4] == '-' && serial.back() == 'B';
}

// Stock 0x4ec10: feature 0xff "b5 01 <mode>".
inline std::array<uint8_t, 64> RadioModeReport(RadioMode mode) {
  std::array<uint8_t, 64> report{};
  report[0] = 0xff;
  report[1] = 0xb5;
  report[2] = 0x01;
  report[3] = static_cast<uint8_t>(mode);
  return report;
}

// Stock 0x48040: feature 0xff "b6 00", then read the feature report back.
inline std::array<uint8_t, 64> RadioModeQuery() {
  std::array<uint8_t, 64> report{};
  report[0] = 0xff;
  report[1] = 0xb6;
  return report;
}

// The reply to RadioModeQuery, report id first: "ff b6 01 <mode>".
inline std::optional<RadioMode> ParseRadioMode(const uint8_t* reply, int size) {
  if (!reply || size < 4 || reply[0] != 0xff || reply[1] != 0xb6 || reply[2] != 0x01) return std::nullopt;
  if (reply[3] > static_cast<uint8_t>(RadioMode::Bluetooth)) return std::nullopt;
  return static_cast<RadioMode>(reply[3]);
}

}  // namespace lh::usb
