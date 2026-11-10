#include "windows.hpp"

#include <windows.h>
#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Devices.Bluetooth.h>
#include <winrt/Windows.Devices.Bluetooth.Advertisement.h>
#include <winrt/Windows.Devices.Bluetooth.GenericAttributeProfile.h>
#include <winrt/Windows.Devices.Radios.h>
#include <winrt/Windows.Storage.Streams.h>
#include <algorithm>
#include <condition_variable>
#include <cctype>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>

namespace lh::bluetooth {
namespace {
using namespace winrt::Windows::Devices::Bluetooth;
using namespace winrt::Windows::Devices::Bluetooth::GenericAttributeProfile;
using namespace winrt::Windows::Devices::Bluetooth::Advertisement;

template<class Operation> auto Await(Operation operation, Clock::time_point deadline) {
  const auto remaining = std::chrono::duration_cast<winrt::Windows::Foundation::TimeSpan>(deadline - Clock::now());
  if (remaining.count() <= 0 ||
      operation.wait_for(remaining) != winrt::Windows::Foundation::AsyncStatus::Completed) {
    operation.Cancel();
    throw winrt::hresult_error(HRESULT_FROM_WIN32(ERROR_TIMEOUT));
  }
  return operation.GetResults();
}

struct Connection {
  BluetoothLEDevice device{nullptr};
  GattDeviceService service{nullptr};
  GattSession session{nullptr};
  GattCharacteristic power{nullptr};
  ~Connection() {
    power = nullptr;
    try { if (session) session.MaintainConnection(false); } catch (...) {}
    try { if (service) service.Close(); } catch (...) {}
    service = nullptr;
    try { if (session) session.Close(); } catch (...) {}
    session = nullptr;
    try { if (device) device.Close(); } catch (...) {}
  }
};

std::unique_ptr<Connection> Connect(uint64_t address, Clock::time_point deadline,
                                    winrt::guid serviceId, winrt::guid characteristicId) {
  auto result = std::make_unique<Connection>();
  result->device = Await(BluetoothLEDevice::FromBluetoothAddressAsync(address, BluetoothAddressType::Random), deadline);
  if (!result->device) throw std::runtime_error("Bluetooth device unavailable");
  const auto services = Await(result->device.GetGattServicesForUuidAsync(
      serviceId, BluetoothCacheMode::Uncached), deadline);
  if (services.Status() != GattCommunicationStatus::Success || services.Services().Size() != 1)
    throw std::runtime_error("Bluetooth service discovery status=" + std::to_string(int(services.Status())));
  result->service = services.Services().GetAt(0);
  result->session = result->service.Session();
  result->session.MaintainConnection(true);
  const auto characteristics = Await(result->service.GetCharacteristicsForUuidAsync(
      characteristicId, BluetoothCacheMode::Uncached), deadline);
  if (characteristics.Status() != GattCommunicationStatus::Success || characteristics.Characteristics().Size() != 1)
    throw std::runtime_error("Bluetooth characteristic discovery status=" + std::to_string(int(characteristics.Status())));
  result->power = characteristics.Characteristics().GetAt(0);
  return result;
}
}  // namespace

bool PcAdapterAvailable() {
  try {
    const auto deadline = Clock::now() + std::chrono::seconds(2);
    const auto adapter = Await(BluetoothAdapter::GetDefaultAsync(), deadline);
    return adapter && adapter.IsLowEnergySupported() &&
           Await(adapter.GetRadioAsync(), deadline).State() == winrt::Windows::Devices::Radios::RadioState::On;
  } catch (...) { return false; }
}

DeviceInfo ReadDeviceInfo(uint64_t address, std::chrono::milliseconds timeout) {
  const auto deadline = Clock::now() + timeout;
  auto connection = Connect(address, deadline, winrt::guid(L"0000180a-0000-1000-8000-00805f9b34fb"),
                            winrt::guid(L"00002a26-0000-1000-8000-00805f9b34fb"));
  const auto read = [&](const GattCharacteristic& characteristic) {
    const auto result = Await(characteristic.ReadValueAsync(BluetoothCacheMode::Uncached), deadline);
    if (result.Status() != GattCommunicationStatus::Success || result.Value().Length() > 256)
      throw std::runtime_error("Bluetooth device information unavailable");
    std::string text(result.Value().Length(), '\0');
    winrt::Windows::Storage::Streams::DataReader::FromBuffer(result.Value()).ReadBytes(
        winrt::array_view<uint8_t>(reinterpret_cast<uint8_t*>(text.data()), reinterpret_cast<uint8_t*>(text.data() + text.size())));
    return text;
  };
  DeviceInfo info;
  info.firmware = read(connection->power);
  info.versions = ParseFirmwareVersions(info.firmware);
  const auto characteristics = Await(connection->service.GetCharacteristicsAsync(BluetoothCacheMode::Uncached), deadline);
  if (characteristics.Status() != GattCommunicationStatus::Success) throw std::runtime_error("Bluetooth device information unavailable");
  for (const auto [id, destination] : {std::pair{0x2a27u, &info.hardware}, {0x2a29u, &info.manufacturer},
                                     {0x2a24u, &info.model}, {0x2a25u, &info.serial}}) {
    bool found = false;
    for (const auto& characteristic : characteristics.Characteristics()) {
      if (characteristic.Uuid() == BluetoothUuidHelper::FromShortId(id)) {
        *destination = read(characteristic);
        found = true;
        break;
      }
    }
    if (!found && id != 0x2a25) throw std::runtime_error("Bluetooth device information incomplete");
  }
  return info;
}

std::vector<Station> Scan(std::chrono::milliseconds duration, std::span<const std::string> candidates) {
  struct State {
    std::mutex mutex;
    std::condition_variable changed;
    std::map<uint64_t, Station> stations;
    bool failed = false;
  };
  auto state = std::make_shared<State>();
  BluetoothLEAdvertisementWatcher watcher;
  watcher.ScanningMode(BluetoothLEScanningMode::Active);
  watcher.Received([state](const auto&, const BluetoothLEAdvertisementReceivedEventArgs& event) {
    const auto name = winrt::to_string(event.Advertisement().LocalName());
    if (name.size() != 12 || !name.starts_with("LHB-") ||
        !std::all_of(name.begin() + 4, name.end(), [](unsigned char c) { return std::isxdigit(c); })) return;
    std::lock_guard lock(state->mutex);
    state->stations[event.BluetoothAddress()] = {name, event.BluetoothAddress(), event.RawSignalStrengthInDBm()};
    state->changed.notify_all();
  });
  watcher.Stopped([state](const auto&, const BluetoothLEAdvertisementWatcherStoppedEventArgs& event) {
    std::lock_guard lock(state->mutex);
    state->failed = event.Error() != BluetoothError::Success;
    state->changed.notify_all();
  });
  const auto deadline = Clock::now() + duration;
  watcher.Start();
  std::unique_lock lock(state->mutex);
  state->changed.wait_until(lock, deadline, [&] {
    if (state->failed) return true;
    return !candidates.empty() && std::all_of(candidates.begin(), candidates.end(), [&](const std::string& candidate) {
      return std::any_of(state->stations.begin(), state->stations.end(), [&](const auto& station) {
        return station.second.name == candidate;
      });
    });
  });
  std::vector<Station> result;
  for (const auto& [address, station] : state->stations) result.push_back(station);
  const bool failed = state->failed;
  lock.unlock();
  watcher.Stop();
  if (failed) throw std::runtime_error("Bluetooth scanning failed");
  return result;
}

std::vector<PowerResult> SetPowerBatch(std::span<const uint64_t> addresses, uint8_t mode,
                                      std::chrono::milliseconds timeout) {
  if (mode > 2 || addresses.size() > 32) throw std::invalid_argument("Invalid Bluetooth power batch");
  std::map<uint64_t, std::unique_ptr<Connection>> connections;
  for (const auto address : addresses) {
    if (!address || address >> 48 || !connections.emplace(address, nullptr).second) throw std::invalid_argument("Invalid Bluetooth address");
  }
  const auto deadline = Clock::now() + timeout;
  auto states = SetPower(addresses, mode, deadline,
      [&](uint64_t address, uint8_t value, Clock::time_point limit) {
        winrt::init_apartment(winrt::apartment_type::multi_threaded);
        struct Uninitialize { ~Uninitialize() { winrt::uninit_apartment(); } } apartment;
        auto connection = Connect(address, limit, winrt::guid(L"00001523-1212-efde-1523-785feabcd124"),
                                  winrt::guid(L"00001525-1212-efde-1523-785feabcd124"));
        if (value == 0) {
          const auto state = Await(connection->power.ReadValueAsync(BluetoothCacheMode::Uncached), limit);
          if (state.Status() == GattCommunicationStatus::Success && state.Value().Length() == 1 &&
              DecodePowerState(winrt::Windows::Storage::Streams::DataReader::FromBuffer(state.Value()).ReadByte()) == 2) {
            // The station firmware requires wake before switching from standby to sleep.
            winrt::Windows::Storage::Streams::DataWriter wake;
            wake.WriteByte(1);
            if (Await(connection->power.WriteValueAsync(wake.DetachBuffer(), GattWriteOption::WriteWithResponse), limit) !=
                GattCommunicationStatus::Success) return false;
            bool awake = false;
            while (Clock::now() < limit) {
              const auto status = Await(connection->power.ReadValueAsync(BluetoothCacheMode::Uncached), limit);
              if (status.Status() == GattCommunicationStatus::Success && status.Value().Length() == 1 &&
                  DecodePowerState(winrt::Windows::Storage::Streams::DataReader::FromBuffer(status.Value()).ReadByte()) == 1) {
                awake = true;
                break;
              }
              std::this_thread::sleep_until(std::min(limit, Clock::now() + std::chrono::milliseconds(100)));
            }
            if (!awake) return false;
          }
        }
        winrt::Windows::Storage::Streams::DataWriter writer;
        writer.WriteByte(value);
        const auto status = Await(connection->power.WriteValueAsync(writer.DetachBuffer(), GattWriteOption::WriteWithResponse), limit);
        connections.at(address) = std::move(connection);
        return status == GattCommunicationStatus::Success;
      },
      [&](uint64_t address, Clock::time_point limit) {
        winrt::init_apartment(winrt::apartment_type::multi_threaded);
        struct Uninitialize { ~Uninitialize() { winrt::uninit_apartment(); } } apartment;
        const auto result = Await(connections.at(address)->power.ReadValueAsync(BluetoothCacheMode::Uncached), limit);
        if (result.Status() != GattCommunicationStatus::Success || result.Value().Length() != 1) return -1;
        return int(winrt::Windows::Storage::Streams::DataReader::FromBuffer(result.Value()).ReadByte());
      });
  std::vector<std::future<void>> closing;
  for (auto& [address, connection] : connections) {
    if (!connection) continue;
    closing.push_back(std::async(std::launch::async, [owned = std::move(connection)]() mutable {
      winrt::init_apartment(winrt::apartment_type::multi_threaded);
      owned.reset();
      winrt::uninit_apartment();
    }));
  }
  for (auto& operation : closing) operation.get();
  return states;
}
}  // namespace lh::bluetooth
