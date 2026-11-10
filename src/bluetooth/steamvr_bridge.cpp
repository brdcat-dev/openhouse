#include <windows.h>
#include <bcrypt.h>
#include <winrt/base.h>
#include <nlohmann/json.hpp>
#include "windows.hpp"
#include "forwards.hpp"

#include <array>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <set>

extern "C" IMAGE_DOS_HEADER __ImageBase;

namespace {
using namespace lh::bluetooth;
using namespace std::chrono_literals;
struct Result { uint32_t status; uint32_t detail; };
// MSVC x64 release string layout, checked against the supported native library.
struct NativeString { union { char text[16]; const char* pointer; }; size_t size; size_t capacity; };
struct NativeVector { const unsigned char* begin; const unsigned char* end; const unsigned char* capacity; };
static_assert(sizeof(NativeString) == 32 && sizeof(Result) == 8);
static_assert(sizeof(std::string) == sizeof(NativeString));
using PowerCall = Result* (*)(void*, Result*, const NativeString*, unsigned);
using ScanCall = Result* (*)(void*, Result*, const NativeString*, const void*, unsigned, int, const NativeVector*);
using ReadCall = Result* (*)(void*, Result*, unsigned, const NativeString*, uint64_t, unsigned, bool, void*);
using CallbackCall = void (*)(void*, void*);
std::array<void*, 14> methods;
PowerCall indexPower = nullptr;
ScanCall indexScan = nullptr;
ReadCall indexRead = nullptr;
HMODULE indexLibrary = nullptr;
CallbackCall indexCallbacks = nullptr;
void* callbacks = nullptr;
void* nativeInterface = nullptr;
std::mutex powerMutex;
std::map<std::string, uint64_t> addresses;
std::map<uint64_t, bool> pending;
// shortcut: no native batch ID; cache unconsumed results for 20s until SteamVR exposes one.
unsigned pendingMode = 3;
Clock::time_point pendingUntil;

std::filesystem::path ModuleDirectory() {
  std::array<wchar_t, 32768> path{};
  const auto count = GetModuleFileNameW(reinterpret_cast<HMODULE>(&__ImageBase), path.data(), DWORD(path.size()));
  if (!count || count == path.size()) throw std::runtime_error("Bluetooth module path unavailable");
  return std::filesystem::path(path.data()).parent_path();
}

void Log(const std::string& text, int level = 0) {
  if (!callbacks) return;
  auto table = *static_cast<void***>(callbacks);
  reinterpret_cast<void(*)(void*, int, const char*)>(table[0])(callbacks, level,
      ("openhouse Bluetooth: " + text + "\n").c_str());
}

template<class F> auto InApartment(F function) {
  return std::async(std::launch::async, [function] {
    winrt::init_apartment(winrt::apartment_type::multi_threaded);
    struct Uninitialize { ~Uninitialize() { winrt::uninit_apartment(); } } apartment;
    return function();
  }).get();
}

std::vector<std::string> ManagedNames() {
  auto runtime = ModuleDirectory().parent_path().parent_path();
  auto config = runtime.parent_path().parent_path().parent_path() / "config";
  std::array<wchar_t, 32768> local{};
  const auto count = GetEnvironmentVariableW(L"LOCALAPPDATA", local.data(), DWORD(local.size()));
  if (count && count < local.size()) {
    std::ifstream paths(std::filesystem::path(local.data()) / "openvr/openvrpaths.vrpath");
    const auto value = nlohmann::json::parse(paths, nullptr, false);
    if (value.is_object() && value.contains("config") && value["config"].is_array() &&
        !value["config"].empty() && value["config"][0].is_string())
      config = std::filesystem::path(winrt::to_hstring(value["config"][0].get<std::string>()).c_str());
  }
  std::ifstream file(config / "steamvr.vrsettings");
  const auto value = nlohmann::json::parse(file, nullptr, false);
  if (!value.is_object() || !value.contains("driver_lighthouse") ||
      !value["driver_lighthouse"].is_object()) return {};
  const auto& section = value["driver_lighthouse"];
  if (!section.contains("PowerManagedBaseStations2") || !section["PowerManagedBaseStations2"].is_string()) return {};
  return ParseManagedStationNames(section["PowerManagedBaseStations2"].get<std::string>());
}

uint64_t ParseAddress(const NativeString& address) {
  if (address.size != 12 || address.capacity < address.size) return 0;
  const char* data = address.capacity < 16 ? address.text : address.pointer;
  if (!data) return 0;
  uint64_t result = 0;
  for (unsigned i = 0; i < 12; ++i) {
    const char c = data[i];
    const int nibble = c >= '0' && c <= '9' ? c - '0' : c >= 'A' && c <= 'F' ? c - 'A' + 10 : c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1;
    if (nibble < 0) return 0;
    result = (result << 4) | unsigned(nibble);
  }
  return result;
}

Result* ScanStations(void* self, Result* result, const NativeString* prefix, const void* candidates,
                     unsigned flags, int seconds, const NativeVector* found) {
  auto* returned = indexScan(self, result, prefix, candidates, flags, seconds, found);
  // Native scan records are 0x90 bytes: name at 8, MAC string at 40.
  if (result->status == 0 && found && found->begin && found->end >= found->begin &&
      found->end - found->begin <= 32 * 0x90 && (found->end - found->begin) % 0x90 == 0) {
    std::lock_guard lock(powerMutex);
    for (auto* entry = found->begin; entry != found->end; entry += 0x90) {
      const auto& name = *reinterpret_cast<const NativeString*>(entry + 8);
      const auto mac = ParseAddress(*reinterpret_cast<const NativeString*>(entry + 40));
      if (mac && name.size == 12 && name.capacity < 16 && std::string_view(name.text, 12).starts_with("LHB-"))
        addresses[std::string(name.text, 12)] = mac;
    }
  }
  return returned;
}

Result* ReadProperties(void* self, Result* result, unsigned type, const NativeString* address,
                       uint64_t nonce, unsigned flags, bool force, void* output) {
  try {
    const auto mac = address ? ParseAddress(*address) : 0;
    if (type == 2 && mac && output && flags && !(flags & ~0x1ff0u) && InApartment([] { return PcAdapterAvailable(); })) {
      const auto info = InApartment([&] { return ReadDeviceInfo(mac, 5s); });
      const auto versions = info.versions;
      // Use the supported library's own JSON allocator and value operations across its ABI.
      const auto base = reinterpret_cast<unsigned char*>(indexLibrary);
      const auto number = reinterpret_cast<void*(*)(void*, unsigned)>(base + 0x60270);
      const auto text = reinterpret_cast<void*(*)(void*, const void*)>(base + 0x60210);
      const auto member = reinterpret_cast<void*(*)(void*, const char*)>(base + 0x609c0);
      const auto assign = reinterpret_cast<void*(*)(void*, void*)>(base + 0x60640);
      for (const auto [flag, value] : {std::pair{0x40u, versions.bootloader}, {0x80u, versions.mcu}, {0x100u, versions.radio}}) {
        if (!(flags & flag)) continue;
        alignas(8) std::array<uint64_t, 6> temporary{};
        number(temporary.data(), value);
        assign(member(output, std::to_string(flag).c_str()), temporary.data());
      }
      const auto macText = std::string(address->capacity < 16 ? address->text : address->pointer, address->size);
      for (const auto [flag, value] : {std::pair{0x10u, &macText}, {0x20u, &info.firmware}, {0x200u, &info.hardware},
                                      {0x400u, &info.manufacturer}, {0x800u, &info.model}, {0x1000u, &info.serial}}) {
        if (!(flags & flag)) continue;
        alignas(8) std::array<uint64_t, 6> temporary{};
        text(temporary.data(), value);
        assign(member(output, std::to_string(flag).c_str()), temporary.data());
      }
      Log("PC firmware versions: radio=" + std::to_string(versions.radio) + " mcu=" + std::to_string(versions.mcu) +
          " bootloader=" + std::to_string(versions.bootloader));
      *result = {};
      return result;
    }
    Log("Index property query: type=" + std::to_string(type) + " flags=" + std::to_string(flags));
  } catch (...) { Log("PC property read failed; using Index", 2); }
  return indexRead(self, result, type, address, nonce, flags, force, output);
}

void SetCallbacks(void* self, void* supplied) {
  callbacks = supplied;
  indexCallbacks(self, supplied);
}

Result* SetPowerState(void* self, Result* result, const NativeString* address, unsigned mode) {
  try {
    const auto current = address ? ParseAddress(*address) : 0;
    if (!current || mode > 2) return indexPower(self, result, address, mode);
    std::lock_guard lock(powerMutex);
    if (pendingMode != mode || Clock::now() >= pendingUntil) pending.clear();
    if (!pending.contains(current)) {
      pending.clear();
      if (!InApartment([] { return PcAdapterAvailable(); })) {
        Log("PC adapter unavailable; using Index controller");
        return indexPower(self, result, address, mode);
      }
      const auto names = ManagedNames();
      if (std::any_of(names.begin(), names.end(), [](const auto& name) { return !addresses.contains(name); })) {
        const auto found = InApartment([&] { return Scan(2s, names); });
        for (const auto& station : found) addresses[station.name] = station.address;
      }
      std::set<uint64_t> selected{current};
      bool currentManaged = false;
      for (const auto& name : names) {
        if (addresses.contains(name) && addresses.at(name) == current) currentManaged = true;
      }
      if (currentManaged) {
        for (const auto& name : names) {
          if (addresses.contains(name)) selected.insert(addresses.at(name));
          else Log("managed station not discovered: " + name, 2);
        }
      }
      const std::vector<uint64_t> targets(selected.begin(), selected.end());
      const auto start = Clock::now();
      Log("PC power batch requested: mode=" + std::to_string(mode) + " stations=" + std::to_string(targets.size()));
      const auto states = InApartment([&] { return SetPowerBatch(targets, uint8_t(mode), 9s); });
      for (const auto& state : states) {
        pending[state.address] = state.confirmed;
        char message[160];
        std::snprintf(message, sizeof message, "%012llX mode=%u acknowledged=%u confirmed=%u attempts=%u reported=%d",
                      static_cast<unsigned long long>(state.address), mode, state.acknowledged, state.confirmed, state.attempts, state.reported);
        Log(std::string(message) + (state.error.empty() ? "" : " error=" + state.error), state.confirmed ? 0 : 2);
      }
      pendingMode = mode;
      pendingUntil = Clock::now() + 20s;
      Log("PC batch finished in " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count()) + "ms");
    }
    const bool confirmed = pending.at(current);
    pending.erase(current);
    if (confirmed) { *result = {}; return result; }
    Log("PC state unconfirmed; retrying this station through Index", 2);
  } catch (const std::exception& error) {
    Log(std::string("PC operation failed; using Index: ") + error.what(), 2);
  } catch (...) { Log("PC operation failed; using Index", 2); }
  return indexPower(self, result, address, mode);
}

bool SupportedLibrary(const std::filesystem::path& path) {
  std::error_code error;
  const auto size = std::filesystem::file_size(path, error);
  if (error || !size || size > 32 * 1024 * 1024) return false;
  std::ifstream file(path, std::ios::binary);
  std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(file)), {});
  std::array<unsigned char, 32> digest{};
  if (bytes.empty() || bytes.size() > 32 * 1024 * 1024 ||
      BCryptHash(BCRYPT_SHA256_ALG_HANDLE, nullptr, 0, bytes.data(), ULONG(bytes.size()), digest.data(), ULONG(digest.size())) < 0) return false;
  std::string hex;
  for (const auto byte : digest) { hex += "0123456789abcdef"[byte >> 4]; hex += "0123456789abcdef"[byte & 15]; }
  return hex == "55b09bdb27e475a10eebf3253b1882380db04b5d6e003344b068d5f189f62e72" ||
         hex == "ab62269c901e92d786462c61ba83b769470d3af05ad084a27852418e29214f94";
}
}  // namespace

extern "C" __declspec(dllexport) void* NRFSerLib_GetBaseStationWirelessInterface() {
  static std::once_flag initialized;
  try {
    std::call_once(initialized, [] {
      const auto path = ModuleDirectory() / "nrfserlib_index.dll";
      const auto library = LoadLibraryExW(path.c_str(), nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);
      indexLibrary = library;
      if (!library) throw std::runtime_error("Original Nordic library unavailable");
      const auto factory = reinterpret_cast<void*(*)()>(GetProcAddress(library, "NRFSerLib_GetBaseStationWirelessInterface"));
      if (!factory) throw std::runtime_error("Original Nordic interface unavailable");
      nativeInterface = factory();
      if (!nativeInterface || !SupportedLibrary(path)) return;
      auto& table = *static_cast<void***>(nativeInterface);
      std::copy_n(table, methods.size(), methods.begin());
      indexCallbacks = reinterpret_cast<CallbackCall>(methods[0]);
      indexScan = reinterpret_cast<ScanCall>(methods[5]);
      indexRead = reinterpret_cast<ReadCall>(methods[6]);
      indexPower = reinterpret_cast<PowerCall>(methods[10]);
      methods[0] = reinterpret_cast<void*>(&SetCallbacks);
      methods[5] = reinterpret_cast<void*>(&ScanStations);
      methods[6] = reinterpret_cast<void*>(&ReadProperties);
      methods[10] = reinterpret_cast<void*>(&SetPowerState);
      table = methods.data();
    });
  } catch (...) { return nullptr; }
  return nativeInterface;
}
