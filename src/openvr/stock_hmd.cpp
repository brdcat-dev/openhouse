#include "stock_hmd.hpp"

#include "pairing_mailbox.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cwchar>
#include <filesystem>
#include <fstream>
#include <iterator>

#include <nlohmann/json.hpp>

#include <hidapi/hidapi.h>

#include <cstring>

#ifdef _WIN32
#include <windows.h>
#include <setupapi.h>
#include <devpkey.h>
extern "C" {
#include <hidsdi.h>
}
#else
#include <dlfcn.h>
#include <elf.h>
#include <link.h>
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace lh::openvr {

using namespace vr;

namespace {
std::mutex g_openMutex;
std::set<std::string> g_allowedHeadsets;  // wired 0x2300 serials, upper case
std::set<std::string> g_deniedLogged;
std::function<void(const std::string&)> g_openLog;

std::string Upper(std::string s) {
  for (auto& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
  return s;
}

// Logs once per device that the stock driver doesn't get to see.
void LogHidden(const std::string& device) {
  std::function<void(const std::string&)> log;
  {
    std::lock_guard lock(g_openMutex);
    if (g_deniedLogged.insert(device).second) log = g_openLog;
  }
  if (log) log("stock driver: hiding " + device + " from it (not the headset)");
}
}  // namespace

#ifndef _WIN32
namespace {

// /dev/hidrawN -> whether the stock driver may open it.
bool StockMayOpen(const char* path) {
  if (!path || std::strncmp(path, "/dev/hidraw", 11) != 0) return true;
  std::ifstream uevent(std::string("/sys/class/hidraw/") + (path + 5) + "/device/uevent");
  unsigned bus = 0, vid = 0, pid = 0;
  std::string uniq, line;
  while (std::getline(uevent, line)) {
    if (line.rfind("HID_ID=", 0) == 0) std::sscanf(line.c_str() + 7, "%x:%x:%x", &bus, &vid, &pid);
    else if (line.rfind("HID_UNIQ=", 0) == 0) uniq = Upper(line.substr(9));
  }
  if (vid != 0x28de) return true;
  if (pid == 0x2101 || pid == 0x2102) return false;  // dongles: ours only
  if (pid != 0x2300) return true;
  std::lock_guard lock(g_openMutex);
  return g_allowedHeadsets.count(uniq) > 0 || g_allowedHeadsets.count("*") > 0;
}

// The stock driver finds devices only by enumerating udev (no hotplug monitor; it
// rescans), so hiding entries there keeps it away from them. Refusing its open() of a
// device it already found is not an option: it crashes on the failure.
struct udev_list_entry;
using GetNext = udev_list_entry* (*)(udev_list_entry*);
using GetName = const char* (*)(udev_list_entry*);
using GetFirst = udev_list_entry* (*)(void*);
GetNext g_realNext = nullptr;
GetFirst g_realFirst = nullptr;
GetName g_name = nullptr;

bool Hidden(udev_list_entry* entry) {
  const char* syspath = g_name ? g_name(entry) : nullptr;
  if (!syspath) return false;
  const char* slash = std::strrchr(syspath, '/');
  const std::string node = slash ? slash + 1 : syspath;
  if (node.rfind("hidraw", 0) != 0) return false;
  const std::string dev = "/dev/" + node;
  if (StockMayOpen(dev.c_str())) return false;
  LogHidden(dev);
  return true;
}

udev_list_entry* SkipHidden(udev_list_entry* entry) {
  while (entry && Hidden(entry)) entry = g_realNext(entry);
  return entry;
}
udev_list_entry* StockNext(udev_list_entry* entry) { return SkipHidden(g_realNext(entry)); }
udev_list_entry* StockFirst(void* enumerate) { return SkipHidden(g_realFirst(enumerate)); }

// Points the library's import `name` (its GOT slot; RTLD_NOW has bound it) at `hook`;
// returns the original target, or nullptr.
void* HookImport(void* library, const char* name, void* hook) {
  link_map* map = nullptr;
  if (dlinfo(library, RTLD_DI_LINKMAP, &map) != 0 || !map) return nullptr;
  const auto base = static_cast<uintptr_t>(map->l_addr);
  auto at = [base](uintptr_t p) { return p < base ? p + base : p; };  // ld.so may have relocated it
  const ElfW(Rela)* rela = nullptr;
  size_t relaSize = 0;
  const ElfW(Sym)* symbols = nullptr;
  const char* strings = nullptr;
  for (const ElfW(Dyn)* d = map->l_ld; d->d_tag != DT_NULL; ++d) {
    if (d->d_tag == DT_JMPREL) rela = reinterpret_cast<const ElfW(Rela)*>(at(d->d_un.d_ptr));
    else if (d->d_tag == DT_PLTRELSZ) relaSize = d->d_un.d_val;
    else if (d->d_tag == DT_SYMTAB) symbols = reinterpret_cast<const ElfW(Sym)*>(at(d->d_un.d_ptr));
    else if (d->d_tag == DT_STRTAB) strings = reinterpret_cast<const char*>(at(d->d_un.d_ptr));
  }
  if (!rela || !symbols || !strings) return nullptr;
  for (size_t i = 0; i < relaSize / sizeof(ElfW(Rela)); ++i) {
    const auto& r = rela[i];
    if (ELF64_R_TYPE(r.r_info) != R_X86_64_JUMP_SLOT) continue;
    if (std::strcmp(strings + symbols[ELF64_R_SYM(r.r_info)].st_name, name) != 0) continue;
    auto* slot = reinterpret_cast<void**>(base + r.r_offset);
    const long page = sysconf(_SC_PAGESIZE);
    const auto pageStart = reinterpret_cast<uintptr_t>(slot) & ~uintptr_t(page - 1);
    // The page's protection as mapped: RELRO leaves it read-only, but the same page
    // can hold writable data too (the stock driver's .data starts on it).
    int prot = PROT_READ;
    {
      std::ifstream maps("/proc/self/maps");
      std::string line;
      while (std::getline(maps, line)) {
        unsigned long lo = 0, hi = 0;
        char perms[5] = {};
        if (std::sscanf(line.c_str(), "%lx-%lx %4s", &lo, &hi, perms) != 3 || pageStart < lo || pageStart >= hi) continue;
        prot = (perms[0] == 'r' ? PROT_READ : 0) | (perms[1] == 'w' ? PROT_WRITE : 0) | (perms[2] == 'x' ? PROT_EXEC : 0);
        break;
      }
    }
    void* start = reinterpret_cast<void*>(pageStart);
    if (!(prot & PROT_WRITE) && mprotect(start, size_t(page), prot | PROT_WRITE) != 0) return nullptr;
    void* original = *slot;
    *slot = hook;
    if (!(prot & PROT_WRITE)) mprotect(start, size_t(page), prot);
    return original;
  }
  return nullptr;
}

// Hides every lighthouse device but the headset from the stock driver's device scans.
bool HideOtherDevices(void* library) {
  g_name = reinterpret_cast<GetName>(dlsym(RTLD_DEFAULT, "udev_list_entry_get_name"));
  if (!g_name) return false;
  void* next = HookImport(library, "udev_list_entry_get_next", reinterpret_cast<void*>(&StockNext));
  if (!next) return false;
  g_realNext = reinterpret_cast<GetNext>(next);
  void* first = HookImport(library, "udev_enumerate_get_list_entry", reinterpret_cast<void*>(&StockFirst));
  if (!first) return false;
  g_realFirst = reinterpret_cast<GetFirst>(first);
  return true;
}
}  // namespace
#else
namespace {
// Windows: the stock DLL lists HID devices with SetupDiEnumDeviceInterfaces /
// SetupDiGetDeviceInterfaceDetailA (paths "\\?\hid#vid_28de&pid_2101&...") and opens
// them with CreateFileA. Its enumeration skips every lighthouse device but the headset,
// as on Linux (refusing the open instead crashes the Linux build of it).
using EnumInterfaces = BOOL(WINAPI*)(HDEVINFO, PSP_DEVINFO_DATA, const GUID*, DWORD, PSP_DEVICE_INTERFACE_DATA);
using InterfaceDetailA = BOOL(WINAPI*)(HDEVINFO, PSP_DEVICE_INTERFACE_DATA, PSP_DEVICE_INTERFACE_DETAIL_DATA_A, DWORD,
                                       PDWORD, PSP_DEVINFO_DATA);
EnumInterfaces g_realEnum = nullptr;
InterfaceDetailA g_realDetail = nullptr;
std::vector<GUID> g_headsetContainers;

bool ContainerOf(HDEVINFO set, SP_DEVINFO_DATA* device, GUID* container) {
  DEVPROPTYPE type = 0;
  const GUID none{};
  return SetupDiGetDevicePropertyW(set, device, &DEVPKEY_Device_ContainerId, &type,
                                 reinterpret_cast<BYTE*>(container), sizeof(*container), nullptr, 0) &&
         type == DEVPROP_TYPE_GUID && !IsEqualGUID(*container, none);
}

unsigned HexAfter(const std::string& path, const char* key) {
  const auto at = path.find(key);
  return at == std::string::npos ? 0 : unsigned(std::strtoul(path.c_str() + at + std::strlen(key), nullptr, 16));
}

// The device's USB serial ("LHR-0053AF37"), opened without access rights.
std::string SerialOf(const char* path) {
  HANDLE h = CreateFileA(path, 0, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
  if (h == INVALID_HANDLE_VALUE) return {};
  wchar_t serial[128] = {};
  const BOOL ok = HidD_GetSerialNumberString(h, serial, sizeof serial);
  CloseHandle(h);
  if (!ok) return {};
  char out[128];
  std::snprintf(out, sizeof out, "%ls", serial);
  return Upper(out);
}

void FindHeadsetContainers() {
  g_headsetContainers.clear();
  const HDEVINFO set = SetupDiGetClassDevsA(nullptr, "USB", nullptr, DIGCF_PRESENT | DIGCF_ALLCLASSES);
  if (set == INVALID_HANDLE_VALUE) return;
  SP_DEVINFO_DATA device{};
  device.cbSize = sizeof device;
  for (DWORD i = 0; SetupDiEnumDeviceInfo(set, i, &device); ++i) {
    char instance[512]{};
    if (!SetupDiGetDeviceInstanceIdA(set, &device, instance, sizeof instance, nullptr)) continue;
    const std::string path = Upper(instance);
    if (HexAfter(path, "VID_") != 0x28de) continue;
    const unsigned pid = HexAfter(path, "PID_");
    bool headset = pid == 0x2000 || pid == 0x2012 || pid == 0x2022;
    if (pid == 0x2300) {
      std::lock_guard lock(g_openMutex);
      const auto slash = path.find_last_of('\\');
      headset = g_allowedHeadsets.count("*") ||
                (slash != std::string::npos && g_allowedHeadsets.count(path.substr(slash + 1)));
    }
    GUID container{};
    if (headset && ContainerOf(set, &device, &container)) g_headsetContainers.push_back(container);
  }
  SetupDiDestroyDeviceInfoList(set);
}

bool StockMayUse(const char* devicePath, HDEVINFO set, SP_DEVINFO_DATA* device) {
  if (!devicePath) return true;
  const std::string path = Upper(devicePath);
  if (HexAfter(path, "VID_") != 0x28de) return true;
  const unsigned pid = HexAfter(path, "PID_");
  bool headsetRadio = false;
  if (pid == 0x2102) {
    GUID container{};
    if (ContainerOf(set, device, &container))
      headsetRadio = std::any_of(g_headsetContainers.begin(), g_headsetContainers.end(),
                                [&](const GUID& hmd) { return IsEqualGUID(container, hmd) != 0; });
  }
  const std::string serial = pid == 0x2300 ? SerialOf(devicePath) : "";
  std::lock_guard lock(g_openMutex);
  return StockWindowsDeviceAllowed(0x28de, pid, g_allowedHeadsets.count("*") || g_allowedHeadsets.count(serial),
                                   headsetRadio);
}

// Whether interface `index` of the set is hidden; reads its path with the real detail call.
bool HiddenInterface(HDEVINFO set, PSP_DEVICE_INTERFACE_DATA data) {
  DWORD size = 0;
  g_realDetail(set, data, nullptr, 0, &size, nullptr);
  if (size == 0 || size > 4096) return false;
  alignas(8) char buffer[4096];
  auto* detail = reinterpret_cast<PSP_DEVICE_INTERFACE_DETAIL_DATA_A>(buffer);
  detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);
  SP_DEVINFO_DATA device{};
  device.cbSize = sizeof device;
  if (!g_realDetail(set, data, detail, size, nullptr, &device)) return false;
  if (StockMayUse(detail->DevicePath, set, &device)) return false;
  LogHidden(detail->DevicePath);
  return true;
}

// Index `wanted` among the visible interfaces: the caller counts 0, 1, 2... until it fails.
BOOL WINAPI StockEnumInterfaces(HDEVINFO set, PSP_DEVINFO_DATA info, const GUID* guid, DWORD wanted,
                                PSP_DEVICE_INTERFACE_DATA data) {
  DWORD visible = 0;
  for (DWORD real = 0;; ++real) {
    if (!g_realEnum(set, info, guid, real, data)) return FALSE;  // its error (no more items)
    if (HiddenInterface(set, data)) continue;
    if (visible++ == wanted) return TRUE;
  }
}

// Points the module's import `dll`!`name` (its IAT slot) at `hook`; returns the original.
void* HookImport(HMODULE module, const char* dll, const char* name, void* hook) {
  auto* base = reinterpret_cast<uint8_t*>(module);
  auto* dos = reinterpret_cast<IMAGE_DOS_HEADER*>(base);
  if (dos->e_magic != IMAGE_DOS_SIGNATURE) return nullptr;
  auto* nt = reinterpret_cast<IMAGE_NT_HEADERS*>(base + dos->e_lfanew);
  const auto& dir = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT];
  if (!dir.VirtualAddress) return nullptr;
  for (auto* imp = reinterpret_cast<IMAGE_IMPORT_DESCRIPTOR*>(base + dir.VirtualAddress); imp->Name; ++imp) {
    if (_stricmp(reinterpret_cast<const char*>(base + imp->Name), dll) != 0) continue;
    auto* names = reinterpret_cast<IMAGE_THUNK_DATA*>(base + (imp->OriginalFirstThunk ? imp->OriginalFirstThunk : imp->FirstThunk));
    auto* slots = reinterpret_cast<IMAGE_THUNK_DATA*>(base + imp->FirstThunk);
    for (; names->u1.AddressOfData; ++names, ++slots) {
      if (IMAGE_SNAP_BY_ORDINAL(names->u1.Ordinal)) continue;
      auto* byName = reinterpret_cast<IMAGE_IMPORT_BY_NAME*>(base + names->u1.AddressOfData);
      if (std::strcmp(reinterpret_cast<const char*>(byName->Name), name) != 0) continue;
      DWORD old = 0;
      if (!VirtualProtect(&slots->u1.Function, sizeof(void*), PAGE_READWRITE, &old)) return nullptr;
      void* original = reinterpret_cast<void*>(slots->u1.Function);
      slots->u1.Function = reinterpret_cast<ULONG_PTR>(hook);
      VirtualProtect(&slots->u1.Function, sizeof(void*), old, &old);
      return original;
    }
  }
  return nullptr;
}

bool HideOtherDevices(HMODULE module) {
  FindHeadsetContainers();
  g_realDetail = reinterpret_cast<InterfaceDetailA>(
      reinterpret_cast<void*>(GetProcAddress(GetModuleHandleA("setupapi.dll"), "SetupDiGetDeviceInterfaceDetailA")));
  if (!g_realDetail) return false;
  void* real = HookImport(module, "SETUPAPI.dll", "SetupDiEnumDeviceInterfaces", reinterpret_cast<void*>(&StockEnumInterfaces));
  if (!real) return false;
  g_realEnum = reinterpret_cast<EnumInterfaces>(real);
  return true;
}
}  // namespace
#endif

void StockHmd::AllowHeadset(const std::string& serial) {
  std::lock_guard lock(g_openMutex);
  g_allowedHeadsets.insert(Upper(serial));
}

// Wraps the stock headset so the host's Activate tells us its object id.
class StockHmd::DeviceProxy : public ITrackedDeviceServerDriver {
 public:
  DeviceProxy(StockHmd* owner, std::string serial, ITrackedDeviceServerDriver* inner)
      : owner_(owner), serial_(std::move(serial)), inner_(inner) {}
  EVRInitError Activate(uint32_t id) override {
    const EVRInitError r = inner_->Activate(id);
    if (r == VRInitError_None) {
      alive_ = true;
      std::lock_guard lock(owner_->mutex_);
      owner_->activated_.push_back({serial_, id, &alive_});
      // SteamVR sends "leave standby" when it comes up and the stock driver powers the
      // headset's panels then; other hosts (Monado) never do, and the panels stay dark.
      // Harmless under SteamVR (it is out of standby already).
      {
        VREvent_t wake{};
        wake.eventType = VREvent_LeaveStandbyMode;
        wake.trackedDeviceIndex = id;
        owner_->events_.push_back(wake);
        wake.eventType = VREvent_TrackedDeviceUserInteractionStarted;
        owner_->events_.push_back(wake);
      }
    }
    return r;
  }
  void Deactivate() override {
    alive_ = false;  // ours stops publishing its pose first
    inner_->Deactivate();
  }
  void EnterStandby() override { inner_->EnterStandby(); }
  void* GetComponent(const char* name) override { return inner_->GetComponent(name); }
  void DebugRequest(const char* request, char* response, uint32_t size) override {
    inner_->DebugRequest(request, response, size);
  }
  DriverPose_t GetPose() override { return inner_->GetPose(); }

 private:
  StockHmd* owner_;
  std::string serial_;
  ITrackedDeviceServerDriver* inner_;
  std::atomic<bool> alive_{false};
};

class StockHmd::Host : public IVRServerDriverHost {
 public:
  explicit Host(StockHmd* owner) : owner_(owner) {}
  bool TrackedDeviceAdded(const char* serial, ETrackedDeviceClass cls, ITrackedDeviceServerDriver* driver) override {
    if (cls != TrackedDeviceClass_HMD) return true;  // ours already; the stock one stays inactive
    auto* proxy = new DeviceProxy(owner_, serial ? serial : "", driver);
    {
      std::lock_guard lock(owner_->mutex_);
      owner_->devices_.push_back(proxy);
    }
    if (owner_->log_) owner_->log_(std::string("stock driver: head-mounted display ") + (serial ? serial : "?"));
    return owner_->realHost_->TrackedDeviceAdded(serial, cls, proxy);
  }
  void TrackedDevicePoseUpdated(uint32_t id, const DriverPose_t& pose, uint32_t size) override {
    {
      std::lock_guard lock(owner_->mutex_);
      if (owner_->owned_.count(id)) return;
    }
    owner_->realHost_->TrackedDevicePoseUpdated(id, pose, size);
  }
  void VsyncEvent(double offset) override { owner_->realHost_->VsyncEvent(offset); }
  void VendorSpecificEvent(uint32_t id, EVREventType type, const VREvent_Data_t& data, double offset) override {
    // Status alerts about the stock driver's own tracking, which isn't used (ours drives
    // the pose): 10002 "Too Many Base Stations" (its TDM tracker in a room of more than
    // two bases, e.g. lhbs multi-base firmware). Other types pass; each is logged once.
    if (int(type) == 10002) return;
    {
      std::lock_guard lock(owner_->mutex_);
      if (owner_->vendorEvents_.insert(int(type)).second && owner_->log_)
        owner_->log_("stock driver: vendor event " + std::to_string(int(type)) + " for device " + std::to_string(id));
    }
    owner_->realHost_->VendorSpecificEvent(id, type, data, offset);
  }
  bool IsExiting() override { return owner_->realHost_->IsExiting(); }
  bool PollNextEvent(VREvent_t* event, uint32_t size) override {
    std::lock_guard lock(owner_->mutex_);
    if (owner_->events_.empty() || size < sizeof(VREvent_t)) return false;
    *event = owner_->events_.front();
    owner_->events_.pop_front();
    return true;
  }
  void GetRawTrackedDevicePoses(float predicted, TrackedDevicePose_t* poses, uint32_t count) override {
    owner_->realHost_->GetRawTrackedDevicePoses(predicted, poses, count);
  }
  void RequestRestart(const char* reason, const char* exe, const char* args, const char* dir) override {
    owner_->realHost_->RequestRestart(reason, exe, args, dir);
  }
  uint32_t GetFrameTimings(Compositor_FrameTiming* timing, uint32_t frames) override {
    return owner_->realHost_->GetFrameTimings(timing, frames);
  }
  void SetDisplayEyeToHead(uint32_t id, const HmdMatrix34_t& left, const HmdMatrix34_t& right) override {
    owner_->realHost_->SetDisplayEyeToHead(id, left, right);
  }
  void SetDisplayProjectionRaw(uint32_t id, const HmdRect2_t& left, const HmdRect2_t& right) override {
    owner_->realHost_->SetDisplayProjectionRaw(id, left, right);
  }
  void SetRecommendedRenderTargetSize(uint32_t id, uint32_t width, uint32_t height) override {
    owner_->realHost_->SetRecommendedRenderTargetSize(id, width, height);
  }

 private:
  StockHmd* owner_;
};

// Settings the host doesn't have come from the stock driver's resources/settings/
// default.vrsettings (SteamVR merges that file itself; other hosts, Monado, don't: without
// it the stock driver has no "usedisambiguation" and never brings its headset up).
class StockHmd::Settings : public IVRSettings {
 public:
  Settings(IVRSettings* real, nlohmann::json defaults) : real_(real), defaults_(std::move(defaults)) {}
  const char* GetSettingsErrorNameFromEnum(EVRSettingsError e) override { return real_->GetSettingsErrorNameFromEnum(e); }
  void SetBool(const char* s, const char* k, bool v, EVRSettingsError* e) override { real_->SetBool(s, k, v, e); }
  void SetInt32(const char* s, const char* k, int32_t v, EVRSettingsError* e) override { real_->SetInt32(s, k, v, e); }
  void SetFloat(const char* s, const char* k, float v, EVRSettingsError* e) override { real_->SetFloat(s, k, v, e); }
  void SetString(const char* s, const char* k, const char* v, EVRSettingsError* e) override { real_->SetString(s, k, v, e); }
  bool GetBool(const char* s, const char* k, EVRSettingsError* e) override {
    EVRSettingsError err = VRSettingsError_None;
    const bool v = real_->GetBool(s, k, &err);
    if (err == VRSettingsError_None) return Ok(e), v;
    const auto* d = Default(s, k);
    if (!d || !(d->is_boolean() || d->is_number())) return Fail(e, err), v;
    return Ok(e), d->is_boolean() ? d->get<bool>() : d->get<double>() != 0;
  }
  int32_t GetInt32(const char* s, const char* k, EVRSettingsError* e) override {
    EVRSettingsError err = VRSettingsError_None;
    const int32_t v = real_->GetInt32(s, k, &err);
    if (err == VRSettingsError_None) return Ok(e), v;
    const auto* d = Default(s, k);
    if (!d || !(d->is_boolean() || d->is_number())) return Fail(e, err), v;
    return Ok(e), d->is_boolean() ? int32_t(d->get<bool>()) : int32_t(d->get<double>());
  }
  float GetFloat(const char* s, const char* k, EVRSettingsError* e) override {
    EVRSettingsError err = VRSettingsError_None;
    const float v = real_->GetFloat(s, k, &err);
    if (err == VRSettingsError_None) return Ok(e), v;
    const auto* d = Default(s, k);
    if (!d || !d->is_number()) return Fail(e, err), v;
    return Ok(e), float(d->get<double>());
  }
  void GetString(const char* s, const char* k, char* v, uint32_t n, EVRSettingsError* e) override {
    EVRSettingsError err = VRSettingsError_None;
    real_->GetString(s, k, v, n, &err);
    if (err == VRSettingsError_None) return Ok(e);
    const auto* d = Default(s, k);
    if (!d || !d->is_string() || !v || n == 0) return Fail(e, err);
    const std::string str = d->get<std::string>();
    std::snprintf(v, n, "%s", str.c_str());
    Ok(e);
  }
  void RemoveSection(const char* s, EVRSettingsError* e) override { real_->RemoveSection(s, e); }
  void RemoveKeyInSection(const char* s, const char* k, EVRSettingsError* e) override { real_->RemoveKeyInSection(s, k, e); }

 private:
  static void Ok(EVRSettingsError* e) { if (e) *e = VRSettingsError_None; }
  static void Fail(EVRSettingsError* e, EVRSettingsError err) { if (e) *e = err; }
  const nlohmann::json* Default(const char* s, const char* k) const {
    if (!s || !k || !defaults_.is_object()) return nullptr;
    const auto sec = defaults_.find(s);
    if (sec == defaults_.end() || !sec->is_object()) return nullptr;
    const auto it = sec->find(k);
    return it == sec->end() ? nullptr : &*it;
  }
  IVRSettings* real_;
  nlohmann::json defaults_;
};

// The stock driver's log, minus repeats: its own tracker runs for the headset too (unused,
// ours drives the pose) and in rooms it can't handle (more than 2 bases on TDM, multi-base
// lhbs firmware) repeats the same lines several times a second. A line repeated within
// a minute is dropped.
class StockHmd::LogFilter : public IVRDriverLog {
 public:
  explicit LogFilter(IVRDriverLog* real) : real_(real) {}
  void Log(const char* text) override {
    if (!text || !real_) return;
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard lock(mutex_);
      auto& last = seen_[text];
      const bool repeat = last.time_since_epoch().count() != 0 && now - last < std::chrono::minutes(1);
      if (!repeat) last = now;
      if (seen_.size() > 4096) seen_.clear();
      if (repeat) return;
    }
    real_->Log(text);
  }

 private:
  IVRDriverLog* real_;
  std::mutex mutex_;
  std::map<std::string, std::chrono::steady_clock::time_point> seen_;
};

class StockHmd::Context : public IVRDriverContext {
 public:
  Context(IVRDriverContext* real, Host* host, Settings* settings, StockHmd::LogFilter* log)
      : real_(real), host_(host), settings_(settings), log_(log) {}
  void* GetGenericInterface(const char* version, EVRInitError* error) override {
    if (version && std::string(version) == IVRServerDriverHost_Version) {
      if (error) *error = VRInitError_None;
      return host_;
    }
    if (log_ && version && std::string(version) == IVRDriverLog_Version) {
      if (error) *error = VRInitError_None;
      return log_;
    }
    if (settings_ && version && std::string(version) == IVRSettings_Version) {
      if (error) *error = VRInitError_None;
      return settings_;
    }
    if (version && std::string(version) == kMailboxVersion) {
      // Its own mailboxes pass; the pairing one is ours (pairing_mailbox.hpp).
      if (!mailbox_)
        if (auto* real = static_cast<IVRMailbox*>(real_->GetGenericInterface(version, error))) mailbox_ = new StockMailbox(real);
      if (error) *error = mailbox_ ? VRInitError_None : VRInitError_Init_InterfaceNotFound;
      return mailbox_;
    }
    return real_->GetGenericInterface(version, error);
  }
  DriverHandle_t GetDriverHandle() override { return real_->GetDriverHandle(); }

 private:
  IVRDriverContext* real_;
  Host* host_;
  Settings* settings_;
  StockHmd::LogFilter* log_;
  StockMailbox* mailbox_ = nullptr;  // created on first request, never freed (as the rest)
};

namespace {
// hidapi's strings are wide; USB names and serials are ASCII ('?' otherwise). Same on
// every backend, unlike printf's %ls.
std::string Narrow(const wchar_t* text) {
  std::string out;
  for (; text && *text; ++text) out += *text < 0x80 ? char(*text) : '?';
  return out;
}
}  // namespace

std::string StockHmd::DefaultPath() {
  if (const char* p = std::getenv("LH_STOCK_DRIVER"); p && *p) return p;
#ifdef _WIN32
  HMODULE self = nullptr;
  if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                          reinterpret_cast<LPCWSTR>(&StockHmd::DefaultPath), &self))
    return {};
  wchar_t name[32768];
  const DWORD n = GetModuleFileNameW(self, name, DWORD(std::size(name)));
  if (n == 0 || n >= std::size(name)) return {};
  const auto path = std::filesystem::path(std::wstring(name, n)).parent_path() / L"driver_lighthouse.dll.stock";
  std::error_code ec;
  if (!std::filesystem::exists(path, ec)) return {};
  const auto u8 = path.u8string();
  return std::string(u8.begin(), u8.end());
#else
  Dl_info info{};
  if (!dladdr(reinterpret_cast<void*>(&StockHmd::DefaultPath), &info) || !info.dli_fname) return {};
  const auto path = std::filesystem::path(info.dli_fname).parent_path() / "driver_lighthouse.so.stock";
  std::error_code ec;
  return std::filesystem::exists(path, ec) ? path.string() : std::string();
#endif
}

StockHmd::Presence StockHmd::HeadsetPresent(std::string* which, std::vector<std::string>* maybe) {
  // LH_STOCK_HMD=1: load the stock driver even if no USB device looks like a headset.
  if (const char* force = std::getenv("LH_STOCK_HMD"); force && *force == '1') {
    if (which) *which = "(forced by LH_STOCK_HMD)";
    AllowHeadset("*");  // headset unknown: any wired 0x2300 may be it
    return Presence::Yes;
  }
  hid_device_info* list = hid_enumerate(0x28de, 0);
  Presence result = Presence::No;
  for (auto* d = list; d; d = d->next) {
    const std::wstring product = d->product_string ? d->product_string : L"";
    Presence p = Presence::No;
    if (d->product_id == 0x2000) p = Presence::Yes;
    else if (d->product_id == 0x2300 && product.find(L"Controller") == std::wstring::npos) p = Presence::Maybe;
    if (p == Presence::No) continue;
    char id[16];
    std::snprintf(id, sizeof id, "28de:%04x ", d->product_id);
    if (p > result && which) *which = id + Narrow(product.c_str());
    result = std::max(result, p);
    if (p == Presence::Maybe && maybe && d->serial_number) {
      const std::string serial = Narrow(d->serial_number);
      if (std::find(maybe->begin(), maybe->end(), serial) == maybe->end()) maybe->push_back(serial);
    }
  }
  hid_free_enumeration(list);
  return result;
}

bool StockHmd::Load(const std::string& path, IVRDriverContext* realContext, LogFn log) {
  log_ = std::move(log);
  if (path.empty()) return false;
  using Factory = void* (*)(const char*, int*);
#ifdef _WIN32
  // path is UTF-8. LOAD_WITH_ALTERED_SEARCH_PATH: its own dependencies come from its
  // directory (bin/win64), as when SteamVR loads it. The ".stock" extension is fine with
  // a full path (LoadLibrary only appends ".dll" to names without one).
  const std::filesystem::path file(std::u8string(path.begin(), path.end()));
  HMODULE module = LoadLibraryExW(file.wstring().c_str(), nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);
  if (!module) {
    if (log_) log_("stock driver: cannot load " + path + ": error " + std::to_string(GetLastError()));
    return false;
  }
  library_ = module;
  auto factory = reinterpret_cast<Factory>(reinterpret_cast<void*>(GetProcAddress(module, "HmdDriverFactory")));
  if (!std::getenv("LH_STOCK_ALL_DEVICES")) {
    {
      std::lock_guard lock(g_openMutex);
      g_openLog = log_;
    }
    if (!HideOtherDevices(module) && log_) log_("stock driver: could not limit it to the headset's devices");
  }
#else
  // RTLD_LOCAL: its symbols (HmdDriverFactory, its OpenVR helpers) stay its own.
  library_ = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!library_) {
    if (log_) log_(std::string("stock driver: cannot load ") + path + ": " + dlerror());
    return false;
  }
  auto factory = reinterpret_cast<Factory>(dlsym(library_, "HmdDriverFactory"));
  if (!std::getenv("LH_STOCK_ALL_DEVICES")) {
    {
      std::lock_guard lock(g_openMutex);
      g_openLog = log_;
    }
    if (!HideOtherDevices(library_) && log_) log_("stock driver: could not limit it to the headset's devices");
  }
#endif
  int code = 0;
  auto* provider = factory ? static_cast<IServerTrackedDeviceProvider*>(factory(IServerTrackedDeviceProvider_Version, &code))
                           : nullptr;
  if (!provider) {
    if (log_) log_("stock driver: no " + std::string(IServerTrackedDeviceProvider_Version) + " in " + path);
#ifdef _WIN32
    FreeLibrary(static_cast<HMODULE>(library_));
#else
    dlclose(library_);
#endif
    library_ = nullptr;
    return false;
  }
  real_ = realContext;
  realHost_ = static_cast<IVRServerDriverHost*>(real_->GetGenericInterface(IVRServerDriverHost_Version, nullptr));
  host_ = new Host(this);
  if (auto* realSettings = static_cast<IVRSettings*>(real_->GetGenericInterface(IVRSettings_Version, nullptr))) {
    // <driver>/bin/{linux64,win64}/<library> -> <driver>/resources/settings/default.vrsettings
    const auto file = std::filesystem::path(std::u8string(path.begin(), path.end())).parent_path().parent_path().parent_path() /
                      "resources" / "settings" / "default.vrsettings";
    nlohmann::json defaults;
    if (std::ifstream in(file); in) defaults = nlohmann::json::parse(in, nullptr, false, true);
    if (defaults.is_discarded() || !defaults.is_object()) {
      if (log_) {
        const auto u8 = file.u8string();
        log_("stock driver: no settings defaults at " + std::string(u8.begin(), u8.end()));
      }
      defaults = nlohmann::json::object();
    }
    settings_ = new Settings(realSettings, std::move(defaults));
  }
  if (auto* realLog = static_cast<IVRDriverLog*>(real_->GetGenericInterface(IVRDriverLog_Version, nullptr)))
    logProxy_ = new LogFilter(realLog);
  context_ = new Context(real_, host_, settings_, logProxy_);
  const EVRInitError r = provider->Init(context_);
  if (r != VRInitError_None) {
    if (log_) log_("stock driver: Init failed (" + std::to_string(int(r)) + ")");
    return false;  // library stays loaded: its provider may hold threads
  }
  provider_ = provider;
  if (log_) log_("stock driver loaded for head-mounted displays: " + path);
  return true;
}

void StockHmd::Cleanup() {
  if (provider_) provider_->Cleanup();
  provider_ = nullptr;
  // The library, proxies and context stay alive: the host may still hold the headset's
  // proxy until it shuts down, and unloading code that may still run is worse than a leak.
}

void StockHmd::PushEvent(const VREvent_t& event) {
  if (!provider_) return;
  // Haptics are ours (its other devices are never activated, but don't risk a double pulse).
  if (event.eventType == VREvent_Input_HapticVibration) return;
  std::lock_guard lock(mutex_);
  if (events_.size() >= 256) events_.pop_front();
  events_.push_back(event);
}

void StockHmd::RunFrame() {
  if (provider_) provider_->RunFrame();
}

std::optional<StockHmd::Activated> StockHmd::TakeActivated() {
  std::lock_guard lock(mutex_);
  if (activated_.empty()) return std::nullopt;
  auto a = activated_.front();
  activated_.pop_front();
  return a;
}

void StockHmd::OwnPose(uint32_t objectId) {
  std::lock_guard lock(mutex_);
  owned_.insert(objectId);
}

}  // namespace lh::openvr
