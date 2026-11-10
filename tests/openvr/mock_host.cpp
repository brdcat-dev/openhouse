// Headless OpenVR host for the drop-in driver, modelled on Monado's steamvr_lh:
// dlopen + HmdDriverFactory, Activate() called from inside TrackedDeviceAdded(),
// devices only accepted during a discovery window, properties/inputs recorded.
// Runs the driver on a replayed capture (LH_REPLAY) and checks what a host sees.
#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif
#include <openvr_driver.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <variant>
#include <vector>

using namespace vr;

namespace {

struct Recorded {
  std::mutex mutex;
  struct Dev {
    std::string serial;
    ETrackedDeviceClass cls;
    ITrackedDeviceServerDriver* driver;
    std::map<ETrackedDeviceProperty, std::vector<uint8_t>> props;
    std::set<std::string> inputs;
    int poses = 0, valid = 0;
    int dropped = 0, held = 0;  // connected, invalid after a valid pose (shown at the origin); held lost poses
    DriverPose_t last{};
  };
  std::vector<Dev> devices;  // index = object id
  std::map<VRInputComponentHandle_t, std::pair<uint32_t, std::string>> components;
  std::map<std::string, int> inputUpdates;
  std::atomic<bool> discovering{true};
  int declined = 0;
  int skeletonCreates = 0, skeletonUpdates = 0, invalidBones = 0;
  int hapticEvents = 0, unroutedHaptics = 0;
  int powerOffRequests = 0;
  bool sentHaptic = false;
  std::chrono::steady_clock::time_point initTime = std::chrono::steady_clock::now();
} g;

std::string PropString(const Recorded::Dev& d, ETrackedDeviceProperty p) {
  auto it = d.props.find(p);
  if (it == d.props.end()) return {};
  return std::string(reinterpret_cast<const char*>(it->second.data()), strnlen(reinterpret_cast<const char*>(it->second.data()), it->second.size()));
}
template <typename T>
T PropValue(const Recorded::Dev& d, ETrackedDeviceProperty p, T fallback) {
  auto it = d.props.find(p);
  if (it == d.props.end() || it->second.size() < sizeof(T)) return fallback;
  T v;
  std::memcpy(&v, it->second.data(), sizeof v);
  return v;
}

class Settings : public IVRSettings {
 public:
  const char* GetSettingsErrorNameFromEnum(EVRSettingsError) override { return "error"; }
  void SetBool(const char*, const char*, bool, EVRSettingsError* e) override { if (e) *e = VRSettingsError_None; }
  void SetInt32(const char*, const char*, int32_t, EVRSettingsError* e) override { if (e) *e = VRSettingsError_None; }
  void SetFloat(const char*, const char*, float, EVRSettingsError* e) override { if (e) *e = VRSettingsError_None; }
  void SetString(const char*, const char*, const char*, EVRSettingsError* e) override { if (e) *e = VRSettingsError_None; }
  bool GetBool(const char*, const char*, EVRSettingsError* e) override { if (e) *e = VRSettingsError_UnsetSettingHasNoDefault; return false; }
  int32_t GetInt32(const char*, const char*, EVRSettingsError* e) override { if (e) *e = VRSettingsError_UnsetSettingHasNoDefault; return 0; }
  float GetFloat(const char*, const char*, EVRSettingsError* e) override { if (e) *e = VRSettingsError_UnsetSettingHasNoDefault; return 0; }
  void GetString(const char*, const char*, char* v, uint32_t n, EVRSettingsError* e) override { if (n) v[0] = 0; if (e) *e = VRSettingsError_UnsetSettingHasNoDefault; }
  void RemoveSection(const char*, EVRSettingsError* e) override { if (e) *e = VRSettingsError_None; }
  void RemoveKeyInSection(const char*, const char*, EVRSettingsError* e) override { if (e) *e = VRSettingsError_None; }
} g_settings;

class Manager : public IVRDriverManager {
 public:
  uint32_t GetDriverCount() const override { return 1; }
  uint32_t GetDriverName(DriverId_t, char* v, uint32_t n) override { if (n) std::snprintf(v, n, "lighthouse"); return 11; }
  DriverHandle_t GetDriverHandle(const char*) override { return 1; }
  bool IsEnabled(DriverId_t) const override { return true; }
} g_manager;

class Resources : public IVRResources {
 public:
  uint32_t LoadSharedResource(const char*, char*, uint32_t) override { return 0; }
  uint32_t GetResourceFullPath(const char*, const char*, char* v, uint32_t n) override { if (n) v[0] = 0; return 0; }
} g_resources;

// "IVRServerInternal_XXX" / "IVRServer_XXX": private SteamVR interfaces the stock driver
// calls without checking (loaded by our driver for headsets, stock_hmd.hpp). Monado's
// steamvr_lh serves a class of empty virtual functions; so do we.
struct ServerStub {
#define LH_STUB(n) virtual void Unknown##n() {}
  LH_STUB(0) LH_STUB(1) LH_STUB(2) LH_STUB(3) LH_STUB(4) LH_STUB(5) LH_STUB(6) LH_STUB(7) LH_STUB(8) LH_STUB(9)
  LH_STUB(10) LH_STUB(11) LH_STUB(12) LH_STUB(13) LH_STUB(14) LH_STUB(15) LH_STUB(16) LH_STUB(17) LH_STUB(18) LH_STUB(19)
  LH_STUB(20) LH_STUB(21) LH_STUB(22) LH_STUB(23) LH_STUB(24) LH_STUB(25) LH_STUB(26) LH_STUB(27) LH_STUB(28) LH_STUB(29)
  LH_STUB(30) LH_STUB(31) LH_STUB(32) LH_STUB(33) LH_STUB(34) LH_STUB(35) LH_STUB(36) LH_STUB(37) LH_STUB(38) LH_STUB(39)
  LH_STUB(40) LH_STUB(41) LH_STUB(42) LH_STUB(43) LH_STUB(44) LH_STUB(45) LH_STUB(46) LH_STUB(47) LH_STUB(48) LH_STUB(49)
#undef LH_STUB
  virtual ~ServerStub() = default;
} g_server;

// "IVRBlockQueue_005" (camera frames): every call fails (EBlockQueueError 1), no queue.
struct BlockQueueStub {
#define LH_STUB(n) virtual int Unknown##n() { return 1; }
  LH_STUB(0) LH_STUB(1) LH_STUB(2) LH_STUB(3) LH_STUB(4) LH_STUB(5) LH_STUB(6) LH_STUB(7) LH_STUB(8) LH_STUB(9)
  LH_STUB(10) LH_STUB(11) LH_STUB(12) LH_STUB(13) LH_STUB(14) LH_STUB(15)
#undef LH_STUB
} g_blockQueue;

class Host : public IVRServerDriverHost, public IVRDriverInput, public IVRProperties, public IVRDriverLog,
             public IVRDriverContext {
 public:
  // IVRDriverContext
  void* GetGenericInterface(const char* version, EVRInitError* err) override {
    if (std::getenv("LH_HOST_VERBOSE")) std::printf("host: interface %s\n", version);
    if (err) *err = VRInitError_None;
    if (!std::strcmp(version, IVRServerDriverHost_Version)) return static_cast<IVRServerDriverHost*>(this);
    if (!std::strcmp(version, IVRDriverInput_Version) || !std::strncmp(version, "IVRDriverInput_", 15))
      return static_cast<IVRDriverInput*>(this);
    if (!std::strcmp(version, IVRProperties_Version)) return static_cast<IVRProperties*>(this);
    if (!std::strcmp(version, IVRDriverLog_Version)) return static_cast<IVRDriverLog*>(this);
    if (!std::strcmp(version, IVRSettings_Version)) return &g_settings;
    if (!std::strcmp(version, IVRDriverManager_Version)) return &g_manager;
    if (!std::strcmp(version, IVRResources_Version)) return &g_resources;
    if (!std::strcmp(version, "IVRServerInternal_XXX") || !std::strcmp(version, "IVRServer_XXX")) return &g_server;
    if (!std::strncmp(version, "IVRBlockQueue_", 14)) return &g_blockQueue;
    if (err) *err = VRInitError_Init_InterfaceNotFound;
    return nullptr;
  }
  DriverHandle_t GetDriverHandle() override { return 1; }

  // IVRServerDriverHost
  bool TrackedDeviceAdded(const char* serial, ETrackedDeviceClass cls, ITrackedDeviceServerDriver* driver) override {
    if (!g.discovering || cls == TrackedDeviceClass_TrackingReference) {  // Monado declines bases too
      std::lock_guard lock(g.mutex);
      ++g.declined;
      if (cls == TrackedDeviceClass_TrackingReference) return false;
      return false;
    }
    if (std::getenv("LH_HOST_VERBOSE"))
      std::printf("host: %s added %.2f s after Init\n", serial,
                  std::chrono::duration<double>(std::chrono::steady_clock::now() - g.initTime).count());
    uint32_t id;
    {
      std::lock_guard lock(g.mutex);
      id = uint32_t(g.devices.size());
      g.devices.push_back({serial, cls, driver, {}, {}, 0, 0, {}});
    }
    // Like Monado: activate synchronously, without host locks held.
    return driver->Activate(id) == VRInitError_None;
  }
  void TrackedDevicePoseUpdated(uint32_t which, const DriverPose_t& pose, uint32_t size) override {
    if (size != sizeof(DriverPose_t)) std::abort();
    std::lock_guard lock(g.mutex);
    if (which >= g.devices.size()) return;
    auto& d = g.devices[which];
    ++d.poses;
    if (d.cls != TrackedDeviceClass_TrackingReference) {
      d.dropped += d.valid > 0 && pose.deviceIsConnected && !pose.poseIsValid;
      d.held += pose.poseIsValid && pose.result == TrackingResult_Running_OutOfRange;
    }
    d.valid += pose.poseIsValid;
    d.last = pose;
  }
  void VsyncEvent(double) override {}
  void VendorSpecificEvent(uint32_t, EVREventType, const VREvent_Data_t&, double) override {}
  bool IsExiting() override { return false; }
  bool PollNextEvent(VREvent_t* event, uint32_t size) override {
    std::lock_guard lock(g.mutex);
    if (g.sentHaptic || size != sizeof(*event)) return false;
    for (const auto& [handle, component] : g.components) {
      if (component.second != "/output/haptic") continue;
      *event = {};
      event->eventType = VREvent_Input_HapticVibration;
      if (g.hapticEvents == 0) {
        // SteamVR: the device's property container
        event->data.hapticVibration = {100 + component.first, handle, .05f, 150.f, .2f};
      } else {
        // Monado's steamvr_lh (ControllerDevice::set_output): one less than the
        // device's property container, and one less than its index; only the
        // component is right.
        event->trackedDeviceIndex = uint32_t(component.first) - 1;
        event->data.hapticVibration = {100 + component.first - 1, handle, -1e-9f, 200.f, .3f};
        g.sentHaptic = true;
      }
      ++g.hapticEvents;
      return true;
    }
    return false;
  }
  void GetRawTrackedDevicePoses(float, TrackedDevicePose_t*, uint32_t) override {}
  void RequestRestart(const char*, const char*, const char*, const char*) override {}
  uint32_t GetFrameTimings(Compositor_FrameTiming*, uint32_t) override { return 0; }
  void SetDisplayEyeToHead(uint32_t, const HmdMatrix34_t&, const HmdMatrix34_t&) override {}
  void SetDisplayProjectionRaw(uint32_t, const HmdRect2_t&, const HmdRect2_t&) override {}
  void SetRecommendedRenderTargetSize(uint32_t, uint32_t, uint32_t) override {}

  // IVRDriverInput
  EVRInputError Add(PropertyContainerHandle_t c, const char* name, VRInputComponentHandle_t* h) {
    std::lock_guard lock(g.mutex);
    const uint32_t id = uint32_t(c - 100);
    if (id >= g.devices.size()) return VRInputError_InvalidHandle;
    g.devices[id].inputs.insert(name);
    *h = VRInputComponentHandle_t(1000 + g.components.size());
    g.components[*h] = {id, name};
    return VRInputError_None;
  }
  EVRInputError Updated(VRInputComponentHandle_t h, double offset) {
    if (offset > 0) std::abort();  // inputs must not be from the future
    std::lock_guard lock(g.mutex);
    auto it = g.components.find(h);
    if (it == g.components.end()) return VRInputError_InvalidHandle;
    ++g.inputUpdates[it->second.second];
    return VRInputError_None;
  }
  EVRInputError CreateBooleanComponent(PropertyContainerHandle_t c, const char* n, VRInputComponentHandle_t* h) override { return Add(c, n, h); }
  EVRInputError UpdateBooleanComponent(VRInputComponentHandle_t h, bool, double o) override { return Updated(h, o); }
  EVRInputError CreateScalarComponent(PropertyContainerHandle_t c, const char* n, VRInputComponentHandle_t* h, EVRScalarType, EVRScalarUnits) override { return Add(c, n, h); }
  EVRInputError UpdateScalarComponent(VRInputComponentHandle_t h, float, double o) override { return Updated(h, o); }
  EVRInputError CreateHapticComponent(PropertyContainerHandle_t c, const char* n, VRInputComponentHandle_t* h) override { return Add(c, n, h); }
  void ValidateBones(const VRBoneTransform_t* bones, uint32_t count) {
    if (!bones || count != 31) { ++g.invalidBones; return; }
    for (uint32_t i = 0; i < count; ++i) {
      const auto& q = bones[i].orientation;
      const double norm = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
      if (!std::isfinite(norm) || std::abs(norm - 1) > 1e-4) ++g.invalidBones;
      for (int k = 0; k < 4; ++k) if (!std::isfinite(bones[i].position.v[k])) ++g.invalidBones;
      if (bones[i].position.v[3] != 1) ++g.invalidBones;
    }
  }
  EVRInputError CreateSkeletonComponent(PropertyContainerHandle_t c, const char* n, const char* path, const char* origin,
      EVRSkeletalTrackingLevel level, const VRBoneTransform_t* bones, uint32_t count, VRInputComponentHandle_t* h) override {
    if (std::string(path) != "/skeleton/hand/right" || std::string(origin) != "/pose/raw" || level != VRSkeletalTracking_Partial)
      return VRInputError_InvalidParam;
    { std::lock_guard lock(g.mutex); ++g.skeletonCreates; ValidateBones(bones, count); }
    return Add(c, n, h);
  }
  EVRInputError UpdateSkeletonComponent(VRInputComponentHandle_t h, EVRSkeletalMotionRange,
      const VRBoneTransform_t* bones, uint32_t count) override {
    std::lock_guard lock(g.mutex);
    if (!g.components.count(h)) return VRInputError_InvalidHandle;
    ++g.skeletonUpdates; ValidateBones(bones, count);
    return VRInputError_None;
  }
  EVRInputError CreatePoseComponent(PropertyContainerHandle_t c, const char* n, VRInputComponentHandle_t* h) override { return Add(c, n, h); }
  EVRInputError UpdatePoseComponent(VRInputComponentHandle_t, const HmdMatrix34_t*, double) override { return VRInputError_None; }
  EVRInputError CreateEyeTrackingComponent(PropertyContainerHandle_t c, const char* n, VRInputComponentHandle_t* h) override { return Add(c, n, h); }
  EVRInputError UpdateEyeTrackingComponent(VRInputComponentHandle_t, const VREyeTrackingData_t*, double) override { return VRInputError_None; }

  // IVRProperties
  ETrackedPropertyError ReadPropertyBatch(PropertyContainerHandle_t, PropertyRead_t* batch, uint32_t n) override {
    for (uint32_t i = 0; i < n; ++i) batch[i].eError = TrackedProp_UnknownProperty;
    return TrackedProp_Success;
  }
  ETrackedPropertyError WritePropertyBatch(PropertyContainerHandle_t c, PropertyWrite_t* batch, uint32_t n) override {
    std::lock_guard lock(g.mutex);
    const uint32_t id = uint32_t(c - 100);
    if (id >= g.devices.size()) return TrackedProp_InvalidContainer;
    for (uint32_t i = 0; i < n; ++i) {
      auto& w = batch[i];
      if (w.writeType == PropertyWrite_Set) {
        const auto* p = static_cast<const uint8_t*>(w.pvBuffer);
        g.devices[id].props[w.prop].assign(p, p + w.unBufferSize);
      }
      w.eError = TrackedProp_Success;
    }
    return TrackedProp_Success;
  }
  const char* GetPropErrorNameFromEnum(ETrackedPropertyError) override { return "error"; }
  PropertyContainerHandle_t TrackedDeviceToPropertyContainer(TrackedDeviceIndex_t d) override { return 100 + d; }

  // IVRDriverLog
  void Log(const char* text) override {
    if (std::strstr(text, "power-off ")) { std::lock_guard lock(g.mutex); ++g.powerOffRequests; }
    if (std::getenv("LH_HOST_VERBOSE")) std::fputs(text, stdout);
    if (std::strstr(text, "unrouted haptic")) { std::lock_guard lock(g.mutex); ++g.unroutedHaptics; }
  }
};

int failures = 0;
#define CHECK(cond, ...)                                       \
  do {                                                         \
    if (!(cond)) {                                             \
      ++failures;                                              \
      std::printf("FAIL %s: ", #cond);                         \
      std::printf(__VA_ARGS__);                                \
      std::printf("\n");                                       \
    }                                                          \
  } while (0)

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s driver_lighthouse.so capture.lhraw [seconds]\n", argv[0]);
    return 2;
  }
  const bool live = std::string(argv[2]) == "live";
  if (!live && !std::filesystem::exists(argv[2])) {
    std::printf("SKIP: capture %s not found\n", argv[2]);
    return 0;
  }
  if (!live) {
#ifdef _WIN32
    _putenv_s("LH_REPLAY", argv[2]);
#else
    setenv("LH_REPLAY", argv[2], 1);
#endif
  }
  const double seconds = argc > 3 ? std::atof(argv[3]) : 12;
  using Factory = void* (*)(const char*, int*);
#ifdef _WIN32
  HMODULE lib = LoadLibraryW(std::filesystem::path(argv[1]).c_str());
  if (!lib) { std::printf("FAIL LoadLibrary: %lu\n", GetLastError()); return 1; }
  auto factory = reinterpret_cast<Factory>(GetProcAddress(lib, "HmdDriverFactory"));
#else
  void* lib = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
  if (!lib) { std::printf("FAIL dlopen: %s\n", dlerror()); return 1; }
  auto factory = reinterpret_cast<Factory>(dlsym(lib, "HmdDriverFactory"));
#endif
  if (!factory) { std::printf("FAIL: no HmdDriverFactory\n"); return 1; }
  int err = 0;
  auto* provider = static_cast<IServerTrackedDeviceProvider*>(factory(IServerTrackedDeviceProvider_Version, &err));
  if (!provider) { std::printf("FAIL: factory returned null (%d)\n", err); return 1; }
  Host host;
  g.initTime = std::chrono::steady_clock::now();
  if (provider->Init(&host) != VRInitError_None) { std::printf("FAIL: Init\n"); return 1; }
  // Discovery window like Monado's LH_DISCOVER_WAIT_MS (real time, replay runs in real time).
  const auto start = std::chrono::steady_clock::now();
  auto elapsed = [&] { return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(); };
  bool standbyTested = false;
  while (elapsed() < seconds) {
    if (!live && elapsed() > 8.0 && !standbyTested) {
      std::vector<ITrackedDeviceServerDriver*> devices;
      { std::lock_guard lock(g.mutex); for (const auto& d : g.devices) devices.push_back(d.driver); }
      for (auto* d : devices) d->EnterStandby();
      standbyTested = true;
    }
    if (elapsed() > 6.0) g.discovering = false;
    provider->RunFrame();
    std::this_thread::sleep_for(std::chrono::milliseconds(11));
  }
  provider->Cleanup();

  std::lock_guard lock(g.mutex);
  std::printf("devices accepted %zu, declined %d\n", g.devices.size(), g.declined);
  const Recorded::Dev* controller = nullptr;
  for (const auto& d : g.devices) {
    std::printf("  %-14s class %d profile %-55s role %d poses %d (valid %d) universe %llu battery %.2f\n",
                d.serial.c_str(), int(d.cls), PropString(d, Prop_InputProfilePath_String).c_str(),
                PropValue<int32_t>(d, Prop_ControllerRoleHint_Int32, -1), d.poses, d.valid,
                static_cast<unsigned long long>(PropValue<uint64_t>(d, Prop_CurrentUniverseId_Uint64, 0)),
                PropValue<float>(d, Prop_DeviceBatteryPercentage_Float, -1));
    CHECK(d.cls != TrackedDeviceClass_HMD, "HMDs must not be exposed");
    if (d.cls == TrackedDeviceClass_TrackingReference) {
      const auto hardware = PropValue<uint64_t>(d, Prop_HardwareRevision_Uint64, 0);
      CHECK(hardware >> 24 == 9 && (hardware & 0xff) != 0, "base station hardware revision for Bluetooth");
      CHECK(d.props.count(Prop_Nonce_Int32), "base station power-management nonce");
    }
    if (d.cls == TrackedDeviceClass_Controller) controller = &d;
  }
  CHECK(controller != nullptr, "the Index controller must be registered");
  if (controller) {
    CHECK(PropString(*controller, Prop_InputProfilePath_String) == "{indexcontroller}/input/index_controller_profile.json", "profile");
    CHECK(PropString(*controller, Prop_ControllerType_String) == "knuckles", "controller type");
    CHECK(PropValue<bool>(*controller, Prop_DeviceCanPowerOff_Bool, false), "power-off capability");
    CHECK(PropValue<bool>(*controller, Prop_Identifiable_Bool, false), "identify capability");
    CHECK(PropString(*controller, Prop_TrackingSystemName_String) == "lighthouse", "tracking system");
    CHECK(PropValue<int32_t>(*controller, Prop_ControllerRoleHint_Int32, 0) == TrackedControllerRole_RightHand, "right hand");
    CHECK(PropValue<uint64_t>(*controller, Prop_CurrentUniverseId_Uint64, 0) != 0, "universe id");
    CHECK(PropValue<float>(*controller, Prop_DeviceBatteryPercentage_Float, -1) > 0, "battery");
    for (const char* path : {"/input/system/click", "/input/a/click", "/input/a/touch", "/input/b/click", "/input/trigger/value",
                             "/input/trigger/click", "/input/grip/force", "/input/grip/value", "/input/thumbstick/x",
                             "/input/thumbstick/y", "/input/trackpad/force", "/input/finger/index", "/output/haptic"})
      CHECK(controller->inputs.count(path), "missing input %s", path);
    CHECK(controller->valid > 50, "valid poses: %d", controller->valid);
    const auto& p = controller->last;
    const double qn = std::sqrt(p.qRotation.w * p.qRotation.w + p.qRotation.x * p.qRotation.x + p.qRotation.y * p.qRotation.y + p.qRotation.z * p.qRotation.z);
    CHECK(std::fabs(qn - 1) < 1e-6, "unit quaternion");
    CHECK(p.poseTimeOffset <= 0 && p.poseTimeOffset > -0.1, "pose time offset %f", p.poseTimeOffset);
    CHECK(p.result == TrackingResult_Running_OK, "result %d", int(p.result));
    std::printf("  last pose (%.3f %.3f %.3f) offset %.1f ms, input updates: trigger %d, finger/index %d\n", p.vecPosition[0],
                p.vecPosition[1], p.vecPosition[2], p.poseTimeOffset * 1000, g.inputUpdates["/input/trigger/value"],
                g.inputUpdates["/input/finger/index"]);
  }
  CHECK(g.hapticEvents == 2, "haptic events were polled");
  CHECK(g.unroutedHaptics == 0, "SteamVR- and Monado-style haptic events reach their controller");
  int dropped = 0, held = 0;
  for (const auto& d : g.devices) { dropped += d.dropped; held += d.held; }
  CHECK(dropped == 0, "lost devices held at their last pose, not dropped to the origin: %d dropped", dropped);
  std::printf("  lost-and-held poses %d\n", held);
  if (!live && seconds > 9) CHECK(g.powerOffRequests > 0, "standby routed a power-off request");
  // Asset-dependent checks are opt-in so a source checkout without SteamVR still tests.
  if (std::getenv("LH_HAND_POSES")) {
    CHECK(g.skeletonCreates == 1, "skeleton created: %d", g.skeletonCreates);
    CHECK(g.skeletonUpdates > 10, "skeleton updates: %d", g.skeletonUpdates);
    CHECK(g.invalidBones == 0, "invalid bone data: %d", g.invalidBones);
  }
  std::printf("  skeletons %d, updates %d, invalid bones %d; haptic events %d\n", g.skeletonCreates, g.skeletonUpdates, g.invalidBones, g.hapticEvents);
  std::printf(failures ? "%d FAILURES\n" : "PASS\n", failures);
  return failures ? 1 : 0;
}
