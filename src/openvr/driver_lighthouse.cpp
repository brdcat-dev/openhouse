// Drop-in OpenVR driver ("lighthouse") on top of liblh.
//
// Replaces SteamVR's drivers/lighthouse/bin/linux64/driver_lighthouse.so for tracked
// devices: Index controllers (with input), trackers and base stations. Head-mounted
// displays are intentionally not exposed (no display/direct-mode support).
//
// Hosts: SteamVR (vrserver) and Monado's steamvr_lh driver. For Monado:
//   * devices are only accepted during its discovery window after Init
//     (LH_DISCOVER_WAIT_MS), so devices are registered as soon as their config is read;
//   * Activate() runs inside TrackedDeviceAdded(): no driver locks are held then;
//   * angular velocity is reported in the device (IMU) frame, like the stock driver;
//   * poses are in our room's frame (origin base, gravity up), the same every session;
//     LH_HOST_FRAME=steamvr puts them in SteamVR's raw frame of lighthousedb.json's
//     universe instead (what Monado's chaperone transform expects without an aligner);
//   * the room is published for alignment tools (room_publisher.hpp, lh/room_shm.h).
//
// Environment: LH_REPLAY=/path/input.lhraw replays a capture instead of live USB.
//   LH_CAPTURE=/path/dir records a diagnostic session while running in the host:
//   input.lhraw (raw USB, replayable with lhviz/lhtool), driver.jsonl (every pose
//   handed to the host, haptic events, session events) and lighthousedb.json.
#include <openvr_driver.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "pairing_mailbox.hpp"
#include "stock_hmd.hpp"

namespace {
#ifdef _WIN32
constexpr const char* kStockFile = "driver_lighthouse.dll.stock";
#else
constexpr const char* kStockFile = "driver_lighthouse.so.stock";
#endif
}  // namespace
#include "lh/session.hpp"
#include "hand_skeleton.hpp"
#include "raw_alignment.hpp"
#include "room_publisher.hpp"
#include "telemetry_server.hpp"
#include "lh/log_level.hpp"
#include "lh/telemetry.hpp"
#include "lh/version.hpp"
#include "lh/bug_report.hpp"
#include "host_frame.hpp"
#include "haptic_routing.hpp"
#include <set>
#include "platform/platform.hpp"
#include <ctime>
#include <filesystem>
#include <fstream>
#ifdef _WIN32
#include <windows.h>
#endif
#include <deque>

namespace {

using namespace vr;

double SteadyNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Buttons, axes and haptics. The stock driver asks for IVRDriverInput_005, whose first
// slots (Create/UpdateBoolean, CreateScalar, CreateHaptic) are laid out as in _004.
// Windows SteamVR accepted booleans sent through _004 but never applied them (no button
// presses, no dashboard; grip/thumbstick/trackpad touch refused as invalid or wrong type).
IVRDriverInput* ComponentInput() {
  static IVRDriverInput* input = [] {
    EVRInitError e = VRInitError_None;
    auto* v5 = static_cast<IVRDriverInput*>(VRDriverContext()->GetGenericInterface("IVRDriverInput_005", &e));
    if (v5 && e == VRInitError_None) return v5;
    return VRDriverInput();
  }();
  return input;
}

// LH_CAPTURE's driver.jsonl: what the host was given, on the session's host clock.
class Recorder {
 public:
  bool Open(const std::string& path) {
    std::lock_guard lock(mutex_);
    file_ = std::fopen(path.c_str(), "w");
    return file_ != nullptr;
  }
  void Close() {
    std::lock_guard lock(mutex_);
    if (file_) std::fclose(file_);
    file_ = nullptr;
  }
  bool On() const { return file_ != nullptr; }
  void Line(const std::string& json) {
    std::lock_guard lock(mutex_);
    if (!file_) return;
    std::fputs(json.c_str(), file_);
    std::fputc('\n', file_);
    if (++lines_ % 20 == 0) std::fflush(file_);  // a killed host keeps nearly everything
  }
  static std::string Quote(const std::string& text) {
    std::string out = "\"";
    for (char c : text) {
      if (c == '"' || c == '\\') { out += '\\'; out += c; }
      else if (uint8_t(c) < 0x20) out += ' ';
      else out += c;
    }
    return out + "\"";
  }
 private:
  std::mutex mutex_;
  std::FILE* file_ = nullptr;
  uint64_t lines_ = 0;
};
Recorder g_recorder;

// Every message goes to the viewers (they filter for themselves); the host's log gets
// Info and up unless LH_LOG_LEVEL says otherwise.
lh::openvr::TelemetryServer g_telemetry;
const lh::LogLevel g_logLevel = lh::MinimumLogLevelFromEnvironment();

void Log(lh::LogLevel level, const std::string& text, const std::string& serial = {}, double time = SteadyNow()) {
  g_telemetry.Log(time, level, serial, text);
  if (level >= g_logLevel && VRDriverLog())
    VRDriverLog()->Log(("lh: [" + std::string(lh::LogLevelName(level)) + "] " + text + "\n").c_str());
}
void Log(const std::string& text) { Log(lh::ClassifyLog(text), text); }

// The process hosting the driver (vrserver, monado-service, wivrn-server, ...).
std::string HostName();

// No stock display for a lighthouse headset: LH_NO_STOCK_HMD, or WiVRn, which streams its
// own headset (a Vive left plugged in crashed wivrn-server inside the stock driver's
// headset setup). LH_STOCK_HMD=1 still forces it.
bool StockHmdUnwanted(std::string* why) {
  if (std::getenv("LH_NO_STOCK_HMD")) { if (why) *why = "LH_NO_STOCK_HMD"; return true; }
  const char* force = std::getenv("LH_STOCK_HMD");
  if (!(force && *force == '1') && HostName() == "wivrn-server") {
    if (why) *why = "WiVRn streams its own headset";
    return true;
  }
  return false;
}

std::string HostName() {
#ifdef _WIN32
  char path[MAX_PATH]{};
  GetModuleFileNameA(nullptr, path, MAX_PATH);
  return std::filesystem::path(path).stem().string();
#else
  std::ifstream comm("/proc/self/comm");
  std::string name;
  std::getline(comm, name);
  return name;
#endif
}

uint64_t UniverseId(const std::string& roomId) {
  // "steamvr-1770781428" (imported SteamVR universe) or "room-<unix time>".
  const auto dash = roomId.rfind('-');
  if (dash == std::string::npos) return 0;
  return std::strtoull(roomId.c_str() + dash + 1, nullptr, 10);
}

HmdQuaternion_t Quat(const std::array<double, 4>& q) { return {q[0], q[1], q[2], q[3]}; }

// The render model name in a device's own config.
// 1) Index controllers' firmware names "{knuckles}valve_controller_knu_1_0_right", a resource
// root SteamVR no longer has (it is "indexcontroller" now): SteamVR found no model and logged
// "Could not find named path knuckles" every frame.
// 2) Vive Tracker 1.0/2.0 ("VIVE Tracker Pro MV") firmware reports "vr_tracker_vive_1_0" without
// resource root. Without "{htc}", SteamVR attempts to load it from resources/rendermodels/
// instead of drivers/htc/resources/rendermodels/, failing with VRRenderModelError_InvalidModel (301).
std::string RenderModel(const std::string& configured) {
  static const std::string kOld = "{knuckles}";
  if (configured.rfind(kOld, 0) == 0) return "{indexcontroller}" + configured.substr(kOld.size());
  if (configured == "vr_tracker_vive_1_0") return "{htc}vr_tracker_vive_1_0";
  return configured;
}

struct DeviceStatic {
  std::string serial, model, manufacturer, renderModel, controllerRole;
  lh::DeviceRole role = lh::DeviceRole::Tracker;
  std::string baseChannel;  // base stations: "A" "b" "c", lhbs multi "1".."4"
  lh::ControllerType controllerType = lh::ControllerType::Index;
  bool lh2 = false;         // base stations: a 2.0 one
  uint8_t baseHardwareVersion = 0;
  uint16_t baseNonce = 0;
};

class Device : public ITrackedDeviceServerDriver {
 public:
  explicit Device(DeviceStatic info) : info_(std::move(info)) {
    pose_.qWorldFromDriverRotation = {1, 0, 0, 0};
    pose_.qDriverFromHeadRotation = {1, 0, 0, 0};
    pose_.qRotation = {1, 0, 0, 0};
    pose_.result = TrackingResult_Uninitialized;
  }

  EVRInitError Activate(uint32_t objectId) override {
    active_ = false;
    lastInputTime_ = lastPoseTime_ = -1;
    skeletonInput_ = {};
    lastSkeletonSent_ = -1e300;
    battery_ = -1; universe_ = 0; hadInput_ = false;
    std::fill(std::begin(sent_), std::end(sent_), false);
    std::fill(std::begin(handles_), std::end(handles_), k_ulInvalidInputComponentHandle);
    haptic_ = skeletonHandle_ = k_ulInvalidInputComponentHandle;
    id_ = objectId;
    container_ = VRProperties()->TrackedDeviceToPropertyContainer(objectId);
    SetProperties();
    if (info_.role == lh::DeviceRole::Controller) CreateInputs();
    active_ = true;
    return VRInitError_None;
  }
  void Deactivate() override {
    active_ = false;
    id_ = k_unTrackedDeviceIndexInvalid;
  }
  // A device the host activated through another driver (a headset from the stock driver,
  // stock_hmd.hpp): we only publish its pose; its properties are that driver's.
  void AttachExternal(uint32_t objectId, const std::atomic<bool>* alive) {
    id_ = objectId;
    container_ = VRProperties()->TrackedDeviceToPropertyContainer(objectId);
    externalAlive_ = alive;
    active_ = true;
  }
  // SteamVR's idle power-off (power/turnOffControllersTimeout) calls this; the stock
  // driver powers the device off here when it can (Prop_DeviceCanPowerOff), and so do we.
  void EnterStandby() override {
    const bool canPowerOff = info_.role == lh::DeviceRole::Controller || info_.role == lh::DeviceRole::Tracker;
    Log("SteamVR put " + info_.serial + " in standby" + (canPowerOff ? ": powering it off" : " (not a device we power off)"));
    if (canPowerOff) powerOffRequested_ = true;
  }
  void* GetComponent(const char*) override { return nullptr; }
  void DebugRequest(const char*, char* response, uint32_t size) override {
    if (size) response[0] = 0;
  }
  DriverPose_t GetPose() override {
    std::lock_guard lock(poseMutex_);
    return pose_;
  }

  bool Active() const { return active_ && (!externalAlive_ || *externalAlive_); }
  bool IsHmd() const { return info_.role == lh::DeviceRole::Hmd; }
  // "left_hand" / "right_hand" for a controller, else "".
  std::string Hand() const { return info_.role == lh::DeviceRole::Controller ? info_.controllerRole : std::string(); }
  std::string StringProperty(ETrackedDeviceProperty prop) const {
    if (!Active()) return {};
    char value[256] = {};
    ETrackedPropertyError error = TrackedProp_Success;
    VRProperties()->GetStringProperty(container_, prop, value, sizeof value, &error);
    return error == TrackedProp_Success ? std::string(value) : std::string();
  }
  const DeviceStatic& Info() const { return info_; }
  bool MatchesHaptic(const VREvent_t& event) const {
    return active_ && lh::openvr::MatchesHaptic(event, id_, container_, haptic_);
  }
  bool TakePowerOffRequest() { return powerOffRequested_.exchange(false); }

  void SetRawTransform(const Eigen::Isometry3d& t) { rawFromWorld_ = t; }
  // A base's channel (Prop_ModeLabel_String): known once its OOTX is, and changed with
  // its mode button.
  void SetChannel(const std::string& channel) {
    if (channel == info_.baseChannel) return;
    info_.baseChannel = channel;
    if (active_) VRProperties()->SetStringProperty(container_, Prop_ModeLabel_String, channel.c_str());
  }
  // 1.0 or 2.0 base station: model, render model and status icons to match.
  void SetLh2(bool lh2) {
    if (lh2 == info_.lh2) return;
    info_.lh2 = lh2;
    if (active_) BaseLook(container_);
  }
  void SetBaseMetadata(uint8_t hardwareVersion, uint16_t nonce) {
    if (hardwareVersion == info_.baseHardwareVersion && nonce == info_.baseNonce) return;
    info_.baseHardwareVersion = hardwareVersion;
    info_.baseNonce = nonce;
    if (active_) BaseMetadata(container_);
  }
  void BaseMetadata(PropertyContainerHandle_t c) {
    // Stock hardware IDs identify base stations with device family 9 in bits 24+.
    if (info_.baseHardwareVersion)
      VRProperties()->SetUint64Property(c, Prop_HardwareRevision_Uint64,
                                       uint64_t(9) << 24 | info_.baseHardwareVersion);
    VRProperties()->SetInt32Property(c, Prop_Nonce_Int32, info_.baseNonce);
  }
  void BaseLook(PropertyContainerHandle_t c) {
    auto* props = VRProperties();
    const bool two = info_.lh2;
    props->SetStringProperty(c, Prop_ManufacturerName_String, two ? "Valve" : "HTC");
    props->SetStringProperty(c, Prop_ModelNumber_String, two ? "Valve SR Imp" : "HTC V2-XD/XE");
    props->SetStringProperty(c, Prop_RenderModelName_String, two ? "lh_basestation_valve_gen2" : "lh_basestation_vive");
    const std::string icons = two ? "{lighthouse}/icons/base2_status_" : "{htc}/icons/base_status_";
    props->SetStringProperty(c, Prop_NamedIconPathDeviceOff_String, (icons + "off.png").c_str());
    props->SetStringProperty(c, Prop_NamedIconPathDeviceSearching_String, (icons + "searching.gif").c_str());
    props->SetStringProperty(c, Prop_NamedIconPathDeviceSearchingAlert_String, (icons + "searching_alert.gif").c_str());
    props->SetStringProperty(c, Prop_NamedIconPathDeviceReady_String, (icons + "ready.png").c_str());
    props->SetStringProperty(c, Prop_NamedIconPathDeviceReadyAlert_String, (icons + "ready_alert.png").c_str());
    props->SetStringProperty(c, Prop_NamedIconPathDeviceNotReady_String, (icons + "error.png").c_str());
    props->SetStringProperty(c, Prop_NamedIconPathDeviceStandby_String, (icons + "standby.png").c_str());
  }
  void SetUniverse(uint64_t universe) {
    if (!active_ || universe == universe_) return;
    universe_ = universe;
    VRProperties()->SetUint64Property(container_, Prop_CurrentUniverseId_Uint64, universe);
  }

  // Tracked devices: called from the driver thread with the latest liblh data.
  void Update(const lh::DeviceUpdate& u, double now) {
    if (!Active()) return;
    DriverPose_t p{};
    const Eigen::Quaterniond rawRotation(rawFromWorld_.rotation());
    p.qWorldFromDriverRotation = {rawRotation.w(), rawRotation.x(), rawRotation.y(), rawRotation.z()};
    for (int i = 0; i < 3; ++i) p.vecWorldFromDriverTranslation[i] = rawFromWorld_.translation()[i];
    const auto& pose = u.pose;
    p.qDriverFromHeadRotation = Quat(pose.imuFromHead.orientation);
    for (int i = 0; i < 3; ++i) p.vecDriverFromHeadTranslation[i] = pose.imuFromHead.position[size_t(i)];
    p.qRotation = Quat(pose.worldFromImu.orientation);
    for (int i = 0; i < 3; ++i) {
      p.vecPosition[i] = pose.worldFromImu.position[size_t(i)];
      p.vecVelocity[i] = pose.velocity[size_t(i)];
      p.vecAcceleration[i] = pose.acceleration[size_t(i)];  // SteamVR predicts with it (swings curve)
    }
    // Angular velocity in the device (IMU) frame, as the stock driver reports it.
    const auto& o = pose.worldFromImu.orientation;
    const Eigen::Quaterniond q(o[0], o[1], o[2], o[3]);
    const Eigen::Vector3d w = q.conjugate() * Eigen::Vector3d(pose.angularVelocity[0], pose.angularVelocity[1],
                                                             pose.angularVelocity[2]);
    for (int i = 0; i < 3; ++i) p.vecAngularVelocity[i] = w[i];
    p.poseTimeOffset = pose.valid ? std::min(0.0, pose.hostTime - now) : 0.0;
    p.result = static_cast<ETrackingResult>(int(pose.result));
    p.poseIsValid = pose.valid;
    p.deviceIsConnected = u.connected && u.enabled;
    p.willDriftInYaw = false;
    p.shouldApplyHeadModel = false;
    bool held = false;
    if (pose.valid) {
      held_ = p;
      haveHeld_ = true;
    } else if (haveHeld_ && p.deviceIsConnected) {
      // Lost while on: held where it was last seen, as the stock driver does, until
      // light finds it again. liblh's lost record is an identity pose, which a host
      // shows at the origin (Monado: any invalid pose is the identity).
      p = held_;
      for (int i = 0; i < 3; ++i) p.vecVelocity[i] = p.vecAngularVelocity[i] = p.vecAcceleration[i] = p.vecAngularAcceleration[i] = 0;
      p.qWorldFromDriverRotation = {rawRotation.w(), rawRotation.x(), rawRotation.y(), rawRotation.z()};
      for (int i = 0; i < 3; ++i) p.vecWorldFromDriverTranslation[i] = rawFromWorld_.translation()[i];
      p.poseTimeOffset = 0;
      p.result = TrackingResult_Running_OutOfRange;
      p.poseIsValid = true;
      held = true;
    } else if (!p.deviceIsConnected) {
      haveHeld_ = false;  // switched off: it comes back wherever it is
    }
    const bool changed = pose.hostTime != lastPoseTime_ || p.poseIsValid != lastValid_ ||
                         p.deviceIsConnected != lastConnected_ || p.result != lastResult_ ||
                         !rawFromWorld_.matrix().isApprox(publishedRaw_.matrix(), 1e-12);
    if (changed) {
      publishedRaw_ = rawFromWorld_;
      lastPoseTime_ = pose.hostTime;
      lastValid_ = p.poseIsValid;
      lastConnected_ = p.deviceIsConnected;
      lastResult_ = p.result;
      {
        std::lock_guard lock(poseMutex_);
        pose_ = p;
      }
      VRServerDriverHost()->TrackedDevicePoseUpdated(id_, p, sizeof(p));
      Record(p, now, held);
    }
    if (u.batteryPercent && (*u.batteryPercent != battery_ || u.charging != charging_)) {
      battery_ = *u.batteryPercent;
      charging_ = u.charging;
      VRProperties()->SetFloatProperty(container_, Prop_DeviceBatteryPercentage_Float, float(battery_) / 100.0f);
      VRProperties()->SetBoolProperty(container_, Prop_DeviceIsCharging_Bool, charging_);
    }
    if (info_.role == lh::DeviceRole::Controller) {
      if (u.input && p.deviceIsConnected) { UpdateInputs(*u.input, now); hadInput_ = true; }
      else if (hadInput_) { lh::ControllerInput neutral; neutral.hostTime = now; UpdateInputs(neutral, now); hadInput_ = false; }
      if (now - lastSkeletonSent_ >= kSkeletonInterval) SendSkeleton(now);
    }
  }

  // Base stations: static pose from the room map.
  void UpdateBase(const lh::BaseStationState& b) {
    if (!active_) return;
    DriverPose_t p{};
    const Eigen::Quaterniond rawRotation(rawFromWorld_.rotation());
    p.qWorldFromDriverRotation = {rawRotation.w(), rawRotation.x(), rawRotation.y(), rawRotation.z()};
    for (int i = 0; i < 3; ++i) p.vecWorldFromDriverTranslation[i] = rawFromWorld_.translation()[i];
    p.qDriverFromHeadRotation = {1, 0, 0, 0};
    p.qRotation = Quat(b.worldFromBase.orientation);
    for (int i = 0; i < 3; ++i) p.vecPosition[i] = b.worldFromBase.position[size_t(i)];
    // Until it's locked (or is the origin) it shows as searching (a blinking icon): its
    // place can still move. Learned but not locked yet: calibrating.
    const bool settled = b.known && (b.origin || b.locked);
    p.result = !b.known ? TrackingResult_Uninitialized : settled ? TrackingResult_Running_OK
                                                                 : TrackingResult_Calibrating_InProgress;
    p.poseIsValid = settled;
    p.deviceIsConnected = b.seen;
    // Room orientation/alignment can change even when the base position doesn't.
    {
      std::lock_guard lock(poseMutex_);
      pose_ = p;
    }
    VRServerDriverHost()->TrackedDevicePoseUpdated(id_, p, sizeof(p));
  }

 private:
  // At most 100 Hz per device, but every change of state.
  void Record(const DriverPose_t& p, double now, bool held) {
    if (!g_recorder.On()) return;
    const bool stateChanged = p.poseIsValid != recValid_ || p.result != recResult_ || held != recHeld_ ||
                              p.deviceIsConnected != recConnected_;
    if (!stateChanged && now - recTime_ < 0.01) return;
    recTime_ = now; recValid_ = p.poseIsValid; recResult_ = p.result; recHeld_ = held; recConnected_ = p.deviceIsConnected;
    char text[512];
    std::snprintf(text, sizeof text,
                  "{\"t\":%.6f,\"pose\":%s,\"valid\":%d,\"held\":%d,\"connected\":%d,\"result\":%d,"
                  "\"p\":[%.5f,%.5f,%.5f],\"q\":[%.6f,%.6f,%.6f,%.6f],\"v\":[%.4f,%.4f,%.4f],\"w\":[%.4f,%.4f,%.4f],"
                  "\"offset\":%.4f,\"world\":[%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f]}",
                  now, Recorder::Quote(info_.serial).c_str(), p.poseIsValid, held, p.deviceIsConnected, int(p.result),
                  p.vecPosition[0], p.vecPosition[1], p.vecPosition[2], p.qRotation.w, p.qRotation.x, p.qRotation.y,
                  p.qRotation.z, p.vecVelocity[0], p.vecVelocity[1], p.vecVelocity[2], p.vecAngularVelocity[0],
                  p.vecAngularVelocity[1], p.vecAngularVelocity[2], p.poseTimeOffset, p.vecWorldFromDriverTranslation[0],
                  p.vecWorldFromDriverTranslation[1], p.vecWorldFromDriverTranslation[2], p.qWorldFromDriverRotation.w,
                  p.qWorldFromDriverRotation.x, p.qWorldFromDriverRotation.y, p.qWorldFromDriverRotation.z);
    g_recorder.Line(text);
  }

  void SetProperties() {
    auto* props = VRProperties();
    const PropertyContainerHandle_t c = container_;
    props->SetStringProperty(c, Prop_TrackingSystemName_String, "lighthouse");
    props->SetStringProperty(c, Prop_SerialNumber_String, info_.serial.c_str());
    props->SetBoolProperty(c, Prop_WillDriftInYaw_Bool, false);
    props->SetBoolProperty(c, Prop_DeviceIsWireless_Bool, info_.role != lh::DeviceRole::BaseStation);
    props->SetBoolProperty(c, Prop_HasDisplayComponent_Bool, false);
    props->SetBoolProperty(c, Prop_HasCameraComponent_Bool, false);
    props->SetBoolProperty(c, Prop_HasDriverDirectModeComponent_Bool, false);
    props->SetBoolProperty(c, Prop_HasVirtualDisplayComponent_Bool, false);
    props->SetBoolProperty(c, Prop_Firmware_UpdateAvailable_Bool, false);
    props->SetBoolProperty(c, Prop_Firmware_ManualUpdate_Bool, false);
    props->SetBoolProperty(c, Prop_CanUnifyCoordinateSystemWithHmd_Bool, true);
    props->SetBoolProperty(c, Prop_DeviceCanPowerOff_Bool,
                           info_.role == lh::DeviceRole::Controller || info_.role == lh::DeviceRole::Tracker);
    switch (info_.role) {
      case lh::DeviceRole::Controller: {
        const bool left = info_.controllerRole == "left_hand";
        if (info_.controllerType == lh::ControllerType::ViveWand) {
          // As SteamVR's own wand: the htc resources (profile, render model, icons).
          props->SetStringProperty(c, Prop_ManufacturerName_String, info_.manufacturer.empty() ? "HTC" : info_.manufacturer.c_str());
          props->SetStringProperty(c, Prop_ModelNumber_String, info_.model.empty() ? "Vive. Controller MV" : info_.model.c_str());
          props->SetStringProperty(c, Prop_ResourceRoot_String, "htc");
          props->SetStringProperty(c, Prop_ControllerType_String, "vive_controller");
          props->SetStringProperty(c, Prop_InputProfilePath_String, "{htc}/input/vive_controller_profile.json");
          props->SetStringProperty(c, Prop_RenderModelName_String,
                                   info_.renderModel.empty() ? "vr_controller_vive_1_5" : info_.renderModel.c_str());
          props->SetStringProperty(c, Prop_RegisteredDeviceType_String, ("htc/vive_controller" + info_.serial).c_str());
          props->SetInt32Property(c, Prop_ControllerRoleHint_Int32,
                                  left ? TrackedControllerRole_LeftHand
                                       : info_.controllerRole == "right_hand" ? TrackedControllerRole_RightHand
                                                                              : TrackedControllerRole_Invalid);
          props->SetInt32Property(c, Prop_DeviceClass_Int32, TrackedDeviceClass_Controller);
          props->SetInt32Property(c, Prop_ControllerHandSelectionPriority_Int32, NextHandPriority());
          lastUse_ = std::chrono::steady_clock::now();
          props->SetInt32Property(c, Prop_Axis0Type_Int32, k_eControllerAxis_TrackPad);
          props->SetInt32Property(c, Prop_Axis1Type_Int32, k_eControllerAxis_Trigger);
          props->SetBoolProperty(c, Prop_DeviceProvidesBatteryStatus_Bool, true);
          props->SetBoolProperty(c, Prop_Identifiable_Bool, true);
          props->SetStringProperty(c, Prop_NamedIconPathDeviceOff_String, "{htc}/icons/controller_status_off.png");
          props->SetStringProperty(c, Prop_NamedIconPathDeviceSearching_String, "{htc}/icons/controller_status_searching.gif");
          props->SetStringProperty(c, Prop_NamedIconPathDeviceSearchingAlert_String, "{htc}/icons/controller_status_searching_alert.gif");
          props->SetStringProperty(c, Prop_NamedIconPathDeviceReady_String, "{htc}/icons/controller_status_ready.png");
          props->SetStringProperty(c, Prop_NamedIconPathDeviceReadyAlert_String, "{htc}/icons/controller_status_ready_alert.png");
          props->SetStringProperty(c, Prop_NamedIconPathDeviceNotReady_String, "{htc}/icons/controller_status_error.png");
          props->SetStringProperty(c, Prop_NamedIconPathDeviceStandby_String, "{htc}/icons/controller_status_standby.png");
          props->SetStringProperty(c, Prop_NamedIconPathDeviceAlertLow_String, "{htc}/icons/controller_status_ready_low.png");
          break;
        }
        props->SetStringProperty(c, Prop_ManufacturerName_String, info_.manufacturer.empty() ? "Valve" : info_.manufacturer.c_str());
        props->SetStringProperty(c, Prop_ModelNumber_String, info_.model.c_str());
        props->SetStringProperty(c, Prop_ResourceRoot_String, "indexcontroller");
        props->SetStringProperty(c, Prop_ControllerType_String, "knuckles");
        props->SetStringProperty(c, Prop_InputProfilePath_String, "{indexcontroller}/input/index_controller_profile.json");
        props->SetStringProperty(c, Prop_RenderModelName_String,
                                 info_.renderModel.empty()
                                     ? (left ? "{indexcontroller}valve_controller_knu_1_0_left" : "{indexcontroller}valve_controller_knu_1_0_right")
                                     : info_.renderModel.c_str());
        props->SetStringProperty(c, Prop_RegisteredDeviceType_String, ("valve/index_controller" + info_.serial).c_str());
        props->SetInt32Property(c, Prop_ControllerRoleHint_Int32,
                                left ? TrackedControllerRole_LeftHand
                                     : info_.controllerRole == "right_hand" ? TrackedControllerRole_RightHand
                                                                            : TrackedControllerRole_Invalid);
        props->SetInt32Property(c, Prop_DeviceClass_Int32, TrackedDeviceClass_Controller);
        // Two controllers for one hand: SteamVR's primary is the higher priority. The one
        // turned on last, then whichever is picked up and used (UpdateInputs).
        props->SetInt32Property(c, Prop_ControllerHandSelectionPriority_Int32, NextHandPriority());
        lastUse_ = std::chrono::steady_clock::now();
        props->SetInt32Property(c, Prop_Axis0Type_Int32, k_eControllerAxis_Joystick);
        props->SetInt32Property(c, Prop_Axis1Type_Int32, k_eControllerAxis_Trigger);
        props->SetInt32Property(c, Prop_Axis2Type_Int32, k_eControllerAxis_TrackPad);
        props->SetBoolProperty(c, Prop_DeviceProvidesBatteryStatus_Bool, true);
        props->SetBoolProperty(c, Prop_Identifiable_Bool, true);  // enables SteamVR's Identify button
        const std::string side = left ? "left" : "right";
        props->SetStringProperty(c, Prop_NamedIconPathDeviceOff_String, ("{indexcontroller}/icons/" + side + "_controller_status_off.png").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceSearching_String, ("{indexcontroller}/icons/" + side + "_controller_status_searching.gif").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceSearchingAlert_String, ("{indexcontroller}/icons/" + side + "_controller_status_searching_alert.gif").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceReady_String, ("{indexcontroller}/icons/" + side + "_controller_status_ready.png").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceReadyAlert_String, ("{indexcontroller}/icons/" + side + "_controller_status_ready_alert.png").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceNotReady_String, ("{indexcontroller}/icons/" + side + "_controller_status_error.png").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceStandby_String, ("{indexcontroller}/icons/" + side + "_controller_status_standby.png").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceAlertLow_String, ("{indexcontroller}/icons/" + side + "_controller_status_ready_low.png").c_str());
        break;
      }
      case lh::DeviceRole::BaseStation: {
        BaseLook(c);
        BaseMetadata(c);
        props->SetStringProperty(c, Prop_ResourceRoot_String, "htc");
        props->SetStringProperty(c, Prop_ModeLabel_String, info_.baseChannel.c_str());
        props->SetInt32Property(c, Prop_DeviceClass_Int32, TrackedDeviceClass_TrackingReference);
        props->SetFloatProperty(c, Prop_FieldOfViewLeftDegrees_Float, 60);
        props->SetFloatProperty(c, Prop_FieldOfViewRightDegrees_Float, 60);
        props->SetFloatProperty(c, Prop_FieldOfViewTopDegrees_Float, 60);
        props->SetFloatProperty(c, Prop_FieldOfViewBottomDegrees_Float, 60);
        props->SetFloatProperty(c, Prop_TrackingRangeMinimumMeters_Float, 0.1f);
        props->SetFloatProperty(c, Prop_TrackingRangeMaximumMeters_Float, 7.0f);
        break;
      }
      default: {  // trackers (inputs ignored by design)
        props->SetStringProperty(c, Prop_ManufacturerName_String, info_.manufacturer.empty() ? "HTC" : info_.manufacturer.c_str());
        props->SetStringProperty(c, Prop_ModelNumber_String, info_.model.empty() ? "VIVE Tracker" : info_.model.c_str());
        props->SetStringProperty(c, Prop_ResourceRoot_String, "htc");
        props->SetStringProperty(c, Prop_ControllerType_String, "vive_tracker");
        props->SetStringProperty(c, Prop_InputProfilePath_String, "{htc}/input/vive_tracker_profile.json");
        props->SetStringProperty(c, Prop_RenderModelName_String,
                                 info_.renderModel.empty() ? "{htc}vr_tracker_vive_1_0" : info_.renderModel.c_str());
        props->SetStringProperty(c, Prop_RegisteredDeviceType_String, ("htc/vive_tracker" + info_.serial).c_str());
        props->SetInt32Property(c, Prop_ControllerRoleHint_Int32, TrackedControllerRole_OptOut);
        props->SetInt32Property(c, Prop_DeviceClass_Int32, TrackedDeviceClass_GenericTracker);
        props->SetBoolProperty(c, Prop_DeviceProvidesBatteryStatus_Bool, true);
        props->SetBoolProperty(c, Prop_Identifiable_Bool, true);  // enables SteamVR's Identify button
        props->SetStringProperty(c, Prop_NamedIconPathDeviceOff_String, "{htc}/icons/tracker_status_off.png");
        props->SetStringProperty(c, Prop_NamedIconPathDeviceSearching_String, "{htc}/icons/tracker_status_searching.gif");
        props->SetStringProperty(c, Prop_NamedIconPathDeviceSearchingAlert_String, "{htc}/icons/tracker_status_searching_alert.gif");
        props->SetStringProperty(c, Prop_NamedIconPathDeviceReady_String, "{htc}/icons/tracker_status_ready.png");
        props->SetStringProperty(c, Prop_NamedIconPathDeviceReadyAlert_String, "{htc}/icons/tracker_status_ready_alert.png");
        props->SetStringProperty(c, Prop_NamedIconPathDeviceNotReady_String, "{htc}/icons/tracker_status_error.png");
        props->SetStringProperty(c, Prop_NamedIconPathDeviceStandby_String, "{htc}/icons/tracker_status_standby.png");
        props->SetStringProperty(c, Prop_NamedIconPathDeviceAlertLow_String, "{htc}/icons/tracker_status_ready_low.png");
        break;
      }
    }
  }

  enum Input {
    SystemClick, SystemTouch, AClick, ATouch, BClick, BTouch, TriggerClick, TriggerTouch, TriggerValue,
    GripTouch, GripValue, GripForce, StickClick, StickTouch, StickX, StickY, PadTouch, PadX, PadY, PadForce,
    FingerIndex, FingerMiddle, FingerRing, FingerPinky, PadClick, GripClick, MenuClick, InputCount
  };

  void CreateInputs() {
    auto* in = ComponentInput();
    const PropertyContainerHandle_t c = container_;
    // Each component's path, and any creation error, logged (Windows SteamVR refused one
    // boolean as "wrong type" without saying which).
    auto created = [&](Input i, const char* path, EVRInputError e) {
      paths_[i] = path;
      if (e != VRInputError_None) Log(info_.serial + ": creating " + path + " failed, error " + std::to_string(int(e)));
    };
    auto b = [&](Input i, const char* path) {
      isBool_[i] = true;
      created(i, path, in->CreateBooleanComponent(c, path, &handles_[i]));
    };
    auto s1 = [&](Input i, const char* path) {
      created(i, path, in->CreateScalarComponent(c, path, &handles_[i], VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided));
    };
    auto s2 = [&](Input i, const char* path) {
      created(i, path, in->CreateScalarComponent(c, path, &handles_[i], VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided));
    };
    if (info_.controllerType == lh::ControllerType::ViveWand) {
      // The wand's components, as vive_controller_profile.json names them. No skeleton:
      // SteamVR estimates the hand from the buttons.
      b(SystemClick, "/input/system/click");
      b(MenuClick, "/input/application_menu/click");
      b(GripClick, "/input/grip/click");
      b(TriggerClick, "/input/trigger/click");
      s1(TriggerValue, "/input/trigger/value");
      b(PadClick, "/input/trackpad/click");
      b(PadTouch, "/input/trackpad/touch");
      s2(PadX, "/input/trackpad/x");
      s2(PadY, "/input/trackpad/y");
      in->CreateHapticComponent(c, "/output/haptic", &haptic_);
      return;
    }
    b(SystemClick, "/input/system/click");
    b(SystemTouch, "/input/system/touch");
    b(AClick, "/input/a/click");
    b(ATouch, "/input/a/touch");
    b(BClick, "/input/b/click");
    b(BTouch, "/input/b/touch");
    b(TriggerClick, "/input/trigger/click");
    b(TriggerTouch, "/input/trigger/touch");
    s1(TriggerValue, "/input/trigger/value");
    b(GripTouch, "/input/grip/touch");
    s1(GripValue, "/input/grip/value");
    s1(GripForce, "/input/grip/force");
    b(StickClick, "/input/thumbstick/click");
    b(StickTouch, "/input/thumbstick/touch");
    s2(StickX, "/input/thumbstick/x");
    s2(StickY, "/input/thumbstick/y");
    b(PadTouch, "/input/trackpad/touch");
    s2(PadX, "/input/trackpad/x");
    s2(PadY, "/input/trackpad/y");
    s1(PadForce, "/input/trackpad/force");
    s1(FingerIndex, "/input/finger/index");
    s1(FingerMiddle, "/input/finger/middle");
    s1(FingerRing, "/input/finger/ring");
    s1(FingerPinky, "/input/finger/pinky");
    in->CreateHapticComponent(c, "/output/haptic", &haptic_);
    std::vector<std::string> paths;
    if (const char* p = std::getenv("LH_HAND_POSES")) paths.emplace_back(p);
    char resource[4096]{};
    if (VRResources()) VRResources()->GetResourceFullPath("{indexcontroller}/anims/vr_glove_right_poses.glb", "", resource, sizeof resource);
    if (*resource) paths.emplace_back(resource);
    paths.emplace_back(lh::platform::SteamVRDirectory() + "/drivers/indexcontroller/resources/anims/vr_glove_right_poses.glb");
    if (const char* home = std::getenv("HOME")) {
      for (const char* root : {"/.local/share/Steam/steamapps/common/SteamVR", "/.steam/steam/steamapps/common/SteamVR"})
        paths.emplace_back(std::string(home) + root + "/drivers/indexcontroller/resources/anims/vr_glove_right_poses.glb");
    }
    const bool left = info_.controllerRole == "left_hand";
    std::string error;
    bool loaded = false;
    for (const auto& path : paths) if (skeleton_.Load(path, left, &error)) { loaded = true; break; }
    if (loaded && !error.empty()) Log("skeleton: " + error);
    if (loaded) {
      const auto& bones = skeleton_.GripLimit();
      // Partial, as the stock Index driver reports it: each finger follows its own
      // capacitive sensor. Monado's steamvr_lh only exposes a skeleton that isn't
      // Estimated as (conforming) hand tracking; without it OpenXR apps and xrizer
      // fall back to their own estimate, middle, ring and pinky as one.
      const auto result = VRDriverInput()->CreateSkeletonComponent(c, left ? "/input/skeleton/left" : "/input/skeleton/right",
          left ? "/skeleton/hand/left" : "/skeleton/hand/right", "/pose/raw", VRSkeletalTracking_Partial,
          bones.data(), uint32_t(bones.size()), &skeletonHandle_);
      if (result != VRInputError_None) { skeletonHandle_ = 0; Log("skeleton creation failed: " + std::to_string(result)); }
    } else Log("skeleton unavailable: " + error);
  }

  // Monado's steamvr_lh stamps each skeleton update with the controller's latest pose
  // and, for the hand-tracking joints xrizer gives VRChat, extrapolates from the newest
  // one along its velocity. Sent only with input reports (30 ms - 1.5 s apart on
  // session-post3), the hand flew along a stale velocity while the controller pose itself
  // (WayVR's clock) was fine: so the skeleton also goes out with the poses.
  // The hand's springs and fades advance here, on the host clock, with the latest
  // input: stepped only when an input report came, they moved in jumps.
  void SendSkeleton(double now) {
    if (!skeletonHandle_) return;
    skeleton_.Update(skeletonInput_, lastSkeletonSent_ < -1e299 ? 0 : now - lastSkeletonSent_);
    lastSkeletonSent_ = now;
    for (bool withController : {true, false}) {
      const auto bones = skeleton_.Current(withController);
      VRDriverInput()->UpdateSkeletonComponent(
          skeletonHandle_, withController ? VRSkeletalMotionRange_WithController : VRSkeletalMotionRange_WithoutController,
          bones.data(), uint32_t(bones.size()));
    }
  }

  void UpdateInputs(const lh::ControllerInput& s, double now) {
    if (s.hostTime == lastInputTime_) return;
    lastInputTime_ = s.hostTime;
    if (active_ && info_.role == lh::DeviceRole::Controller &&
        (s.systemClick || s.aClick || s.bClick || s.triggerClick || s.thumbstickClick || s.trackpadClick ||
         s.gripClick || s.menuClick || s.trigger > 0.5)) {
      // Used after a minute of rest: this one is in the hand now (the other on a hip,
      // a shelf): it becomes the primary for its hand.
      const auto t = std::chrono::steady_clock::now();
      if (t - lastUse_ > std::chrono::seconds(60))
        VRProperties()->SetInt32Property(container_, Prop_ControllerHandSelectionPriority_Int32, NextHandPriority());
      lastUse_ = t;
    }
    // Clamped: the first report after activation can carry a host time from long before.
    const double offset = std::clamp(s.hostTime - now, -0.1, 0.0);
    // Squeeze: the Index reports finger proximity (middle/ring/pinky) and grip force.
    const double curl = (s.fingerMiddle + s.fingerRing + s.fingerPinky) / 3.0;
    const double grip = std::max(curl, s.gripForce);
    const double values[InputCount] = {
        double(s.systemClick), double(s.systemTouch), double(s.aClick), double(s.aTouch), double(s.bClick),
        double(s.bTouch), double(s.triggerClick), double(s.triggerTouch), s.trigger, double(curl > 0.7),
        grip, s.gripForce, double(s.thumbstickClick), double(s.thumbstickTouch), s.thumbstickX, s.thumbstickY,
        double(s.trackpadTouch), s.trackpadX, s.trackpadY, s.trackpadForce, s.fingerIndex, s.fingerMiddle,
        s.fingerRing, s.fingerPinky, double(s.trackpadClick), double(s.gripClick), double(s.menuClick)};
    auto* in = ComponentInput();
    if (skeletonHandle_) {
      skeletonInput_ = s;
      SendSkeleton(now);
    }
    for (int i = 0; i < InputCount; ++i) {
      if (handles_[i] == k_ulInvalidInputComponentHandle || (sent_[i] && values[i] == last_[i])) continue;
      const EVRInputError e = isBool_[i] ? in->UpdateBooleanComponent(handles_[i], values[i] != 0, offset)
                                         : in->UpdateScalarComponent(handles_[i], float(values[i]), offset);
      if (e != VRInputError_None && failLogged_[i] < 5) {
        ++failLogged_[i];
        Log(info_.serial + ": updating " + (paths_[i] ? paths_[i] : "?") + " (handle " + std::to_string(handles_[i]) +
            ") value " + std::to_string(values[i]) + " offset " + std::to_string(offset) + " -> error " +
            std::to_string(int(e)));
      }
      last_[i] = values[i];
      sent_[i] = true;
    }
  }

  Eigen::Isometry3d rawFromWorld_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d publishedRaw_ = Eigen::Isometry3d::Identity();
  bool hadInput_ = false;
  DeviceStatic info_;
  std::atomic<bool> active_{false};
  const std::atomic<bool>* externalAlive_ = nullptr;  // AttachExternal: the host still has it
  std::atomic<bool> powerOffRequested_{false};
  std::chrono::steady_clock::time_point lastUse_{};
  static int32_t NextHandPriority() {
    static std::atomic<int32_t> next{1000};
    return ++next;
  }
  std::atomic<uint32_t> id_{k_unTrackedDeviceIndexInvalid};
  PropertyContainerHandle_t container_ = k_ulInvalidPropertyContainer;
  std::mutex poseMutex_;
  DriverPose_t pose_{};
  double recTime_ = -1e300;
  bool recValid_ = false, recHeld_ = false, recConnected_ = false;
  ETrackingResult recResult_ = TrackingResult_Uninitialized;
  DriverPose_t held_{};    // the last valid pose: what a device that loses tracking keeps
  bool haveHeld_ = false;
  double lastPoseTime_ = -1;
  bool lastValid_ = false, lastConnected_ = false;
  ETrackingResult lastResult_ = TrackingResult_Uninitialized;
  int battery_ = -1;
  bool charging_ = false;
  uint64_t universe_ = 0;
  VRInputComponentHandle_t handles_[InputCount] = {};
  bool isBool_[InputCount] = {};
  const char* paths_[InputCount] = {};
  int failLogged_[InputCount] = {};
  double last_[InputCount] = {};
  bool sent_[InputCount] = {};
  VRInputComponentHandle_t haptic_ = k_ulInvalidInputComponentHandle;
  double lastInputTime_ = -1;
  double lastSkeletonSent_ = -1e300;
  lh::ControllerInput skeletonInput_;
  static constexpr double kSkeletonInterval = 0.005;  // s: at most 200 Hz, with the poses
  lh::openvr::HandSkeleton skeleton_;
  VRInputComponentHandle_t skeletonHandle_ = 0;
};

class Provider : public IServerTrackedDeviceProvider {
 public:
  EVRInitError Init(IVRDriverContext* context) override {
    VR_INIT_SERVER_DRIVER_CONTEXT(context);
    context_ = context;
    lh::SessionOptions options;
    if (const char* replay = std::getenv("LH_REPLAY")) options.replayPath = replay;
    if (const char* room = std::getenv("LH_ROOM_MAP")) options.roomMapPath = room;
    // Our room map is the only source of base poses: SteamVR's rooms are imported only
    // on request (LH_STEAM_DB=auto or a path), and the host frame is ours unless
    // LH_HOST_FRAME=steamvr pins it to lighthousedb.json's anchor.
    options.steamDatabasePath = "-";
    if (const char* db = std::getenv("LH_STEAM_DB"); db && *db)
      options.steamDatabasePath = std::string(db) == "auto" ? "" : db;
    const char* hostFrame = std::getenv("LH_HOST_FRAME");
    steamFrame_ = hostFrame && std::string(hostFrame) == "steamvr";
    std::string captureDir;
    // Or "capture" in steamvr.vrsettings under driver_lighthouse (or driver_openhouse): on
    // Windows a new environment variable reaches SteamVR only after Steam restarts.
    // SteamVR only: older Monado's steamvr_lh (Envision's) aborts on a GetString of a
    // key missing from default.vrsettings, and Monado/WiVRn see LH_CAPTURE anyway.
    std::string captureSetting;
    const bool steamvr = HostName().rfind("vrserver", 0) == 0;
    if (steamvr) for (const char* section : {"driver_lighthouse", "driver_openhouse"}) {
      char value[1024]{};
      VRSettings()->GetString(section, "capture", value, sizeof value);
      if (*value) { captureSetting = value; break; }
    }
    const char* c = std::getenv("LH_CAPTURE");
    if (!(c && *c) && !captureSetting.empty()) c = captureSetting.c_str();
    if (c && *c) {
      // Never overwrite an earlier session: a used directory gets a numbered sibling.
      captureDir = c;
      for (int n = 2; std::filesystem::exists(captureDir); ++n) captureDir = std::string(c) + "-" + std::to_string(n);
      std::error_code ec;
      std::filesystem::create_directories(captureDir, ec);
      if (ec || !g_recorder.Open(captureDir + "/driver.jsonl")) {
        Log("LH_CAPTURE: cannot create " + captureDir);
        captureDir.clear();
      } else if (options.replayPath.empty()) {
        options.capturePath = captureDir + "/input.lhraw";  // a replay already is one
      }
    }
    // Keep the host's chaperone frame separate from our optimizable room frame.
    std::string reference;
    if (const char* p = std::getenv("LH_REFERENCE_DB")) reference = p;
    else reference = lh::platform::SteamConfigDirectory() + "/lighthousedb.json";
    std::string alignmentError;
    if (steamFrame_ && !reference.empty() && !alignment_.Load(reference, &alignmentError)) Log(alignmentError);
    if (!steamFrame_) {
      const std::string roomMap = options.roomMapPath.empty() ? lh::platform::ConfigDirectory() + "/roommap.json"
                                                              : options.roomMapPath;
      hostFrame_.Load(std::filesystem::path(roomMap).parent_path().string() + "/hostframe.json");
    }
    Log(steamFrame_ ? "host frame: SteamVR's (" + reference + ")" : "host frame: our room's, pinned to its origin base");
    if (!captureDir.empty()) {
      std::error_code ec;
      std::filesystem::copy_file(reference, captureDir + "/lighthousedb.json", ec);
      Log("recording a diagnostic session to " + captureDir);
    }
    session_ = lh::Session::Create(options);
    std::string error;
    if (!session_->Start(&error)) {
      Log("could not start: " + error);
      session_.reset();
      return VRInitError_Init_HmdNotFound;
    }
    ApplyTuning(true);
    Log("openhouse " + std::string(lh::kVersion) + ": liblh session started (" + std::string(options.replayPath.empty() ? "live USB" : "replay " + options.replayPath) +
        "); head-mounted displays: tracked by us, displayed through the stock driver when one is present");
    Log(std::string("input components through ") +
        (ComponentInput() != VRDriverInput() ? "IVRDriverInput_005" : IVRDriverInput_Version));
    // SteamVR's dongle pairing menu: ours owns the dongles (registered before the stock
    // driver loads, which is kept off this mailbox).
    if (options.replayPath.empty()) pairing_.Open(context_, [](const std::string& text) { Log(text); });
    {
      std::lock_guard lock(radioMutex_);
      radioSession_ = session_.get();
    }
    stock_.SetRequestHandler([this](const char* request, char* response, uint32_t size) {
      return RadioRequest(request, response, size);
    });
    // A headset on USB: bring up the stock driver for its display now, inside the host's
    // device discovery window.
    using Presence = lh::openvr::StockHmd::Presence;
    std::string which;
    std::vector<std::string> maybe;
    std::string unwanted;
    const bool noStock = StockHmdUnwanted(&unwanted);
    Presence presence = noStock ? Presence::No : lh::openvr::StockHmd::HeadsetPresent(&which, &maybe);
    if (noStock && unwanted != "LH_NO_STOCK_HMD") Log("lighthouse headsets aren't displayed here (" + unwanted + ")");
    if (presence == Presence::Maybe) {
      // Ask the device configs (role from device_class), up to 2 s: inside Monado's 3 s.
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
      std::set<std::string> pending(maybe.begin(), maybe.end());
      while (presence == Presence::Maybe && !pending.empty() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        for (const auto& d : session_->Snapshot().devices) {
          if (!pending.count(d.info.serial)) continue;
          pending.erase(d.info.serial);
          if (d.info.role == lh::DeviceRole::Hmd) {
            presence = Presence::Yes;
            which = d.info.serial + " (headset by its config)";
            lh::openvr::StockHmd::AllowHeadset(d.info.serial);
          }
        }
      }
      if (presence != Presence::Yes) {
        Log(pending.empty() ? "wired lighthouse devices are not headsets (by their configs)"
                            : "no config from a wired lighthouse device within 2 s; not loading the stock driver now");
        presence = Presence::No;
      }
    }
    if (presence == Presence::Yes) {
      triedStock_ = true;
      const std::string path = lh::openvr::StockHmd::DefaultPath();
      if (path.empty()) Log("headset " + which + " on USB, but SteamVR's own lighthouse driver wasn't found to display it (" + std::string(kStockFile) + " or SteamVR's drivers/lighthouse)");
      else {
        Log("headset " + which + " on USB: loading the stock driver for its display");
        stock_.Load(path, context_, [](const std::string& text) { Log(text); });
      }
    }
    if (const int port = lh::telemetry::PortFromEnvironment(); port > 0) {
      std::string telemetryError;
      if (g_telemetry.Start(port, HostName(), &telemetryError)) Log(lh::LogLevel::Debug, "viewer telemetry on 127.0.0.1:" + std::to_string(port));
      else Log(lh::LogLevel::Warn, "viewer telemetry unavailable: " + telemetryError);
    }
    stop_ = false;
    thread_ = std::thread([this] { Main(); });
    return VRInitError_None;
  }

  void Cleanup() override {
    // SteamVR's "Turn off controllers when exiting SteamVR" (power/powerOffOnExit,
    // default on): the stock driver powers its wireless devices off at shutdown; so do
    // we, while the USB thread still runs to send it.
    if (session_) {
      EVRSettingsError error = VRSettingsError_None;
      const bool powerOff = VRSettings() ? VRSettings()->GetBool("power", "powerOffOnExit", &error) : false;
      if ((error == VRSettingsError_None ? powerOff : true) && !std::getenv("LH_NO_POWER_OFF_ON_EXIT")) {
        int queued = 0;
        for (const auto& d : session_->Snapshot().devices)
          if (d.info.connected && (d.info.role == lh::DeviceRole::Controller || d.info.role == lh::DeviceRole::Tracker))
            queued += session_->PowerOff(d.info.serial);
        if (queued) {
          Log("SteamVR exiting: powering off " + std::to_string(queued) + " device(s)");
          std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
      }
    }
    stop_ = true;
    if (thread_.joinable()) thread_.join();
    if (reportThread_.joinable()) reportThread_.join();  // a report being saved finishes first
    g_telemetry.Stop();
    {
      std::lock_guard lock(radioMutex_);  // waits for a request being answered
      radioSession_ = nullptr;
    }
    if (session_) session_->Stop();
    session_.reset();
    g_recorder.Close();
    devices_.clear();
    snapshot_ = {};
    alignment_ = {};
    roomPublisher_.Close();
    warnedRoom_.clear();
    lastSnapshot_ = -1e300;
    { std::lock_guard lock(eventMutex_); hapticEvents_.clear(); }
    loggedHaptic_.clear();
    loggedUnrouted_ = false;
    stock_.Cleanup();
    VR_CLEANUP_SERVER_DRIVER_CONTEXT();
  }

  const char* const* GetInterfaceVersions() override { return k_InterfaceVersions; }

  void RunFrame() override {
    VREvent_t event;
    while (VRServerDriverHost()->PollNextEvent(&event, sizeof(event))) {
      stock_.PushEvent(event);  // the stock driver (headset display) sees the host's events too
      if (event.eventType == VREvent_Input_HapticVibration) {
        std::lock_guard lock(eventMutex_);
        if (hapticEvents_.size() < 64) hapticEvents_.push_back(event);
      }
    }
    // A headset is attached: bring up the stock driver for its display (host thread).
    if (wantStock_ && !triedStock_) {
      triedStock_ = true;
      const std::string path = lh::openvr::StockHmd::DefaultPath();
      std::string unwanted;
      if (StockHmdUnwanted(&unwanted)) Log("a head-mounted display is attached; not exposed (" + unwanted + ")");
      else if (path.empty()) Log("a head-mounted display is attached, but SteamVR's own lighthouse driver wasn't found to display it (" + std::string(kStockFile) + " or SteamVR's drivers/lighthouse)");
      else stock_.Load(path, context_, [](const std::string& text) { Log(text); });
    }
    stock_.RunFrame();
    if (session_) pairing_.Poll(*session_, ExpectedControllerType());
  }

  // The headset's expected controllers (Prop_ExpectedControllerType_String), "" if none.
  std::string ExpectedControllerType() const {
    for (const auto& [serial, device] : devices_)
      if (device && device->IsHmd() && device->Active()) return device->StringProperty(Prop_ExpectedControllerType_String);
    return {};
  }

  // Only leaving standby reaches the stock driver: it powers the headset's panels back up
  // there (the headset's own EnterStandby, through DeviceProxy, turned them off). Without
  // it a headset that went to sleep stayed dark (#20). Its provider EnterStandby isn't
  // forwarded: it would power off controllers it doesn't own here.
  bool ShouldBlockStandbyMode() override { return false; }
  void EnterStandby() override { Log("SteamVR entering standby"); }
  void LeaveStandby() override {
    Log("SteamVR leaving standby");
    stock_.LeaveStandby();
  }

 private:
  void Main() {
    while (!stop_) {
      const double now = session_->HostNow();
      for (auto& e : session_->PollEvents()) {
        Log(e.level, e.text, e.serial, now);
        if (g_recorder.On()) {
          char t[32];
          std::snprintf(t, sizeof t, "%.6f", now);
          g_recorder.Line(std::string("{\"t\":") + t + ",\"level\":\"" + lh::LogLevelName(e.level) + "\",\"event\":" +
                          Recorder::Quote(e.text) + "}");
        }
      }
      std::deque<VREvent_t> haptics;
      { std::lock_guard lock(eventMutex_); haptics.swap(hapticEvents_); }
      for (const auto& event : haptics) {
        bool matched = false;
        if (g_recorder.On()) {
          const auto& h = event.data.hapticVibration;
          std::string target;
          for (const auto& [serial, d] : devices_) if (d->MatchesHaptic(event)) target = serial;
          char text[320];
          std::snprintf(text, sizeof text,
                        "{\"t\":%.6f,\"haptic\":{\"device\":%u,\"container\":%llu,\"component\":%llu,\"duration\":%.6g,"
                        "\"frequency\":%.6g,\"amplitude\":%.6g,\"age\":%.4f},\"routed\":%s}",
                        now, event.trackedDeviceIndex, (unsigned long long)h.containerHandle,
                        (unsigned long long)h.componentHandle, h.fDurationSeconds, h.fFrequency, h.fAmplitude,
                        event.eventAgeSeconds, target.empty() ? "null" : Recorder::Quote(target).c_str());
          g_recorder.Line(text);
        }
        for (const auto& [serial, d] : devices_) if (d->MatchesHaptic(event)) {
          matched = true;
          const auto& h = event.data.hapticVibration;
          // Values go to the transport unchanged; HapticReport ports stock 0x3f4e50,
          // including duration 0 (one pulse, as SteamVR Identify sends every frame).
          // A zero amplitude is a no-op here (as in Monado's steamvr_lh): forwarding it
          // as a stop would replace a still-pending pulse in the transport queue.
          if (!(h.fAmplitude > 0)) continue;
          if (!loggedHaptic_.count(serial)) {
            loggedHaptic_.insert(serial);
            char details[160];
            std::snprintf(details, sizeof details, " (first event: duration=%.6g frequency=%.6g amplitude=%.6g)", h.fDurationSeconds, h.fFrequency, h.fAmplitude);
            Log("haptics routed to " + serial + details);
          }
          const bool queued = session_->Haptic(serial, h.fFrequency, h.fAmplitude, h.fDurationSeconds);
          char details[160];
          std::snprintf(details, sizeof details, " duration=%.6g frequency=%.6g amplitude=%.6g", h.fDurationSeconds, h.fFrequency, h.fAmplitude);
          Log(lh::LogLevel::Debug, "haptic " + serial + (queued ? " queued" : " ignored (replay/offline/disabled)") + details, serial);
        }
        if (!matched) {
          const auto& h = event.data.hapticVibration;
          Log(loggedUnrouted_.exchange(true) ? lh::LogLevel::Debug : lh::LogLevel::Info, "unrouted haptic: device=" + std::to_string(event.trackedDeviceIndex) + " container=" +
              std::to_string(h.containerHandle) + " component=" + std::to_string(h.componentHandle));
        }
      }
      for (const auto& [serial, d] : devices_) if (d->TakePowerOffRequest()) {
        const bool queued = session_->PowerOff(serial);
        Log("power-off " + serial + (queued ? " queued" : " ignored (replay/offline/disabled)"));
      }
      const auto updates = session_->Updates();
      // Base stations, configs and the room only change slowly.
      if (now - lastSnapshot_ > 0.25 || devices_.empty()) {
        lastSnapshot_ = now;
        snapshot_ = session_->Snapshot();
        RegisterNewDevices();
        for (const auto& d : snapshot_.devices)
          if (d.info.role == lh::DeviceRole::Hmd && d.info.enabled && d.info.connected) wantStock_ = true;
        while (const auto hmd = stock_.TakeActivated()) {
          // The stock driver's headset is up: our tracking drives its pose.
          DeviceStatic info{hmd->serial, {}, {}, {}, {}, lh::DeviceRole::Hmd, {}};
          auto device = std::make_unique<Device>(info);
          device->AttachExternal(hmd->objectId, hmd->alive);
          devices_[hmd->serial] = std::move(device);
          stock_.OwnPose(hmd->objectId);
          Log("head-mounted display " + hmd->serial + ": pose from our tracking");
        }
        const bool aligned = steamFrame_ ? alignment_.Update(snapshot_) : !snapshot_.roomId.empty();
        const Eigen::Isometry3d rawFromWorld = steamFrame_ ? alignment_.RawFromWorld() : hostFrame_.Update(snapshot_, now);
        if (steamFrame_ && !aligned && !snapshot_.roomId.empty() && snapshot_.roomId != warnedRoom_) {
          warnedRoom_ = snapshot_.roomId;
          Log("no shared reference base for room " + snapshot_.roomId + "; host room setup is required for a standing origin");
        }
        const uint64_t universe = UniverseId(aligned && steamFrame_ ? alignment_.RoomId() : snapshot_.roomId);
        roomPublisher_.Publish(snapshot_, rawFromWorld, now, session_->RoomResets());
        if (const std::string line = roomPublisher_.TakeLog(); !line.empty()) Log(line);
        for (auto& [serial, d] : devices_) {
          d->SetRawTransform(rawFromWorld);
          d->SetUniverse(universe);
        }
        for (const auto& b : snapshot_.bases) {
          auto found = devices_.find(b.serial);
          if (found != devices_.end()) found->second->UpdateBase(b);
        }
      }
      if (now - lastTuning_ >= 1.0) {  // the SteamVR settings page: applies while playing
        lastTuning_ = now;
        ApplyTuning(false);
      }
      if (now - lastTelemetry_ >= 0.05) {  // viewers: ~20 Hz is plenty to watch
        lastTelemetry_ = now;
        g_telemetry.PublishState(now, snapshot_, updates, session_->Receivers(),
                                 {roomPublisher_.Settled(), roomPublisher_.Epoch(), lh::openvr::RoomSettle::kMaxSigma,
                                  session_->RoomResets()});
      }
      for (const auto& u : updates) {
        auto found = devices_.find(u.serial);
        if (found != devices_.end()) found->second->Update(u, SteadyNowFor(now));
      }
      lh::platform::SleepFor(std::chrono::microseconds(500));
    }
  }

  double SteadyNowFor(double sessionNow) const {
    // Live sessions use the steady clock already; replays run on recorded host time.
    return session_ && std::getenv("LH_REPLAY") ? sessionNow : SteadyNow();
  }

  void RegisterNewDevices() {
    // SteamVR fills an empty hand slot with whichever controller comes next, also a
    // second one for the other hand (live: a right controller on a hip took the left
    // slot because the left one registered after it). So per batch: one controller per
    // free hand first (left, then right), then extra same-hand controllers, then trackers.
    std::set<std::string> handsTaken;
    for (const auto& [serial, device] : devices_)
      if (device && device->Hand().size()) handsTaken.insert(device->Hand());
    std::vector<const lh::DeviceState*> order;
    for (const auto& d : snapshot_.devices) order.push_back(&d);
    auto rank = [&](const lh::DeviceState* d) {
      if (d->info.role != lh::DeviceRole::Controller) return 3;
      if (d->info.controllerRole == "left_hand") return 0;
      if (d->info.controllerRole == "right_hand") return 1;
      return 2;
    };
    std::stable_sort(order.begin(), order.end(), [&](auto* a, auto* b) { return rank(a) < rank(b); });
    std::vector<const lh::DeviceState*> extras;
    for (auto* d : order) {
      const std::string& hand = d->info.controllerRole;
      if (d->info.role == lh::DeviceRole::Controller && !hand.empty() && !handsTaken.insert(hand).second) extras.push_back(d);
    }
    std::stable_partition(order.begin(), order.end(), [&](auto* d) { return std::find(extras.begin(), extras.end(), d) == extras.end(); });
    std::stable_partition(order.begin(), order.end(), [&](auto* d) { return d->info.role == lh::DeviceRole::Controller; });
    for (const auto* dp : order) {
      const auto& d = *dp;
      if (devices_.count(d.info.serial) || !d.info.enabled || d.info.role == lh::DeviceRole::Hmd) continue;
      DeviceStatic info{d.info.serial, d.info.model, d.info.manufacturer, RenderModel(d.info.renderModel), d.info.controllerRole,
                        d.info.role, {}, d.info.controllerType};
      Add(info, d.info.role == lh::DeviceRole::Controller ? TrackedDeviceClass_Controller
                                                          : TrackedDeviceClass_GenericTracker);
    }
    for (const auto& b : snapshot_.bases) {
      if (const auto found = devices_.find(b.serial); found != devices_.end()) {
        if (!b.channel.empty()) found->second->SetChannel(b.channel);
        found->second->SetLh2(b.lh2);
        found->second->SetBaseMetadata(b.hardwareVersion, b.nonce);
        continue;
      }
      if (!b.seen) continue;
      DeviceStatic info;
      info.serial = b.serial;
      info.role = lh::DeviceRole::BaseStation;
      info.baseChannel = b.channel;
      info.lh2 = b.lh2;
      info.baseHardwareVersion = b.hardwareVersion;
      info.baseNonce = b.nonce;
      Add(info, TrackedDeviceClass_TrackingReference);
    }
  }

  void Add(const DeviceStatic& info, ETrackedDeviceClass cls) {
    auto device = std::make_unique<Device>(info);
    Device* raw = device.get();
    devices_[info.serial] = std::move(device);
    // The host may call Activate() from inside this call (Monado does).
    if (!VRServerDriverHost()->TrackedDeviceAdded(info.serial.c_str(), cls, raw))
      Log("host declined " + info.serial + (cls == TrackedDeviceClass_TrackingReference ? " (base station)" : ""));
    else
      Log("registered " + info.serial);
  }

  // SteamVR's base station power management asks the headset to switch the Bluetooth
  // receiver's radio (usb/radio_mode.hpp). That receiver is ours, so ours answers, the way
  // the stock driver does (0x1eaa9). False: there is none, and the stock driver says so.
  bool RadioRequest(const char* request, char* response, uint32_t size) {
    if (!request || !response || !size) return false;
    const bool toBluetooth = !std::strcmp(request, "set_radio_mode_ble");
    const bool toValve = !std::strcmp(request, "set_radio_mode_valve");
    const bool read = !std::strcmp(request, "get_radio_mode");
    if (!toBluetooth && !toValve && !read) return false;
    std::lock_guard lock(radioMutex_);
    if (!radioSession_ || !radioSession_->HasBluetoothRadio()) return false;
    const char* answer = "0";
    if (read) {
      if (const auto on = radioSession_->BluetoothRadioOn()) answer = *on ? "B" : "V";
    } else if (radioSession_->SetBluetoothRadio(toBluetooth)) {
      answer = "1";
    }
    std::snprintf(response, size, "%s", answer);
    return true;
  }

  std::set<std::string> loggedHaptic_;
  std::atomic<bool> loggedUnrouted_{false};
  std::mutex eventMutex_;
  std::deque<VREvent_t> hapticEvents_;
  std::unique_ptr<lh::Session> session_;
  std::mutex radioMutex_;
  lh::Session* radioSession_ = nullptr;  // session_ while it can answer; under radioMutex_
  IVRDriverContext* context_ = nullptr;
  lh::openvr::StockHmd stock_;
  lh::openvr::PairingMailbox pairing_;
  std::atomic<bool> wantStock_{false};
  bool triedStock_ = false;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::map<std::string, std::unique_ptr<Device>> devices_;
  lh::SessionSnapshot snapshot_;
  lh::openvr::RawAlignment alignment_;
  bool steamFrame_ = false;
  lh::openvr::HostFrame hostFrame_;
  lh::openvr::RoomPublisher roomPublisher_;
  std::string warnedRoom_;
  double lastSnapshot_ = -1e300;
  double lastTelemetry_ = -1e300;
  std::string recording_;  // the settings page's recording, while on
  std::thread reportThread_;  // saving a bug report
  std::atomic<bool> reportDone_{true};

  // The bug report zip (lh/bug_report.hpp) with this recording, off the host thread.
  void SaveReport(const std::string& recording) {
    if (reportThread_.joinable() && reportDone_) reportThread_.join();
    if (reportThread_.joinable()) {
      Log("a bug report is still being saved; this recording stays in " + recording);
      return;
    }
    reportDone_ = false;
    lh::BugReportOptions options;
    options.summary = ReportSummary();
    options.recording = recording;
    reportThread_ = std::thread([this, options] {
      // The USB thread closes the capture on its next pass: let it finish the file.
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      std::string error;
      const std::string path = lh::SaveBugReport(options, &error);
      Log(path.empty() ? "bug report failed: " + error : "bug report saved: " + path);
      reportDone_ = true;
    });
  }

  // The devices, bases and room as they are now, for a bug report's info.txt.
  std::string ReportSummary() const {
    std::string out = "room " + (snapshot_.roomId.empty() ? std::string("(none)") : snapshot_.roomId) +
                      (roomPublisher_.Settled() ? ", settled" : ", not settled") + "\n\ndevices:\n";
    char line[256];
    for (const auto& d : snapshot_.devices) {
      std::snprintf(line, sizeof line, "  %s %s %s: %s, %s%s\n", d.info.serial.c_str(), lh::ToString(d.info.role),
                    d.info.model.c_str(), d.info.connected ? "connected" : "disconnected",
                    d.pose.valid ? "tracking" : "not tracking", d.info.enabled ? "" : ", disabled");
      out += line;
    }
    out += "\nbase stations:\n";
    for (const auto& b : snapshot_.bases) {
      std::snprintf(line, sizeof line, "  %s channel %s %s: %s%s%s, %.1f mm, %d keyframes\n", b.serial.c_str(),
                    b.channel.empty() ? "?" : b.channel.c_str(), b.lh2 ? "2.0" : "1.0", b.seen ? "seen" : "not seen",
                    !b.known ? ", not placed" : b.learned ? ", learned" : ", learning", b.locked ? ", locked" : "",
                    b.sigma * 1000, b.observations);
      out += line;
    }
    return out;
  }
  double lastTuning_ = -1e300;

  // SteamVR settings, section driver_openhouse (the page from settingsschema.vrsettings,
  // which the installer puts in the lighthouse driver's resources). Missing values are
  // written with their defaults first, so the page shows them.
  static constexpr const char* kSection = "driver_openhouse";
  void ApplyTuning(bool writeDefaults) {
    if (!session_ || !VRSettings()) return;
    auto number = [&](const char* key, float fallback) {
      EVRSettingsError e = VRSettingsError_None;
      const float v = VRSettings()->GetFloat(kSection, key, &e);
      if (e == VRSettingsError_None) return v;
      if (writeDefaults) VRSettings()->SetFloat(kSection, key, fallback);
      return fallback;
    };
    auto flag = [&](const char* key, bool fallback) {
      EVRSettingsError e = VRSettingsError_None;
      const bool v = VRSettings()->GetBool(kSection, key, &e);
      if (e == VRSettingsError_None) return v;
      if (writeDefaults) VRSettings()->SetBool(kSection, key, fallback);
      return fallback;
    };
    const double position = number("positionSmoothingMs", 60) / 1000.0;
    const double rotation = number("rotationSmoothingMs", 60) / 1000.0;
    const bool acceleration = flag("predictWithAcceleration", true);
    session_->SetOutputTuning(position, rotation, acceleration);
    // Record session: raw device data to <data>/recordings while on (replay with
    // `lhtool session`); turning it off saves the bug report zip with it.
    const bool record = flag("record", false);
    if (record != !recording_.empty()) {
      if (record) {
        const std::time_t t = std::time(nullptr);
        char name[64];
        std::strftime(name, sizeof name, "recording-%Y%m%d-%H%M%S.lhraw", std::localtime(&t));
        const std::string dir = lh::platform::DataDirectory() + "/recordings";
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        if (!ec && session_->SetCapture(dir + "/" + name)) {
          recording_ = dir + "/" + name;
          Log("recording to " + recording_);
        } else {
          Log("cannot record to " + dir);
          VRSettings()->SetBool(kSection, "record", false);
        }
      } else {
        session_->SetCapture("");
        Log("recording stopped: " + recording_ + "; saving the bug report");
        SaveReport(recording_);
        recording_.clear();
      }
    }
    // A one-shot switch: the settings page turns it on, we reset the room and turn it off.
    if (flag("resetRoom", false)) {
      VRSettings()->SetBool(kSection, "resetRoom", false);
      Log("room reset from the SteamVR settings");
      session_->ResetRoom();
    }
  }
};

Provider g_provider;

}  // namespace

#ifdef _WIN32
#define LH_DRIVER_EXPORT __declspec(dllexport)
#else
#define LH_DRIVER_EXPORT __attribute__((visibility("default")))
#endif
extern "C" LH_DRIVER_EXPORT void* HmdDriverFactory(const char* interfaceName, int* returnCode) {
  if (std::strcmp(interfaceName, IServerTrackedDeviceProvider_Version) == 0) return &g_provider;
  if (returnCode) *returnCode = VRInitError_Init_InterfaceNotFound;
  return nullptr;
}
