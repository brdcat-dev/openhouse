// lh::Session implementation: transport -> per-device trackers + shared room map.
#include "lh/session.hpp"
#include "platform/platform.hpp"

#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <condition_variable>
#include <mutex>
#include <sstream>
#include <thread>

#include "device_role.hpp"
#include "device_tracker.hpp"
#include "index_input.hpp"
#include "lh/input_sink.hpp"
#include "room_map.hpp"
#include "room_solver.hpp"
#include "tracking/tracked_object.hpp"

#ifdef LH_RUNTIME_USB
#include "raw_capture.hpp"
#include "usb_transport.hpp"
#endif

namespace lh {

const char* ToString(DeviceRole role) {
  switch (role) {
    case DeviceRole::Hmd: return "hmd";
    case DeviceRole::Controller: return "controller";
    case DeviceRole::Tracker: return "tracker";
    case DeviceRole::BaseStation: return "base_station";
  }
  return "?";
}

const char* ToString(TrackingState state) {
  switch (state) {
    case TrackingState::Disconnected: return "disconnected";
    case TrackingState::Uninitialized: return "uninitialized";
    case TrackingState::Acquiring: return "acquiring";
    case TrackingState::Tracking: return "tracking";
    case TrackingState::Coasting: return "coasting";
    case TrackingState::Lost: return "lost";
  }
  return "?";
}

namespace runtime {
namespace {
using nlohmann::json;

double SteadyNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string BaseName(uint32_t serial) {
  char text[16];
  std::snprintf(text, sizeof text, "LHB-%08X", serial);
  return text;
}

std::string DefaultRoomMapPath() { return platform::ConfigDirectory() + "/roommap.json"; }
std::string DefaultSettingsPath() { return platform::ConfigDirectory() + "/settings.json"; }
std::string DefaultPreinitDirectory() { return platform::ConfigDirectory() + "/preinit"; }

std::string FirstExisting(const std::vector<std::string>& paths) {
  for (const auto& p : paths)
    if (std::filesystem::exists(p)) return p;
  return {};
}

std::string ReadFile(const std::string& path) {
  std::ifstream in(path);
  std::stringstream s;
  s << in.rdbuf();
  return s.str();
}

Pose ToPose(const Eigen::Isometry3d& t) {
  Pose p;
  const Eigen::Quaterniond q(t.rotation());
  p.position = {t.translation().x(), t.translation().y(), t.translation().z()};
  p.orientation = {q.w(), q.x(), q.y(), q.z()};
  return p;
}

// Rotation with +X = plus_x, +Z = plus_z (same construction as the IMU block).
bool AxesRotation(const json& block, Eigen::Isometry3d* out) {
  if (!block.contains("plus_x") || !block.contains("plus_z")) return false;
  const auto px = block.at("plus_x").get<std::array<double, 3>>();
  const auto pz = block.at("plus_z").get<std::array<double, 3>>();
  Eigen::Vector3d x(px[0], px[1], px[2]), z(pz[0], pz[1], pz[2]);
  if (x.norm() < 1e-6 || z.norm() < 1e-6 || x.cross(z).norm() < 1e-6) return false;
  x.normalize(); z.normalize();
  Eigen::Matrix3d r;
  r.col(0) = x; r.col(1) = z.cross(x).normalized(); r.col(2) = x.cross(r.col(1)).normalized();
  *out = Eigen::Isometry3d::Identity();
  out->linear() = r;
  if (block.contains("position")) {
    const auto p = block.at("position").get<std::array<double, 3>>();
    out->translation() = Eigen::Vector3d(p[0], p[1], p[2]);
  }
  return true;
}

bool PoseArray(const json& a, Eigen::Isometry3d* out) {  // [qx qy qz qw x y z]
  if (!a.is_array() || a.size() != 7) return false;
  *out = Eigen::Isometry3d::Identity();
  out->linear() = Eigen::Quaterniond(a[3].get<double>(), a[0].get<double>(), a[1].get<double>(), a[2].get<double>())
                      .normalized().toRotationMatrix();
  out->translation() = Eigen::Vector3d(a[4].get<double>(), a[5].get<double>(), a[6].get<double>());
  return true;
}

std::string Lower(std::string s) {
  for (auto& c : s) c = char(std::tolower(static_cast<unsigned char>(c)));
  return s;
}
}  // namespace

namespace {
std::string ModelHint(const json& j) {
  return Lower(j.value("model_number", "") + " " + j.value("render_model", "") + " " + j.value("input_profile", ""));
}
bool IsIndexHint(const std::string& hint) {
  return hint.find("knuckles") != std::string::npos || hint.find("index") != std::string::npos ||
         hint.find("knu_") != std::string::npos;
}
// Vive wands: "Vive. Controller MV" (1.0), "VIVE Controller Pro MV" (2.0),
// render model vr_controller_vive_1_5. Not seen on hardware here; from their configs.
bool IsWandHint(const std::string& hint) {
  return hint.find("vive") != std::string::npos && hint.find("controller") != std::string::npos;
}
}  // namespace

DeviceRole RoleFromConfig(const std::string& configJson) {
  const json j = json::parse(configJson);
  const std::string cls = Lower(j.value("device_class", ""));
  if (cls == "hmd") return DeviceRole::Hmd;
  // As the stock driver: "generic_tracker" makes a controller a tracker (e.g. on a hip).
  if (Lower(j.value("tracked_controller_role", "")) == "generic_tracker") return DeviceRole::Tracker;
  if (cls == "controller") {
    const std::string hint = ModelHint(j);
    if (IsIndexHint(hint) || IsWandHint(hint)) return DeviceRole::Controller;
  }
  return DeviceRole::Tracker;
}

ControllerType ControllerTypeFromConfig(const std::string& configJson) {
  const std::string hint = ModelHint(json::parse(configJson));
  return !IsIndexHint(hint) && IsWandHint(hint) ? ControllerType::ViveWand : ControllerType::Index;
}

class SessionImpl final : public Session, public InputSinkBase {
 public:
  void OnTransportLog(LogLevel level, const std::string& text) override {
    std::lock_guard lock(mutex_);
    Emit(SessionEventType::Log, "", text, level);
  }
  explicit SessionImpl(SessionOptions options) : options_(std::move(options)) {}
  ~SessionImpl() override { Stop(); }

  bool Start(std::string* error) override;
  void Stop() override;
  bool Running() const override { return running_; }
  bool Finished() const override { return finished_; }
  SessionSnapshot Snapshot() const override;
  std::vector<DeviceUpdate> Updates() const override;
  double HostNow() const override {
    std::lock_guard lock(mutex_);
    if (options_.replayPath.empty() && running_) now_ = std::max(now_, SteadyNow());
    return now_;
  }
  std::vector<SessionEvent> PollEvents() override;
  void ResetRoom() override;
  void SetDeviceEnabled(const std::string& serial, bool enabled) override;
  std::vector<Receiver> Receivers() const override {
    std::vector<Receiver> out;
    if (usb_)
      for (const auto& r : usb_->Receivers()) out.push_back({r.serial, r.connected});
    return out;
  }
  bool SetCapture(const std::string& path) override {
#ifdef LH_RUNTIME_USB
    if (!usb_) return false;
    usb_->SetCapture(path);
    return true;
#else
    (void)path;
    return false;
#endif
  }
  bool SetPairing(const std::string& receiver, double seconds) override {
    return usb_ && usb_->SetPairing(receiver, int(std::lround(seconds * 1000)));
  }
  bool Haptic(const std::string& serial, double frequency, double amplitude, double duration) override;
  bool PowerOff(const std::string& serial) override;
  bool SaveRoom(std::string* error) override;

  // IInputSink (transport thread)
  void OnDeviceAttached(const DeviceId&, const std::string& configJson) override;
  // Device JSON -> tracker, role, input. False (and logged) if the config is unusable.
  bool SetUpDevice(const DeviceId& id, const std::string& config);
  // preinit/*.json: devices created at start, disconnected until they really connect.
  void LoadPreinit();
  void UpdatePreinit(const DeviceId& id, const std::string& config);
  void OnDeviceDetached(const DeviceId&) override;
  void OnClockOffset(const DeviceId&, double offset) override;
  void OnImu(const DeviceId&, const ImuSample&) override;
  void OnLightPulse(const DeviceId&, const LightPulse&) override;
  void OnLh2BeamWord(const DeviceId&, const Lh2BeamWord&) override;
  void OnLh2SensorWord(const DeviceId&, const Lh2SensorWord&) override;
  void OnInput(const DeviceId&, const InputReport&) override;
  void OnBattery(const DeviceId&, const BatteryReport&) override;

 private:
  struct Device {
    DeviceInfo info;
    std::unique_ptr<DeviceTracker> tracker;
    std::optional<ControllerInputDecoder> input;
    Eigen::Isometry3d imuFromHead = Eigen::Isometry3d::Identity();
    double clockOffset = 0;
    bool haveClock = false;
    bool attached = false;
    // Preinitialized and never attached yet: reported connected (uninitialized) so hosts
    // that only take devices at startup (Monado) keep it. Cleared on the first attach.
    bool preinitConnected = false;
    double lastData = -1e300;  // host time
    TrackingState lastState = TrackingState::Disconnected;
    // Light mode (live only): 0 = LH1 pulses, 1 = LH2 words; -1 not set yet.
    int lightMode = -1;
    double modeSince = 0, lastGoodLight = -1e300;  // host time
  };
  struct Base {
    BaseConfig config;
    BaseDynamicState dynamic;
    bool seen = false;
  };

  // level: as given, else ClassifyLog(text).
  void Emit(SessionEventType type, const std::string& serial, std::string text, std::optional<LogLevel> level = {});
  void Tick() { if (options_.replayPath.empty()) now_ = std::max(now_, SteadyNow()); }
  void Touch(Device& d, double deviceTime);
  void Process(const DeviceId& id, Device& d);
  void UpdateLightMode(const DeviceId& id, Device& d);
  void PushIdentities();
  void TakeKeyframes(Device& d);
  bool SolveDue() const;
  RoomSolveInput SolveInput();
  void ApplySolve(const RoomSolveOutput& out);
  void SolverMain();
  void LoadSettings();
  void SaveSettings();
  void SetOutputTuning(double positionSmoothing, double rotationSmoothing, bool reportAcceleration) override;
  bool Enabled(const DeviceId& id) const { return !disabled_.count(id); }
  void CollectLogs(Device* d);
  void MaybeSave();
  void ResetRoomLocked(const std::string& why);
  void WatchRoomFile(bool now = false);
  uint64_t RoomResets() const override { std::lock_guard lock(mutex_); return roomResets_; }
  void ReplayMain();
  DevicePose MakePose(const Device& d) const;
  double HostTime(const Device& d, double deviceTime) const { return deviceTime + d.clockOffset; }

  SessionOptions options_;
  mutable std::mutex mutex_;
  std::map<DeviceId, Device> devices_;
  std::map<uint32_t, Base> bases_;
  std::map<uint32_t, tdm::TdmDisambiguator::Identity> identities_;
  RoomMap room_;
  std::string roomPath_, settingsPath_, preinitDir_;
  std::map<std::string, std::string> preinitFiles_;
  uint64_t roomResets_ = 0;
  bool roomFileSeen_ = false;
  double lastRoomFileCheck_ = -1e300;  // serial -> preinit file
  bool explicitControllers_ = false;
  std::string leftController_, rightController_;
  std::set<std::string> disabled_;
  double smoothPosition_ = DeviceTracker::kDefaultSmoothing, smoothRotation_ = DeviceTracker::kDefaultSmoothing;
  bool reportAcceleration_ = true;
  std::map<std::string, std::string> roles_;  // settings.json "roles": serial -> tracked_controller_role
  bool persist_ = false;
  std::deque<SessionEvent> events_;
  mutable double now_ = 0;  // latest host time seen (live: steady clock)
  double lastSave_ = 0;
  std::atomic<bool> running_{false}, finished_{false}, stop_{false};
  std::atomic<bool> warming_{false};  // replay: before the window
  std::thread replay_;
  // Room solver.
  KeyframeStore keyframes_;
  std::map<std::string, std::shared_ptr<const TrackedObjectModel>> models_;
  bool newKeyframes_ = false;
  int addedKeyframes_ = 0;      // since the last SolveInput: the newest entries of the store
  bool syncSolve_ = false;
  double lastSolve_ = -1e300;   // session time of the last solve
  double solveInterval_ = 1.0;  // grows while solves stop moving the bases (see ApplySolve)
  std::map<uint32_t, Eigen::Isometry3d> lastSolved_;  // previous solve's bases
  double lastAdvance_ = -1e300;  // room bases gliding to solved poses
  std::condition_variable solverWake_;
  std::thread solver_;
#ifdef LH_RUNTIME_USB
  std::unique_ptr<usb::UsbTransport> usb_;
#endif
};

bool SessionImpl::Start(std::string* error) {
  if (running_) return true;
  const bool replay = !options_.replayPath.empty();
  std::string err;
  // Room map: replays use an in-memory map unless a path is given explicitly.
  roomPath_ = options_.roomMapPath.empty() && !replay ? DefaultRoomMapPath() : options_.roomMapPath;
  persist_ = options_.saveRoomMap && !roomPath_.empty();
  if (!roomPath_.empty() && !room_.Load(roomPath_, &err)) Emit(SessionEventType::Log, "", err);
  room_.SetFrozen(std::getenv("LH_EVALUATE_ROOM") != nullptr);
  settingsPath_ = options_.settingsPath.empty() && !replay ? DefaultSettingsPath() : options_.settingsPath;
  LoadSettings();
  for (const auto& serial : options_.disabledDevices) disabled_.insert(serial);
  preinitDir_ = options_.preinitDirectory == "-" ? "" : options_.preinitDirectory.empty() && !replay ? DefaultPreinitDirectory()
                                                                                                    : options_.preinitDirectory;
  LoadPreinit();
  std::string steamDb = options_.steamDatabasePath;
  if (steamDb.empty()) {
    std::vector<std::string> candidates;
    if (replay) candidates.push_back((std::filesystem::path(options_.replayPath).parent_path() / "lighthousedb.json").string());
    else {
      candidates.push_back(platform::SteamConfigDirectory() + "/lighthousedb.json");
#ifndef _WIN32
      candidates.push_back(platform::Home() + "/.steam/steam/config/lighthouse/lighthousedb.json");
      candidates.push_back(platform::Home() + "/.local/share/Steam/config/lighthouse/lighthousedb.json");
#endif
    }
    steamDb = FirstExisting(candidates);
  }
  if (!steamDb.empty() && steamDb != "-") {
    if (!room_.ImportSteamDatabase(ReadFile(steamDb), &err)) Emit(SessionEventType::Log, "", err);
  }
  {
    std::lock_guard lock(mutex_);
    for (const auto& [serial, c] : room_.KnownConfigs()) {
      bases_[serial] = {c.first, c.second, false};
      const auto frame = room_.OotxFrames().find(serial);
      if (frame != room_.OotxFrames().end()) identities_[serial] = {c.first, c.second, frame->second};
    }
    for (auto& text : room_.TakeLogs()) Emit(SessionEventType::Log, "", text);
  }
  stop_ = false;
  finished_ = false;
  syncSolve_ = replay && !options_.replayRealtime;
  if (!syncSolve_) solver_ = std::thread([this] { SolverMain(); });
  if (replay) {
#ifdef LH_RUNTIME_USB
    if (!std::filesystem::exists(options_.replayPath)) {
      if (error) *error = "capture not found: " + options_.replayPath;
      return false;
    }
    running_ = true;
    replay_ = std::thread([this] { ReplayMain(); });
#else
    if (error) *error = "built without the USB/capture layer";
    return false;
#endif
  } else {
#ifdef LH_RUNTIME_USB
    usb::UsbTransportOptions o;
    o.steamLighthouseConfigDir = options_.steamConfigDir;
    o.capturePath = options_.capturePath;
    o.onCommandResult = [this](const usb::DeviceCommandResult& result) {
      const bool routine = std::string(result.status) == "written";
      std::lock_guard lock(mutex_);
      std::ostringstream text;
      text << (result.powerOff ? "power-off " : "haptic ") << result.serial << ": " << result.status
           << " (HID bytes=" << result.bytes;
      if (!result.powerOff) text << ", on=" << (result.report[4] | result.report[5] << 8)
                                << ", off=" << (result.report[6] | result.report[7] << 8)
                                << ", repeats=" << (result.report[8] | result.report[9] << 8);
      text << ")";
      Emit(SessionEventType::Log, result.serial, text.str(), routine ? LogLevel::Debug : LogLevel::Info);
    };
    usb_ = std::make_unique<usb::UsbTransport>(this, o);
    if (!usb_->Start()) {
      usb_.reset();
      if (error) *error = "no Lighthouse USB devices could be opened (is SteamVR running?)";
      return false;
    }
    running_ = true;
#else
    if (error) *error = "built without the USB transport";
    return false;
#endif
  }
  if (error) error->clear();
  return true;
}

bool SessionImpl::Haptic(const std::string& serial, double frequency, double amplitude, double duration) {
#ifdef LH_RUNTIME_USB
  std::lock_guard lock(mutex_);
  const auto d = devices_.find(serial);
  if (!running_ || !usb_ || !Enabled(serial) || d == devices_.end() || !d->second.attached ||
      d->second.info.role != DeviceRole::Controller || !std::isfinite(frequency) ||
      !std::isfinite(amplitude) || !std::isfinite(duration)) return false;
  usb_->Haptic(serial, frequency, amplitude, duration);
  return true;
#else
  return false;
#endif
}

bool SessionImpl::PowerOff(const std::string& serial) {
#ifdef LH_RUNTIME_USB
  std::lock_guard lock(mutex_);
  const auto d = devices_.find(serial);
  if (!running_ || !usb_ || !Enabled(serial) || d == devices_.end() || !d->second.attached ||
      (d->second.info.role != DeviceRole::Controller && d->second.info.role != DeviceRole::Tracker)) return false;
  usb_->PowerOff(serial);
  return true;
#else
  return false;
#endif
}

void SessionImpl::Stop() {
  stop_ = true;
#ifdef LH_RUNTIME_USB
  std::unique_ptr<usb::UsbTransport> transport;
  { std::lock_guard lock(mutex_); transport = std::move(usb_); }
  if (transport) transport->Stop();  // callbacks take mutex_: never hold it here
#endif
  if (replay_.joinable()) replay_.join();
  solverWake_.notify_all();
  if (solver_.joinable()) solver_.join();
  if (running_ && persist_) {
    std::string err;
    std::lock_guard lock(mutex_);
    if (room_.Dirty() && room_.Save(roomPath_, &err)) room_.ClearDirty();
  }
  running_ = false;
}

void SessionImpl::ReplayMain() {
#ifdef LH_RUNTIME_USB
  usb::ReportDecoder decoder(this);
  usb::RawCaptureReader reader;
  if (!reader.Open(options_.replayPath)) {
    std::lock_guard lock(mutex_);
    Emit(SessionEventType::Log, "", "cannot open capture " + options_.replayPath);
    finished_ = true;
    return;
  }
  double firstHost = 0, wallStart = 0, captureStart = -1;
  bool first = true;
  std::set<uint16_t> skipped;  // endpoints of devices outside replayOnly
  std::FILE* trace = options_.replayTracePath.empty() ? nullptr : std::fopen(options_.replayTracePath.c_str(), "w");
  double nextTrace = 0;
  const auto& only = options_.replayOnly;
  reader.ReadAll([&](const usb::EndpointInfo& e) {
                   if (!only.empty() && std::find(only.begin(), only.end(), e.serial) == only.end()) { skipped.insert(e.ep); return; }
                   decoder.AddEndpoint(e);
                 },
                 [&](const usb::RawReport& r) {
                   if (stop_) return;
                   if (captureStart < 0 && r.hostTime > 0) captureStart = r.hostTime;  // header-time reports carry 0
                   const double at = captureStart < 0 ? 0 : r.hostTime - captureStart;
                   if (skipped.count(r.ep)) return;
                   // Before the window: decoded (attach, protocol negotiation, clock), but
                   // no IMU or light reaches the trackers.
                   warming_ = at < options_.replayFrom;
                   if (options_.replayTo > 0 && at > options_.replayTo) { stop_ = true; return; }
                   if (options_.replayRealtime && !warming_) {  // the warm-up runs at full speed
                     if (first) { firstHost = r.hostTime; wallStart = SteadyNow(); first = false; }
                     const double wait = (r.hostTime - firstHost) - (SteadyNow() - wallStart);
                     if (wait > 0.0005) platform::SleepFor(std::chrono::microseconds(int64_t(wait * 1e6)));
                   }
                   {
                     std::lock_guard lock(mutex_);
                     now_ = std::max(now_, r.hostTime);
                   }
                   decoder.OnReport(r);
                   if (trace && !warming_ && r.hostTime >= nextTrace) {
                     nextTrace = std::max(nextTrace + 1.0 / options_.replayTraceRate, r.hostTime);
                     for (const auto& u : Updates()) {
                       const auto& p = u.pose;
                       std::fprintf(trace,
                                    "{\"t\":%.6f,\"serial\":\"%s\",\"valid\":%d,\"state\":\"%s\",\"result\":%d,"
                                    "\"position\":[%.6f,%.6f,%.6f],\"orientation\":[%.7f,%.7f,%.7f,%.7f],\"velocity\":[%.5f,%.5f,%.5f],\"acceleration\":[%.3f,%.3f,%.3f],\"ft\":%.6f,\"fp\":[%.6f,%.6f,%.6f],\"pt\":%.6f}\n",
                                    r.hostTime, u.serial.c_str(), int(p.valid), ToString(p.state), int(p.result),
                                    p.worldFromImu.position[0], p.worldFromImu.position[1], p.worldFromImu.position[2],
                                    p.worldFromImu.orientation[0], p.worldFromImu.orientation[1], p.worldFromImu.orientation[2],
                                    p.worldFromImu.orientation[3], p.velocity[0], p.velocity[1], p.velocity[2],
                                    p.acceleration[0], p.acceleration[1], p.acceleration[2],
                                    p.filterTime, p.filterPosition[0], p.filterPosition[1], p.filterPosition[2], p.hostTime);
                     }
                   }
                 });
  if (trace) std::fclose(trace);
  finished_ = true;
#endif
}

void SessionImpl::Emit(SessionEventType type, const std::string& serial, std::string text, std::optional<LogLevel> level) {
  if (events_.size() > 4096) events_.pop_front();
  const LogLevel l = level ? *level : ClassifyLog(text);
  events_.push_back({now_, type, serial, std::move(text), l});
}

void SessionImpl::OnDeviceAttached(const DeviceId& id, const std::string& configJson) {
  std::lock_guard lock(mutex_);
  Tick();
  auto& d = devices_[id];
  const bool wasAttached = d.attached;
  const std::string config = configJson.empty() ? d.info.configJson : configJson;
  d.attached = true;
  d.preinitConnected = false;
  d.info.serial = id;
  d.lastData = now_;
  if (config.empty() || (wasAttached && d.tracker && d.info.configJson == config)) {
    if (d.tracker) Emit(SessionEventType::DeviceConnected, id, id + " connected");
    return;
  }
  // A radio reconnect is a new timestamp/protocol epoch, even when the device
  // JSON is identical; never take the duplicate-attach shortcut above for it.
  Emit(SessionEventType::Log, id, id + (wasAttached ? ": reattached with a new radio epoch" : ": attached"));
  if (SetUpDevice(id, config) && !configJson.empty()) UpdatePreinit(id, configJson);
}

bool SessionImpl::SetUpDevice(const DeviceId& id, const std::string& config) {
  auto& d = devices_[id];
  try {
    const json j = json::parse(config);
    TrackedObjectModel model;
    std::string err;
    if (!TrackedObjectModel::FromJson(config, &model, &err)) throw std::runtime_error(err);
    // A radio reconnect is a new timestamp/protocol epoch, not a duplicate attach.
    d.haveClock = false;
    d.clockOffset = 0;
    d.info.batteryPercent.reset();
    d.info.charging = false;
    d.input.reset();
    d.info.configJson = config;
    d.info.role = RoleFromConfig(config);
    d.info.controllerType = ControllerTypeFromConfig(config);
    d.info.deviceClass = j.value("device_class", "");
    d.info.model = j.value("model_number", "");
    d.info.manufacturer = j.value("manufacturer", "");
    d.info.renderModel = j.value("render_model", "");
    d.info.controllerRole = j.value("tracked_controller_role", "");
    if (const auto r = roles_.find(id); r != roles_.end() && d.info.role != DeviceRole::Hmd) {
      // settings.json "roles": the values of tracked_controller_role.
      if (r->second == "generic_tracker") {
        d.info.role = DeviceRole::Tracker;
        d.info.controllerRole.clear();
      } else {
        d.info.role = DeviceRole::Controller;
        d.info.controllerRole = r->second;
      }
    } else if (explicitControllers_ && d.info.role != DeviceRole::Hmd) {
      d.info.controllerRole.clear();
      d.info.role = DeviceRole::Tracker;
      if (id == leftController_ || id == rightController_) {
        d.info.role = DeviceRole::Controller;
        d.info.controllerRole = id == leftController_ ? "left_hand" : "right_hand";
      }
    }
    d.info.sensors = model.InImuFrame().points;
    // imuFromHead = (objectFromImu)^-1 * objectFromHead.
    Eigen::Isometry3d objectFromImu = Eigen::Isometry3d::Identity();
    const auto& r = model.objectFromImuRotation;
    objectFromImu.linear() = Eigen::Quaterniond(r[0], r[1], r[2], r[3]).normalized().toRotationMatrix();
    objectFromImu.translation() = Eigen::Vector3d(model.imuPosition.data());
    Eigen::Isometry3d objectFromHead = Eigen::Isometry3d::Identity();
    if (j.contains("head")) AxesRotation(j.at("head"), &objectFromHead);
    else if (j.contains("trackref_from_head")) PoseArray(j.at("trackref_from_head"), &objectFromHead);
    d.imuFromHead = objectFromImu.inverse() * objectFromHead;
    model.serial = id;
    d.tracker = std::make_unique<DeviceTracker>(id, model, &room_);
    d.tracker->SetSmoothing(smoothPosition_, smoothRotation_);
    d.tracker->SetReportAcceleration(reportAcceleration_);
    models_[id] = d.tracker->ImuModel();
    for (const auto& [serial, b] : bases_) d.tracker->OnBaseConfig(b.config, b.dynamic);
    PushIdentities();
    if (d.info.role == DeviceRole::Controller) d.input.emplace(d.info.controllerType);
    else d.input.reset();
    Emit(SessionEventType::DeviceActivated, id,
         id + " activated as " + ToString(d.info.role) + (d.info.model.empty() ? "" : " (" + d.info.model + ")") +
             (Enabled(id) ? "" : " [disabled]"));
    return true;
  } catch (const std::exception& e) {
    Emit(SessionEventType::Log, id, id + ": unusable device configuration: " + e.what());
    return false;
  }
}

void SessionImpl::LoadPreinit() {
  if (preinitDir_.empty()) return;
  std::error_code ec;
  std::vector<std::filesystem::path> files;
  for (const auto& e : std::filesystem::directory_iterator(preinitDir_, ec))
    if (e.path().extension() == ".json") files.push_back(e.path());
  std::sort(files.begin(), files.end());
  std::lock_guard lock(mutex_);
  for (const auto& file : files) {
    const std::string config = ReadFile(file.string());
    // The serial: a file named like one (LHR-0704E07A.json, as `lhtool configs` saves
    // them), else the config's own (wired headsets' configs say "LHR-XXXXXXXX").
    std::string id = file.stem().string();
    std::transform(id.begin(), id.end(), id.begin(), [](unsigned char c) { return char(std::toupper(c)); });
    try {
      const json j = json::parse(config);
      if (id.rfind("LHR-", 0) != 0) id = j.value("device_serial_number", "");
    } catch (const std::exception& e) {
      Emit(SessionEventType::Log, "", "preinit " + file.string() + ": " + e.what(), LogLevel::Warn);
      continue;
    }
    if (id.empty()) {
      Emit(SessionEventType::Log, "", "preinit " + file.string() + ": no serial (name the file <serial>.json)", LogLevel::Warn);
      continue;
    }
    if (devices_.count(id)) continue;
    preinitFiles_[id] = file.string();
    auto& d = devices_[id];
    d.info.serial = id;
    if (SetUpDevice(id, config)) {  // created, not attached: connected but uninitialized until it connects
      // Not a headset: a "connected" one would start the stock driver's display for nothing.
      d.preinitConnected = d.info.role != DeviceRole::Hmd;
      Emit(SessionEventType::Log, id, id + ": preinitialized from " + file.filename().string() + " (uninitialized)");
    }
  }
}

void SessionImpl::UpdatePreinit(const DeviceId& id, const std::string& config) {
  const auto file = preinitFiles_.find(id);
  if (file == preinitFiles_.end()) return;
  try {
    // Same JSON (formatting aside): keep the file as it is.
    if (json::parse(ReadFile(file->second)) == json::parse(config)) return;
    const std::string temp = file->second + ".tmp";
    {
      std::ofstream out(temp, std::ios::binary | std::ios::trunc);
      out << config;
      if (!out) throw std::runtime_error("cannot write " + temp);
    }
    std::filesystem::rename(temp, file->second);
    Emit(SessionEventType::Log, id, id + ": device config differs from preinit; using the device's and updating " + file->second);
  } catch (const std::exception& e) {
    Emit(SessionEventType::Log, id, id + ": updating preinit " + file->second + ": " + e.what(), LogLevel::Warn);
  }
}

void SessionImpl::OnDeviceDetached(const DeviceId& id) {
  std::lock_guard lock(mutex_);
  Tick();
  auto found = devices_.find(id);
  if (found == devices_.end()) return;
  found->second.attached = false;
  Emit(SessionEventType::DeviceDisconnected, id, id + " disconnected");
}

void SessionImpl::OnClockOffset(const DeviceId& id, double offset) {
  if (!std::isfinite(offset)) return;
  std::lock_guard lock(mutex_);
  auto& d = devices_[id];
  d.clockOffset = offset;
  d.haveClock = true;
}

void SessionImpl::Touch(Device& d, double deviceTime) {
  const double host = d.haveClock ? HostTime(d, deviceTime) : now_;
  if (host > d.lastData) d.lastData = std::min(host, now_ + 1.0);
}

void SessionImpl::OnImu(const DeviceId& id, const ImuSample& sample) {
  if (warming_) return;
  std::lock_guard lock(mutex_);
  Tick();
  if (!Enabled(id)) return;
  auto found = devices_.find(id);
  if (found == devices_.end() || !found->second.tracker) return;
  Touch(found->second, sample.time);
  found->second.tracker->OnImu(sample);
  Process(id, found->second);
  UpdateLightMode(id, found->second);
}

// A device reports LH1 or LH2 light, whichever mode the host set (LH1 after power-on). In
// LH2 mode it reports nothing from LH1 bases; in LH1 mode, LH2 bases give stray pulses but
// no sync flash. Usable light: an LH1 sync-width pulse, or an LH2 beam word.
//  * Start from the room's known bases (SteamVR's database): LH2 if all are 2.0 bases.
//  * Once bases of one kind only have been seen, keep every device in that mode (a device
//    that sees nothing is occluded; re-send the mode now and then in case it was lost).
//  * Otherwise (nothing seen yet, or a mixed room), a device without usable light for
//    kDark seconds tries the other mode.
void SessionImpl::UpdateLightMode(const DeviceId& id, Device& d) {
  constexpr double kDark = 2.5, kResend = 5.0;
  if (!usb_ || !d.attached) return;
  bool knownLh1 = false, knownLh2 = false, seenLh1 = false, seenLh2 = false;
  for (const auto& [serial, b] : bases_) {
    const bool lh2 = b.config.model != 1;
    (lh2 ? knownLh2 : knownLh1) = true;
    if (b.seen) (lh2 ? seenLh2 : seenLh1) = true;
  }
  auto set = [&](int mode, const char* why) {
    usb_->SetOpticalSubProtocol(id, mode);
    if (mode != d.lightMode) Emit(SessionEventType::Log, id, id + ": light mode " + (mode ? "LH2" : "LH1") + " (" + why + ")");
    d.lightMode = mode;
    d.modeSince = now_;
  };
  if (d.lightMode < 0) {
    set(knownLh2 && !knownLh1 ? 1 : 0, knownLh2 && !knownLh1 ? "room has 2.0 bases" : "default");
    return;
  }
  if (now_ - d.lastGoodLight < kDark || now_ - d.modeSince < kDark) return;
  if (seenLh1 != seenLh2) {
    const int want = seenLh2 ? 1 : 0;
    if (want != d.lightMode) set(want, "room's bases");
    else if (now_ - d.modeSince >= kResend) set(want, "re-send");
    return;
  }
  if (knownLh2 != knownLh1 && !seenLh1 && !seenLh2) {
    // Nothing seen yet, but the room is known to be one kind: stay in its mode (an
    // occluded device would otherwise spend seconds in the wrong one).
    const int want = knownLh2 ? 1 : 0;
    if (want != d.lightMode) set(want, "room's bases (known)");
    else if (now_ - d.modeSince >= kResend) set(want, "re-send");
    return;
  }
  set(1 - d.lightMode, "no usable light");
}

void SessionImpl::OnLightPulse(const DeviceId& id, const LightPulse& pulse) {
  if (warming_) return;
  std::lock_guard lock(mutex_);
  Tick();
  if (!Enabled(id)) return;
  auto found = devices_.find(id);
  if (found == devices_.end() || !found->second.tracker) return;
  Touch(found->second, pulse.time);
  if (pulse.width >= 56e-6 && pulse.width <= 140e-6) found->second.lastGoodLight = now_;  // a sync flash
  found->second.tracker->OnPulse(pulse);
}

void SessionImpl::OnLh2BeamWord(const DeviceId& id, const Lh2BeamWord& word) {
  if (warming_) return;
  std::lock_guard lock(mutex_);
  Tick();
  if (!Enabled(id)) return;
  auto found = devices_.find(id);
  if (found == devices_.end() || !found->second.tracker) return;
  Touch(found->second, word.time);
  found->second.lastGoodLight = now_;
  found->second.tracker->OnLh2BeamWord(word);
}

void SessionImpl::OnLh2SensorWord(const DeviceId& id, const Lh2SensorWord& word) {
  if (warming_) return;
  std::lock_guard lock(mutex_);
  Tick();
  if (!Enabled(id)) return;
  auto found = devices_.find(id);
  if (found == devices_.end() || !found->second.tracker) return;
  Touch(found->second, word.time);
  found->second.tracker->OnLh2SensorWord(word);
}

void SessionImpl::OnInput(const DeviceId& id, const InputReport& report) {
  std::lock_guard lock(mutex_);
  Tick();
  if (!Enabled(id)) return;
  auto found = devices_.find(id);
  if (found == devices_.end() || !found->second.input) return;  // trackers' inputs are ignored
  auto& d = found->second;
  d.input->Apply(report, d.haveClock ? HostTime(d, report.time) : now_);
}

void SessionImpl::OnBattery(const DeviceId& id, const BatteryReport& b) {
  std::lock_guard lock(mutex_);
  Tick();
  if (!Enabled(id)) return;
  auto found = devices_.find(id);
  if (found == devices_.end()) return;
  const int percent = b.raw & 0x7f;
  const bool charging = b.raw & 0x80;
  if (percent == 0 && !charging) return;
  found->second.info.batteryPercent = percent;
  found->second.info.charging = charging;
}

void SessionImpl::Process(const DeviceId& id, Device& d) {
  if (now_ - lastAdvance_ >= 0.01) { lastAdvance_ = now_; room_.Advance(now_); }
  std::vector<std::pair<BaseConfig, BaseDynamicState>> decoded;
  d.tracker->Process(&decoded);
  tdm::TdmDisambiguator::Identity identity;
  bool newIdentity = false;
  while (d.tracker->Source().PopIdentity(&identity)) {
    room_.SetOotxFrame(identity.config.serial, identity.frame);
    auto& stored = identities_[identity.config.serial];
    newIdentity |= stored.frame != identity.frame;
    stored = identity;
  }
  if (newIdentity) PushIdentities();
  for (const auto& [config, dynamic] : decoded) {
    auto& base = bases_[config.serial];
    const bool first = !base.seen;
    if (first) {
      char text[128];
      std::snprintf(text, sizeof text, "base %08X: firmware %u, gravity %.3f %.3f %.3f", config.serial,
                    unsigned(dynamic.firmwareVersion), dynamic.accelDir[0], dynamic.accelDir[1], dynamic.accelDir[2]);
      Emit(SessionEventType::Log, "", text, LogLevel::Debug);
    }
    const bool newChannel = dynamic.sobChannel >= 0 && base.dynamic.sobChannel != dynamic.sobChannel;
    base.config = config;
    base.dynamic = dynamic;
    if (newChannel) PushIdentities();  // the other devices identify this LH2 base at once
    base.seen = true;
    room_.OnBaseSeen(config, dynamic);
    for (auto& [otherId, other] : devices_)
      if (other.tracker && otherId != id) other.tracker->OnBaseConfig(config, dynamic);
    if (first) {
      char text[96];
      std::snprintf(text, sizeof text, "%s seen by %s (mode %d)", BaseName(config.serial).c_str(), id.c_str(), dynamic.mode);
      Emit(SessionEventType::BaseStationSeen, BaseName(config.serial), text);
    }
  }
  TakeKeyframes(d);
  CollectLogs(&d);
  const auto out = d.tracker->Output();
  if (out.state != d.lastState) {
    Emit(SessionEventType::TrackingStateChanged, id,
         id + ": " + ToString(d.lastState) + " -> " + ToString(out.state));
    d.lastState = out.state;
  }
  MaybeSave();
}

void SessionImpl::TakeKeyframes(Device& d) {
  for (uint32_t base : room_.TakeRelearned()) keyframes_.DropBase(base);
  for (auto& k : d.tracker->TakeKeyframes()) {
    std::optional<Eigen::Isometry3d> pose;
    if (const RoomBase* primary = room_.Base(k->primary)) pose = primary->worldFromBase * k->primaryFromDevice;
    if (keyframes_.Add(std::move(k), pose)) { newKeyframes_ = true; ++addedKeyframes_; }
  }
  if (!newKeyframes_ || !SolveDue()) return;
  if (syncSolve_) {
    lastSolve_ = now_;
    newKeyframes_ = false;
    ApplySolve(SolveRoom(SolveInput()));
  } else {
    solverWake_.notify_one();
  }
}

bool SessionImpl::SolveDue() const {
  if (!room_.Active() || !room_.Origin()) return false;
  // Locating a seen base quickly matters most; refinement can be slower.
  bool missing = false;
  for (const auto& [serial, b] : bases_) missing |= b.seen && !room_.Base(serial);
  return now_ - lastSolve_ >= (missing ? 0.1 : solveInterval_);
}

RoomSolveInput SessionImpl::SolveInput() {
  RoomSolveInput in;
  in.keyframes = keyframes_.All();
  in.fresh = std::min(addedKeyframes_, int(in.keyframes.size()));
  for (const auto& b : room_.Bases()) in.known[b.serial] = b.worldFromBase;
  // A knocked origin is relearned against another placed base, held fixed meanwhile.
  in.origin = room_.OriginRelearning() && room_.OriginGauge() ? room_.OriginGauge() : room_.Origin();
  for (const auto& [serial, b] : bases_)
    if (b.config.cal.size() == 14) {
      std::array<double, 14> cal;
      std::copy(b.config.cal.begin(), b.config.cal.end(), cal.begin());
      in.calibration[serial] = cal;
      in.baseModels[serial] = b.config.model;
    }
  in.models = models_;
  in.refineBases = !std::getenv("LH_EVALUATE_ROOM");  // diagnostics: score a fixed room
  addedKeyframes_ = 0;
  return in;
}

void SessionImpl::ApplySolve(const RoomSolveOutput& out) {
  Emit(SessionEventType::Log, "", "room solve: " + out.summary, LogLevel::Debug);
  if (!out.ok || std::getenv("LH_EVALUATE_ROOM")) return;
  const uint32_t origin = room_.Origin();
  // A settled room: re-solving every second only burns CPU (up to 400 keyframes, two
  // passes). Back off while the solved bases agree with the room, up to 4 s (it also bounds how late a knocked base shows up); any real
  // change (or a base still being located, see SolveDue) brings it back to 1 s.
  // Compared with the previous solve, not the room map: the map moves toward solves
  // gradually, so it always trails them by a little.
  bool settled = !lastSolved_.empty();
  double worstMove = 0, worstTurn = 0;
  for (const auto& [serial, b] : out.bases) {
    if (serial == origin || b.keyframes <= 0) continue;
    const auto previous = lastSolved_.find(serial);
    if (previous == lastSolved_.end()) { settled = false; continue; }
    worstMove = std::max(worstMove, (b.worldFromBase.translation() - previous->second.translation()).norm());
    worstTurn = std::max(worstTurn, Eigen::AngleAxisd(previous->second.rotation().transpose() * b.worldFromBase.rotation()).angle());
  }
  settled = settled && worstMove < 0.003 && worstTurn < 0.15 * M_PI / 180;
  lastSolved_.clear();
  for (const auto& [serial, b] : out.bases) lastSolved_[serial] = b.worldFromBase;
  solveInterval_ = settled ? std::min(2 * solveInterval_, 4.0) : 1.0;
  {
    char text[128];
    std::snprintf(text, sizeof text, "room solve: moved %.1f mm %.3f deg since last; next in %.0f s", worstMove * 1e3,
                  worstTurn * 57.2958, solveInterval_);
    Emit(SessionEventType::Log, "", text, LogLevel::Debug);
  }
  if (room_.OriginRelearning()) {
    const auto o = out.bases.find(origin);
    if (o != out.bases.end() && o->second.keyframes > 0)
      room_.ApplyOriginSolved(o->second.worldFromBase, o->second.positionVariance, o->second.keyframes);
  } else {
    // Every other base far off at once (three or more bases): it's the origin that moved.
    // With two, a knocked origin and a knocked other look alike; only the
    // accelerometers tell them apart.
    int others = 0, farOff = 0;  // (not `far`: a macro in the Windows headers)
    bool locked = false;  // a locked room: only the origin's accelerometer says it was knocked
    for (const auto& b : room_.Bases()) locked |= RoomMap::Locked(b);
    for (const auto& [serial, b] : out.bases) {
      if (locked) break;
      if (serial == origin || b.keyframes <= 0 || !room_.Base(serial)) continue;
      ++others;
      farOff += room_.FarOff(serial, b.worldFromBase, b.positionVariance, b.rotationVariance);
    }
    if (others >= 2 && farOff == others) {
      room_.Bump(origin, "every other base appears moved by it");
      CollectLogs(nullptr);
      return;
    }
  }
  const uint32_t gauge = room_.OriginRelearning() ? room_.OriginGauge() : 0;  // held fixed by the solve
  for (const auto& [serial, b] : out.bases)
    if (serial != origin && serial != gauge && b.keyframes > 0)
      room_.ApplySolved(serial, b.worldFromBase, b.positionVariance, b.rotationVariance, b.keyframes, b.rms, b.fresh,
                          b.devices, b.span);
  CollectLogs(nullptr);
}

void SessionImpl::SolverMain() {
  std::unique_lock lock(mutex_);
  while (!stop_) {
    solverWake_.wait_for(lock, std::chrono::milliseconds(100));
    if (stop_) break;
    Tick();
    if (!newKeyframes_ || !SolveDue()) continue;
    newKeyframes_ = false;
    lastSolve_ = now_;
    const RoomSolveInput input = SolveInput();
    const uint64_t resets = roomResets_;
    lock.unlock();
    const RoomSolveOutput out = SolveRoom(input);
    lock.lock();
    if (resets == roomResets_) ApplySolve(out);  // a reset meanwhile: solved against the old room
  }
}

void SessionImpl::PushIdentities() {
  std::vector<tdm::TdmDisambiguator::Identity> all;
  for (const auto& [serial, id] : identities_) all.push_back(id);
  std::map<int, std::pair<BaseConfig, BaseDynamicState>> channels;  // LH2: one channel per base, every device
  for (const auto& [serial, b] : bases_)
    if (b.dynamic.sobChannel >= 0 && b.config.valid && b.config.cal.size() == 14)
      channels[b.dynamic.sobChannel] = {b.config, b.dynamic};
  for (auto& [id, d] : devices_)
    if (d.tracker) {
      d.tracker->Source().SetKnownIdentities(all);
      d.tracker->Source().SetKnownSobChannels(channels);
    }
}

void SessionImpl::CollectLogs(Device* d) {
  if (d)
    for (auto& text : d->tracker->TakeLogs()) Emit(SessionEventType::Log, d->info.serial, std::move(text));
  const uint64_t before = room_.Revision();
  for (auto& text : room_.TakeLogs()) {
    const bool located = text.rfind("Located base", 0) == 0 || text.rfind("Setting ", 0) == 0;
    Emit(located ? SessionEventType::BaseStationLocated : SessionEventType::RoomChanged, "", std::move(text));
  }
  (void)before;
}

void SessionImpl::MaybeSave() {
  WatchRoomFile();
  if (!persist_ || !room_.Dirty()) return;
  const double wall = SteadyNow();
  if (wall - lastSave_ < 10.0) return;
  lastSave_ = wall;
  WatchRoomFile(true);  // never write over a deletion before it's seen
  if (!room_.Dirty()) return;
  std::string err;
  if (room_.Save(roomPath_, &err)) room_.ClearDirty();
  else Emit(SessionEventType::Log, "", err);
}

DevicePose SessionImpl::MakePose(const Device& d) const {
  DevicePose pose;
  pose.imuFromHead = ToPose(d.imuFromHead);
  if (!d.tracker) return pose;
  const auto out = d.tracker->Output();
  pose.state = out.state;
  pose.result = out.result;
  pose.valid = out.valid;
  pose.hostTime = d.haveClock ? HostTime(d, out.deviceTime) : now_;
  pose.worldFromImu = ToPose(out.worldFromImu);
  pose.worldFromHead = ToPose(out.worldFromImu * d.imuFromHead);
  pose.velocity = {out.velocity.x(), out.velocity.y(), out.velocity.z()};
  pose.angularVelocity = {out.angularVelocity.x(), out.angularVelocity.y(), out.angularVelocity.z()};
  pose.acceleration = {out.acceleration.x(), out.acceleration.y(), out.acceleration.z()};
  pose.primaryBase = out.primary;
  pose.bases = out.bases;
  pose.coverage = out.coverage;
  pose.updates = out.updates;
  pose.filterPosition = {out.filterPosition.x(), out.filterPosition.y(), out.filterPosition.z()};
  pose.filterTime = d.haveClock ? HostTime(d, out.filterTime) : now_;
  return pose;
}

SessionSnapshot SessionImpl::Snapshot() const {
  std::lock_guard lock(mutex_);
  SessionSnapshot snap;
  if (options_.replayPath.empty() && running_) now_ = std::max(now_, SteadyNow());
  snap.hostTime = now_;
  snap.roomRevision = room_.Revision();
  if (const Room* room = room_.Active()) snap.roomId = room->id;
  snap.originEpoch = room_.OriginEpoch();
  for (const auto& [id, d] : devices_) {
    if (!d.tracker) continue;  // receivers / devices without a usable tracking config
    DeviceState s;
    s.info = d.info;
    s.info.connected = d.preinitConnected || (d.attached && now_ - d.lastData < 1.0);
    s.info.enabled = Enabled(id);
    s.pose = MakePose(d);
    if (!s.info.connected || !s.info.enabled || d.preinitConnected) {
      s.pose.state = TrackingState::Disconnected;
      s.pose.valid = false;
      s.pose.result = TrackingResult::Uninitialized;
    }
    if (d.input && s.info.enabled) s.input = d.input->State();
    snap.devices.push_back(std::move(s));
  }
  std::map<uint32_t, BaseStationState> bases;
  for (const auto& [serial, b] : bases_) {
    auto& s = bases[serial];
    s.id = serial;
    s.serial = BaseName(serial);
    s.seen = b.seen;
    s.mode = b.dynamic.mode;
    // lhbs multi channels are its slot (1-4 on the display), stock ones A, b, c.
    s.lh2 = b.dynamic.sobChannel >= 0;
    s.hardwareVersion = b.config.hardwareVersion;
    s.nonce = b.dynamic.extra;
    if (b.dynamic.sobChannel >= 0) s.channel = std::to_string(b.dynamic.sobChannel + 1);  // LH2: 1-16
    else if (b.dynamic.multiSlot >= 0) s.channel = std::to_string(b.dynamic.multiSlot + 1);
    else if (b.dynamic.mode >= 1 && b.dynamic.mode <= 3) s.channel = std::string(1, "Abc"[b.dynamic.mode - 1]);
    s.faults = b.dynamic.faults;
  }
  for (const auto& rb : room_.Bases()) {
    auto& s = bases[rb.serial];
    s.id = rb.serial;
    s.serial = BaseName(rb.serial);
    s.known = true;
    s.worldFromBase = ToPose(rb.worldFromBase);
    s.positionVariance = rb.positionVariance;
    s.rotationVariance = rb.rotationVariance;
    s.origin = rb.origin;
    s.locked = RoomMap::Locked(rb);
    s.observations = std::max(rb.observations, rb.storedObservations);
    s.sigma = rb.origin ? 0.0 : std::sqrt(std::max(rb.positionVariance, 0.0));
    s.learned = rb.origin || s.sigma <= BaseStationState::kLearnedSigma ||
                ((s.locked || s.observations >= BaseStationState::kLearnedObservations) &&
                 s.sigma <= BaseStationState::kLearnedLockedSigma);
  }
  for (auto& [serial, s] : bases)
    if (s.seen || s.known) snap.bases.push_back(s);
  return snap;
}

std::vector<DeviceUpdate> SessionImpl::Updates() const {
  std::lock_guard lock(mutex_);
  if (options_.replayPath.empty() && running_) now_ = std::max(now_, SteadyNow());
  std::vector<DeviceUpdate> out;
  out.reserve(devices_.size());
  for (const auto& [id, d] : devices_) {
    if (!d.tracker) continue;
    DeviceUpdate u;
    u.serial = id;
    u.role = d.info.role;
    u.connected = d.attached && now_ - d.lastData < 1.0;
    u.enabled = Enabled(id);
    u.batteryPercent = d.info.batteryPercent;
    u.charging = d.info.charging;
    u.pose = MakePose(d);
    if (!u.connected || !u.enabled) {
      u.pose.state = TrackingState::Disconnected;
      u.pose.valid = false;
      u.pose.result = TrackingResult::Uninitialized;
    }
    if (d.input && u.enabled) u.input = d.input->State();
    out.push_back(std::move(u));
  }
  return out;
}

std::vector<SessionEvent> SessionImpl::PollEvents() {
  std::lock_guard lock(mutex_);
  std::vector<SessionEvent> out(events_.begin(), events_.end());
  events_.clear();
  return out;
}

void SessionImpl::LoadSettings() {
  if (settingsPath_.empty()) return;
  std::ifstream in(settingsPath_);
  if (!in) return;
  try {
    const json j = json::parse(in);
    if (j.contains("controllers")) {
      const auto& controllers = j.at("controllers");
      if (!controllers.is_object()) throw std::runtime_error("controllers must be an object");
      const auto left = controllers.value("left", std::string{});
      const auto right = controllers.value("right", std::string{});
      if (!left.empty() && left == right) throw std::runtime_error("left and right controller serials must differ");
      leftController_ = left;
      rightController_ = right;
      explicitControllers_ = true;
    }
    for (const auto& serial : j.value("disabledDevices", std::vector<std::string>{})) disabled_.insert(serial);
    if (j.contains("roles")) {
      // {"LHR-...": "left_hand" | "right_hand" | "generic_tracker"}, per device; the rest
      // follow their config's tracked_controller_role.
      for (const auto& [serial, role] : j.at("roles").items()) {
        const std::string r = Lower(role.get<std::string>());
        if (r != "left_hand" && r != "right_hand" && r != "generic_tracker")
          throw std::runtime_error("roles: " + serial + ": expected left_hand, right_hand or generic_tracker");
        roles_[serial] = r;
      }
    }
  } catch (const std::exception& e) {
    Emit(SessionEventType::Log, "", "settings " + settingsPath_ + ": " + e.what());
  }
}

void SessionImpl::SetOutputTuning(double positionSmoothing, double rotationSmoothing, bool reportAcceleration) {
  std::lock_guard lock(mutex_);
  const double position = std::clamp(positionSmoothing, 0.0, 1.0), rotation = std::clamp(rotationSmoothing, 0.0, 1.0);
  if (position == smoothPosition_ && rotation == smoothRotation_ && reportAcceleration == reportAcceleration_) return;
  smoothPosition_ = position;
  smoothRotation_ = rotation;
  reportAcceleration_ = reportAcceleration;
  for (auto& [id, d] : devices_) {
    if (!d.tracker) continue;
    d.tracker->SetSmoothing(position, rotation);
    d.tracker->SetReportAcceleration(reportAcceleration);
  }
  char text[160];
  std::snprintf(text, sizeof text, "smoothing: position %.0f ms, rotation %.0f ms%s; acceleration %s", position * 1000,
                rotation * 1000, position == 0 && rotation == 0 ? " (off)" : "", reportAcceleration ? "reported" : "not reported");
  Emit(SessionEventType::Log, "", text, LogLevel::Info);
}

void SessionImpl::SaveSettings() {
  if (settingsPath_.empty()) return;
  try {
    json j = json::object();
    {
      std::ifstream in(settingsPath_);
      if (in) j = json::parse(in);
    }
    j["disabledDevices"] = std::vector<std::string>(disabled_.begin(), disabled_.end());
    const std::filesystem::path target(settingsPath_);
    if (target.has_parent_path()) std::filesystem::create_directories(target.parent_path());
    const std::string temp = settingsPath_ + ".tmp";
    {
      std::ofstream out(temp);
      out << j.dump(2) << "\n";
    }
    platform::ReplaceFile(temp, target);
  } catch (const std::exception& e) {
    Emit(SessionEventType::Log, "", "saving settings " + settingsPath_ + ": " + e.what());
  }
}

void SessionImpl::SetDeviceEnabled(const std::string& serial, bool enabled) {
  std::lock_guard lock(mutex_);
  if (Enabled(serial) == enabled) return;
  if (enabled) disabled_.erase(serial);
  else disabled_.insert(serial);
  auto found = devices_.find(serial);
  if (found != devices_.end() && found->second.tracker) {
    // Start clean when re-enabled; stale filter state is worse than a fresh bootstrap.
    auto& d = found->second;
    TrackedObjectModel model;
    if (TrackedObjectModel::FromJson(d.info.configJson, &model)) {
      model.serial = serial;
      d.tracker = std::make_unique<DeviceTracker>(serial, model, &room_);
      d.tracker->SetSmoothing(smoothPosition_, smoothRotation_);
      d.tracker->SetReportAcceleration(reportAcceleration_);
      for (const auto& [base, b] : bases_) d.tracker->OnBaseConfig(b.config, b.dynamic);
      PushIdentities();
    }
    d.lastState = TrackingState::Disconnected;
  }
  Emit(SessionEventType::DeviceEnabledChanged, serial, serial + (enabled ? " enabled" : " disabled"));
  SaveSettings();
}

void SessionImpl::ResetRoom() {
  std::lock_guard lock(mutex_);
  ResetRoomLocked("reset requested");
}

void SessionImpl::ResetRoomLocked(const std::string& why) {
  // Every room goes, and with it everything learned against it: keyframes, solves in
  // flight, and each device's filter (its base frames and alt-base relationships).
  room_.ForgetAll();
  keyframes_.Clear();
  lastSolved_.clear();
  solveInterval_ = 1.0;
  newKeyframes_ = false;
  ++roomResets_;
  for (auto& [id, d] : devices_) {
    if (!d.tracker) continue;
    TrackedObjectModel model;
    if (!TrackedObjectModel::FromJson(d.info.configJson, &model)) continue;
    model.serial = id;
    d.tracker = std::make_unique<DeviceTracker>(id, model, &room_);
    d.tracker->SetSmoothing(smoothPosition_, smoothRotation_);
    d.tracker->SetReportAcceleration(reportAcceleration_);
    for (const auto& [base, b] : bases_) d.tracker->OnBaseConfig(b.config, b.dynamic);
    d.lastState = TrackingState::Disconnected;
  }
  PushIdentities();
  // Seen bases are seen again: the first one becomes the new room's origin.
  for (const auto& [serial, b] : bases_)
    if (b.seen) room_.OnBaseSeen(b.config, b.dynamic);
  if (persist_) {
    std::string err;
    if (room_.Save(roomPath_, &err)) room_.ClearDirty();
    roomFileSeen_ = true;
  }
  Emit(SessionEventType::RoomChanged, "", "room reset (" + why + "): every base station is relearned");
  CollectLogs(nullptr);
}

void SessionImpl::WatchRoomFile(bool now) {
  // Deleting the room map file while running resets the room (an external switch).
  if (!persist_ || roomPath_.empty()) return;
  const double wall = SteadyNow();
  if (!now && wall - lastRoomFileCheck_ < 1.0) return;
  lastRoomFileCheck_ = wall;
  std::error_code ec;
  const bool exists = std::filesystem::exists(roomPath_, ec);
  if (ec) return;
  if (exists) roomFileSeen_ = true;
  else if (roomFileSeen_) { roomFileSeen_ = false; ResetRoomLocked("room map file deleted"); }
}

bool SessionImpl::SaveRoom(std::string* error) {
  std::lock_guard lock(mutex_);
  if (roomPath_.empty()) {
    if (error) *error = "no room map path (replay without --room)";
    return false;
  }
  if (!room_.Save(roomPath_, error)) return false;
  room_.ClearDirty();
  return true;
}

}  // namespace runtime

std::unique_ptr<Session> Session::Create(const SessionOptions& options) {
  return std::make_unique<runtime::SessionImpl>(options);
}

}  // namespace lh
