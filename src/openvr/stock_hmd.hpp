// Head-mounted displays through the original driver ("hybrid"): our driver tracks every
// device, but a headset's display (distortion, projection, direct mode, IPD, proximity,
// camera, audio) comes from the user's installed stock driver_lighthouse, loaded at run
// time from the copy the installer kept (driver_lighthouse.so.stock). Nothing of it is
// shipped with this project.
//
// The stock provider gets a driver context whose IVRServerDriverHost is a proxy:
//  * TrackedDeviceAdded: only head-mounted displays reach the real host; everything else
//    the stock driver finds is ours already and is swallowed (reported as added).
//  * TrackedDevicePoseUpdated for its headset: dropped once ours publishes the pose (our
//    tracking, our room frame); every other call passes through.
//  * PollNextEvent: the host's events reach both drivers (ours polls the host and hands
//    the stock driver a copy), so neither steals the other's.
// The stock provider's standby calls are not forwarded: it would power off controllers
// it doesn't own here. The headset's own device calls are.
#pragma once

#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <openvr_driver.h>

namespace lh::openvr {

// Only the headset's own radios may be shared with the stock Bluetooth service.
inline bool StockWindowsDeviceAllowed(unsigned vid, unsigned pid, bool headset, bool headsetRadio) {
  if (vid != 0x28de) return true;
  if (pid == 0x2101) return false;
  if (pid == 0x2102) return headsetRadio;
  if (pid == 0x2300) return headset;
  return true;
}

class StockHmd {
 public:
  using LogFn = std::function<void(const std::string&)>;
  // Loads the stock driver and runs its provider's Init. path: the stock library.
  bool Load(const std::string& path, vr::IVRDriverContext* realContext, LogFn log);
  bool Loaded() const { return provider_ != nullptr; }
  void Cleanup();

  // Host thread: our RunFrame calls these.
  void PushEvent(const vr::VREvent_t& event);
  void RunFrame();

  // A headset the stock driver added and the host activated: (serial, object id). alive
  // stays valid for the process's life and turns false when the host deactivates it.
  struct Activated { std::string serial; uint32_t objectId; const std::atomic<bool>* alive; };
  std::optional<Activated> TakeActivated();
  // Our driver publishes this device's pose from now on: drop the stock driver's.
  void OwnPose(uint32_t objectId);

  // The stock library next to ours (or LH_STOCK_DRIVER), "" if none.
  static std::string DefaultPath();
  // Lighthouse headsets on USB right now. Hosts accept devices only for a few seconds
  // after Init (Monado: 3 s), so the stock driver is loaded at Init when one is.
  //  * Yes: 28de:2000 (Vive / Vive Pro / Index headset board).
  //  * Maybe: a 28de:2300 that isn't a controller. Wired 0x2300 devices name themselves
  //    by hardware family: a Bigscreen Beyond says "Tundra Tracker". Its device config
  //    decides (the session's role from device_class); `maybe` gets their serials.
  enum class Presence { No, Maybe, Yes };
  static Presence HeadsetPresent(std::string* which = nullptr, std::vector<std::string>* maybe = nullptr);
  // A wired 28de:2300 headset (Bigscreen Beyond) the stock driver may open; call before
  // Load. On Linux the stock driver never sees any other lighthouse device: dongles and
  // other wired 0x2300 devices are left out of its udev scans (it would otherwise drive
  // them too: in LH2 rooms every tracker then paused its IMU stream ~30 ms once a
  // second). LH_STOCK_ALL_DEVICES=1 turns this off.
  static void AllowHeadset(const std::string& serial);

 private:
  class Host;
  class LogFilter;
  class Settings;
  class Context;
  class DeviceProxy;
  friend class Host;
  friend class DeviceProxy;

  void* library_ = nullptr;
  vr::IServerTrackedDeviceProvider* provider_ = nullptr;
  vr::IVRDriverContext* real_ = nullptr;
  vr::IVRServerDriverHost* realHost_ = nullptr;
  Host* host_ = nullptr;
  Settings* settings_ = nullptr;
  LogFilter* logProxy_ = nullptr;
  Context* context_ = nullptr;
  LogFn log_;

  std::mutex mutex_;
  std::deque<vr::VREvent_t> events_;
  std::deque<Activated> activated_;
  std::set<uint32_t> owned_;
  std::set<int> vendorEvents_;  // types seen (logged once)
  std::deque<DeviceProxy*> devices_;
};

}  // namespace lh::openvr
