#include "common/test_paths.hpp"
#include <array>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include "openvr/hand_skeleton.hpp"
#include "openvr/raw_alignment.hpp"
#include "openvr/room_publisher.hpp"
#include "openvr/host_frame.hpp"
#include "openvr/haptic_routing.hpp"
#include "openvr/telemetry_server.hpp"
#include "openvr/stock_hmd.hpp"
#include "common/net.hpp"
#include "lh/log_level.hpp"
#include <chrono>
#include <nlohmann/json.hpp>
#include <thread>
#include "platform/platform.hpp"
#include "transport/usb/haptics.hpp"
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>

TEST(StockHmd, SharesOnlyTheHeadsetsBluetoothRadios) {
  using lh::openvr::StockWindowsDeviceAllowed;
  EXPECT_TRUE(StockWindowsDeviceAllowed(0x28de, 0x2102, false, true));
  EXPECT_FALSE(StockWindowsDeviceAllowed(0x28de, 0x2102, false, false));
  EXPECT_FALSE(StockWindowsDeviceAllowed(0x28de, 0x2101, false, true));
  EXPECT_FALSE(StockWindowsDeviceAllowed(0x28de, 0x2300, false, true));
  EXPECT_TRUE(StockWindowsDeviceAllowed(0x28de, 0x2300, true, false));
  EXPECT_TRUE(StockWindowsDeviceAllowed(0x28de, 0x2000, false, false));
  EXPECT_TRUE(StockWindowsDeviceAllowed(0x1234, 0x2102, false, false));
}

TEST(Haptics, MatchesStockEncoding) {
  // Captured from stock on SteamVR Identify (duration 0, 200 Hz, amplitude 0.125):
  // ff 8f 08 00 f4 01 f4 01 01 00 ff.
  const auto identify = lh::usb::HapticReport(200, .125, 0);
  const std::array<uint8_t, 11> captured{0xff, 0x8f, 0x08, 0x00, 0xf4, 0x01, 0xf4, 0x01, 0x01, 0x00, 0xff};
  for (size_t i = 0; i < captured.size(); ++i) EXPECT_EQ(identify[i], captured[i]) << i;
  // 0x3f4e50, regular pulse: period 5 ms <= 7.998 ms, so on = a*(T/2 - 80us) + 80us.
  const auto r = lh::usb::HapticReport(200, .5, .1);
  EXPECT_NEAR(r[4] + 256 * r[5], 1290, 1);
  EXPECT_NEAR(r[6] + 256 * r[7], 3710, 1);
  EXPECT_EQ(r[8] + 256 * r[9], 20);
  const auto stop = lh::usb::HapticReport(0, 0, 0);
  EXPECT_EQ(stop[4] + stop[5] + stop[6] + stop[7] + stop[8] + stop[9], 0);
  const auto invalid = lh::usb::HapticReport(NAN, 1, 1);
  EXPECT_EQ(invalid[6] + invalid[7] + invalid[8] + invalid[9], 0);
  const auto bounded = lh::usb::HapticReport(1e9, 1e9, 1e9);  // stock caps at 300 us x 33333
  EXPECT_EQ(bounded[8] + 256 * bounded[9], 0x8235);
}

TEST(DeviceCommands, PowerOffWireFormat) {
  const auto report = lh::usb::PowerOffReport();
  EXPECT_EQ(report.size(), 64u);
  EXPECT_EQ(report[0], 0xff); EXPECT_EQ(report[1], 0x9f); EXPECT_EQ(report[2], 4);
  EXPECT_EQ(std::string(report.begin() + 3, report.begin() + 7), "off!");
  for (size_t i = 7; i < report.size(); ++i) EXPECT_EQ(report[i], 0);
}

TEST(DeviceCommands, RoutesModernAndLegacyHapticsWithoutBroadcast) {
  vr::VREvent_t e{}; e.eventType = vr::VREvent_Input_HapticVibration;
  e.trackedDeviceIndex = vr::k_unTrackedDeviceIndexInvalid;
  e.data.hapticVibration.containerHandle = 100;
  e.data.hapticVibration.componentHandle = 200;
  EXPECT_TRUE(lh::openvr::MatchesHaptic(e, 2, 100, 200));
  e.data.hapticVibration.containerHandle = 0;
  EXPECT_TRUE(lh::openvr::MatchesHaptic(e, 2, 100, 200));
  e.data.hapticVibration.componentHandle = 0;
  EXPECT_FALSE(lh::openvr::MatchesHaptic(e, 2, 100, 200));
  e.trackedDeviceIndex = 2;
  EXPECT_TRUE(lh::openvr::MatchesHaptic(e, 2, 100, 200));
  EXPECT_FALSE(lh::openvr::MatchesHaptic(e, 3, 101, 201));
  e.data.hapticVibration.componentHandle = 999;
  EXPECT_FALSE(lh::openvr::MatchesHaptic(e, 2, 100, 200));
}

// Monado's steamvr_lh (ControllerDevice::set_output): the device Activate(n)'d gets
// container n + 1 from TrackedDeviceToPropertyContainer, but its haptic events carry
// container n and device index n - 1. The component handle is what identifies it.
TEST(DeviceCommands, RoutesMonadoHapticsByComponent) {
  vr::VREvent_t e{}; e.eventType = vr::VREvent_Input_HapticVibration;
  const uint32_t first = 1, second = 2;  // Activate(device_idx + 1)
  e.trackedDeviceIndex = second - 1;
  e.data.hapticVibration.containerHandle = second;      // Monado's container_handle
  e.data.hapticVibration.componentHandle = 0x2000000a;  // second's /output/haptic
  EXPECT_TRUE(lh::openvr::MatchesHaptic(e, second, second + 1, 0x2000000a));
  // the first device's container is the event's: it must still not vibrate
  EXPECT_FALSE(lh::openvr::MatchesHaptic(e, first, first + 1, 0x20000005));
  EXPECT_FALSE(lh::openvr::MatchesHaptic(e, first, second, 0x20000005));
}

namespace {
// Model-space (root-relative) joint positions and rotations, OpenVR's parent chain.
std::array<Eigen::Isometry3f, 26> Globals(const lh::openvr::HandSkeleton::Bones& b) {
  static constexpr int parent[26] = {-1,0,1,2,3,4,1,6,7,8,9,1,11,12,13,14,1,16,17,18,19,1,21,22,23,24};
  std::array<Eigen::Isometry3f, 26> g;
  for (int i = 0; i < 26; ++i) {
    Eigen::Isometry3f local = Eigen::Isometry3f::Identity();
    local.linear() = Eigen::Quaternionf(b[i].orientation.w, b[i].orientation.x, b[i].orientation.y, b[i].orientation.z).toRotationMatrix();
    local.translation() = Eigen::Vector3f(b[i].position.v[0], b[i].position.v[1], b[i].position.v[2]);
    g[i] = parent[i] < 0 ? local : g[parent[i]] * local;
  }
  return g;
}
float Deg(const vr::VRBoneTransform_t& b) { return 2 * std::acos(std::min(1.f, std::abs(b.orientation.w))) * 57.29578f; }
}  // namespace

TEST(Skeleton, ValidArticulationAndOpenVRLeftHand) {
  const char* file = std::getenv("LH_HAND_POSES");
  if (!file) GTEST_SKIP() << "set LH_HAND_POSES to Valve's installed vr_glove_right_poses.glb";
  lh::openvr::HandSkeleton right, left;
  std::string error;
  ASSERT_TRUE(right.Load(file, false, &error)) << error;
  ASSERT_TRUE(left.Load(file, true, &error)) << error;
  lh::ControllerInput input;
  const auto open = right.Evaluate(input, true);
  input.fingerIndex = .9; input.fingerMiddle = .8; input.fingerRing = .7; input.fingerPinky = .6;
  input.aTouch = true; input.bTouch = true;
  for (bool controller : {true, false}) {
    const auto r = right.Evaluate(input, controller), l = left.Evaluate(input, controller);
    const auto gr = Globals(r), gl = Globals(l);
    bool moved = false, thumb = false;
    for (int i = 0; i < 31; ++i) {
      const auto& q = r[i].orientation;
      EXPECT_NEAR(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z, 1, 1e-5);
      if (i > 6 && std::abs(q.z - open[i].orientation.z) > .01) moved = true;
      if (i >= 3 && i <= 4 && std::abs(q.z - open[i].orientation.z) > .01) thumb = true;
    }
    EXPECT_TRUE(moved);
    EXPECT_TRUE(thumb) << "a touched face button curls the thumb";
    for (int i = 0; i < 26; ++i) {
      // The hands are mirror images in model space...
      EXPECT_NEAR(gl[i].translation().x(), -gr[i].translation().x(), 1e-5) << i;
      EXPECT_NEAR(gl[i].translation().y(), gr[i].translation().y(), 1e-5) << i;
      EXPECT_NEAR(gl[i].translation().z(), gr[i].translation().z(), 1e-5) << i;
      // ...with OpenVR's bone frames: inside a finger the left bone has the right one's
      // rotation and the negated offset (Valve's reference left/right open hands).
      const bool inFinger = i >= 3 && i != 6 && i != 11 && i != 16 && i != 21;
      if (inFinger) {
        for (int k = 0; k < 3; ++k) EXPECT_NEAR(l[i].position.v[k], -r[i].position.v[k], 1e-5) << i;
        EXPECT_NEAR(std::abs(l[i].orientation.w * r[i].orientation.w + l[i].orientation.x * r[i].orientation.x +
                             l[i].orientation.y * r[i].orientation.y + l[i].orientation.z * r[i].orientation.z), 1, 1e-5) << i;
      }
    }
    // Aux bones sit on the fingertips they summarize.
    for (int f = 0; f < 5; ++f)
      EXPECT_LT((Eigen::Vector3f(l[26 + f].position.v) - gl[f == 0 ? 5 : 10 + (f - 1) * 5].translation()).norm(), 1e-5);
  }
  // Without the controller (all Monado reads): a full grip closes every finger into
  // a fist, the index included; with it, the fingers wrap the handle.
  input = {};
  input.fingerIndex = input.fingerMiddle = input.fingerRing = input.fingerPinky = 1;
  input.trigger = 1;
  const auto fist = right.Evaluate(input, false);
  const auto openHand = right.Evaluate(lh::ControllerInput{}, false);
  for (int f = 0; f < 4; ++f) {
    float bend = 0;
    for (int k = 7; k <= 9; ++k) bend += Deg(fist[size_t(k + 5 * f)]);
    EXPECT_GT(bend, 150) << "finger " << f << " closes";
    EXPECT_LT(bend, 300) << "finger " << f << " stays a fist";
  }
  EXPECT_GT(Deg(fist[8]), Deg(openHand[8]) + 40) << "the index closes all the way";
}

// The thumb has its own pose for what it rests on, as the stock Index hand: A, B,
// both, the stick (by deflection) and the pad (by position), in both motion ranges.
TEST(Skeleton, ThumbFollowsWhatItTouchesSmoothly) {
  const char* file = std::getenv("LH_HAND_POSES");
  if (!file) GTEST_SKIP() << "set LH_HAND_POSES to Valve's installed vr_glove_right_poses.glb";
  lh::openvr::HandSkeleton hand;
  std::string error;
  ASSERT_TRUE(hand.Load(file, false, &error)) << error;
  auto tip = [](const lh::openvr::HandSkeleton::Bones& b) { return Eigen::Vector3f(b[26].position.v); };
  for (bool controller : {true, false}) {
    std::vector<std::pair<std::string, lh::ControllerInput>> cases;
    lh::ControllerInput in;
    cases.push_back({"rest", in});
    in = {}; in.aClick = in.aTouch = true; cases.push_back({"A pressed", in});
    in = {}; in.bClick = in.bTouch = true; cases.push_back({"B pressed", in});
    in = {}; in.thumbstickTouch = true; cases.push_back({"stick", in});
    in.thumbstickX = 1; cases.push_back({"stick right", in});
    in = {}; in.trackpadTouch = true; in.trackpadY = 1; cases.push_back({"pad top", in});
    in.trackpadY = -1; cases.push_back({"pad bottom", in});
    std::vector<Eigen::Vector3f> tips;
    for (const auto& [name, input] : cases) tips.push_back(tip(hand.Evaluate(input, controller)));
    for (size_t i = 0; i < tips.size(); ++i)
      for (size_t k = i + 1; k < tips.size(); ++k) {
        if (!controller && (cases[i].first == "A pressed" || cases[i].first == "B pressed") &&
            (cases[k].first == "A pressed" || cases[k].first == "B pressed")) continue;  // both curl onto the buttons
        if (!controller && cases[i].first == "rest" && cases[k].first == "pad top") continue;  // stock: the pad's top is the rest pose
        EXPECT_GT((tips[i] - tips[k]).norm(), 0.002f) << (controller ? "with" : "without") << " controller: "
                                                      << cases[i].first << " vs " << cases[k].first;
      }
  }
  // Moving from the stick to the pad fades: no frame jumps the thumb tip by > 1 cm.
  lh::openvr::HandSkeleton live = hand;
  lh::ControllerInput in;
  in.thumbstickTouch = true;
  for (int i = 0; i < 100; ++i) live.Update(in, 0.005);
  for (bool controller : {true, false}) {
    lh::openvr::HandSkeleton step = live;
    lh::ControllerInput pad;
    pad.trackpadTouch = true; pad.trackpadY = -1;
    Eigen::Vector3f last = tip(step.Current(controller));
    float largest = 0;
    for (int i = 0; i < 200; ++i) {
      step.Update(pad, 0.005);
      const Eigen::Vector3f now = tip(step.Current(controller));
      largest = std::max(largest, (now - last).norm());
      last = now;
    }
    EXPECT_LT(largest, 0.01f) << (controller ? "with" : "without") << " controller";
    EXPECT_LT((last - tip(hand.Evaluate(pad, controller))).norm(), 1e-4f) << "settles on the pad pose";
  }
  live.Update(in, 0.005);
  EXPECT_FALSE(hand.Load("/nonexistent.glb", false, &error));
}

// lighthousedb.json still levels the room the way the stock driver last saw the origin (a
// base since moved: 40 deg off); our room is level (RoomMap::LevelOrigin). The host frame
// keeps SteamVR's origin and yaw but comes out level.
TEST(RawAlignment, TakesTheLevelFromOurRoomNotAStaleDatabase) {
  const auto path = (std::filesystem::temp_directory_path() / ("lh-test-rawalign-" + std::to_string(lh::platform::ProcessId()) + ".json")).string();
  {
    std::ofstream f(path);
    f << R"({"known_universes": [{"id": "77", "tilt": {"pitch": 0.698, "roll": 0.0},
             "base_stations": [{"base_serial_number": 1,
               "target_pose": {"pose": [0, 0, 0, 1, 0, 0, 0], "variance": 0}}]}],
           "base_stations": [{"config": {"serialNumber": 1, "baseCalibration": [0,0,0,0,0,0,0,0,0,0,0,0,0,0]},
             "dynamic_states": [{"dynamic_state": {"reset_count": 1}}]}]})";
  }
  lh::openvr::RawAlignment alignment;
  std::string error;
  ASSERT_TRUE(alignment.Load(path, &error)) << error;
  std::filesystem::remove(path);
  // our room: the origin levelled by its accelerometer, which says it's upright
  lh::SessionSnapshot snapshot;
  lh::BaseStationState base;
  base.id = 1; base.known = base.seen = base.origin = true;
  base.worldFromBase.orientation = {1, 0, 0, 0};
  snapshot.bases.push_back(base);
  ASSERT_TRUE(alignment.Update(snapshot));
  const Eigen::Isometry3d raw = alignment.RawFromWorld();
  EXPECT_NEAR((raw.rotation() * Eigen::Vector3d::UnitY()).y(), 1.0, 1e-9);  // level
  EXPECT_LT(raw.translation().norm(), 1e-9);                                 // SteamVR's origin
  // yaw: the smallest turn from SteamVR's frame, so a horizontal heading stays put
  const Eigen::Vector3d heading = raw.rotation() * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(heading.y(), 0.0, 1e-9);
  EXPECT_EQ(alignment.RoomId(), "steamvr-77");
}

// A small disagreement with lighthousedb.json's level (Valve's accelerometers, ours:
// 2 deg) keeps SteamVR's frame exactly: the anchor where SteamVR has it, so tools
// aligning that frame (some reset past a few degrees) see no move. Our room turning
// inside (re-levelling) changes nothing in the host frame either.
TEST(RawAlignment, KeepsSteamVRsFrameForSmallLevelDifferencesAndNeverDrifts) {
  const auto path = (std::filesystem::temp_directory_path() / ("lh-test-rawalign2-" + std::to_string(lh::platform::ProcessId()) + ".json")).string();
  {
    std::ofstream f(path);
    f << R"({"known_universes": [{"id": "77", "tilt": {"pitch": 0.035, "roll": 0.0},
             "base_stations": [{"base_serial_number": 1,
               "target_pose": {"pose": [0, 0, 0, 1, 0, 0, 0], "variance": 0}}]}],
           "base_stations": [{"config": {"serialNumber": 1, "baseCalibration": [0,0,0,0,0,0,0,0,0,0,0,0,0,0]},
             "dynamic_states": [{"dynamic_state": {"reset_count": 1}}]}]})";
  }
  lh::openvr::RawAlignment alignment;
  std::string error;
  ASSERT_TRUE(alignment.Load(path, &error)) << error;
  std::filesystem::remove(path);
  lh::SessionSnapshot snapshot;
  lh::BaseStationState base;
  base.id = 1; base.known = base.seen = base.origin = true;
  base.worldFromBase.orientation = {1, 0, 0, 0};
  snapshot.bases.push_back(base);
  ASSERT_TRUE(alignment.Update(snapshot));
  const Eigen::Isometry3d first = alignment.RawFromWorld();
  const Eigen::Isometry3d anchorRaw = first;  // the anchor at our identity
  EXPECT_GT(std::acos((anchorRaw.rotation() * Eigen::Vector3d::UnitY()).y()), 0.03) << "SteamVR's 2 deg tilt kept";
  // our room re-levels by 1 deg: the anchor's raw pose, and every device's, stay
  const Eigen::Quaterniond turn(Eigen::AngleAxisd(0.017, Eigen::Vector3d::UnitX()));
  snapshot.bases[0].worldFromBase.orientation = {turn.w(), turn.x(), turn.y(), turn.z()};
  ASSERT_TRUE(alignment.Update(snapshot));
  const Eigen::Isometry3d worldFromAnchor(turn);
  EXPECT_TRUE((alignment.RawFromWorld() * worldFromAnchor).isApprox(anchorRaw, 1e-9));
}

TEST(RawAlignment, PreservesHostFrameAcrossRoomRebuild) {
  const std::string file = TestCapture("session-20261016-142931Z-147083/lighthousedb.json");
  if (!std::filesystem::exists(file)) GTEST_SKIP();
  lh::openvr::RawAlignment alignment;
  std::string error;
  ASSERT_TRUE(alignment.Load(file, &error)) << error;
  lh::SessionSnapshot snapshot;
  lh::BaseStationState base;
  base.id = 0xA101B835; base.known = base.seen = base.origin = true;
  base.worldFromBase.orientation = {1, 0, 0, 0};
  snapshot.bases.push_back(base);
  ASSERT_TRUE(alignment.Update(snapshot));
  const Eigen::Isometry3d original = alignment.RawFromWorld();
  snapshot.bases[0].worldFromBase.position = {1, 2, 3};
  ASSERT_TRUE(alignment.Update(snapshot));
  const auto fixed = alignment.RawFromWorld() * Eigen::Vector3d(1, 2, 3);
  EXPECT_LT((fixed - original.translation()).norm(), 1e-9);
  EXPECT_EQ(alignment.RoomId(), "steamvr-1770781428");
  snapshot.bases.clear();
  EXPECT_FALSE(alignment.Update(snapshot));
  EXPECT_TRUE(alignment.RoomId().empty());
}

TEST(RoomPublisher, SettlesAfterTheRoomStopsMovingAndBumpsTheEpochOnARealChange) {
  using lh::openvr::RoomSettle;
  RoomSettle s;
  Eigen::Isometry3d a = Eigen::Isometry3d::Identity(), b = Eigen::Isometry3d::Identity();
  b.translation() = Eigen::Vector3d(3, 0, 1);
  auto room = [&](double sigmaB) {
    return std::vector<RoomSettle::Base>{{"LHB-A", true, true, a, 0, false, true},
                                         {"LHB-B", true, true, b, sigmaB, false, sigmaB <= RoomSettle::kMaxSigma}};
  };
  double t = 0;
  EXPECT_FALSE(s.Update({{"LHB-A", true, true, a, 0, false, true}, {"LHB-B", true, false, b, 0, false, true}}, t));  // B not placed
  for (; t < 2.9; t += 0.25) EXPECT_FALSE(s.Update(room(0.001), t)) << t;
  for (; t < 8; t += 0.25) {  // B moves 2.4 cm per second: never still
    b.translation().x() += 0.006;
    EXPECT_FALSE(s.Update(room(0.001), t)) << t;
  }
  const double stop = t;
  for (; t < stop + 2.6; t += 0.25) EXPECT_FALSE(s.Update(room(0.001), t));
  bool settled = false;
  for (; t < stop + 4; t += 0.25) settled = s.Update(room(0.001), t);
  EXPECT_TRUE(settled);
  EXPECT_EQ(s.epoch(), 1u);
  // Jitter under 5 mm keeps it settled; an uncertain base unsettles it, the same room again keeps the epoch.
  b.translation().x() += 0.001;
  EXPECT_TRUE(s.Update(room(0.001), t += 0.25));
  EXPECT_FALSE(s.Update(room(0.02), t += 0.25));
  for (int i = 0; i < 16; ++i) settled = s.Update(room(0.001), t += 0.25);
  EXPECT_TRUE(settled);
  EXPECT_EQ(s.epoch(), 1u);
  // A base moved 5 cm: a new epoch once it is settled again.
  b.translation().z() += 0.05;
  EXPECT_FALSE(s.Update(room(0.001), t += 0.25));
  for (int i = 0; i < 16; ++i) settled = s.Update(room(0.001), t += 0.25);
  EXPECT_TRUE(settled);
  EXPECT_EQ(s.epoch(), 2u);
}

TEST(HostFrame, PinsTheOriginAcrossRelevelsAndSessions) {
  const std::string path = (std::filesystem::temp_directory_path() / "lh_hostframe_test.json").string();
  std::filesystem::remove(path);
  auto snapshot = [](double tiltDeg) {
    lh::SessionSnapshot s;
    s.roomId = "room-1";
    lh::BaseStationState o;
    o.serial = "LHB-00000001"; o.origin = o.known = o.seen = true;
    const Eigen::Quaterniond q(Eigen::AngleAxisd(tiltDeg * M_PI / 180, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitY()));
    o.worldFromBase.orientation = {q.w(), q.x(), q.y(), q.z()};
    o.worldFromBase.position = {0, 0, 0};
    s.bases.push_back(o);
    return s;
  };
  auto originInHost = [](const lh::SessionSnapshot& s, const Eigen::Isometry3d& rawFromWorld) {
    const auto& p = s.bases[0].worldFromBase;
    Eigen::Isometry3d w = Eigen::Isometry3d::Identity();
    w.linear() = Eigen::Quaterniond(p.orientation[0], p.orientation[1], p.orientation[2], p.orientation[3]).toRotationMatrix();
    return rawFromWorld * w;
  };
  Eigen::Isometry3d anchored;
  {
    lh::openvr::HostFrame h;
    h.Load(path);
    double t = 0;
    for (; t < 2.9; t += 0.25) { h.Update(snapshot(0), t); EXPECT_FALSE(h.Anchored()); }
    for (; t < 3.6; t += 0.25) h.Update(snapshot(0), t);
    ASSERT_TRUE(h.Anchored());
    anchored = originInHost(snapshot(0), h.Update(snapshot(0), t));
    // The room re-levels 2.5 deg inside: the host still sees the origin where it was,
    // heading the same way; and the host frame stays level (no tilt, only a turn about
    // the vertical): its tilt is the room's own, now the better level.
    const auto turned = snapshot(2.5);
    const Eigen::Isometry3d raw = h.Update(turned, t + 1);
    EXPECT_TRUE(originInHost(turned, raw).translation().isApprox(anchored.translation(), 1e-9));
    EXPECT_NEAR((raw.linear() * Eigen::Vector3d::UnitY()).y(), 1.0, 1e-12);
    const Eigen::Vector3d f0 = -anchored.linear().col(2), f1 = -originInHost(turned, raw).linear().col(2);
    EXPECT_NEAR(std::atan2(f0.x(), f0.z()), std::atan2(f1.x(), f1.z()), 1e-9);
    // However far off the anchor's tilt is (a remounted origin, re-levelled 51 deg),
    // the host frame stays level.
    const Eigen::Isometry3d tilted = h.Update(snapshot(51), t + 2);
    EXPECT_NEAR((tilted.linear() * Eigen::Vector3d::UnitY()).y(), 1.0, 1e-12);
  }
  // The next session: the room's level differs again; the origin is at once where it
  // was, heading the same way.
  lh::openvr::HostFrame again;
  again.Load(path);
  const auto other = snapshot(-1.7);
  EXPECT_TRUE(originInHost(other, again.Update(other, 0)).translation().isApprox(anchored.translation(), 1e-6));
  EXPECT_TRUE(again.Anchored());
  std::filesystem::remove(path);
}

// Without the controller (what Monado and xrizer show): a lifted thumb rises to the
// open hand's thumb instead of staying on the last button, and moving between buttons,
// with the moment of nothing touched in between, glides instead of snapping.
TEST(Skeleton, ThumbReleasesAndGlidesBetweenButtons) {
  const char* file = std::getenv("LH_HAND_POSES");
  if (!file) GTEST_SKIP() << "set LH_HAND_POSES to Valve's installed vr_glove_right_poses.glb";
  lh::openvr::HandSkeleton hand;
  std::string error;
  ASSERT_TRUE(hand.Load(file, false, &error)) << error;
  auto tip = [](const lh::openvr::HandSkeleton::Bones& b) { return Eigen::Vector3f(b[26].position.v); };
  const lh::ControllerInput none;
  lh::ControllerInput a, stick;
  a.aTouch = true;
  stick.thumbstickTouch = true; stick.thumbstickX = 1;
  const Eigen::Vector3f rest = tip(hand.Evaluate(none, false)), onA = tip(hand.Evaluate(a, false));
  ASSERT_GT((rest - onA).norm(), 0.01f);
  // Lifted, the thumb is the open hand's (raised), not the one lying on the controller.
  lh::ControllerInput pad;
  pad.trackpadTouch = true; pad.trackpadY = 1;  // stock: the pad's top is the thumb's resting pose
  EXPECT_GT((rest - tip(hand.Evaluate(pad, false))).norm(), 0.01f) << "lifted thumb doesn't rise";
  // On A, then lifted: raised within a second.
  for (int i = 0; i < 200; ++i) hand.Update(a, 0.005);
  EXPECT_LT((tip(hand.Current(false)) - onA).norm(), 1e-3f);
  for (int i = 0; i < 200; ++i) hand.Update(none, 0.005);
  EXPECT_LT((tip(hand.Current(false)) - rest).norm(), 2e-3f) << "the thumb stays on A after letting go";
  // A -> 30 ms of nothing -> the stick: no 5 ms step moves the tip by more than 1 cm.
  for (int i = 0; i < 200; ++i) hand.Update(a, 0.005);
  Eigen::Vector3f last = tip(hand.Current(false));
  float largest = 0;
  for (int i = 0; i < 200; ++i) {
    hand.Update(i < 6 ? none : stick, 0.005);
    const Eigen::Vector3f now = tip(hand.Current(false));
    largest = std::max(largest, (now - last).norm());
    last = now;
  }
  EXPECT_LT(largest, 0.01f) << "the thumb snapped";
  EXPECT_LT((last - tip(hand.Evaluate(stick, false))).norm(), 2e-3f) << "settles on the stick";
}

TEST(Telemetry, ServesHelloBacklogStateAndLogsOnLoopback) {
  lh::openvr::TelemetryServer server;
  const int port = 36840 + 100 + int(lh::platform::ProcessId() % 500);
  std::string error;
  ASSERT_TRUE(server.Start(port, "test-host", &error)) << error;
  server.Log(1.0, lh::LogLevel::Warn, "LHR-1", "before the client");
  lh::net::Socket s = lh::net::kInvalid;
  for (int i = 0; i < 50 && s == lh::net::kInvalid; ++i) {
    s = lh::net::ConnectLoopback(port);
    if (s == lh::net::kInvalid) std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_NE(s, lh::net::kInvalid);
  auto readLines = [&](int want) {
    std::vector<nlohmann::json> out;
    std::string buf;
    for (int i = 0; i < 200 && int(out.size()) < want; ++i) {
      char chunk[4096];
      const long n = lh::net::RecvSome(s, chunk, sizeof chunk);
      if (n > 0) buf.append(chunk, size_t(n));
      for (size_t nl; (nl = buf.find('\n')) != std::string::npos; buf.erase(0, nl + 1)) out.push_back(nlohmann::json::parse(buf.substr(0, nl)));
      if (int(out.size()) < want) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return out;
  };
  auto first = readLines(2);
  ASSERT_GE(first.size(), 2u);
  EXPECT_EQ(first[0]["type"], "hello");
  EXPECT_EQ(first[0]["host"], "test-host");
  EXPECT_EQ(first[1]["type"], "log");
  EXPECT_EQ(first[1]["level"], "WARN");
  EXPECT_EQ(first[1]["text"], "before the client");
  lh::SessionSnapshot snapshot;
  lh::BaseStationState base;
  base.serial = "LHB-D7B1AD76";
  base.seen = base.known = true;
  base.channel = "3";
  base.positionVariance = 0.017 * 0.017;
  base.sigma = 0.017;
  snapshot.bases.push_back(base);
  snapshot.roomId = "steamvr-1";
  server.PublishState(2.0, snapshot, {}, {}, {false, 3, 0.005});
  server.Log(2.1, lh::LogLevel::Debug, "", "[multi] chatter");
  auto next = readLines(2);
  ASSERT_GE(next.size(), 2u);
  EXPECT_EQ(next[0]["type"], "state");
  EXPECT_EQ(next[0]["room"]["epoch"], 3);
  EXPECT_FALSE(next[0]["room"]["settled"].get<bool>());
  EXPECT_NEAR(next[0]["bases"][0]["sigma"].get<double>(), 0.017, 1e-9);
  EXPECT_EQ(next[0]["bases"][0]["channel"], "3");
  EXPECT_EQ(next[1]["level"], "DEBUG");
  lh::net::Close(s);
  server.Stop();
}

TEST(LogLevels, ClassifiesAndParses) {
  EXPECT_EQ(lh::ClassifyLog("LHR-1: [multi] multi: base X at relative slot 3"), lh::LogLevel::Debug);
  EXPECT_EQ(lh::ClassifyLog("LHR-1: Resetting tracking: no optical samples for 2000ms"), lh::LogLevel::Warn);
  EXPECT_EQ(lh::ClassifyLog("cannot open capture /x"), lh::LogLevel::Error);
  EXPECT_EQ(lh::ClassifyLog("LHR-1 activated as tracker"), lh::LogLevel::Info);
  lh::LogLevel l = lh::LogLevel::Info;
  EXPECT_TRUE(lh::ParseLogLevel("DEBUG", &l));
  EXPECT_EQ(l, lh::LogLevel::Debug);
  EXPECT_TRUE(lh::ParseLogLevel("warning", &l));
  EXPECT_EQ(l, lh::LogLevel::Warn);
  EXPECT_FALSE(lh::ParseLogLevel("loud", &l));
  EXPECT_STREQ(lh::LogLevelName(lh::LogLevel::Error), "ERR");
}
