// Synchronous LH1 orchestration of the ported filter and bootstrap solver.
// Algorithmic baseline; whole-driver parity is not yet established.
#pragma once

#include <deque>
#include <map>
#include <optional>
#include <string>

#include <Eigen/Geometry>

#include "lh/tracker.hpp"
#include "direct_filter.hpp"
#include "sample_history.hpp"
#include "tracked_object.hpp"

namespace lh {

class LighthouseDirect {
 public:
  static constexpr double kSaturatedJerkScale = 10;  // process noise while the accelerometer saturates
  explicit LighthouseDirect(TrackedObjectModel model, double opticalTimeout = 0.5);
  // Not stock, off by default (the oracle tests compare against the original):
  // multi-start acquisition (solver::SolveMultiStart) and per-row innovation gating
  // of optical updates at `gateSigma`.
  // Swing handling: saturated IMU axes as one-sided bounds, and the slow states held
  // during violent motion (DirectFilter::holdSlowStates).
  void SetImprovements(bool multiStart, double gateSigma, bool swing = false) {
    multiStart_ = multiStart; filter_.gateSigma = gateSigma; swing_ = swing;
    filter_.gravityTolerance = multiStart ? 0.98 : 0.49033249999999996;
  }
  void OnBaseConfig(const BaseConfig&, const BaseDynamicState&);
  void SetRecordedTilt(uint32_t serial, std::array<double, 2> tilt, double variance);
  void OnImu(const ImuSample&);
  bool OnFrame(const OpticalFrame&);
  // Consume inertial data while optical data is unavailable. This is an explicit
  // fallback policy, not the original CImuOnlyTracker implementation.
  bool AdvanceInertial(double time);
  // Time between sweeps of the bases seen (1/60 stock; N/60 for a lhbs multi-station
  // slave): advancing on the IMU alone waits at least this long for light.
  void SetSweepInterval(double seconds) { sweepInterval_ = seconds; }
  std::optional<TrackedPose> Pose() const;
  // Not stock: the filter state carried forward through the IMU samples queued
  // since its last update (strapdown with the filter's own biases, scales, IMU
  // rotation and gravity), to the newest one. The filter only advances on optical
  // frames, so its own pose is 10-30 ms old and moves in bursts; this is the pose now.
  struct Predicted {
    double time = 0;
    double p[3] = {}, v[3] = {}, q[4] = {1, 0, 0, 0}, w[3] = {};  // base frame; w body
    double a[3] = {};  // base frame, gravity removed: the newest sample's (or the state's)
  };
  Predicted PredictNow() const;
  DeviceTrackingInfo Info() const;
  const direct::DirectFilter& Filter() const { return filter_; }
  // Mutable access for deterministic fault-injection tests only.
  direct::DirectFilter& FilterForTest() { return filter_; }
  // Read-only synchronous diagnostics, also used by the original-function oracle.
  const TrackedObjectModel& Model() const { return model_; }
  const std::deque<ImuSample>& PendingImu() const { return imu_; }
  uint32_t BaseSerial(int slot) const { return slot >= 0 && slot < 4 ? slots_[slot] : 0; }
  double LastOpticalTime() const { return lastOpticalTime_; }
  size_t PrimarySwitches() const { return primarySwitches_; }
  size_t SecondaryBases() const { return secondaryBases_; }
  uint32_t Counters() const { return filter_.flags; }
  // Process the buffered IMU prefix before a measurement time without adding a
  // measurement; useful to independently verify prediction/update scheduling.
  bool ProcessImuUntil(double time) { return PredictTo(time); }

  // ---- hooks for the shared room map (not part of the stock driver) ----------
  // Pose estimate of an alternate base relative to the primary, with the
  // translation / rotation variance traces the original uses for RELATIONSHIP
  // reports (0x3509e0: < 6.4e-5 and < 1.22e-5 count as converged).
  struct AltEstimate {
    uint32_t serial = 0;
    Eigen::Isometry3d altFromPrimary = Eigen::Isometry3d::Identity();
    double translationVariance = 0, rotationVariance = 0;
  };
  std::vector<AltEstimate> AltEstimates() const;
  // Start tracking against `base` from a known pose instead of a cold bootstrap.
  // `carry` (optional) supplies IMU calibration states 18..30 and their covariance
  // from a previous track of the same device; `tilt` is the base tilt (x16, x17).
  struct Seed {
    uint32_t base = 0;
    double time = 0;
    Eigen::Isometry3d baseFromImu = Eigen::Isometry3d::Identity();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();         // base frame
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();  // IMU frame
    double positionVariance = 1e-4, rotationVariance = 1e-3, velocityVariance = 0.04;
    std::array<double, 2> tilt{};
    double tiltVariance = 1e-4;
    const direct::DirectFilter* carry = nullptr;
    bool calibrated = false;  // start past the 200-update calibration phase
  };
  bool SeedPose(const Seed& seed);
  // Add an alternate base whose pose relative to the primary is already known.
  bool AddKnownAltBase(uint32_t serial, const Eigen::Isometry3d& altFromPrimary,
                       double translationVariance, double rotationVariance);
  // Overwrite an alternate base already in the filter (e.g. after the room solver
  // refined it). Cross-covariances with the rest of the state are cleared.
  void SetAltBaseRelearn(uint32_t serial, bool relearn);  // see DirectFilter::altBaseRelearn
  bool SetKnownAltBase(uint32_t serial, const Eigen::Isometry3d& altFromPrimary,
                       double translationVariance, double rotationVariance);
  // Stock reset checks (0x358300). Returns the reason the original would log after
  // "Resetting tracking:", or "" if tracking is healthy. `now` is device time.
  std::string HealthCheck(double now) const;
  bool Bootstrapped() const { return slots_[0] != 0; }
  double Time() const { return time_; }
  std::optional<double> ObservedTime() const { return observedTime_; }

 private:
  bool Bootstrap(const OpticalFrame&, const BaseConfig&);
  bool CanSwitchPrimary(uint32_t serial) const;
  bool ApplyOpticalResiduals(const OpticalFrame&, const BaseConfig&, int slot);
  bool OpticalUpdate(const OpticalFrame&, const BaseConfig&, int slot);
  bool ImuUpdate(const ImuSample&);
  bool PredictTo(double time);
  int Slot(uint32_t serial) const;
  double BaseDistance(const double* state, int slot) const;
  std::vector<OpticalSample> Gather(const std::vector<OpticalSample>&) const;
  void FilterOutliers(std::vector<OpticalSample>* samples, double distance) const;

  TrackedObjectModel model_;
  direct::DirectFilter filter_;
  struct Base { BaseConfig config; BaseDynamicState dynamic; };
  std::map<uint32_t, Base> bases_;
  std::map<uint32_t, SampleHistory> history_;
  struct RecordedTilt { std::array<double, 2> angles; double variance; };
  std::map<uint32_t, RecordedTilt> recordedTilts_;
  std::deque<ImuSample> imu_;
  uint32_t slots_[4]{};
  double time_ = 0, lastOpticalTime_ = 0, opticalTimeout_ = 0.5;
  bool swing_ = false;
  double violentUntil_ = -1e300;  // IMU calibration held until (freezeImuCalibration)
  double sweepInterval_ = 1.0 / 60;
  std::optional<double> observedTime_;
  uint64_t bootstrapAttempts_ = 0;
  std::map<uint32_t, double> lastMultiStart_, multiStartWait_;  // per base
  bool multiStart_ = false;  // SetImprovements
  double diameter_ = 0, radius_ = 0;
  bool calibrationSeen_ = false;  // filter_.flags & 0x100
  size_t primarySwitches_ = 0, secondaryBases_ = 0;
  uint64_t altGateUntil_ = 0;  // after a reseed: the other bases' light isn't frame-gated before this
};

}  // namespace lh
