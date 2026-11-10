#include "report_parser.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace lh::usb {
namespace {

constexpr double kTick = 1.0 / 48e6;       // 0x1a96f0
constexpr double kHalfTick = 1.0 / 96e6;   // 0x1a97b8

// 0x1a40a0 / 0x1a4080
constexpr double kAccelScale[4] = {
    2.0 * 9.80665 / 32768.0, 4.0 * 9.80665 / 32768.0, 8.0 * 9.80665 / 32768.0, 16.0 * 9.80665 / 32768.0};
constexpr double kGyroScale[4] = {
    250.0 * M_PI / 180.0 / 32768.0, 500.0 * M_PI / 180.0 / 32768.0, 1000.0 * M_PI / 180.0 / 32768.0,
    2000.0 * M_PI / 180.0 / 32768.0};

// LH1 sync-like pulse window and rate limit (0x1a9bb8, 0x1a97f0, 0x1a9768).
constexpr double kSyncWidthMin = 23.0 / 48000.0;
constexpr double kSyncWidthMax = 25.0 / 48000.0;
constexpr double kSyncEventHoldoff = 50.0 / 48000.0;

template <typename T>
T Rd(const uint8_t* p) {
  T v;
  std::memcpy(&v, p, sizeof(T));
  return v;
}

}  // namespace

// ---------------------------------------------------------------- IMU

bool ImuReportParser::Parse(double hostTime, const uint8_t* data, size_t len, bool lastInBatch) {
  if (len < 1) return false;
  if (data[0] != 0x20) return false;  // wireless (0x23/0x24...) handled elsewhere
  if (len < 1 + 3 * 17) return false;

  // 0x3d0a00: start at the oldest of the three ring entries.
  int64_t stamps[3];
  int oldest = 0;
  for (int i = 0; i < 3; i++) {
    stamps[i] = dev_->ticks.Unwrap(Rd<uint32_t>(data + 1 + 17 * i + 12), 32);
    if (stamps[i] < stamps[oldest]) oldest = i;
  }
  int emitted = 0;
  for (int k = 0; k < 3; k++) {
    const int i = (k + oldest) % 3;
    // Original re-unwraps here; the value is identical since the reference only advanced.
    const int64_t t = dev_->ticks.Unwrap(Rd<uint32_t>(data + 1 + 17 * i + 12), 32);
    if (!imu_.IsNew(t)) continue;
    if (k == 2 && lastInBatch) {
      dev_->clock.Update(hostTime - 0.0009, double(t) * kTick);
      sink_->OnClockOffset(dev_->id, dev_->clock.Offset());
    }
    imu_.Emit(data + 1 + 17 * i, t);
    emitted++;
  }
  if (emitted == 3) sink_->OnEvent(dev_->id, {double(imu_.LastTicks()) * kTick, int(DeviceEventType::ImuReportComplete), 0});
  return true;
}

void ImuEmitter::Emit(const uint8_t* s, int64_t ticks) {
  // 0x3d07c0
  lastTicks_ = ticks;
  if (dev_->accelRange < 0 || dev_->gyroRange < 0) return;
  ImuSample out;
  out.time = double(ticks) * kTick;
  const double as = kAccelScale[dev_->accelRange & 3];
  const double gs = kGyroScale[dev_->gyroRange & 3];
  for (int i = 0; i < 6; i++) {
    const int16_t raw = Rd<int16_t>(s + 2 * i);
    // (u16)(raw + 0x7fff) > 0xfffd  <=>  raw in {-32768, -32767, 32767}
    if (uint16_t(raw + 0x7fff) > 0xfffd) out.saturated |= 1u << i;
    if (i < 3) out.accel[i] = double(int(raw)) * as;
    else out.gyro[i - 3] = double(int(raw)) * gs;
  }
  sink_->OnImu(dev_->id, out);
}

// ---------------------------------------------------------------- Optical

bool OpticalReportParser::Parse(double /*hostTime*/, const uint8_t* data, size_t len) {
  if (len < 1) return false;
  switch (data[0]) {
    case 0x21: {  // 7 x {u8 sensor, u8 ?, u16 width, u32 ts28}
      if (len < 0x39) return true;
      for (const uint8_t* p = data + 1; p != data + 0x39; p += 8) {
        if (p[0] == 0xff) continue;
        const int64_t t = dev_->ticks.Unwrap(Rd<uint32_t>(p + 4), 28);
        const uint16_t w = Rd<uint16_t>(p + 2);
        OnRawPulse(double(t) * kTick, w == 0xffff ? -1.0 : double(w) * kTick, p[0]);
      }
      return true;
    }
    case 0x25: {  // 9 x {u8 sensor, u16 width, u32 ts28}
      if (len < 0x40) return true;
      for (const uint8_t* p = data + 1; p != data + 0x40; p += 7) {
        const uint16_t w = Rd<uint16_t>(p + 1);
        if (w == 0xffff) continue;
        const int64_t t = dev_->ticks.Unwrap(Rd<uint32_t>(p + 3), 28);
        OnRawPulse(double(t) * kTick, double(w) * kTick, p[0]);
      }
      return true;
    }
    case 0x27: {
      if (len >= 0x3b && Rd<uint32_t>(data + 0x37) == 0x0de40daa) {
        // Alternate layout (0x3cef10 + demod count). TODO: decode 0x3cef10 fully.
        if (data[1] != 0xff) {
          const int64_t t = dev_->ticks.Unwrap(Rd<uint32_t>(data + 2), 28);
          sink_->OnDemodCount(dev_->id, double(t) * kTick, Rd<uint16_t>(data + 0x35));
        }
        return true;
      }
      for (const uint8_t* p = data + 1; p != data + 0x35 && p + 13 <= data + len; p += 13) {
        if (p[0] == 0xff) continue;
        const int64_t t = dev_->ticks.Unwrap(Rd<uint32_t>(p + 1), 28);
        const int8_t s = int8_t(p[0]);
        // Slot +0x18 (raw entry) is a no-op in CSwSyncDetectSource; only the demod count matters.
        if (s >= 0 && (Rd<uint32_t>(p + 5) & Rd<uint32_t>(p + 9)) != 0)
          sink_->OnDemodCount(dev_->id, double(t) * kTick, 1);
      }
      return true;
    }
    case 0x28:
      if (len >= 2) ParseLh2Stream(data, len);
      return true;
    default:
      return false;
  }
}

void OpticalReportParser::OnRawPulse(double t, double width, int sensor) {
  // 0x3cec20
  if (width < 0.0) {
    sink_->OnEvent(dev_->id, {t, int(DeviceEventType::PulseWidthUnknown), 0});
    return;
  }
  if (sensor < 0xf7) {
    if (kSyncWidthMin < width && width < kSyncWidthMax &&
        (syncEventHoldoff_ == 0.0 || syncEventHoldoff_ < t)) {
      sink_->OnEvent(dev_->id, {t, int(DeviceEventType::SyncPulseSeen), 0});
      syncEventHoldoff_ = kSyncEventHoldoff + t;
    }
    sink_->OnLightPulse(dev_->id, {t, width, sensor & 0x1f});
  } else {
    sink_->OnAux(dev_->id, {t, 0xfe - sensor, 0});
  }
}

void OpticalReportParser::ParseLh2Stream(const uint8_t* data, size_t len) {
  // 0x3cf040
  const uint8_t* p = data + 2;
  const uint8_t* end = p + std::min<size_t>(data[1], len - 2);
  int channel = -1;
  while (p < end) {
    const uint8_t b = *p;
    if (b & 1) {
      switch (b & 0xf) {
        case 1:
          channel = b >> 4;
          p += 1;
          break;
        case 5:
          sink_->OnEvent(dev_->id, {0.0, int(DeviceEventType::Lh2ChannelConflict), b >> 4});
          p += 1;
          break;
        case 3: {
          if (p + 4 > end) return;
          const uint32_t w = Rd<uint32_t>(p);
          const int64_t t = dev_->ticks.Unwrap((w >> 4) & 0xffffff, 24);
          sink_->OnAux(dev_->id, {double(t) * kTick, int(w >> 28), 0});
          p += 4;
          break;
        }
        default:
          p += 1;
      }
      continue;
    }
    if (p + 4 > end) return;
    const uint32_t w = Rd<uint32_t>(p);
    p += 4;
    if (channel == -1) return;
    if ((w & 2) == 0) {
      const int64_t t = dev_->ticks.Unwrap((w >> 2) & 0xffffff, 24);
      sink_->OnLh2BeamWord(dev_->id, {double(t) * kTick, channel, int((w >> 26) & 1), int((w >> 27) & 1)});
    } else {
      const int64_t t = dev_->ticks.Unwrap((w >> 3) & 0xffffff, 24);
      double tt = double(t) * kTick;
      if (w & 4) tt += kHalfTick;
      sink_->OnLh2SensorWord(dev_->id, {tt, channel, int(w >> 27)});
    }
  }
}

}  // namespace lh::usb

namespace lh::usb {

// ---------------------------------------------------------------- Wireless

bool WirelessReportParser::DecodePacket(const uint8_t* pkt, int64_t* imuTicks) {
  // 0x3d18d0. Each radio slot is 29 bytes, including the two framing bytes.
  if (pkt[1] < 1 || pkt[1] > 27) return false;
  bool ok;
  if (version_ == 0) {
    ok = LegacyDecoder().Decode(pkt[0], pkt + 2, pkt[1], &pkt_);
    if (ok) {
      probeDecoded_[0] = probeDecoded_[1] = 0;
    } else {
      // Its sub-protocols are announced only now and then, never in a short recording:
      // each probe assumes one (LH2 light, LH1 light; controller 0 as live devices
      // announce), and the one that keeps decoding while legacy fails is the device.
      for (int k = 0; k < 2; ++k) {
        WirelessPacket probed;
        if (!probe_[k].Decode(pkt[0], pkt + 2, pkt[1], &probed)) { probeDecoded_[k] = 0; continue; }
        if (++probeDecoded_[k] < 20) continue;  // ~0.1 s of a device's packets
        version_ = 1;
        wv2_ = probe_[k];
        pkt_ = probed;
        ok = true;
        break;
      }
    }
  } else if (version_ == 1) ok = wv2_.Decode(pkt[0], pkt + 2, pkt[1], &pkt_);
  else {
    // 0x3ccaf0: waiting for "len 3, 0xe2, version"
    if (pkt[1] == 3 && pkt[3] == 0xe2) {
      version_ = pkt[4];
      return true;
    }
    return false;
  }
  if (!ok) return false;  // "dropped a malformed radio packet"
  if (pkt_.versionChange && version_ != pkt_.newVersion) {
    version_ = pkt_.newVersion;  // "device switched to report protocol %d" — rest of packet dropped
    return true;
  }
  if (pkt_.hasImu) {
    const int64_t t = dev_->ticks.Unwrap(pkt_.imuTs, 32);
    *imuTicks = t;
    if (imu_.IsNew(t)) imu_.Emit(pkt_.imuRaw, t);
  }
  // Input/battery carry no stamp of their own: use the IMU stamp when present, else
  // the latest device time, without moving the shared unwrapper reference.
  const double packetTime = double(pkt_.hasImu ? *imuTicks : dev_->ticks.Last()) * kTick;
  if (pkt_.hasBattery) sink_->OnBattery(dev_->id, {packetTime, pkt_.battery});
  if (pkt_.hasButtons || pkt_.hasTrigger || pkt_.hasTouchpad || pkt_.hasInput8 || pkt_.hasInputV2 ||
      pkt_.hasRawInput) {
    InputReport in;
    in.time = packetTime;
    if (pkt_.hasInputV2) {
      in.format = InputFormat::Compact;
      in.hasButtons = in.hasTrigger = in.hasAxes = true;
      in.extra = pkt_.inputA2;
    } else if (pkt_.hasRawInput) {
      in.format = InputFormat::Raw;
      in.raw = pkt_.rawInput;
    } else {
      in.hasButtons = pkt_.hasButtons; in.hasTrigger = pkt_.hasTrigger; in.hasAxes = pkt_.hasTouchpad;
    }
    in.buttons = pkt_.buttons; in.trigger = pkt_.trigger;
    in.axisX = int16_t(pkt_.touchpad & 0xffff); in.axisY = int16_t(pkt_.touchpad >> 16);
    if (pkt_.hasInput8) {
      in.hasExtended = true;
      for (int i = 0; i < 8; ++i) in.extended[size_t(i)] = uint8_t(pkt_.input8 >> (8 * i));
    }
    sink_->OnInput(dev_->id, in);
  }
  if (pkt_.hasEvent) {
    const int64_t t = dev_->ticks.Unwrap(pkt_.eventTs, 32);
    sink_->OnAux(dev_->id, {double(t) * kTick, 0x100, pkt_.eventValue});
  }
  for (const auto& r : pkt_.lh2) {
    const int64_t t = dev_->ticks.Unwrap(r.ts24, 24);
    if (r.type == 1) {
      sink_->OnLh2BeamWord(dev_->id, {double(t) * kTick, r.channel, r.bit26, r.bit27});
    } else {
      double tt = double(t) * kTick;
      if (r.halfTick) tt += kHalfTick;
      sink_->OnLh2SensorWord(dev_->id, {tt, r.channel, r.sensor});
    }
  }
  for (const auto& p : pkt_.pulses) {
    const int64_t t = dev_->ticks.Unwrap(p.ts, 32);
    if (p.sensor < 0xf7) sink_->OnLightPulse(dev_->id, {double(t) * kTick, double(p.width) * kTick, int(p.sensor & 0x1f)});
    else sink_->OnAux(dev_->id, {double(t) * kTick, int(0xfe - p.sensor), 0});
  }
  return true;
}

bool WirelessReportParser::Parse(double hostTime, const uint8_t* data, size_t len, bool lastInBatch) {
  // 0x3d1920
  if (len < 1 || (data[0] != 0x23 && data[0] != 0x24)) return false;
  if (len < 30) return true;
  int64_t ticks = INT64_MIN;
  const bool ok1 = DecodePacket(data + 1, &ticks);
  bool sync = lastInBatch && ok1;
  if (data[0] == 0x24 && len >= 59) {
    int64_t t2 = INT64_MIN;
    if (DecodePacket(data + 0x1e, &t2)) {
      ticks = t2;
      sync = lastInBatch;
    } else {
      if (!ok1) return true;
      hostTime -= 0.0015;  // 0x1a9580
      sync = lastInBatch;
    }
  }
  // Original syncs with the last decoded IMU timestamp (uninitialized if the packet had
  // no IMU); we only sync when one was present.
  if (sync && ticks != INT64_MIN) {
    dev_->clock.Update(hostTime - 0.0044, double(ticks) * kTick);  // 0x1a99c8
    sink_->OnClockOffset(dev_->id, dev_->clock.Offset());
  }
  return true;
}

}  // namespace lh::usb
