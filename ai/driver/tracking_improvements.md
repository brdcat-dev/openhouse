# Beyond stock: what the runtime adds on top of the verified filter

The filter core (`LighthouseDirect`, `DirectFilter`) stays the verified port of the
stock driver and is stock by default; the oracle tests compare it against the
original. `DeviceTracker` switches the additions on. Everything here was measured on a
10-minute VRChat session under Monado (2026-10-19: two Index controllers, an Index as
a tracker, two Vive trackers, two LH1 bases; one controller lying on a bed, trackers
near the floor).

## Results (tools/eval_replay.sh, whole session, 2154 device-seconds)

| | valid | position spikes > 2 cm | > 5 cm | rotation spikes > 5 deg | still jitter |
|---|---|---|---|---|---|
| stock filter (`LH_STOCK_FILTER=1`, room fixes kept) | 90.3 % | 1301 | 67 | 1294 | 412 um |
| now | 95.9 % | 36 | 0 | 132 | 80 um |

Spikes are second differences of the 100 Hz pose a host is given (accelerations no
hand makes). The remaining rotation spikes are real fast wrist turns.

Display error (the pose a host extrapolates to display time, against the filter's
later, light-corrected estimate), 20 ms ahead: mean 1.5 mm vs 1.9 mm, p95 5.9 vs 7.9 mm;
40 ms ahead: 2.4 vs 3.2 mm, p95 9.3 vs 13.5 mm; and 23 ms less host extrapolation.

Hidden controller (`captures/controller-moving-hidden`, `OutputStage` test): largest
frame-to-frame jump 19.7 cm -> 5.4 cm (real motion), drift while hidden 23.4 cm -> 0.9 cm.

## Acquisition (`solver::SolveMultiStart`, lighthouse_direct.cpp)

The stock bootstrap solves from one start (2 m straight ahead, unrotated). A device
lying on its side below a base, seeing ~5 sensors, never converges from there: live, a
controller on a bed took **207 s** to acquire and two trackers lost 58 s stretches.
When the stock solve fails (at most every 0.25 s per base, backing off to 2 s; ~11 ms):

* starts along the ray the sweep angles point at, 3 distances x 24 orientations,
  with a 2.5x looser convergence noise (a still device on few sensors sits at
  ~1.7x the stock noise for minutes);
* **reflections**: light on a sensor facing away from the base (cos < 0.2) at the
  solved pose can only be reflected (live: 4 of 11 sensors of a tracker on the floor,
  3-6 mrad off). Those sensors are dropped and the pose solved again; a pose that
  needs one is rejected;
* the best near misses are trimmed of their worst-fitting sensor up to 3 times (one
  strong reflection pulls even a start at the true pose 26 cm away);
* a fit is not taken if a clearly different pose fits within 2x (ambiguous).

The controller on the bed now acquires in 3 s; the floor trackers in seconds.

## Output stage (`DeviceTracker::UpdateOutput`)

* **Prediction to now.** The filter only advances on optical frames, so its pose is
  10-30 ms old and moves in bursts (frozen for a sample or two, then a jump). The pose
  is carried forward through the queued IMU samples (strapdown with the filter's own
  biases, scales, IMU rotation and gravity; `LighthouseDirect::PredictNow`).
* **Correction smoothing.** A discontinuity of the filter's pose (optical correction
  after drift, reacquisition, a room change) becomes an offset decaying over 60 ms
  (150 ms above 10 cm); motion passes through untouched.
* **Coasting.** Without light for > 60 ms the position follows the last velocity,
  decaying over 120 ms, instead of 2 s of free inertial integration (live: 71 cm of
  drift, snapped back when light returned). Orientation stays the IMU's.
* **Stationary hold.** Gyro (less bias) < 0.86 deg/s and accelerometer within 0.04
  m/s^2 of its mean for 0.5 s: the pose is held (one base and a few sensors drift the
  depth of a resting device by 20 cm). Any motion releases it and it glides.

## Inconsistent poses (`DeviceTracker::SyncRoom`, `CheckHealth`)

A device can bootstrap on one base into a pose that fits that base's light but not
the other's (session-post: the left controller on E236DC40; 14 % of its A101B835 frames
rejected for 50 s while it drifted, then "position out of bounds"). A rejected frame
teaches the filter nothing, so it never recovers on its own. When a base's well-lit
frames (>= 4 sensors) are all rejected 20 in a row:

* the room's relationship replaces the filter's own where they differ by more than
  1 cm / 3 room sigmas (the filter's view of that base is wrong);
* otherwise, with the room confident (< 5 mm) and agreeing with the filter, twice
  running: the pose is wrong. The device starts over, on the base it was rejecting
  first (1 s), with its IMU calibration kept.

Room corrections also reach a filter whenever the room is more certain than the
filter's own estimate (it used to need a fixed 5 mm combined sigma). session-post: the left
controller's rejected A101B835 frames 1339 -> 87 (two reacquisitions, at 3 s and 40 s);
no device on vrchat-01 triggered it.

## Late optical frames

Frames reach the filter 20-25 ms after their sweep (the sweep-decoding pipeline), with
a tail past 50 ms. Stock advances the filter on the IMU to 20 ms behind the newest
sample once light is 50 ms old, then drops any frame older than that: ~1.4 % of all
usable frames on the session. The output stage predicts to now itself, so the filter
now waits up to 120 ms: dropped frames 2142 -> 76 (first 300 s), time coasting 1.1 %
-> 0.5 %, one tracker's 2.1 % -> 0.0 %.

Measured too (with a per-frame visibility print, since removed): lost with light split across both
bases but enough on neither (where a joint two-base bootstrap would help) was 1.8 s
of 2154 device-seconds; not worth building yet.

## Room (`RoomMap`)

* The base accelerometer re-levels a stored room only beyond 8 deg (a moved base);
  Valve's accelerometers are a few degrees off (live: 2.6 deg one way at start, the
  devices' gravity 2.24 deg back a second later: every device jumped 17 cm).
* Tilt refinements turn the room at 2 deg/s the first time, 0.05 deg/s after that.
* Room-solver refinements glide at 2 cm/s and 0.5 deg/s (it used to apply 30 % per
  solve: 7-10 mm steps every second). First placements and moves are at once.
* The room keeps a running estimate of each base instead of taking every solve. The
  solver sees the newest 400 keyframes and treated every sample as independent: it
  claimed 0.5 mm while solves from each device alone disagreed by 1-4 cm, and its
  answers wandered 1-5 cm (up to 9 cm) as keyframes rotated out. Now a keyframe counts
  as one measurement at the fit's own noise, and a solve moves the base by the share
  of information its new keyframes add (Kalman-style, 1 mm floor). A solve more than
  max(5 cm, 5 sigma) off three times running is a moved base, taken at once.
* A stored or imported pose counts as 20 cm uncertain until the solver confirms it
  (at 1-5 cm, vrchat-01's imported room, 7 cm off, lingered for minutes). The OOTX reset count (rotor locks since power-up: it changes on every
  wake from standby) no longer throws the room away; the stored pose just keeps
  little weight. Re-levelling the room turns pending glide targets too (left behind,
  the next solve looked 8 cm off and was taken for a move).
* Result, session-ms-01 from the stored room: the second base moves 1.6 cm in total over
  90 s, smoothly, and < 2 mm after 50 s (it moved ~3 cm, in 1-2 cm steps, before);
  vrchat from an empty room: no move events after the first placement (were 11-50 mm
  jumps, one of 9 cm after a re-level).

### Bumped and turned bases

* A solve off in place (max(5 cm, 5 sigma)) or in turn (max(0.5 deg, 5 sigma)) three
  times running: a moved base, re-placed at once; its room epoch changes and every
  device takes the new relationship (> 2 mm / 0.03 deg) instead of waiting for 1 cm.
* lhbs bases refresh their accelerometer in the OOTX frame: a turn of more than 1.2 deg,
  twice running, relearns the base (20 cm prior, re-placed on the first solve under
  2 cm). Recalled (cached) frames are ignored. Stock bases send it once at power-up, so
  for them only the solves and the devices' own filters notice.
* Locked bases (`RoomMap::Locked`: 100+ keyframes and within 20 mm) used to move only on
  an accelerometer bump, so a nudge under 1.2 deg was never corrected (GitHub #4: every
  device jumping in turn between two poses). `Session::LightMovedLockedBase` treats a
  locked base as knocked (`RoomMap::Bump`, relearned from fresh keyframes) when, 3
  solves running (5 for one device whose keyframes span < 10 s):
  * the solve places it off by max(15 mm, 1 % of its distance) or 0.75 deg, beyond
    4 sigma; or
  * the fit rejects at least a quarter of the keyframes that see it (15+) while
    rejecting under a tenth of the others. This is what a nudge shows first: the fit
    keeps the old pose by rejecting every new keyframe (nudge-01, ch3 tapped, turned and
    moved: rejections 1 -> 85, the solve moving 2 mm; flagged at 40 s, re-placed 10.6 mm
    / 9.15 deg at 44 s, rejections back to 1-5). `SolvedBase::seen/rejected` carry it,
    and a base whose keyframes were all rejected is still reported at its room pose.
  One light relearn per base a minute. On vrchat-01 / qls-post / ms4-01 / fastmotion /
  testswing (learned room, then replayed against it) the rejection rule never fired;
  the distance rule fired once per vrchat-01 pass, on a 5 m base the learned room had
  locked 172 mm off. Open: a moved unlocked base drops all its keyframes, not only
  older ones; a relearn that never gets under 2 cm has no timeout.
* The origin can't move in the room (it is the gauge): when it is knocked, or two other
  bases are off together, it is re-solved against the best other base, and the host
  frame re-anchors on the room's epoch so the world stays put.
* Pairs with the origin are synced into the device filters only once the room is sure
  (< 5 mm), or when a filter keeps rejecting the light. Syncing onto still-converging
  solves left vrchat-01 at 5-10 % rejected light for minutes (0 % at HEAD and now).
* Simulated (a since-removed debug switch turned a base rigidly about its y axis) on
  vrchat-01, 1.5 deg at 120 s: E236DC40 5 % of light rejected for 20 s, then 0 %; the
  origin A101B835 14 % for 20 s, 4 % the next 20 s, then 0 %. Both through refinement
  and the filters' rejection path; a stock base's 1.5 deg turn stays under the move
  threshold (room sigma there is 2-5 cm), so no "moved" event.

Not effective on this data: per-row innovation gating of optical updates
(`DirectFilter::gateSigma`, 4 sigma) changed nothing measurable; it is kept as a cheap
safety net.

## Output stage (`DeviceTracker::UpdateOutput`)

From a GitHub patch (Pimax Dream Air, seated): the stillness hold is off for headsets
(`SetHoldStill`, a still head froze the view, then caught up 0.1-0.65 deg), and near
still every unexplained step glides (thresholds ramp from 10 um / 5 urad under
0.05 m/s / 0.15 rad/s back to 2 mm / 0.5 deg by 0.3 m/s / 1 rad/s; the expected motion
uses the mean of both velocities). The gravity state may sit 10 % from 9.80665 with the
improvements (stock 5 %, `DirectFilter::gravityTolerance`): a controller reading 5 % low
reset every time (#14). Reacquires back off (0.5 s doubling to 8 s, 2 s minimum on the
same base, cleared after 10 s quiet; PR #9).

## The host frame

The host sees our room's frame: the origin base at the origin, Y up by gravity, the
same every session (it is the stored room). Nothing comes from SteamVR's
lighthousedb.json, whose room was 19 cm / 4.3 deg off and went stale whenever a base
was nudged; its rooms are imported only with `LH_STEAM_DB=auto` (or a path).
`LH_HOST_FRAME=steamvr` restores the old frame: the anchor base exactly where
lighthousedb.json has it (`RawAlignment`).

The room is published in shared memory (`/dev/shm/openhouse-room-<uid>`,
`include/lh/room_shm.h`): every base's pose in the host frame, whether the room has
settled (no base moved more than 5 mm / 0.15 deg for 3 s, every visible base under 5 mm
uncertainty) and an epoch that changes when a settled room differs from the last by
1 cm / 0.3 deg. A tool that aligns another tracking system to the lighthouse room can
wait while the room settles and drop what it measured at a new epoch. Without it (the
stock driver), such tools use the runtime's base station devices.

On session-ms-01 the room settles about 27 s in, from the stored room as from none. Then
the solver still moves the second base about 3 cm over 20 s (a second epoch at 45 s):
the room solver is the next thing to steady.

## A/B switches (environment)

`LH_STOCK_FILTER` turns off all of the above except the room. The individual switches
and the stderr diagnostics were removed for the release (2026-10-25); to A/B one change,
add a temporary switch locally and remove it again. Diagnostics now go to the log at
DEBUG level (`LH_LOG_LEVEL=DEBUG`, or the viewer's DEBUG box).

## Evaluating a change

```sh
tools/eval_replay.sh ~/lh-captures/vrchat-01/input.lhraw /tmp/eval 50 8   # ~25 s on 16 cores
LH_STOCK_FILTER=1 tools/eval_replay.sh ~/lh-captures/vrchat-01/input.lhraw /tmp/eval-stock 50 8
```

It replays 50 s windows in parallel (`lhtool session --from/--to --trace`, every pose
at 100 Hz of capture time; reports before a window are still decoded so the radio
protocol and clocks are right, but feed no tracker), skipping 8 s of acquisition per
window. `--only SERIAL` replays one device.

### A knocked base the solver can't re-place yet (ms4-01)

A101B835, the origin, turned ~3 deg when it was switched off and on (accelerometer:
"knocked"); the controller then lay still, too still for the solver to gather the
keyframes to re-place it, and the controller rejected 40 % of that base's light (its
axis-0 frames: all of them) for the rest of the capture. Every correction asked to move
the base ~20 cm, beyond the stock per-update limit (10 cm, 1 deg), so each was rejected
and nothing was learned. Now, while a base of the pair relearns, the device's prior on
it is loosened (5 cm, 2 deg) and the room's stale relationship isn't synced in; for 10 s,
an oversized correction moves that base alone, by the largest step the limit allows,
and leaves the device pose and covariance as they were (scaling the whole correction
instead dragged the device off the other bases: 47 % rejected when the fourth joined).
ms4-01 after the knock: 38-40 % -> 0-5 % of the light rejected, no lost tracking.
vrchat-01 and its simulated knocks: unchanged.

## Violent motion (`LighthouseDirect::ImuUpdate`, `DirectFilter::holdSlowStates`)

testswing (Beat Saber-style swings and spins, 4 bases, 24 s): 27 % of the Index
controller's IMU samples saturated (gyro +-34.9 rad/s, accel +-78 m/s2 per axis). The
filter put the errors this made into its slow states, and each such correction tripped
a stock per-update limit, so the light was thrown away: 56 % of it rejected (altTrans,
altRot, gravTilt), the IMU misalignment run off to 15-19 deg, tracking lost and reset.

* Violent (a saturated sample, |gyro| > 25 rad/s or |accel| > 60 m/s2 on an axis, and
  0.3 s after): the slow states (16-18 base tilt and gravity, 19-30 IMU calibration,
  31+ the other bases' poses) are held, Schmidt-style: their gain rows are zeroed, so
  the light corrects the device's motion only and their covariance stays consistent.
* Held, the other bases' step limits no longer catch a bad frame, so optical frames of
  fewer than three sensor rows are rejected (controller-moving-hidden: one reflected
  sensor 0.24 rad off moved the pose 1 m in 0.3 s). A chi-square gate doesn't work
  there: the predicted spread is far too small in violent motion (median 71 per row).
* A saturated axis is a one-sided bound: the true value is at least the reading, that
  way. Predicted short of it, the filter is pulled to it (variance 0.25 rad2/s2, 4
  m2/s4); stock drops the axis, and the light alone made spins of 4500 deg/s.

testswing from the driver's room (old -> new): tracking 86.9 -> 90.6 %, valid 97.5 ->
99.5 %, lost 4 -> 0, output jumps > 5 cm 142 -> 53, > 15 cm 7 -> 3, the light used 42
-> 82 %. vrchat-01, controller-moving-hidden, the oracle tests: unchanged. Off with
LH_STOCK_FILTER; off by default in LighthouseDirect (SetImprovements).

## Beat Saber fly-aways (2026-10-22)

Fast Beat Saber play on the 4-base lhbs room threw controllers metres out of the room
("position out of bounds" resets). Captures: `/mnt/lh-captures/beatsaber-01`, `-01-2`,
`-01-3` (the last one live-tested good on Windows). A cold replay from `--from 230` of
`-01-2` reproduced it; replays from 0 often don't, so test from several `--from` points
(0 120 200 230 260). Two causes, two fixes:

* **Slave bases mislabel the sync axis.** A slave sometimes flashes a clean axis-1 sync
  while its laser sweeps axis 0 (same per-sensor angles as its previous axis-0 sweep):
  A101B835 9.5 % of its sweeps, 89AE90F2 4.8 %, D7B1AD76 1.5 %, the master E236DC40
  never. Horizontal angles read as vertical (0.33 rad off) threw the pose 0.7-1.8 m.
  `MultiDisambiguator` now drops a sweep whose axis breaks the station's 0 -> 1 order
  (logged as "swept axis N out of sequence"). The firmware cause is open: `ai/firmware/MULTISTATION.md`.
* **Whole frames far off the prediction.** The row gate stands aside when most rows are
  out, so such a frame (40-90 cm away, mostly single-axis, often an alternate base) was
  applied in full and the filter followed the wrong track. `DirectFilter::Update` rejects
  a frame with most rows beyond the gate whose correction would move the position more
  than 10 cm, once settled (200 updates since bootstrap/reacquire), at most 6 in a row.
  It acts in violent motion too (nothing separated bad light from real corrections
  there). Tried and dropped: a sigma gate (> 100) doubled vrchat-01's coasting
  (still devices have a tiny predicted spread); skipping violent motion let a mid-game
  fly-away in `-01-3` back.

Results (Linux, gate on vs off): 15 Beat Saber replays clean except a startup reset
(6.4 s into `-01-2`, both ways); vrchat-01 valid 94.2 vs 93.7 %, lost 5.8 vs 6.3 %, pos
> 5 cm 887 vs 994, coast 5.9 vs 4.9 %; testswing valid 84 vs ~94 % (the accepted cost);
ms4-01 unchanged.

Also fixed: `SolveRoom` marked a base used before any of its residuals passed the
facing gate, then called `SetManifold` on a block not in the problem: a fatal Ceres
abort (one vrchat-01 replay window crashed).

## Room solver and Windows performance (2026-10-22)

Windows replays ran at about half real time; Linux far faster. Profiled: 87 % in
`Eigen::SparseQR::factorize` from `ceres::Covariance` (vcpkg Ceres has no SuiteSparse,
so it falls back to Eigen's sparse QR, hundreds of MB). Changes, each checked against
the old path:

* Base covariance by Schur complement (`BaseCovariances`): every residual ties one
  keyframe to one base, so keyframes are eliminated as 6x6 blocks and only the base
  system is inverted. Matches `ceres::Covariance` to 1e-10.
* `AngleCost`: the rigid transform differentiated by hand, autodiff only for the sweep
  model against the base-frame point (Jet<3> instead of Jet<14>). Matches autodiff in
  the tangent space to 1e-15.
* Solve backoff: after a solve that moved no base > 3 mm / 0.15 deg since the previous
  solve, the interval doubles up to 4 s (1 s on any change, 0.1 s while a base is being
  located). The movement is logged at DEBUG level.
* `platform::SleepFor`: Windows `sleep_for` rounds to the 15.6 ms timer tick (measured
  15.4 ms for 500 us), which held the driver loop and USB reads to ~64 passes/s; a
  high-resolution waitable timer sleeps ~1 ms.

Full 298 s Beat Saber replay on Windows: 148 s -> 65 s. Real-time replay (`--realtime`)
now runs the warm-up at full speed instead of sleeping through it.

## Fast motion and fresh rooms (2026-10-26)

Recordings (owner's machine, `~/lh-captures`): `fastmotion3`/`4`/`5` (one tracker shaken
harder and harder, 4 lhbs bases; `5` has 70-95 % of IMU samples saturated for 8 s) and
`fastmotionindex` (both Index controllers, Beat Saber-style swings up to 6-7 m/s). The
Beat Saber captures are gone; `fastmotionindex` replaces them. Score continuous replays
against a copy of the room; the windowed scorer learns each window's room from scratch.

* **Multi-station slot numbering** (`disambiguator_multi`): one stray early pulse shifted
  every slot for good, pushing slots 3-4 out of range (the tracker never saw ch3/ch4).
  Early stations now need 30 of the last 64 groups; a station's own OOTX slot renumbers.
* **Fly-away calibration**: after an out-of-bounds reset the filter's calibration is
  dropped and the one from the last calm, well-lit stretch (`calm_`, every 100 updates
  without held slow states) is carried instead.
* **Saturated accelerometer**: jerk process noise x10 on a saturated accel sample
  (`kSaturatedJerkScale`); x100 was worse, gyro too added rotation spikes.
* **After a reseed** (starts at 201 updates, so the frame gate was on at once): the other
  bases' light isn't frame-gated for 100 updates; the reacquired base stays gated.
* **Refused light**: 0.5 s of well-lit frames all refused -> reacquire with the calm
  calibration (`kRefusedLightReset`), instead of the 2 s timeout / running out of bounds.
* **Room**: a base is first placed from >= 12 keyframes (`kMinLocateKeyframes`); from one,
  mid-swing, it was 1.3 m off.
* **Output glide** across a deliberate base switch: the glide clock advances while the
  pose is held, so the switch glides instead of stepping (vrchat-01: an 18 cm step).
* Tried and dropped: keyframes only below a speed (slowed vrchat-01's room), loosening
  the motion covariance instead of reacquiring, counting gated frames as contradictions.

Results vs the start of the day: fastmotion3 fly-aways 9 -> 0, fastmotion4 5 -> 0,
fastmotion5 5 -> 0 (coasting ~5 -> 0.6 %); fastmotionindex in the room: no losses, 1 jump
> 5 cm in 32 s; fresh-room jumps > 5 cm: fastmotionindex 92 -> 28, spin-01 258 -> 57;
vrchat-01 > 5 cm 5 -> 3; ms4-01 and the LH2 sessions unchanged. Cost: testswing valid
37 -> 29 % (drops out instead of tracking a wrong pose; fewer jumps).
