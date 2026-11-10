# Windows base-station Bluetooth power management

Owner scope: Windows and SteamVR, Index headset, four LH2 stations. Overnight
SteamVR restarts and station sleep/wake tests are authorized. The owner requested
concurrent commands followed by verification, preferring PC Bluetooth when
available and the default Index controller otherwise. The Bluetooth work is preserved in the upstream merge.

**Latest checkpoint (22:24 AEDT): partial fixes installed; standby reconnect remains open.**
Follow-up source changes are uncommitted. The Windows await helper now preserves
failed/cancelled operation HRESULTs rather than replacing every non-completion with
a timeout. Error messages identify discovery, write and verification stages. Failed
connection attempts are retained until verification finishes, so synchronous Windows
cleanup cannot exhaust the responsive stations' verification window. Each retry still
creates a fresh connection. Added Windows-only tests for success/error/expired awaits
and invalid batch inputs; all ten CTest suites pass.

Installed bridge SHA256: `6D0B45726DE6EEEF64261F9DCCFE8E1A012CD3C015828B36B28842E197FE0FF9`.
Tracking DLL remains the merged A1338668... build below. Native SteamVR PID 27724
passed four PC wake confirmations (3.328s), four metadata requests (one Index fallback),
four PC sleep confirmations (2.708s), and independent uncached awake/asleep readbacks.
Native shutdown completed in 2.75s, with no startup sleep, initialization retry or
NRF timeout. SteamVR is closed and all four stations are asleep. Evidence:
`.cache/diagnostic-fixes-native-cycle.txt` and `.cache/diagnostic-fixes-native-*`.

The remaining reproduction is four stations put into standby (2), followed immediately
by a separate process requesting wake (1) or sleep (0). Some service discoveries time
out at the six-second write deadline; subsequent Close calls can block another 10-13s.
Responsive stations now get verified despite that failure. A separate-process normal
wake-to-sleep batch passed all four (2.608s/2.827s), as did one single-station
standby-to-sleep test (2.329s/3.056s). These are observations, not proof of a root cause.

Experiments with cached discovery, an earlier session/connection wait, a longer request
budget, default address lookup, removing MaintainConnection, serial cleanup, a 100ms
cleanup delay and two-at-a-time discovery did not fix the four-station standby case;
all those production experiments were removed. Advertisements confirm random address
type 1 for all four stations. An ETW BTHPORT trace could not start: Windows returned
Access denied and requires administrator rights. No trace session or instrumentation
was left running. The installed bridge retains Index fallback; do not mark the
broader Bluetooth reliability goal complete.

**Previous checkpoint (21:43 AEDT): merged upstream b4070bc into the Windows Bluetooth branch.**
Native MSVC Release build and all nine CTest suites pass, including six Bluetooth
unit cases and the native installer's unknown-SDK preservation test. The merged
native installer deployed both libraries successfully. Installed SHA256:

* tracking driver (4,532,224 bytes): `A13386686C7B3D0FC91906DA828158E669DA36E4DBABC0A45BA607E66ECE3429`
* Bluetooth bridge (580,608 bytes): `18721D371967F96A2ED5D290EB5789BD080AC595579658D1AD680B619B33CC21`

SteamVR PID 2544 woke all four stations through one PC batch (3.218s); independent
uncached reads confirmed all awake. All four metadata requests succeeded, including
one automatic fallback to Index after a failed PC read. Normal shutdown confirmed
all four asleep in one PC batch (2.804s; native shutdown 2.84s), without transport
initialization retries, NRF timeouts, or a startup sleep command.

A same-process standby-to-sleep test passed all four in both phases. A subsequent
separate-process standby attempt failed all four PC writes within the budget.
The intermittent Windows GATT issue therefore remains open; do not describe the
broader Bluetooth reliability goal as complete. The earlier forced-Index test
passed before the upstream merge; no forced-Index-only power test was repeated yet.
No Frida instrumentation remains attached.

## Current implementation

SteamVR retains its management UI, selected stations, wake/shutdown/inactivity
policy and sleep/standby setting. The bridge hooks the existing station-operation
interface instead of adding a competing driver standby worker.

The native MSVC build produces an additional **bluetooth/nrfserlib.dll**, installed
in SteamVR/bin/win64. The original Nordic library remains available as
nrfserlib_index.dll; an untouched original backup is kept separately. All 81 public
Nordic C exports are forwarded to the original implementation.

For LH2 power requests, the bridge:

* Checks for an enabled PC BLE adapter; absence/errors retain the original Index
  operation. Failed or unconfirmed PC station operations also fall back to Index.
* Reads the enabled entries from driver_lighthouse/PowerManagedBaseStations2 in
  SteamVR's active settings file. It uses addresses from native discovery, with
  PC scanning if a selected station is missing from that cache.
* Starts all selected station writes concurrently, waits for all write attempts,
  then polls all acknowledged stations concurrently under a shared nine-second
  GATT budget. Each target has its own acknowledgement/confirmation result.
* Keeps the confirmed batch results for SteamVR's remaining per-station calls.
  Only a station whose result is unconfirmed needs the Index operation.
* Performs routine LH2 device-information queries through PC Bluetooth as well,
  avoiding an immediate switch to Index GATT connections after a PC batch.
* Leaves native discovery, LH1 operations, firmware update and other maintenance
  methods on their existing implementation.

Commands are 01 awake, 00 sleep, 02 standby. Sleep from standby performs the
firmware's required wake-before-sleep sequence. Reported awake states 01, 09 and
0B normalize to awake; 00 means sleep and 02 means standby. Unknown values never
count as confirmation. The observed 0B state is normal, not failed wake-up.
Readback proves the reported power mode, not measured motor RPM.

Source is under src/bluetooth/. The power coordinator and protocol parsers have
portable unit tests; the Windows transport uses the installed C++/WinRT SDK with
no new third-party dependency. GATT devices, services and sessions are closed after
use. Native JSON values are constructed/assigned with the original library's own
allocator and operations, avoiding cross-runtime ownership of its objects.

The driver also exposes headset-contained 2102 radios to the stock HMD component
on Windows. Device-container matching preserves external 2101/2102 isolation.
OOTX hardware version and nonce are published as SteamVR station properties.
USB transport and tracking remain separate.

## Version boundary and BT-213

The installed OS is **Windows 10 Pro 22H2, build 19045**. Do not assume Windows 11.

BT-213 was reproduced as a timeout in the Index radio's Nordic H5 serial handshake
on COM3. A standalone test using the same native library passed eight scans at
default timing; adding timeBeginPeriod(1) reproduced 15 timeouts. Passive COM3
tracing showed the final 100ms CONFIG retry preceding the peer's CONFIG. Increasing
the data-link retry interval to 200ms removed the observed handshake failure in
native startup/shutdown tests. This changes the data-link interval, not the BLE
power value or the RPC response timeout.

The installer applies that one-byte correction only to the working Index copy.
The untouched backup remains available for restore. It accepts only these original
library hashes; an unknown SteamVR library requires revalidation, not guessed offsets:

* Original: 55B09BDB27E475A10EEBF3253B1882380DB04B5D6E003344B068D5F189F62E72
* Corrected: AB62269C901E92D786462C61BA83B769470D3AF05AD084A27852418E29214F94

The correction is file offset 0x17e88 (instruction immediate at RVA 0x18a88):
100 becomes 200. Runtime interface hooks are also hash-guarded. The supported
interface has 14 vtable entries: callbacks 0, discovery 5, properties 6, LH2 power 10.
Native scan entries are 0x90 bytes (name at 8, MAC string at 40); MSVC release string
layout is 32 bytes. JSON number/string constructors, member access and move
assignment are at RVAs 0x60270, 0x60210, 0x609c0 and 0x60640. These are compatibility
boundaries, not a claim that another library build has the same ABI.

Other original provenance (Windows image base 0x180000000):

* Stock driver 0x180061af0: hardware ID = 0x09000000 + OOTX hardwareVersion.
* Stock driver 0x180011550 / 0x180061fd0: station properties and nonce.
* Stock driver 0x180020770 / 0x18002cb80: Index radio selection and BLE mode.
* Nordic wrapper 0x180018a00: serial stack setup; 0x18001b1e0: close.
* Nordic 0x180008890: DIS characteristic mapping and firmware components.
* Nordic 0x18001cb10: property flags and native JSON results.

No Valve binaries, decompiled bodies or firmware are distributed in the source.
Analysis snapshots, traces, hardware logs and original-file test fixtures are
ignored under .cache/ and build-bt-probe/. The patch/bridge is superseded if SteamVR
replaces it; an updated library must pass the version checks before reinstalling.

## Installation and restore

Close SteamVR first. From the repo after a native MSVC build:

    powershell -NoProfile -ExecutionPolicy Bypass -File scripts/windows-bluetooth.ps1 -Action Apply -SteamVRPath "C:/Program Files (x86)/Steam/steamapps/common/SteamVR" -BridgePath build-windows/bluetooth/nrfserlib.dll

Status and restore use the same command with -Action Status or -Action Restore;
BridgePath is unnecessary for those actions. The original backup is
bin/win64/nrfserlib.dll.openhouse-backup. Restore keeps it.

The Windows packager includes the bridge when that build artifact exists and
copies the helper as openhouse-bluetooth.ps1. The driver installer applies/restores
it with the rest of the package. Apply/restore are tested on a workspace copy;
unknown files and unexpected backups are preserved. File replacement is atomic.

The separate tracking driver is still
drivers/lighthouse/bin/win64/driver_lighthouse.dll, with its original .stock backup.
Only the C: SteamVR installation was changed; the other F: installation is untouched.

## Hardware evidence (2026-11-10, AEDT)

Index headset: LHR-CDAF5410. PC BLE radio: Realtek USB 0BDA:A728. Index radio:
28DE:2102, serial 86CB03F38F-RYB, COM3. Headset/radios share Windows container
00000000-0000-0000-0000-0083C4551486.

| Station | BLE address |
|---|---|
| LHB-87660BC4 | FD:BB:F1:DC:AC:7D |
| LHB-2E59B706 | FF:39:26:AE:37:A4 |
| LHB-1055D06E | CD:E8:96:FA:0E:6B |
| LHB-F4A0EE85 | E6:E8:AD:CB:2A:D8 |

The owner changed basestationPowerManagement to 1 (sleep) after the overnight
standby tests. This current value is preserved; do not restore the earlier value 2.

Three normal SteamVR cycles passed between 20:48 and 20:52: exactly one PC batch per
wake and one per sleep, four confirmed states in each batch, four successful routine
information queries, no startup sleep command, no serial retry or timeout, and
independent uncached GATT readback for all four awake and asleep states.
Native shutdown was about three seconds (2.85 seconds in cycle 1).
Evidence: .cache/bt-pc-cycle-1-*.txt and .cache/bt-persistent-cycle-{2,3}-*.txt.

The final compiled bridge adds only managed-name validation/refactoring after those
three cycles. Its additional startup/shutdown passed with four PC confirmations and
successful routine information queries; shutdown was 3.15s. Saved log:
.cache/bt-final-build-pc-cycle.txt.

Pre-merge tracking DLL SHA256 (superseded by the checkpoint above):
D6FD9E66444645D8A310CC2CB5A9BB9ECFC99608E79AC47D7F5ED0811B1D06F0.
Pre-merge bridge SHA256 (superseded by the checkpoint above):
DD950F6BCA55C44BE99DAA2CE8EE48CA6131459359E67D0B3A46F780A83D4F7D.

## Build and regression checks

Native MSVC 14.44, Windows SDK 10.0.26100.0, Release, static AVX2 vcpkg triplet.
Ignored build-bt-probe/configure.cmd and build-project.cmd contain the exact local
commands. Native build command uses the pinned workspace vcpkg checkout, OpenVR
headers under .cache/openvr, LH_BUILD_VIEWER=OFF and the absolute triplet overlay.

All nine configured CTest suites pass after the upstream merge. Six additional
Bluetooth checks live in the unit suite: managed selection, version parsing, reported
mode decoding, concurrent writes before polling, missing/delayed station state, and failed-write retries.
Real-DLL OpenVR tests still check station properties and normal tracking/replay.

A complete release ZIP, a new MinGW cross-build, Linux hardware and LH1 hardware
have not been validated here. The C++/WinRT bridge currently requires native MSVC.

## DLL size question

The tracking DLL is 4,486,656 bytes (4.28 MiB), MSVC linker 14.44. The pre-existing
.new fork build is 8,029,184 bytes (7.66 MiB), GNU/MinGW linker 2.47.
Their executable-code sections are 0x366400 and 0x678600 bytes; most of the reduction
is compiled code. USB, OpenVR, tracking and Ceres Schur specializations are enabled.
Only separate diagnostic viewers were disabled. No tracking algorithm was removed,
but no function-by-function binary-equivalence proof has been made. The new Bluetooth
bridge is a separate 563,712-byte DLL.
