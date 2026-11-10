// openhouse-installer.exe: installs or removes openhouse in SteamVR on Windows. It
// replaces the Install-Driver.bat / openhouse-driver.ps1 pair: a .bat starting
// "powershell -ExecutionPolicy Bypass" is a common malware pattern, and Windows Defender
// flagged the package for it. Same steps as the script it replaces:
//   * finds SteamVR (openvrpaths.vrpath, else Steam's registry key; --steamvr PATH);
//   * offers to close Steam / SteamVR (they hold the driver DLL open);
//   * install: keeps SteamVR's driver as driver_lighthouse.dll.stock whenever the one in
//     place isn't openhouse's (a SteamVR update's newer one replaces an older backup),
//     copies ours over it, the settings page schema, and the update check (an app SteamVR
//     starts with every session: Steam's appconfig.json + vrappconfig);
//   * uninstall: puts the backup back, unless a SteamVR update already put its own
//     newer driver there (then the outdated backup goes instead), and removes the update
//     check's files from %LOCALAPPDATA%\openhouse, not the folder: recordings live there.
//
//   openhouse-installer [--install | --uninstall | --status] [--steamvr PATH] [--yes]
// Without an action it asks. Files are overwritten in place, retrying for a few seconds
// (a file a closed process just released can stay locked briefly).
#include <windows.h>
#include <tlhelp32.h>
#include <bcrypt.h>

#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using nlohmann::ordered_json;

namespace {

constexpr const char* kMarker = "liblh session started";  // only openhouse's driver has it
constexpr const char* kCheckKey = "openhouse.check";
const wchar_t* kHosts[] = {L"steam.exe",       L"steamwebhelper.exe", L"vrserver.exe",  L"vrmonitor.exe",
                           L"vrcompositor.exe", L"vrdashboard.exe",    L"vrwebhelper.exe", L"vrstartup.exe",
                           L"vrservice.exe",   L"monado-service.exe"};

struct Failure : std::runtime_error {
  using std::runtime_error::runtime_error;
};

bool g_yes = false;

std::string Narrow(const std::wstring& w) {
  if (w.empty()) return {};
  const int n = WideCharToMultiByte(CP_UTF8, 0, w.data(), int(w.size()), nullptr, 0, nullptr, nullptr);
  std::string s(size_t(n), '\0');
  WideCharToMultiByte(CP_UTF8, 0, w.data(), int(w.size()), s.data(), n, nullptr, nullptr);
  return s;
}
std::string Show(const fs::path& p) { return Narrow(p.wstring()); }

fs::path EnvPath(const wchar_t* name) {
  wchar_t buffer[MAX_PATH * 2];
  const DWORD n = GetEnvironmentVariableW(name, buffer, DWORD(std::size(buffer)));
  return n && n < std::size(buffer) ? fs::path(buffer) : fs::path();
}

std::string ReadFile(const fs::path& p) {
  std::ifstream in(p, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

void WriteText(const fs::path& p, const std::string& text) {  // UTF-8, no byte order mark
  fs::create_directories(p.parent_path());
  std::ofstream out(p, std::ios::binary | std::ios::trunc);
  out << text << "\n";
  if (!out) throw Failure("Could not write " + Show(p));
}

bool Ask(const std::string& question) {
  if (g_yes) return true;
  std::cout << question << " [y/N] " << std::flush;
  std::string answer;
  std::getline(std::cin, answer);
  return !answer.empty() && (answer[0] == 'y' || answer[0] == 'Y');
}

// Overwrites `to`, retrying for ~5 s.
void CopyOver(const fs::path& from, const fs::path& to) {
  for (int i = 0;; ++i) {
    std::error_code ec;
    fs::copy_file(from, to, fs::copy_options::overwrite_existing, ec);
    if (!ec) return;
    if (i >= 20)
      throw Failure("Could not write " + Show(to) + " - a process still has it open. Close Steam and SteamVR completely, then run this again. (" +
                    ec.message() + ")");
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
}

void RemoveFile(const fs::path& p) {
  std::error_code ec;
  fs::remove(p, ec);
}

// Only this SDK version has the private interface used by the Bluetooth bridge.
constexpr const char* kSdkHash = "55b09bdb27e475a10eebf3253b1882380db04b5d6e003344b068d5f189f62e72";
constexpr const char* kFixedSdkHash = "ab62269c901e92d786462c61ba83b769470d3af05ad084a27852418e29214f94";
constexpr const char* kBluetoothMarker = "openhouse Bluetooth: ";

std::string Sha256(const std::string& bytes) {
  unsigned char hash[32];
  if (BCryptHash(BCRYPT_SHA256_ALG_HANDLE, nullptr, 0,
                 reinterpret_cast<PUCHAR>(const_cast<char*>(bytes.data())), ULONG(bytes.size()), hash, sizeof hash) < 0)
    throw Failure("Could not hash Bluetooth library");
  std::string result;
  for (unsigned char b : hash) {
    result += "0123456789abcdef"[b >> 4];
    result += "0123456789abcdef"[b & 15];
  }
  return result;
}

bool KnownSdk(const std::string& bytes) {
  const auto hash = Sha256(bytes);
  return hash == kSdkHash || hash == kFixedSdkHash;
}

void ReplaceBinary(const fs::path& target, const std::string& bytes) {
  const fs::path temporary = fs::path(target) += ".openhouse-tmp";
  try {
    std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
    out.write(bytes.data(), std::streamsize(bytes.size()));
    out.close();
    if (!out || ReadFile(temporary) != bytes) throw Failure("Could not prepare " + Show(target));
    if (!MoveFileExW(temporary.c_str(), target.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
      throw Failure("Could not replace " + Show(target) + "; close SteamVR and try again");
  } catch (...) {
    RemoveFile(temporary);
    throw;
  }
  if (ReadFile(target) != bytes) throw Failure("Installed Bluetooth library differs: " + Show(target));
}

void Bluetooth(const fs::path& here, const fs::path& steamvr, bool install) {
  const auto source = here / "bluetooth" / "nrfserlib.dll";
  const auto target = steamvr / "bin" / "win64" / "nrfserlib.dll";
  const auto backup = fs::path(target) += ".openhouse-backup";
  const auto index = target.parent_path() / "nrfserlib_index.dll";
  if (install && !fs::exists(source)) return;  // MinGW packages have no PC Bluetooth bridge.
  const auto current = ReadFile(target);
  const bool ours = current.find(kBluetoothMarker) != std::string::npos;
  if (!ours && !KnownSdk(current)) {
    if (!install) return;  // Preserve a newer SteamVR SDK on removal.
    throw Failure("Unsupported Bluetooth SDK; nothing changed. Revalidate this SteamVR version first.");
  }
  if (fs::exists(backup) && !KnownSdk(ReadFile(backup)))
    throw Failure("Unexpected Bluetooth backup; preserving " + Show(backup));
  if (ours && !fs::exists(backup)) throw Failure("Original Bluetooth backup is missing: " + Show(backup));
  if (!install) {
    if (fs::exists(backup)) ReplaceBinary(target, ReadFile(backup));
    if (fs::exists(index) && Sha256(ReadFile(index)) == kFixedSdkHash) RemoveFile(index);
    return;
  }
  const auto bridge = ReadFile(source);
  if (bridge.find(kBluetoothMarker) == std::string::npos) throw Failure("Invalid Bluetooth bridge: " + Show(source));
  if (fs::exists(index) && !KnownSdk(ReadFile(index))) throw Failure("Unexpected Index fallback: " + Show(index));
  if (!fs::exists(backup)) {
    fs::copy_file(target, backup);  // Never overwrite the original SDK backup.
    if (ReadFile(backup) != current) throw Failure("Bluetooth backup differs from original");
  }
  auto sdk = ReadFile(backup);
  if (Sha256(sdk) == kSdkHash) {
    if (sdk.size() <= 0x17e88 || static_cast<unsigned char>(sdk[0x17e88]) != 100)
      throw Failure("Unexpected Index handshake interval");
    sdk[0x17e88] = static_cast<char>(200);
  }
  if (Sha256(sdk) != kFixedSdkHash) throw Failure("Corrected Index SDK hash differs");
  ReplaceBinary(index, sdk);
  ReplaceBinary(target, bridge);
  std::cout << "Installed PC Bluetooth batching with Index fallback.\n";
}

// ---- finding SteamVR ----

ordered_json ReadJson(const fs::path& p) {
  try {
    const std::string text = ReadFile(p);
    if (text.empty()) return ordered_json::object();
    return ordered_json::parse(text);
  } catch (...) {
    return ordered_json::object();
  }
}

fs::path VrPathFile() { return EnvPath(L"LOCALAPPDATA") / "openvr" / "openvrpaths.vrpath"; }

fs::path FromUtf8(const std::string& s) {
  const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), int(s.size()), nullptr, 0);
  std::wstring w(size_t(n), L'\0');
  MultiByteToWideChar(CP_UTF8, 0, s.data(), int(s.size()), w.data(), n);
  return fs::path(w);
}

fs::path FindSteamVR() {
  const ordered_json paths = ReadJson(VrPathFile());
  if (paths.contains("runtime") && paths["runtime"].is_array())
    for (const auto& r : paths["runtime"])
      if (r.is_string() && fs::is_directory(FromUtf8(r.get<std::string>()) / "drivers" / "lighthouse" / "bin" / "win64"))
        return FromUtf8(r.get<std::string>());
  HKEY key;
  if (RegOpenKeyExW(HKEY_CURRENT_USER, L"Software\\Valve\\Steam", 0, KEY_READ, &key) == ERROR_SUCCESS) {
    wchar_t value[MAX_PATH * 2]{};
    DWORD size = sizeof value;
    const LONG r = RegQueryValueExW(key, L"SteamPath", nullptr, nullptr, reinterpret_cast<BYTE*>(value), &size);
    RegCloseKey(key);
    if (r == ERROR_SUCCESS) return fs::path(value) / "steamapps" / "common" / "SteamVR";
  }
  return {};
}

fs::path SteamConfig(const fs::path& steamvr) {
  const ordered_json paths = ReadJson(VrPathFile());
  if (paths.contains("config") && paths["config"].is_array() && !paths["config"].empty() && paths["config"][0].is_string())
    return FromUtf8(paths["config"][0].get<std::string>());
  return steamvr.parent_path().parent_path().parent_path() / "config";  // .../Steam/config
}

// ---- closing Steam / SteamVR ----

std::vector<std::pair<DWORD, std::wstring>> RunningHosts() {
  std::vector<std::pair<DWORD, std::wstring>> found;
  HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
  if (snap == INVALID_HANDLE_VALUE) return found;
  PROCESSENTRY32W e{};
  e.dwSize = sizeof e;
  for (BOOL ok = Process32FirstW(snap, &e); ok; ok = Process32NextW(snap, &e))
    for (const wchar_t* name : kHosts)
      if (_wcsicmp(e.szExeFile, name) == 0) found.emplace_back(e.th32ProcessID, e.szExeFile);
  CloseHandle(snap);
  return found;
}

void CloseHosts() {
  auto running = RunningHosts();
  if (running.empty()) return;
  std::set<std::string> names;
  for (const auto& [pid, name] : running) names.insert(Narrow(name));
  std::string list;
  for (const auto& n : names) list += (list.empty() ? "" : ", ") + n;
  std::cout << "\nSteam / SteamVR is running (" << list << "). It has the lighthouse driver open, so it\n"
               "must be closed before the driver can be changed.\n";
  if (!Ask("Close them now? Unsaved work in games is lost.")) throw Failure("Close Steam and SteamVR completely, then run this again.");
  for (const auto& [pid, name] : running)
    if (HANDLE h = OpenProcess(PROCESS_TERMINATE, FALSE, pid)) {
      TerminateProcess(h, 0);
      CloseHandle(h);
    }
  for (int i = 0; i < 50 && !RunningHosts().empty(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(200));
  if (!RunningHosts().empty()) throw Failure("Couldn't close Steam / SteamVR. Close them by hand (Task Manager), then run this again.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));  // file handles go a moment after the process
  std::cout << "Closed.\n";
}

// ---- the update check's registration ----

struct Paths {
  fs::path here, steamvr, directory, target, backup, schema, checkDir, checkManifest;
};

void SetCheckRegistration(const Paths& p, bool registerIt) {
  const fs::path config = SteamConfig(p.steamvr);
  const fs::path appconfig = config / "appconfig.json";
  ordered_json data = ReadJson(appconfig);
  if (!data.is_object()) data = ordered_json::object();
  ordered_json kept = ordered_json::array();
  const std::string manifest = Show(p.checkManifest);
  if (data.contains("manifest_paths") && data["manifest_paths"].is_array())
    for (const auto& m : data["manifest_paths"])
      if (m.is_string() && fs::path(FromUtf8(m.get<std::string>())).lexically_normal() != p.checkManifest.lexically_normal())
        kept.push_back(m);
  if (registerIt) kept.push_back(manifest);
  data["manifest_paths"] = kept;
  WriteText(appconfig, data.dump(2));
  const fs::path app = config / "vrappconfig" / (std::string(kCheckKey) + ".vrappconfig");
  if (registerIt) WriteText(app, R"({ "autolaunch" : true, "last_launch_time" : "0" })");
  else RemoveFile(app);
}

// ---- the settings page schema ----

bool HasOurSection(const fs::path& schema) { return ReadFile(schema).find("driver_openhouse") != std::string::npos; }

void InstallSchema(const Paths& p) {
  const fs::path source = p.here / "settingsschema.vrsettings";
  if (!fs::exists(source)) return;
  fs::create_directories(p.schema.parent_path());
  if (fs::exists(p.schema) && !HasOurSection(p.schema)) CopyOver(p.schema, fs::path(p.schema) += ".stock");
  CopyOver(source, p.schema);
}

void RemoveSchema(const Paths& p) {
  const fs::path stock = fs::path(p.schema) += ".stock";
  if (fs::exists(p.schema) && HasOurSection(p.schema)) RemoveFile(p.schema);
  if (fs::exists(stock)) {
    // Still there: a SteamVR update replaced ours with a newer one, so the backup is older.
    if (fs::exists(p.schema)) RemoveFile(stock);
    else { CopyOver(stock, p.schema); RemoveFile(stock); }
  }
}

// ---- actions ----

void Install(const Paths& p) {
  const fs::path ours = p.here / "driver_lighthouse.dll";
  if (!fs::exists(ours)) throw Failure("The package's driver is missing: " + Show(ours));
  Bluetooth(p.here, p.steamvr, true);
  const bool isOurs = ReadFile(p.target).find(kMarker) != std::string::npos;
  if (!isOurs) {
    CopyOver(p.target, p.backup);
    std::cout << "Saved SteamVR's driver as " << Show(p.backup) << "\n";
  } else if (!fs::exists(p.backup)) {
    throw Failure("The installed driver is already openhouse's and there is no stock backup (" + Show(p.backup) +
                  "). Verify SteamVR's files in Steam, then install again.");
  }
  CopyOver(ours, p.target);
  if (ReadFile(p.target) != ReadFile(ours)) throw Failure("The installed driver doesn't match the package's (" + Show(p.target) + ")");
  InstallSchema(p);
  const fs::path checkExe = p.here / "apps" / "openhouse-check.exe";
  if (fs::exists(checkExe)) {
    fs::create_directories(p.checkDir);
    const fs::path installed = p.checkDir / "openhouse-check.exe";
    CopyOver(checkExe, installed);
    ordered_json app;
    app["app_key"] = kCheckKey;
    app["launch_type"] = "binary";
    app["binary_path_windows"] = Show(installed);
    app["arguments"] = "\"" + Show(p.directory) + "\"";
    app["is_dashboard_overlay"] = true;
    app["strings"]["en_us"] = {{"name", "openhouse update check"},
                               {"description", "Tells you when a SteamVR update replaced the openhouse driver"}};
    ordered_json manifest;
    manifest["source"] = "openhouse";
    manifest["applications"] = ordered_json::array({app});
    WriteText(p.checkManifest, manifest.dump(2));
    SetCheckRegistration(p, true);
  }
  std::cout << "Installed. Start SteamVR.\n";
}

void Uninstall(const Paths& p) {
  Bluetooth(p.here, p.steamvr, false);
  // First, so the check goes even when there's no driver backup.
  RemoveSchema(p);
  SetCheckRegistration(p, false);
  // Only the files we put there: the folder is also the driver's data directory, with the
  // user's recordings (Record Session) in it.
  for (const char* name : {"openhouse-check.exe", "openhouse-check.vrmanifest"}) RemoveFile(p.checkDir / name);
  std::error_code ec;
  fs::remove(p.checkDir, ec);  // only goes when nothing else is in it
  if (ReadFile(p.target).find(kMarker) == std::string::npos) {
    // A SteamVR update puts its own, newer driver in place of ours. The backup is then the
    // older one: moving it back would undo the update, so it goes instead.
    RemoveFile(p.backup);
    std::cout << "SteamVR's own driver is already in place (a SteamVR update put it back). Your room and settings in "
                 "%APPDATA%\\lh are kept.\n";
    return;
  }
  if (!fs::exists(p.backup))
    throw Failure("SteamVR's driver backup isn't there (" + Show(p.backup) + "). Nothing to undo, or verify SteamVR's files in Steam.");
  CopyOver(p.backup, p.target);
  RemoveFile(p.backup);
  std::cout << "SteamVR's own driver is back. Your room and settings in %APPDATA%\\lh are kept.\n";
}

int Run(int argc, wchar_t** argv) {
  std::string action;
  fs::path steamvr;
  for (int i = 1; i < argc; ++i) {
    const std::wstring a = argv[i];
    if (a == L"--install" || a == L"install") action = "install";
    else if (a == L"--uninstall" || a == L"uninstall") action = "uninstall";
    else if (a == L"--status" || a == L"status") action = "status";
    else if (a == L"--yes" || a == L"-y") g_yes = true;
    else if (a == L"--steamvr" && i + 1 < argc) steamvr = argv[++i];
    else throw Failure("Unknown option " + Narrow(a) + ". Usage: openhouse-installer [--install | --uninstall | --status] [--steamvr PATH] [--yes]");
  }
  std::cout << "openhouse driver installer\n\n";
  if (action.empty()) {
    std::cout << "  1. Install (or update) openhouse\n  2. Uninstall openhouse (put SteamVR's driver back)\n\nChoose 1 or 2: " << std::flush;
    std::string choice;
    std::getline(std::cin, choice);
    if (choice == "1") action = "install";
    else if (choice == "2") action = "uninstall";
    else { std::cout << "Nothing changed.\n"; return 0; }
  }
  wchar_t self[MAX_PATH * 2]{};
  GetModuleFileNameW(nullptr, self, DWORD(std::size(self)));
  Paths p;
  p.here = fs::path(self).parent_path();
  p.steamvr = steamvr.empty() ? FindSteamVR() : steamvr;
  if (p.steamvr.empty()) throw Failure("SteamVR not found. Run this with --steamvr \"C:\\path\\to\\SteamVR\".");
  p.directory = p.steamvr / "drivers" / "lighthouse" / "bin" / "win64";
  p.target = p.directory / "driver_lighthouse.dll";
  p.backup = fs::path(p.target) += ".stock";
  p.schema = p.steamvr / "drivers" / "lighthouse" / "resources" / "settings" / "settingsschema.vrsettings";
  p.checkDir = EnvPath(L"LOCALAPPDATA") / "openhouse";
  p.checkManifest = p.checkDir / "openhouse-check.vrmanifest";
  if (!fs::exists(p.target)) throw Failure("Driver not found at " + Show(p.target) + ". Run this with --steamvr PATH.");
  if (action == "status") {
    std::cout << "Driver: " << Show(p.target) << "\n"
              << "openhouse installed: " << (ReadFile(p.target).find(kMarker) != std::string::npos ? "yes" : "no") << "\n"
              << "Stock backup present: " << (fs::exists(p.backup) ? "yes" : "no") << "\n";
    return 0;
  }
  CloseHosts();
  if (action == "install") Install(p);
  else Uninstall(p);
  return 0;
}

}  // namespace

int wmain(int argc, wchar_t** argv) {
  SetConsoleOutputCP(CP_UTF8);
  int rc;
  try {
    rc = Run(argc, argv);
  } catch (const std::exception& e) {
    std::cout << "\nError: " << e.what() << "\n";
    rc = 1;
  }
  bool launchedByExplorer = false;  // double-clicked: keep the window open to be read
  {
    DWORD ids[2];
    launchedByExplorer = GetConsoleProcessList(ids, 2) <= 1;
  }
  if (launchedByExplorer && !g_yes) {
    std::cout << "\nPress Enter to close." << std::flush;
    std::string line;
    std::getline(std::cin, line);
  }
  return rc;
}
