#define wmain installer_main
#include "../../apps/openhouse-installer/main.cpp"
#undef wmain
#include <gtest/gtest.h>

TEST(WindowsInstaller, HashAndUnknownSdkGuard) {
  EXPECT_EQ(Sha256("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  const auto root = fs::temp_directory_path() / ("openhouse-installer-test-" + std::to_string(GetCurrentProcessId()));
  const auto target = root / "runtime/bin/win64/nrfserlib.dll";
  fs::create_directories(target.parent_path());
  WriteText(target, "unknown SDK");
  WriteText(root / "package/bluetooth/nrfserlib.dll", kBluetoothMarker);
  EXPECT_THROW(Bluetooth(root / "package", root / "runtime", true), Failure);
  EXPECT_EQ(ReadFile(target), "unknown SDK\n");
  EXPECT_FALSE(fs::exists(fs::path(target) += ".openhouse-backup"));
  EXPECT_NO_THROW(Bluetooth(root / "package", root / "runtime", false));
  EXPECT_EQ(ReadFile(target), "unknown SDK\n");
  ReplaceBinary(target, "replacement");
  EXPECT_EQ(ReadFile(target), "replacement");
  for (const auto& file : {target, root / "package/bluetooth/nrfserlib.dll"}) fs::remove(file);
  for (const auto& dir : {target.parent_path(), root / "runtime/bin", root / "runtime",
                          root / "package/bluetooth", root / "package", root}) fs::remove(dir);
}
