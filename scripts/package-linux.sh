#!/usr/bin/env bash
# Packages the Linux driver and tools from a release build:
#   scripts/package-linux.sh [BUILD_DIR]     (default build-release)
# Output: dist/openhouse-driver/ and dist/openhouse-driver-linux.tar.gz
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
build="${1:-$repo/build-release}"
out="$repo/dist/openhouse-driver"
rm -rf "$out"
mkdir -p "$out/tools"
python3 "$repo/scripts/package-driver.py" "$build/openvr/lighthouse/bin/linux64/driver_lighthouse.so" "$out" | tail -n1
cp "$repo/packaging/install-linux.sh" "$out/install.sh"
cp "$repo/packaging/uninstall-linux.sh" "$out/uninstall.sh"
cp "$repo/packaging/steamvr-dir.sh" "$out/.steamvr-dir.sh"
cp "$repo/packaging/settingsschema.vrsettings" "$out/"
# Notices: openhouse's own, the vcpkg libraries built in, and what the tools link or embed.
cp "$repo/LICENSE" "$out/LICENSE.txt"
prefix=$(sed -n 's/^VCPKG_INSTALLED_DIR:[A-Z]*=//p' "$build/CMakeCache.txt")/$(sed -n 's/^VCPKG_TARGET_TRIPLET:[A-Z]*=//p' "$build/CMakeCache.txt")
for copyright in "$prefix"/share/*/copyright; do
  [[ -f "$copyright" ]] && cp "$copyright" "$out/lh-licenses/$(basename "$(dirname "$copyright")").txt"
done
cp "$repo/third_party/hidapi/LICENSE-bsd.txt" "$out/lh-licenses/hidapi.txt"
cp "$repo/third_party/imgui/LICENSE.txt" "$out/lh-licenses/imgui.txt"
for package in openvr glfw systemd; do
  # -L: copies of what the notices link to (systemd's MIT-0.txt is a link into /usr/share/doc)
  [[ -d /usr/share/licenses/$package ]] && { cp -rL "/usr/share/licenses/$package" "$out/lh-licenses/" 2>/dev/null || true; }
done
find "$out/lh-licenses" -xtype l -delete  # a link whose target wasn't there
# lhtool and openhouse-compare use liblh, shipped next to them; no tool keeps the build
# directory as its RUNPATH.
cp -L "$build/liblh.so.1" "$out/tools/"
strip --strip-unneeded "$out/tools/liblh.so.1"
for tool in lhtool lhcapture openhouse-viewer openhouse-check openhouse-compare; do
  cp "$build/$tool" "$out/tools/"
  strip "$out/tools/$tool"
  if readelf -d "$out/tools/$tool" | grep -q 'liblh\.so'; then
    patchelf --set-rpath '$ORIGIN' "$out/tools/$tool"
  else
    patchelf --remove-rpath "$out/tools/$tool"
  fi
done
tar czf "$repo/dist/openhouse-driver-linux.tar.gz" -C "$repo/dist" openhouse-driver
echo "Packaged $repo/dist/openhouse-driver-linux.tar.gz"
