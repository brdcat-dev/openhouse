#!/usr/bin/env bash
# Installs openhouse into SteamVR: replaces drivers/lighthouse/bin/linux64/driver_lighthouse.so
# and keeps SteamVR's own as driver_lighthouse.so.stock (openhouse loads it for the
# headset's display). Run it again after a SteamVR update. STEAMVR_DIR overrides where
# SteamVR is.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
. "$here/.steamvr-dir.sh"
marker="liblh session started"
if [[ ! -f "$backup" ]] || ! grep -qa "$marker" "$target"; then
  # Not ours: SteamVR's (first install, or a SteamVR update put a newer one in place).
  grep -qa "$marker" "$target" && { echo "openhouse is installed but SteamVR's driver backup is gone. Verify SteamVR's files in Steam, then run this again." >&2; exit 1; }
  cp -pf "$target" "$backup"
  echo "Saved SteamVR's driver as $backup"
fi
mkdir -p "$dir/lh-libs"
cp -a "$here/lh-libs/." "$dir/lh-libs/"
rm -f "$target.new"
cp "$here/driver_lighthouse.so" "$target.new" && mv -f "$target.new" "$target"
install_schema  # the openhouse page in SteamVR's settings
# The update check: warns in SteamVR's next session if an update replaced this driver.
mkdir -p "$check_dir"
cp "$here/tools/openhouse-check" "$check_dir/openhouse-check"
cat > "$check_dir/openhouse-check.vrmanifest" <<MANIFEST
{
   "source": "openhouse",
   "applications": [{
      "app_key": "$check_key",
      "launch_type": "binary",
      "binary_path_linux": "$check_dir/openhouse-check",
      "arguments": "\"$dir\"",
      "is_dashboard_overlay": true,
      "strings": { "en_us": { "name": "openhouse update check", "description": "Tells you when a SteamVR update replaced the openhouse driver" } }
   }]
}
MANIFEST
check_registration register
if [[ -f "$monado_unit" ]]; then
  install_monado_check   # installed before: keep it, with this install's paths
elif [[ -t 0 ]] && command -v systemctl >/dev/null; then
  echo
  echo "If you use Monado or WiVRn: a SteamVR update can also replace openhouse there, and"
  echo "you wouldn't be told. A small background service can check each time Monado or"
  echo "WiVRn starts, and show the same warning."
  read -r -p "Install it? [y/N] " answer
  if [[ $answer == [yY]* ]]; then install_monado_check; echo "Installed the Monado / WiVRn update check."; fi
fi
echo "Installed. Start SteamVR."
