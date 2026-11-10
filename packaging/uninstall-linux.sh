#!/usr/bin/env bash
# Puts SteamVR's own lighthouse driver back. Your room and settings in ~/.config/lh are kept.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
. "$here/.steamvr-dir.sh"
# The update check goes first, so it's removed even when there's no driver backup.
check_registration unregister
remove_monado_check
# Only the files the installer put there: the folder is also the driver's data directory,
# with your recordings (Record Session) in it.
rm -f "$check_dir/openhouse-check" "$check_dir/openhouse-check.vrmanifest" "$check_dir/openhouse-monado-watch"
rmdir "$check_dir" 2>/dev/null || true  # only goes when nothing else is in it
remove_schema
if ! grep -qa "liblh session started" "$target"; then
  # A SteamVR update puts its own, newer driver in place of ours. The backup is then the
  # older one: moving it back would undo the update, so it goes instead.
  rm -f "$backup"
  rm -rf "$dir/lh-libs"
  echo "SteamVR's own driver is already in place (a SteamVR update put it back). Your room and settings in ~/.config/lh are kept."
  exit 0
fi
[[ -f "$backup" ]] || { echo "SteamVR's driver backup isn't there ($backup). Nothing to undo, or verify SteamVR's files in Steam." >&2; exit 1; }
mv -f "$backup" "$target"
rm -rf "$dir/lh-libs"
echo "SteamVR's own driver is back. Your room and settings in ~/.config/lh are kept."
