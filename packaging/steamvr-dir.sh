# Shared by install.sh and uninstall.sh: finds SteamVR and refuses while a runtime runs.
find_steamvr() {
  if [[ -n "${STEAMVR_DIR:-}" ]]; then echo "$STEAMVR_DIR"; return; fi
  for d in "$HOME/.local/share/Steam/steamapps/common/SteamVR" "$HOME/.steam/steam/steamapps/common/SteamVR" \
           "$HOME/.steam/root/steamapps/common/SteamVR"; do
    [[ -d "$d/drivers/lighthouse" ]] && { echo "$d"; return; }
  done
  echo "SteamVR not found; set STEAMVR_DIR" >&2; exit 1
}
# Anything with the driver loaded (or about to load it) must be closed first: the
# installer offers to close it.
runtimes=(vrserver vrmonitor vrcompositor vrdashboard vrwebhelper vrstartup monado-service wivrn-server)
running_runtimes() {
  local name found=()
  for name in "${runtimes[@]}"; do pgrep -x "$name" >/dev/null && found+=("$name"); done
  echo "${found[*]:-}"
}
running=$(running_runtimes)
if [[ -n $running ]]; then
  echo "These are running and use the lighthouse driver: $running"
  if [[ ! -t 0 ]]; then echo "Close them first, then run this again." >&2; exit 1; fi
  read -r -p "Close them now? Unsaved work in VR apps is lost. [y/N] " answer
  [[ $answer == [yY]* ]] || { echo "Close them first, then run this again." >&2; exit 1; }
  for name in $running; do pkill -x "$name" 2>/dev/null || true; done
  for _ in $(seq 1 50); do [[ -z $(running_runtimes) ]] && break; sleep 0.2; done
  if [[ -n $(running_runtimes) ]]; then
    for name in $(running_runtimes); do pkill -9 -x "$name" 2>/dev/null || true; done
    sleep 1
  fi
  [[ -z $(running_runtimes) ]] || { echo "Couldn't close: $(running_runtimes). Close them by hand, then run this again." >&2; exit 1; }
  echo "Closed."
fi
dir="$(find_steamvr)/drivers/lighthouse/bin/linux64"
target="$dir/driver_lighthouse.so"
backup="$target.stock"
[[ -f "$target" ]] || { echo "No lighthouse driver at $target; set STEAMVR_DIR" >&2; exit 1; }

# The update check (openhouse-check): an app SteamVR starts with every session, registered
# in Steam's config (appconfig.json lists app manifests, vrappconfig/ holds auto-launch).
check_key="openhouse.check"
check_dir="${XDG_DATA_HOME:-$HOME/.local/share}/openhouse"
steam_config() {
  local vrpath="${XDG_CONFIG_HOME:-$HOME/.config}/openvr/openvrpaths.vrpath" c=""
  [[ -f "$vrpath" ]] && command -v python3 >/dev/null &&
    c=$(python3 -c 'import json,sys; print((json.load(open(sys.argv[1])).get("config") or [""])[0])' "$vrpath" 2>/dev/null)
  [[ -n "$c" ]] || c="$(cd "$dir/../../../../../../.." && pwd)/config"   # .../Steam/config
  echo "$c"
}
# register|unregister: edits appconfig.json and the app's vrappconfig.
check_registration() {
  command -v python3 >/dev/null || { echo "python3 not found: SteamVR update warning not $1ed." >&2; return 0; }
  python3 - "$1" "$(steam_config)" "$check_dir/openhouse-check.vrmanifest" "$check_key" <<'PY'
import json, os, sys
mode, config, manifest, key = sys.argv[1:]
path = os.path.join(config, 'appconfig.json')
try:
    with open(path) as f: data = json.load(f)
except (FileNotFoundError, ValueError): data = {}
paths = [p for p in data.get('manifest_paths') or [] if os.path.normpath(p) != os.path.normpath(manifest)]
if mode == 'register': paths.append(manifest)
data['manifest_paths'] = paths
os.makedirs(config, exist_ok=True)
with open(path + '.new', 'w') as f: json.dump(data, f, indent=3); f.write('\n')
os.replace(path + '.new', path)
app = os.path.join(config, 'vrappconfig', key + '.vrappconfig')
if mode == 'register':
    os.makedirs(os.path.dirname(app), exist_ok=True)
    with open(app, 'w') as f: json.dump({'autolaunch': True, 'last_launch_time': '0'}, f, indent=3); f.write('\n')
elif os.path.exists(app):
    os.remove(app)
PY
}

# For Monado / WiVRn (they don't start SteamVR apps): an optional systemd user service
# that runs the same check whenever monado-service or wivrn-server starts.
monado_unit="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user/openhouse-check.service"
install_monado_check() {
  cat > "$check_dir/openhouse-monado-watch" <<WATCH
#!/usr/bin/env bash
# Runs openhouse-check once each time Monado or WiVRn starts.
running=0
while sleep 3; do
  if pgrep -x monado-service >/dev/null || pgrep -x wivrn-server >/dev/null; then
    [[ \$running == 0 ]] && "$check_dir/openhouse-check" "$dir"
    running=1
  else
    running=0
  fi
done
WATCH
  chmod +x "$check_dir/openhouse-monado-watch"
  mkdir -p "$(dirname "$monado_unit")"
  cat > "$monado_unit" <<UNIT
[Unit]
Description=openhouse: warn when a SteamVR update replaced the openhouse driver (Monado / WiVRn)

[Service]
ExecStart=$check_dir/openhouse-monado-watch
Restart=on-failure

[Install]
WantedBy=default.target
UNIT
  systemctl --user daemon-reload 2>/dev/null || true
  systemctl --user enable openhouse-check.service >/dev/null 2>&1 || echo "Couldn't enable the service (no systemd user session?)." >&2
  systemctl --user restart openhouse-check.service >/dev/null 2>&1 || true
}
remove_monado_check() {
  [[ -f "$monado_unit" ]] || return 0
  systemctl --user disable --now openhouse-check.service >/dev/null 2>&1 || true
  rm -f "$monado_unit"
  systemctl --user daemon-reload 2>/dev/null || true
}

# The openhouse page in SteamVR's settings: a settings schema in the lighthouse driver's
# resources (SteamVR has none there today; one it ships later is kept as .stock).
schema="$(cd "$dir/../.." && pwd)/resources/settings/settingsschema.vrsettings"
install_schema() {
  mkdir -p "$(dirname "$schema")"
  if [[ -f "$schema" ]] && ! grep -q driver_openhouse "$schema"; then cp -p "$schema" "$schema.stock"; fi
  cp "$here/settingsschema.vrsettings" "$schema"
}
remove_schema() {
  [[ -f "$schema" ]] && grep -q driver_openhouse "$schema" && rm -f "$schema"
  if [[ -f "$schema.stock" ]]; then
    # Still there: a SteamVR update replaced ours with a newer one, so the backup is older.
    if [[ -f "$schema" ]]; then rm -f "$schema.stock"; else mv "$schema.stock" "$schema"; fi
  fi
  return 0
}
