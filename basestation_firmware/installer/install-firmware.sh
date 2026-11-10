#!/usr/bin/env bash
# Installs the lhbs firmware on a 1.0 base station (Linux).
#
#   ./install-firmware.sh                 back up the base station, then install
#   ./install-firmware.sh --restore FILE  put a backup back
#
# Needs: udisks2 (udisksctl), util-linux (blkid, findmnt), coreutils. No root.
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
label="CRP DISABLD"
restore=""
if [[ ${1:-} == --restore ]]; then
  restore=${2:?"usage: $0 --restore BACKUP.bin"}
fi

fail() { printf '\nError: %s\n' "$*" >&2; exit 1; }

for tool in blkid findmnt udisksctl dd od cmp; do
  command -v "$tool" >/dev/null || fail "'$tool' is missing. Install udisks2 and util-linux, then try again."
done

if [[ -n $restore ]]; then
  image=$restore
else
  image=$(ls "$here"/firmware/*.bin 2>/dev/null | head -n1 || true)
  [[ -n $image ]] || fail "no firmware file in $here/firmware"
fi
[[ -f $image ]] || fail "$image not found"

# The LPC11U37 refuses to boot an image whose first 8 words don't add up to zero.
vector_ok() {
  local sum
  sum=$(od -An -v -tu4 -N32 "$1" | tr -s ' ' '\n' | awk 'NF {s = (s + $1) % 4294967296} END {print s + 0}')
  [[ $sum == 0 ]]
}
vector_ok "$image" || fail "$image is not a valid firmware image (bad checksum)"
# A code read protection word at 0x2FC would lock out the USB bootloader for good
# (NO_ISP, CRP1-3): such an image is never written.
crp=$(od -An -v -tx4 -j 764 -N4 "$image" | tr -d ' ')
case $crp in
  4e697370|12345678|87654321|43218765) fail "$image would lock the base station's bootloader; refusing to write it" ;;
esac
size=$(stat -c %s "$image")
(( size <= 131072 )) || fail "$image is larger than the base station's 128 KB flash"

# Prints the mount point of the base station's drive, mounting it if needed.
find_drive() {
  local dev mnt
  dev=$(blkid -L "$label" 2>/dev/null || true)
  [[ -n $dev ]] || return 1
  mnt=$(findmnt -n -o TARGET -S "$dev" 2>/dev/null | head -n1 || true)
  if [[ -z $mnt ]]; then
    udisksctl mount -b "$dev" >/dev/null 2>&1 || return 1
    mnt=$(findmnt -n -o TARGET -S "$dev" | head -n1)
  fi
  [[ -f $mnt/firmware.bin ]] || return 1
  printf '%s\n' "$mnt"
}

wait_for_drive() {
  local mnt
  if ! mnt=$(find_drive); then
    cat >&2 <<'TXT'
Waiting for the base station...
  1. Unplug its power cable.
  2. Hold the button on the back.
  3. Plug the micro-USB cable into this PC, then the power cable, and let go.
TXT
    until mnt=$(find_drive); do sleep 1; done
  fi
  printf '%s\n' "$mnt"
}

wait_for_unplug() {
  while blkid -L "$label" >/dev/null 2>&1; do sleep 1; done
}

eject() {
  local dev
  dev=$(findmnt -n -o SOURCE -T "$1" 2>/dev/null || true)
  [[ -n $dev ]] && udisksctl unmount -b "$dev" >/dev/null 2>&1 || true
}

# Writes in place, sector by sector, straight to the device. Copying the file over
# (or dragging it in a file manager) can scramble the flash; this doesn't.
write_image() {
  local padded
  padded=$(mktemp)
  cp "$1" "$padded"
  local pad=$(( (512 - size % 512) % 512 ))
  (( pad )) && head -c "$pad" /dev/zero | tr '\0' '\377' >> "$padded"
  dd if="$padded" of="$2/firmware.bin" bs=512 conv=notrunc,nocreat oflag=direct status=none
  sync
  rm -f "$padded"
}

red=$'\e[1;37;41m' yellow=$'\e[1;33m' reset=$'\e[0m'
[[ -t 1 ]] || red="" yellow="" reset=""
cat <<TXT

${red}                                                                              ${reset}
${red}   !!!  WARNING: ONLY FOR LIGHTHOUSE 1.0 BASE STATIONS (HTC VIVE, 2016)  !!!   ${reset}
${red}                                                                              ${reset}

${yellow}  This firmware is ONLY for Lighthouse 1.0 base stations: the HTC ones that came
  with the original Vive (model 1.0, a mode button and channel letter on the back).

  NEVER use it on a Lighthouse 2.0 base station (Valve Index / SteamVR Base
  Station 2.0, or HTC's Base Station 2.0). It will BRICK a 2.0 base station.

  If you're not sure which one you have, STOP and close this window.${reset}

TXT
read -r -p "Type 1.0 to confirm this is a Lighthouse 1.0 base station: " generation
[[ $generation == "1.0" ]] || { echo "Stopped. Nothing was changed."; exit 0; }
echo

echo "lhbs base station firmware installer"
echo
if [[ -n $restore ]]; then
  echo "This puts back: $(basename "$image")"
else
  echo "This installs: $(basename "$image")"
  echo "Your base station's current firmware is saved first, so you can always go back."
fi
echo

mnt=$(wait_for_drive)
echo "Found the base station."
# A Lighthouse 1.0 base station's bootloader drive holds its whole 128 KiB flash.
# Anything else isn't one: stop before reading or writing anything.
if [[ $(stat -c %s "$mnt/firmware.bin" 2>/dev/null) != 131072 ]]; then
  eject "$mnt"
  fail "this isn't a Lighthouse 1.0 base station (its firmware.bin isn't 128 KiB). Nothing was written."
fi

if [[ -z $restore ]]; then
  echo
  echo "Type the serial number from the sticker on the base station (it looks like"
  read -r -p "LHB-1A2B3C4D), or just press Enter to skip: " serial
  serial=$(printf '%s' "$serial" | tr '[:lower:]' '[:upper:]' | tr -cd 'A-Z0-9-')
  [[ -z $serial ]] && serial="basestation"
  [[ $serial =~ ^[0-9A-F]{8}$ ]] && serial="LHB-$serial"
  mkdir -p "$here/backups"
  backup="$here/backups/$serial-$(date +%Y%m%d-%H%M%S).bin"
  dd if="$mnt/firmware.bin" of="$backup" bs=512 iflag=direct status=none
  [[ -s $backup ]] || fail "could not read the current firmware"
  echo "Saved the current firmware to backups/$(basename "$backup")"
  if cmp -s -n "$size" "$image" "$backup"; then
    echo
    echo "This firmware is already installed. Nothing to do."
    eject "$mnt"
    exit 0
  fi
fi

echo
read -r -p "Write it now? [y/N] " answer
[[ $answer =~ ^[Yy] ]] || { echo "Nothing written."; eject "$mnt"; exit 0; }

write_image "$image" "$mnt"
eject "$mnt"
echo "Written."
echo
echo "Now check it: unplug the base station, hold the button again and plug it back in."
wait_for_unplug
mnt=$(wait_for_drive)
check=$(mktemp)
dd if="$mnt/firmware.bin" of="$check" bs=512 iflag=direct status=none
if cmp -s -n "$size" "$image" "$check"; then
  rm -f "$check"
  eject "$mnt"
  echo
  echo "All good. Unplug the base station and plug its power back in (without holding"
  echo "the button). It starts with the new firmware."
else
  rm -f "$check"
  eject "$mnt"
  echo
  echo "The check failed: the base station doesn't have the right firmware."
  [[ -z $restore ]] && echo "Run it again, or put your backup back with:  $0 --restore \"$backup\""
  exit 2
fi
