> Former firmware README. Paths are relative to `basestation_firmware/`.

# Lighthouse 1.0 base station firmware

Reverse engineering of the HTC Vive Lighthouse 1.0 base station (NXP LPC11U37,
stock firmware `lighthouse_tx_htc_2_0-436-2016-09-20`) and **lhbs**, a readable
reimplementation with stock feature parity. lhbs is tracked by SteamVR alone,
as optical slave of a stock master, and as master of a stock slave.

| Path | What |
|---|---|
| `ai/firmware/HARDWARE.md` | Hardware/firmware reference: MCU, pin map, peripherals, protocols (sync/OOTX, EEPROM parameters, Bluetooth power management), verification log |
| `src/`, `test/`, `Makefile` | The firmware rewrite. One module per stock source file; every function names the stock address it replaces |
| `fw/` (local only) | `lhfw`, a minimal bring-up firmware (display, LEDs, button, EEPROM snapshot) |
| `tools/` | `lpcsum.py` (the boot ROM's vector checksum, run by the Makefile) |
| `installer/` | The flashing scripts shipped in the firmware package (a "Lighthouse 1.0 only" warning that needs `1.0` typed, a 128 KiB flash check, backup, write in place, verify, restore) |
| `re/` (local only) | Analysis scripts (headless Ghidra, Unicorn RAM-init dump), function names, parameter table |
| `patches/` (local only) | Byte patches for the stock image |

Not in git (see `.gitignore`): the stock firmware image, boot ROM dump, decompiled
stock code, and personal flash/EEPROM backups.

## Build and flash

```sh
make                               # -> build/firmware.bin (checksummed)
# hold the mode button while powering on -> "CRP DISABLD" USB drive appears
../scripts/build-firmware.sh && ../dist/openhouse-basestation-firmware/install-firmware.sh
# power-cycle; console on /dev/lhbs (any serial terminal)
```

USB console access (udev rule, creates `/dev/lhbs`):

```sh
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1fc9", ATTRS{idProduct}=="0083", TAG+="uaccess", SYMLINK+="lhbs"' | sudo tee /etc/udev/rules.d/70-lhbs.rules
```

Never copy firmware onto the ISP drive with a file manager: it corrupts flash
(see `ai/firmware/HARDWARE.md`); the installer writes in place.

## Safety

lhbs drives Class 1 lasers only while both rotors are locked, with several
independent interlocks. A stationary beam is not eye safe: keep the rotor
interlocks intact in any modification.
