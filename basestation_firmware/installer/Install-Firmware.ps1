# Installs the lhbs firmware on a 1.0 base station (Windows).
#
#   Install-Firmware.bat                       back up the base station, then install
#   Install-Firmware.bat -Restore BACKUP.bin   put a backup back
param([string]$Restore = "")

$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$label = "CRP DISABLD"

function Fail([string]$text) {
  Write-Host ""
  Write-Host "Error: $text" -ForegroundColor Red
  exit 1
}

if ($Restore) {
  $image = $Restore
} else {
  $found = Get-ChildItem -Path (Join-Path $here "firmware") -Filter *.bin -ErrorAction SilentlyContinue | Select-Object -First 1
  if (-not $found) { Fail "no firmware file in $here\firmware" }
  $image = $found.FullName
}
if (-not (Test-Path $image)) { Fail "$image not found" }
$imageBytes = [IO.File]::ReadAllBytes((Resolve-Path $image))

# The LPC11U37 refuses to boot an image whose first 8 words don't add up to zero.
function Test-Vectors([byte[]]$bytes) {
  if ($bytes.Length -lt 32) { return $false }
  [uint64]$sum = 0
  for ($i = 0; $i -lt 8; $i++) { $sum = ($sum + [BitConverter]::ToUInt32($bytes, $i * 4)) % 4294967296 }
  return $sum -eq 0
}
if (-not (Test-Vectors $imageBytes)) { Fail "$image is not a valid firmware image (bad checksum)" }
# A code read protection word at 0x2FC would lock out the USB bootloader for good
# (NO_ISP, CRP1-3): such an image is never written.
if ($imageBytes.Length -ge 0x300) {
  $crp = [BitConverter]::ToUInt32($imageBytes, 0x2FC)
  if (@(0x4E697370, 0x12345678, 0x87654321, 0x43218765) -contains $crp) { Fail "$image would lock the base station's bootloader; refusing to write it" }
}
if ($imageBytes.Length -gt 131072) { Fail "$image is larger than the base station's 128 KB flash" }

function Find-Drive {
  $disk = Get-CimInstance Win32_LogicalDisk | Where-Object { $_.VolumeName -eq $label } | Select-Object -First 1
  if ($disk -and (Test-Path "$($disk.DeviceID)\firmware.bin")) { return $disk.DeviceID }
  return $null
}

function Wait-Drive {
  $drive = Find-Drive
  if (-not $drive) {
    Write-Host "Waiting for the base station..."
    Write-Host "  1. Unplug its power cable."
    Write-Host "  2. Hold the button on the back."
    Write-Host "  3. Plug the micro-USB cable into this PC, then the power cable, and let go."
    while (-not $drive) { Start-Sleep -Seconds 1; $drive = Find-Drive }
  }
  return $drive
}

function Wait-Unplug {
  while (Find-Drive) { Start-Sleep -Seconds 1 }
}

function Eject([string]$drive) {
  try { (New-Object -ComObject Shell.Application).Namespace(17).ParseName($drive).InvokeVerb("Eject") } catch { }
  Start-Sleep -Seconds 2
}

function Same([byte[]]$a, [byte[]]$b, [int]$count) {
  if ($a.Length -lt $count -or $b.Length -lt $count) { return $false }
  for ($i = 0; $i -lt $count; $i++) { if ($a[$i] -ne $b[$i]) { return $false } }
  return $true
}

# Writes in place, straight through to the device. Copying the file over (or dragging
# it in Explorer) replaces it, and that can scramble the flash; this doesn't.
function Write-Image([byte[]]$bytes, [string]$drive) {
  $size = [int]([math]::Ceiling($bytes.Length / 512) * 512)
  $padded = New-Object byte[] $size
  for ($i = 0; $i -lt $size; $i++) { $padded[$i] = 0xFF }
  [Array]::Copy($bytes, $padded, $bytes.Length)
  $stream = New-Object IO.FileStream("$drive\firmware.bin", [IO.FileMode]::Open, [IO.FileAccess]::Write,
                                     [IO.FileShare]::None, 512, [IO.FileOptions]::WriteThrough)
  try {
    $stream.Write($padded, 0, $padded.Length)
    $stream.Flush($true)
  } finally {
    $stream.Close()
  }
}

Write-Host ""
Write-Host "                                                                              " -BackgroundColor Red
Write-Host "   !!!  WARNING: ONLY FOR LIGHTHOUSE 1.0 BASE STATIONS (HTC VIVE, 2016)  !!!   " -ForegroundColor White -BackgroundColor Red
Write-Host "                                                                              " -BackgroundColor Red
Write-Host ""
Write-Host "  This firmware is ONLY for Lighthouse 1.0 base stations: the HTC ones that came" -ForegroundColor Yellow
Write-Host "  with the original Vive (model 1.0, a mode button and channel letter on the back)." -ForegroundColor Yellow
Write-Host ""
Write-Host "  NEVER use it on a Lighthouse 2.0 base station (Valve Index / SteamVR Base" -ForegroundColor Yellow
Write-Host "  Station 2.0, or HTC's Base Station 2.0). It will BRICK a 2.0 base station." -ForegroundColor Yellow
Write-Host ""
Write-Host "  If you're not sure which one you have, STOP and close this window." -ForegroundColor Yellow
Write-Host ""
$generation = Read-Host "Type 1.0 to confirm this is a Lighthouse 1.0 base station"
if ($generation.Trim() -ne "1.0") { Write-Host "Stopped. Nothing was changed."; exit 0 }
Write-Host ""

Write-Host "lhbs base station firmware installer"
Write-Host ""
if ($Restore) {
  Write-Host "This puts back: $(Split-Path -Leaf $image)"
} else {
  Write-Host "This installs: $(Split-Path -Leaf $image)"
  Write-Host "Your base station's current firmware is saved first, so you can always go back."
}
Write-Host ""

$drive = Wait-Drive
Write-Host "Found the base station ($drive)."
# A Lighthouse 1.0 base station's bootloader drive holds its whole 128 KiB flash.
# Anything else isn't one: stop before reading or writing anything.
$flash = Get-Item -LiteralPath "$drive\firmware.bin" -ErrorAction SilentlyContinue
if (-not $flash -or $flash.Length -ne 131072) {
  Eject $drive
  Fail "this isn't a Lighthouse 1.0 base station (its firmware.bin isn't 128 KiB). Nothing was written."
}

$backup = ""
if (-not $Restore) {
  Write-Host ""
  Write-Host "Type the serial number from the sticker on the base station (it looks like"
  $serial = Read-Host "LHB-1A2B3C4D), or just press Enter to skip"
  $serial = ($serial.ToUpper() -replace '[^A-Z0-9-]', '')
  if (-not $serial) { $serial = "basestation" }
  if ($serial -match '^[0-9A-F]{8}$') { $serial = "LHB-$serial" }
  $folder = Join-Path $here "backups"
  New-Item -ItemType Directory -Force -Path $folder | Out-Null
  $backup = Join-Path $folder ("$serial-" + (Get-Date -Format "yyyyMMdd-HHmmss") + ".bin")
  $current = [IO.File]::ReadAllBytes("$drive\firmware.bin")
  if ($current.Length -eq 0) { Fail "could not read the current firmware" }
  [IO.File]::WriteAllBytes($backup, $current)
  Write-Host "Saved the current firmware to backups\$(Split-Path -Leaf $backup)"
  if (Same $imageBytes $current $imageBytes.Length) {
    Write-Host ""
    Write-Host "This firmware is already installed. Nothing to do."
    Eject $drive
    exit 0
  }
}

Write-Host ""
$answer = Read-Host "Write it now? [y/N]"
if ($answer -notmatch '^[Yy]') { Write-Host "Nothing written."; Eject $drive; exit 0 }

Write-Image $imageBytes $drive
Eject $drive
Write-Host "Written."
Write-Host ""
Write-Host "Now check it: unplug the base station, hold the button again and plug it back in."
Wait-Unplug
$drive = Wait-Drive
$check = [IO.File]::ReadAllBytes("$drive\firmware.bin")
Eject $drive
if (Same $imageBytes $check $imageBytes.Length) {
  Write-Host ""
  Write-Host "All good. Unplug the base station and plug its power back in (without holding"
  Write-Host "the button). It starts with the new firmware."
  exit 0
}
Write-Host ""
Write-Host "The check failed: the base station doesn't have the right firmware." -ForegroundColor Red
if ($backup) { Write-Host "Run it again, or put your backup back with:  Install-Firmware.bat -Restore `"$backup`"" }
exit 2
