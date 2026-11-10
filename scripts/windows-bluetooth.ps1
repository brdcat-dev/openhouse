# Install the PC Bluetooth power bridge, retaining SteamVR's Index implementation.
param(
    [ValidateSet('Status', 'Apply', 'Restore')][string]$Action = 'Status',
    [Parameter(Mandatory = $true)][string]$SteamVRPath,
    [string]$BridgePath = ''
)
$ErrorActionPreference = 'Stop'
if (-not $BridgePath) { $BridgePath = Join-Path $PSScriptRoot 'bluetooth\nrfserlib.dll' }
$SteamVRPath = (Resolve-Path -LiteralPath $SteamVRPath).Path
$target = Join-Path $SteamVRPath 'bin\win64\nrfserlib.dll'
$backup = "$target.openhouse-backup"
$index = Join-Path $SteamVRPath 'bin\win64\nrfserlib_index.dll'
$originalHash = '55B09BDB27E475A10EEBF3253B1882380DB04B5D6E003344B068D5F189F62E72'
$fixedHash = 'AB62269C901E92D786462C61BA83B769470D3AF05AD084A27852418E29214F94'
function Hash([string]$path) { (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash }
function Is-Bridge([string]$path) {
    [Text.Encoding]::ASCII.GetString([IO.File]::ReadAllBytes($path)).Contains('openhouse Bluetooth: ')
}
function Replace-File([string]$path, [byte[]]$bytes, [string]$expectedHash) {
    $temporary = "$path.$([Guid]::NewGuid().ToString('N')).tmp"
    try {
        [IO.File]::WriteAllBytes($temporary, $bytes)
        if ((Hash $temporary) -ne $expectedHash) { throw "Prepared file hash mismatch: $path" }
        if (Test-Path -LiteralPath $path) {
            [IO.File]::Replace($temporary, $path, [NullString]::Value)
        } else {
            [IO.File]::Move($temporary, $path)
        }
    } finally {
        if (Test-Path -LiteralPath $temporary) { Remove-Item -LiteralPath $temporary }
    }
    if ((Hash $path) -ne $expectedHash) { throw "Installed file hash mismatch: $path" }
}
if (-not (Test-Path -LiteralPath $target -PathType Leaf)) { throw "Nordic library not found: $target" }
$installedHash = Hash $target
$isBridge = Is-Bridge $target
$isNative = $installedHash -eq $originalHash -or $installedHash -eq $fixedHash
if ($Action -eq 'Status') {
    $state = if ($isBridge) { 'PC Bluetooth bridge' } elseif ($isNative) { 'native Index controller' } else { 'unsupported version' }
    Write-Output "Bluetooth library: $target ($state)"
    Write-Output "SHA256: $installedHash"
    Write-Output "Original backup present: $(Test-Path -LiteralPath $backup)"
    Write-Output "Index fallback present: $(Test-Path -LiteralPath $index)"
    return
}
$hosts = @(Get-Process -Name vrserver,vrmonitor,vrcompositor -ErrorAction SilentlyContinue)
if ($hosts) { throw "Close $($hosts.ProcessName -join ', ') before changing the Bluetooth library." }
if (-not $isBridge -and -not $isNative) {
    throw 'Unsupported Nordic library. Nothing changed; revalidate the interface for this SteamVR version.'
}
if (Test-Path -LiteralPath $backup) {
    $backupHash = Hash $backup
    if ($backupHash -ne $originalHash -and $backupHash -ne $fixedHash) { throw "Unexpected original backup; preserving files: $backup" }
} elseif ($isBridge) {
    throw "Bridge installed without its original backup: $backup"
}
if ($Action -eq 'Restore') {
    if (-not (Test-Path -LiteralPath $backup)) { Write-Output 'Native library already installed; no backup to restore.'; return }
    Replace-File $target ([IO.File]::ReadAllBytes($backup)) (Hash $backup)
    if ((Test-Path -LiteralPath $index) -and ((Hash $index) -eq $fixedHash)) { Remove-Item -LiteralPath $index }
    Write-Output 'Restored the original SteamVR Bluetooth library. Backup retained.'
    return
}
if (-not (Test-Path -LiteralPath $BridgePath -PathType Leaf) -or -not (Is-Bridge $BridgePath)) {
    throw "PC Bluetooth bridge missing or invalid: $BridgePath (native MSVC build required)"
}
if ((Test-Path -LiteralPath $index) -and ((Hash $index) -ne $originalHash) -and ((Hash $index) -ne $fixedHash)) {
    throw "Unexpected Index fallback file; preserving files: $index"
}
if (-not (Test-Path -LiteralPath $backup)) { Copy-Item -LiteralPath $target -Destination $backup }
$indexBytes = [IO.File]::ReadAllBytes($backup)
if ((Hash $backup) -eq $originalHash) {
    # Give the radio time to finish H5 CONFIG negotiation at 1ms timer precision.
    if ($indexBytes[0x17e88] -ne 100) { throw 'Unexpected native handshake interval.' }
    $indexBytes[0x17e88] = 200
}
Replace-File $index $indexBytes $fixedHash
Replace-File $target ([IO.File]::ReadAllBytes((Resolve-Path -LiteralPath $BridgePath).Path)) (Hash $BridgePath)
Write-Output "Installed PC Bluetooth batching with Index fallback. Original kept at $backup."
