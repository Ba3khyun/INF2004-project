$uf2Path = Join-Path $PSScriptRoot 'build\pico_w_project.uf2'
$rp2Volume = Get-Volume | Where-Object { $_.FileSystemLabel -eq 'RPI-RP2' } | Select-Object -First 1

if (-not (Test-Path $uf2Path)) {
    throw "Firmware not found at '$uf2Path'. Build the project first."
}

if ($null -eq $rp2Volume) {
    throw 'RPI-RP2 drive not found. Hold BOOTSEL while connecting the Pico W over USB.'
}

Copy-Item $uf2Path -Destination (Join-Path $rp2Volume.DriveLetter 'pico_w_project.uf2') -Force
Write-Host "Flashed $uf2Path to $($rp2Volume.DriveLetter):"
