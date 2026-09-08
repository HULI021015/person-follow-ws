$ErrorActionPreference = 'Stop'

$resolverLine = (& wsl.exe -d Ubuntu-22.04 -- cat /etc/resolv.conf |
    Where-Object { $_ -match '^nameserver\s+' } |
    Select-Object -First 1)

$gateway = (($resolverLine -split '\s+')[1]).Trim()

if (-not $gateway) {
    throw 'Cannot read WSL gateway address'
}

$relay = Join-Path $PSScriptRoot 'forward_camera_windows.py'

Write-Host "Camera relay starting: ${gateway}:37778 -> 192.168.1.21:37777"
Write-Host 'Keep this window open. Press Ctrl+C to stop.'

python $relay --listen-host $gateway --listen-port 37778
