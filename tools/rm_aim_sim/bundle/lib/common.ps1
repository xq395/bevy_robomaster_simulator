# 部署包公共函数：路径、配置、Python/Godot 解析、PID 记录、端口探测。
# 由 deploy.ps1 与各组件 start.ps1 点源（. lib\common.ps1）使用。

$script:BundleRoot = Split-Path -Parent $PSScriptRoot
$script:ComponentRoot = Join-Path $BundleRoot 'components'

# Windows PowerShell 5.1 的 Env: 驱动器不区分大小写：同时存在 HTTPS_PROXY 与
# https_proxy（Clash / 公司代理常见）时会抛“已添加相同键”，Start-Process 随之失败。
# 这些子进程都不需要代理，直接清掉。
function Clear-ProxyEnv {
    foreach ($name in @('http_proxy', 'https_proxy', 'no_proxy', 'all_proxy',
            'HTTP_PROXY', 'HTTPS_PROXY', 'NO_PROXY', 'ALL_PROXY')) {
        try { [Environment]::SetEnvironmentVariable($name, $null, 'Process') } catch { }
    }
}

function Get-BundleRoot { return $script:BundleRoot }

function Get-BundleConfig {
    $path = Join-Path $script:BundleRoot 'config.json'
    $defaults = [ordered]@{
        channel        = 'default'
        operator_port  = 5001
        video_port     = 5000
        client_port    = 5002
        tolerance_deg  = 0.45
        python         = ''
        godot          = ''
        bind           = '127.0.0.1'
    }
    if (Test-Path -LiteralPath $path) {
        $raw = Get-Content -LiteralPath $path -Raw | ConvertFrom-Json
        foreach ($property in $raw.PSObject.Properties) { $defaults[$property.Name] = $property.Value }
    }
    return [pscustomobject]$defaults
}

function Get-RuntimeDir {
    $dir = Join-Path $script:BundleRoot 'runtime'
    if (-not (Test-Path -LiteralPath $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }
    return $dir
}

function Resolve-Python {
    param([object]$Config, [string]$Explicit)
    $candidates = @()
    if ($Explicit) { $candidates += $Explicit }
    if ($Config -and $Config.python) { $candidates += $Config.python }
    $candidates += (Join-Path $script:BundleRoot '.venv\Scripts\python.exe')
    $command = Get-Command python, python3 -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($command) { $candidates += $command.Source }
    $candidates += (Join-Path $env:LOCALAPPDATA 'Programs\Python\Python311\python.exe')
    $candidates += (Join-Path $env:LOCALAPPDATA 'Programs\Python\Python312\python.exe')
    foreach ($candidate in $candidates) {
        if ($candidate -and (Test-Path -LiteralPath $candidate)) { return $candidate }
    }
    return ''
}

function Resolve-Godot {
    param([object]$Config, [string]$Explicit)
    $candidates = @()
    if ($Explicit) { $candidates += $Explicit }
    if ($Config -and $Config.godot) { $candidates += $Config.godot }
    if ($env:GODOT_BIN) { $candidates += $env:GODOT_BIN }
    $candidates += (Join-Path $script:BundleRoot 'third_party\Godot_v4.7.2-stable_win64.exe')
    $found = Get-ChildItem -LiteralPath (Join-Path $script:BundleRoot 'third_party') -Filter 'Godot*.exe' -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -notlike '*console*' } | Select-Object -First 1
    if ($found) { $candidates += $found.FullName }
    $command = Get-Command godot4, godot -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($command) { $candidates += $command.Source }
    foreach ($candidate in $candidates) {
        if ($candidate -and (Test-Path -LiteralPath $candidate)) { return $candidate }
    }
    return ''
}

# ---- PID 记录（runtime\pids.json），供 deploy.ps1 -Action stop 使用 -------------

function Read-PidMap {
    $path = Join-Path (Get-RuntimeDir) 'pids.json'
    $map = [ordered]@{}
    if (Test-Path -LiteralPath $path) {
        try {
            $json = Get-Content -LiteralPath $path -Raw | ConvertFrom-Json
            foreach ($property in $json.PSObject.Properties) { $map[$property.Name] = [int]$property.Value }
        } catch { }
    }
    return $map
}

function Save-Pid([string]$Name, [int]$ProcessId) {
    $map = Read-PidMap
    $map[$Name] = $ProcessId
    $path = Join-Path (Get-RuntimeDir) 'pids.json'
    ($map | ConvertTo-Json) | Set-Content -LiteralPath $path -Encoding UTF8
}

function Remove-Pid([string]$Name) {
    $map = Read-PidMap
    if ($map.Contains($Name)) { $map.Remove($Name) }
    $path = Join-Path (Get-RuntimeDir) 'pids.json'
    ($map | ConvertTo-Json) | Set-Content -LiteralPath $path -Encoding UTF8
}

function Test-Alive([int]$ProcessId) {
    if ($ProcessId -le 0) { return $false }
    return [bool](Get-Process -Id $ProcessId -ErrorAction SilentlyContinue)
}

# ---- 端口 --------------------------------------------------------------------

function Test-TcpPort([string]$Address, [int]$Port, [int]$TimeoutMs = 250) {
    $client = New-Object System.Net.Sockets.TcpClient
    try {
        $async = $client.BeginConnect($Address, $Port, $null, $null)
        if (-not $async.AsyncWaitHandle.WaitOne($TimeoutMs)) { return $false }
        $client.EndConnect($async)
        return $true
    } catch { return $false } finally { $client.Close() }
}

function Test-UdpPort([int]$Port) {
    $lines = @(netstat -ano -p UDP | Select-String -Pattern (":{0}\s" -f $Port))
    return $lines.Count -gt 0
}

function Wait-TcpPort([string]$Address, [int]$Port, [int]$Seconds = 15) {
    $deadline = (Get-Date).AddSeconds($Seconds)
    while ((Get-Date) -lt $deadline) {
        if (Test-TcpPort $Address $Port 250) { return $true }
        Start-Sleep -Milliseconds 250
    }
    return $false
}

function Wait-UdpPort([int]$Port, [int]$Seconds = 15) {
    $deadline = (Get-Date).AddSeconds($Seconds)
    while ((Get-Date) -lt $deadline) {
        if (Test-UdpPort $Port) { return $true }
        Start-Sleep -Milliseconds 250
    }
    return $false
}

# 仿真的共享邮箱目录（Linux 上是 /dev/shm，其余平台是系统临时目录）
function Get-InboxDir([string]$Channel) {
    $root = $env:TEMP
    if (-not $root) { $root = [System.IO.Path]::GetTempPath() }
    return (Join-Path $root ("rm_aim_sim_" + $Channel))
}

function Wait-Inbox([string]$Channel, [int]$Seconds = 30) {
    $dir = Get-InboxDir $Channel
    $deadline = (Get-Date).AddSeconds($Seconds)
    while ((Get-Date) -lt $deadline) {
        if (Test-Path -LiteralPath $dir) { return $true }
        Start-Sleep -Milliseconds 250
    }
    return $false
}
