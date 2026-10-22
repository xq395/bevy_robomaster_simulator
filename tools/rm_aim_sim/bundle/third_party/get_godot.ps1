[CmdletBinding()]
param(
    # 覆盖下载地址（默认走国内可用的 GitHub 加速镜像，失败自动回官方）
    [string]$Url = 'https://ghfast.top/https://github.com/godotengine/godot/releases/download/4.7.2-stable/Godot_v4.7.2-stable_win64.exe.zip'
)

$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$target = Join-Path $here 'Godot_v4.7.2-stable_win64.exe'
if (Test-Path -LiteralPath $target) {
    Write-Host "已存在：$target"
    exit 0
}

$zip = Join-Path $here 'godot.zip'
$mirrors = @($Url,
    'https://github.com/godotengine/godot/releases/download/4.7.2-stable/Godot_v4.7.2-stable_win64.exe.zip')

$ok = $false
foreach ($mirror in $mirrors) {
    try {
        Write-Host "下载：$mirror"
        Invoke-WebRequest -Uri $mirror -OutFile $zip -UseBasicParsing -TimeoutSec 900
        if ((Get-Item -LiteralPath $zip).Length -gt 50MB) { $ok = $true; break }
        Write-Host '  文件过小，换下一个源'
    } catch {
        Write-Host "  失败：$($_.Exception.Message)"
    }
}
if (-not $ok) {
    Write-Host '下载失败。手动下载后解压到本目录：' -ForegroundColor Red
    Write-Host '  https://github.com/godotengine/godot/releases/tag/4.7.2-stable'
    Write-Host '  文件名 Godot_v4.7.2-stable_win64.exe.zip'
    exit 3
}

$hash = (Get-FileHash -LiteralPath $zip -Algorithm SHA256).Hash.ToLower()
Write-Host "SHA256：$hash"

Expand-Archive -LiteralPath $zip -DestinationPath $here -Force
Remove-Item -LiteralPath $zip -Force -ErrorAction SilentlyContinue

if (-not (Test-Path -LiteralPath $target)) {
    Write-Host '解压后没找到 exe，请手动确认。' -ForegroundColor Red
    exit 3
}

$version = & $target --version 2>&1 | Select-Object -First 1
Write-Host "版本：$version"
if ($version -notmatch '4\.7\.2\.stable\.official') {
    Write-Host '警告：版本不是 4.7.2.stable.official，仿真可能行为不同。' -ForegroundColor Yellow
}
Write-Host '完成。' -ForegroundColor Green
