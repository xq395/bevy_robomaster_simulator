[CmdletBinding()]
param(
    [ValidateSet('sp_standard', 'hnu_vision')]
    [string]$Target = 'sp_standard',
    [Parameter(Mandatory = $true)]
    [string]$ConfigPath,
    [ValidateSet('Release', 'Debug', 'RelWithDebInfo', 'MinSizeRel')]
    [string]$Config = 'Release'
)

$ErrorActionPreference = 'Stop'

$RepoRoot = Split-Path -Parent $PSScriptRoot
$BuildRoot = Join-Path $RepoRoot '.deps\build\deploy_dev-win64'
$Executable = Join-Path $BuildRoot "pipeline\$Config\$Target.exe"
$OpenCvBin = Join-Path $RepoRoot '.deps\install\opencv\build\x64\vc16\bin'
$OpenVinoBin = 'C:\Users\34741\AppData\Local\Programs\Python\Python311\Lib\site-packages\openvino\libs'

foreach ($Path in @($Executable, $ConfigPath, $OpenCvBin, $OpenVinoBin)) {
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "Required path does not exist: $Path"
    }
}

$env:PATH = "$OpenCvBin;$OpenVinoBin;$env:PATH"
& $Executable $ConfigPath
exit $LASTEXITCODE
