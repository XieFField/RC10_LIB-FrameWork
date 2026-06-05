$ErrorActionPreference = 'Stop'

$testsRoot = Split-Path -Parent $PSScriptRoot
$aiRoot = Split-Path -Parent $testsRoot
$projectRoot = Split-Path -Parent $aiRoot
$repo = Join-Path $projectRoot 'RC10_LIB-FrameWork'
$setupHeader = Join-Path $repo 'User\Setup\Inc\omni_chassisSetup.h'

$content = Get-Content -LiteralPath $setupHeader -Raw

if ($content -match 'wheels_\s*\[\s*3\s*\]') {
    Write-Error 'omni_setup_static test: FAIL Chassis_Omni<3> setup must not access wheels_[3]'
}

Write-Host 'omni_setup_static test: PASS'
