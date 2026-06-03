$ErrorActionPreference = 'Stop'

$legacyRoot = Split-Path -Parent $PSScriptRoot
$testsRoot = Split-Path -Parent $legacyRoot
$jiaDocsRoot = Split-Path -Parent $testsRoot
$repo = Split-Path -Parent $jiaDocsRoot
$outDir = Join-Path $PSScriptRoot 'build'
if (-not (Test-Path $outDir)) {
    New-Item -ItemType Directory -Path $outDir | Out-Null
}

$exe = Join-Path $outDir ("test_serial1_protocol_{0}.exe" -f [System.Guid]::NewGuid().ToString('N'))
$stubDir = Join-Path $PSScriptRoot 'stubs'
$moduleInc = Join-Path $repo 'RC10_LIB\Module\Inc'
$moduleSrc = Join-Path $repo 'RC10_LIB\Module\Src\Module_Serial1Protocol.cpp'
$globalsSrc = Join-Path $PSScriptRoot 'stubs\test_host_globals.cpp'
$testSrc = Join-Path $PSScriptRoot 'test_serial1_protocol.cpp'

& 'C:\Qt\Tools\mingw1310_64\bin\g++.exe' `
    '-std=c++17' `
    '-I' $stubDir `
    '-I' $moduleInc `
    $moduleSrc `
    $globalsSrc `
    $testSrc `
    '-o' $exe

& $exe
$nativeExitCode = if ($null -eq $LASTEXITCODE) { 0 } else { [int]$LASTEXITCODE }
if ($nativeExitCode -ne 0) {
    exit $nativeExitCode
}

Write-Host 'serial1_protocol_host test: PASS'
