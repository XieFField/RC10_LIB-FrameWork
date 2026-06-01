$ErrorActionPreference = 'Stop'

$testsDir = $PSScriptRoot
$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $testsDir)))
$outDir = Join-Path $testsDir 'build'
if (-not (Test-Path $outDir)) {
    New-Item -ItemType Directory -Path $outDir | Out-Null
}

$exe = Join-Path $outDir ("test_chassis_semantics_{0}.exe" -f [System.Guid]::NewGuid().ToString('N'))
$stubDir = Join-Path $testsDir 'stubs'
$userInc = Join-Path $root 'User\Setup\Inc'
$appInc = Join-Path $root 'RC10_LIB\APP\Inc'
$testSupportSrc = Join-Path $testsDir 'stubs_src\test_host_globals.cpp'
$chassisSrc = Join-Path $root 'User\Setup\Src\chassis.cpp'
$testSrc = Join-Path $testsDir 'test_chassis_semantics.cpp'

& 'C:\Qt\Tools\mingw1310_64\bin\g++.exe' `
    '-std=c++17' `
    '-ffunction-sections' `
    '-fdata-sections' `
    '-I' $stubDir `
    '-I' $userInc `
    '-I' $appInc `
    $testSupportSrc `
    $chassisSrc `
    $testSrc `
    '-Wl,--gc-sections' `
    '-o' $exe

if (-not (Test-Path $exe)) {
    throw "compile failed, executable not generated: $exe"
}

& $exe
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

& (Join-Path $testsDir 'run_app_utils_backend_test.ps1')
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
