$ErrorActionPreference = 'Stop'

$testsDir = $PSScriptRoot
$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $testsDir)))
$outDir = Join-Path $testsDir 'build'
if (-not (Test-Path $outDir)) {
    New-Item -ItemType Directory -Path $outDir | Out-Null
}

$exe = Join-Path $outDir ("test_steer_pid_reconnect_{0}.exe" -f [System.Guid]::NewGuid().ToString('N'))
$stubDir = Join-Path $testsDir 'pid_reconnect_stubs'
$appInc = Join-Path $root 'RC10_LIB\APP\Inc'
$motorInc = Join-Path $root 'RC10_LIB\Motor\Inc'
$moduleInc = Join-Path $root 'RC10_LIB\Module\Inc'
$bspDriverInc = Join-Path $root 'RC10_LIB\BSP_Driver\Inc'

$appToolSrc = Join-Path $root 'RC10_LIB\APP\Src\APP_tool.cpp'
$appPidSrc = Join-Path $root 'RC10_LIB\APP\Src\APP_PID.cpp'
$motorDjiSrc = Join-Path $root 'RC10_LIB\Motor\Src\Motor_DJI.cpp'
$encoderSrc = Join-Path $root 'RC10_LIB\Module\Src\Module_Encoder.cpp'
$testSrc = Join-Path $testsDir 'test_steer_pid_reconnect.cpp'

& 'C:\Qt\Tools\mingw1310_64\bin\g++.exe' `
    '-std=c++17' `
    '-ffunction-sections' `
    '-fdata-sections' `
    '-I' $stubDir `
    '-I' $appInc `
    '-I' $motorInc `
    '-I' $moduleInc `
    '-I' $bspDriverInc `
    $appToolSrc `
    $appPidSrc `
    $motorDjiSrc `
    $encoderSrc `
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
