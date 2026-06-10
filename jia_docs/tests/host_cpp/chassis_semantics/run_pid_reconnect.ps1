$ErrorActionPreference = 'Stop'

$suiteDir = $PSScriptRoot
$repoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $suiteDir)))
$sharedScript = Join-Path (Split-Path -Parent $suiteDir) '..\shared\common.ps1'
. $sharedScript

$outDir = New-JiaBuildDir -SuiteDir $suiteDir
$exe = Join-Path $outDir ("test_steer_pid_reconnect_{0}.exe" -f [System.Guid]::NewGuid().ToString('N'))
$stubDir = Join-Path $suiteDir 'pid_reconnect_stubs'
$appInc = Join-Path $repoRoot 'RC10_LIB\APP\Inc'
$motorInc = Join-Path $repoRoot 'RC10_LIB\Motor\Inc'
$moduleInc = Join-Path $repoRoot 'RC10_LIB\Module\Inc'
$bspDriverInc = Join-Path $repoRoot 'RC10_LIB\BSP_Driver\Inc'

$appToolSrc = Join-Path $repoRoot 'RC10_LIB\APP\Src\APP_tool.cpp'
$appPidSrc = Join-Path $repoRoot 'RC10_LIB\APP\Src\APP_PID.cpp'
$motorDjiSrc = Join-Path $repoRoot 'RC10_LIB\Motor\Src\Motor_DJI.cpp'
$encoderSrc = Join-Path $repoRoot 'RC10_LIB\Module\Src\Module_Encoder.cpp'
$testSrc = Join-Path $suiteDir 'test_steer_pid_reconnect.cpp'

Invoke-JiaGppCompile -Arguments @(
    '-std=c++17',
    '-ffunction-sections',
    '-fdata-sections',
    '-I', $stubDir,
    '-I', $appInc,
    '-I', $motorInc,
    '-I', $moduleInc,
    '-I', $bspDriverInc,
    $appToolSrc,
    $appPidSrc,
    $motorDjiSrc,
    $encoderSrc,
    $testSrc,
    '-Wl,--gc-sections'
) -OutputPath $exe

Invoke-JiaNativeExecutable -Path $exe
