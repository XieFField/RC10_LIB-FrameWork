$ErrorActionPreference = 'Stop'

$suiteDir = $PSScriptRoot
$repoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $suiteDir)))
$sharedScript = Join-Path (Split-Path -Parent $suiteDir) '..\shared\common.ps1'
. $sharedScript

$outDir = New-JiaBuildDir -SuiteDir $suiteDir
$exe = Join-Path $outDir ("test_chassis_semantics_{0}.exe" -f [System.Guid]::NewGuid().ToString('N'))
$stubDir = Join-Path $suiteDir 'stubs'
$doctestInc = Join-Path (Split-Path -Parent (Split-Path -Parent $suiteDir)) 'third_party\doctest'
$userInc = Join-Path $repoRoot 'User\Setup\Inc'
$appInc = Join-Path $repoRoot 'RC10_LIB\APP\Inc'
$testSupportSrc = Join-Path $suiteDir 'stubs_src\test_host_globals.cpp'
$chassisSrc = Join-Path $repoRoot 'User\Setup\Src\chassis.cpp'
$testSrcs = @(
    (Join-Path $suiteDir 'test_chassis_semantics_main.cpp'),
    (Join-Path $suiteDir 'test_chassis_semantics_harness.cpp'),
    (Join-Path $suiteDir 'test_chassis_semantics_drive_pid_and_mapping.cpp'),
    (Join-Path $suiteDir 'test_chassis_semantics_single_wheel_debug.cpp'),
    (Join-Path $suiteDir 'test_chassis_semantics_drive_delivery_zero_stop.cpp'),
    (Join-Path $suiteDir 'test_chassis_semantics_yaw_and_motion_profile.cpp'),
    (Join-Path $suiteDir 'test_chassis_semantics_swerve_planner_flip_reverse.cpp'),
    (Join-Path $suiteDir 'test_chassis_semantics_xpark_gate_and_hold.cpp'),
    (Join-Path $suiteDir 'test_chassis_semantics_steer_fault_homing_recovery.cpp')
)

$compileArgs = @(
    '-std=c++17',
    '-ffunction-sections',
    '-fdata-sections',
    '-DJIA_CHASSIS_PROFILE=JIA_CHASSIS_PROFILE_FULL_DEBUG',
    '-DJIA_CHASSIS_FIRST_BOOT_HOMING_DELAY_ENABLE=1',
    '-DJIA_CHASSIS_FIRST_BOOT_HOMING_DELAY_MS=3',
    '-I', $stubDir,
    '-I', $doctestInc,
    '-I', $userInc,
    '-I', $appInc,
    $testSupportSrc,
    $chassisSrc
) + $testSrcs + @(
    '-Wl,--gc-sections'
)

Invoke-JiaGppCompile -Arguments $compileArgs -OutputPath $exe

Invoke-JiaNativeExecutable -Path $exe -Arguments @(
    '--reporters=console',
    '--success=false',
    '--duration=false',
    '--no-version'
)
& (Join-Path $suiteDir 'run_app_utils_backend.ps1')
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
