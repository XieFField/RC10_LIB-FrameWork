$ErrorActionPreference = 'Stop'

$testsRoot = Split-Path -Parent $PSScriptRoot
$aiRoot = Split-Path -Parent $testsRoot
$projectRoot = Split-Path -Parent $aiRoot
$repo = Join-Path $projectRoot 'RC10_LIB-FrameWork'

$chassisCpp = Join-Path $repo 'User\Setup\Src\chassis.cpp'
$chassisH = Join-Path $repo 'User\Setup\Inc\chassis.h'

$cppContent = Get-Content $chassisCpp -Raw
$hContent = Get-Content $chassisH -Raw

$failures = 0

function Check-Match($content, $pattern, $description) {
    if ($content -match $pattern) {
        Write-Host "OK: $description"
    } else {
        Write-Host "FAIL: $description"
        $script:failures++
    }
}

Check-Match $cppContent 'is_wheel_speed_mode_' 'single-wheel debug path (is_wheel_speed_mode_) preserved'
Check-Match $cppContent 'is_wheel_current_mode_' 'single-wheel debug path (is_wheel_current_mode_) preserved'
Check-Match $cppContent 'is_wheel_single_position_mode_' 'single-wheel debug path (is_wheel_single_position_mode_) preserved'
Check-Match $cppContent 'runRuntimeSwerveControl' 'runRuntimeSwerveControl() exists'
Check-Match $cppContent 'is_power_on_cailbration_' 'photogate calibration path preserved'
Check-Match $hContent 'SwerveController' 'SwerveController referenced in chassis.h'
Check-Match $hContent 'JiaChassisDebugWatch' 'JiaChassisDebugWatch defined'
Check-Match $hContent 'extern.*volatile.*JiaChassisDebugWatch' 'extern volatile JiaChassisDebugWatch declared'

Write-Host ""
Write-Host "$failures failures"
exit $failures
