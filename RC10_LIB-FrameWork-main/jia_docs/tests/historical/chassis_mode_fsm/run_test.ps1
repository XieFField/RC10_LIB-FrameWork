$ErrorActionPreference = 'Stop'

$testsRoot = Split-Path -Parent $PSScriptRoot
$aiRoot = Split-Path -Parent $testsRoot
$projectRoot = Split-Path -Parent $aiRoot
$repo = Join-Path $projectRoot 'RC10_LIB-FrameWork'

$chassisH = Join-Path $repo 'User\Setup\Inc\chassis.h'
$chassisCpp = Join-Path $repo 'User\Setup\Src\chassis.cpp'

$hContent = Get-Content $chassisH -Raw
$cppContent = Get-Content $chassisCpp -Raw

$failures = 0

function Check-Match($content, $pattern, $description) {
    if ($content -match $pattern) {
        Write-Host "OK: $description"
    } else {
        Write-Host "FAIL: $description"
        $script:failures++
    }
}

# 检查 TriOmniChassis 九种 Mode
$triModes = @(
    'kWheelTorqueFreeMode',
    'kBodySpeedMode',
    'kWorldSpeedMode',
    'kBodySpeedLockNowRotZMode',
    'kWorldSpeedLockNowRotZMode',
    'kBodySpeedLockToRotZMode',
    'kWorldSpeedLockToRotZMode',
    'kBodySpeedLockNowRotZWithNoOmegaZMode',
    'kWorldSpeedLockNowRotZWithNoOmegaZMode'
)

foreach ($mode in $triModes) {
    Check-Match $hContent $mode "TriOmni Mode::$mode"
}

# 检查 setModeFlag 覆盖关键模式
Check-Match $cppContent 'kWheelTorqueFreeMode' 'setModeFlag covers kWheelTorqueFreeMode'
Check-Match $cppContent 'kBodySpeedMode' 'setModeFlag covers kBodySpeedMode'
Check-Match $cppContent 'kWorldSpeedMode' 'setModeFlag covers kWorldSpeedMode'

# 检查 using 声明
Check-Match $hContent 'using.*Chassis' 'using declaration for Chassis exists'

# 检查 TwoSteer 命名空间存在
Check-Match $hContent 'TriOmniChassis' 'TriOmniChassis namespace exists'
Check-Match $hContent 'FourSteerChassis' 'FourSteerChassis namespace exists'

Write-Host ""
Write-Host "$failures failures"
exit $failures
