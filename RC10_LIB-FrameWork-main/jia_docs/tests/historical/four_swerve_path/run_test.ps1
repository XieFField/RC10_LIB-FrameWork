$ErrorActionPreference = 'Stop'

$repo = 'D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork'
$headerPath = Join-Path $repo 'User\Setup\Inc\chassis.h'
$sourcePath = Join-Path $repo 'User\Setup\Src\chassis.cpp'

function Assert-Pattern {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if (-not (Select-String -Path $Path -Pattern $Pattern -Quiet)) {
        throw $Message
    }
}

Assert-Pattern -Path $headerPath -Pattern 'jia::swerve::SwerveConfig\s+swerve_config_' -Message 'Missing FourSteer runtime SwerveConfig member.'
Assert-Pattern -Path $headerPath -Pattern 'jia::swerve::SwerveController\s+swerve_controller_' -Message 'Missing FourSteer runtime SwerveController member.'
Assert-Pattern -Path $sourcePath -Pattern 'swerve_controller_\.configure\(swerve_config_\)' -Message 'FourSteer init() does not configure the runtime swerve controller.'
Assert-Pattern -Path $sourcePath -Pattern 'swerve_controller_\.step\(' -Message 'FourSteer normal path does not call into SwerveController::step().' 
Assert-Pattern -Path $sourcePath -Pattern 'runRuntimeSwerveControl\(\);' -Message 'FourSteer task loop does not execute the runtime swerve control path.'
Assert-Pattern -Path $sourcePath -Pattern 'applyDriveWheelDebugCommand\(wheel,\s*drive_target_rpm\)' -Message 'Single-wheel debug drive path regressed.'

Write-Host 'four_swerve_path test: PASS'
