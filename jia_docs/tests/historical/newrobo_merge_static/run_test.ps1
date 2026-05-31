$ErrorActionPreference = 'Stop'

$legacyRoot = Split-Path -Parent $PSScriptRoot
$testsRoot = Split-Path -Parent $legacyRoot
$jiaDocsRoot = Split-Path -Parent $testsRoot
$repo = Split-Path -Parent $jiaDocsRoot

$omniCpp = Join-Path $repo 'User\Setup\Src\omni_chassisSetup.cpp'
$fsmCpp = Join-Path $repo 'User\Setup\Src\FSM_Controller.cpp'
$fsmEnumH = Join-Path $repo 'User\Setup\Inc\FSMstauts_enum.h'
$mainC = Join-Path $repo 'Core\Src\main.c'
$mainCpp = Join-Path $repo 'Core\Src\main.cpp'

$omniContent = Get-Content -LiteralPath $omniCpp -Raw
$fsmContent = Get-Content -LiteralPath $fsmCpp -Raw
$fsmEnumContent = Get-Content -LiteralPath $fsmEnumH -Raw
$mainCContent = Get-Content -LiteralPath $mainC -Raw
$mainCppContent = Get-Content -LiteralPath $mainCpp -Raw

$failures = 0

function Check-Match($content, $pattern, $description) {
    if ($content -match $pattern) {
        Write-Host "OK: $description"
    } else {
        Write-Host "FAIL: $description"
        $script:failures++
    }
}

function Check-NotMatch($content, $pattern, $description) {
    if ($content -notmatch $pattern) {
        Write-Host "OK: $description"
    } else {
        Write-Host "FAIL: $description"
        $script:failures++
    }
}

Check-Match $omniContent 'CrsfReceiver::GetInstance\(&huart7\)->getControlData\(&airjoy_data_\);' 'omni setup keeps CRSF as a supported control-data source'
Check-Match $omniContent 'case\s+CHASSIS_MANUAL_CONTROL_A' 'manual control mode A still exists'
Check-Match $omniContent 'case\s+CHASSIS_MANUAL_CONTROL_B' 'manual control mode B still exists'
Check-Match $omniContent 'case\s+CHASSIS_LOCK_FORWEAPON' 'weapon lock mode still exists'
Check-Match $omniContent 'case\s+CHASSIS_MANUAL_CONTROL_C' 'manual control mode C still exists'
Check-Match $omniContent 'setSpeed_LockNowYaw\(Chassis::Coordinate::kWorld,\s*target_vel_x,\s*target_vel_y,\s*target_omega_z\);' 'manual mode A still routes to lock-now-yaw with omega input'
Check-Match $omniContent 'setSpeed_LockNowYaw\(Chassis::Coordinate::kWorld,\s*target_vel_x,\s*target_vel_y\);' 'manual mode B still routes to lock-now-yaw without omega input'
Check-Match $omniContent 'setSpeed_LockToYaw\(Chassis::Coordinate::kWorld,\s*target_vel_x,\s*target_vel_y,\s*target_rot_z\);' 'weapon lock mode still routes to lock-to-yaw'
Check-Match $omniContent 'const float target_yaw_angle = 90\.0f;' 'weapon lock mode still targets a fixed 90-degree platform heading'
Check-Match $omniContent 'target_chassis_twist_\.yaw_rate = 0\.0f;' 'manual control mode C still forces omega to zero'
Check-Match $omniContent 'chassis\.setSpeed\(Chassis::Coordinate::kWorld,\s*target_chassis_twist_\.vx,\s*target_chassis_twist_\.vy,\s*target_chassis_twist_\.yaw_rate\);' 'manual control mode C still routes through world-speed output'

Check-Match $fsmEnumContent 'CHASSIS_LOCK_FORWEAPON' 'FSM enum still exposes weapon lock chassis mode'
Check-Match $fsmEnumContent 'CHASSIS_MANUAL_CONTROL_C' 'FSM enum still exposes manual control mode C'
Check-Match $fsmEnumContent 'WEAPONSAGE_AUTO_CONTROL' 'FSM enum still exposes current weapon auto mode'
Check-NotMatch $fsmEnumContent 'WEAPONSAGE_AUTO_CONTROL_CATCH' 'legacy weapon auto-catch enum has not been restored into the main enum set'

Check-Match $fsmContent 'if\(arm_setup_->isArmcalibrated\(\) == false \)' 'FSM still hard-gates on arm calibration state'
Check-Match $fsmContent 'if\(arm_setup_->isArmcalibrated\(\) == false \)\s*\{\s*robot_status_ = ALL_STOP;\s*\}' 'FSM active global stop gate still depends only on arm calibration'
Check-Match $fsmContent 'weaponSage_setup_->setWeaponSageControlStatus\(WEAPONSAGE_AUTO_CONTROL\);' 'FSM still uses the current weapon auto-control handshake'
Check-NotMatch $fsmContent 'weaponSage_setup_->setWeaponSageControlStatus\(WEAPONSAGE_AUTO_CONTROL_CATCH\);' 'FSM does not fall back to the legacy weapon auto-catch handshake'
Check-Match $fsmContent 'chassis_setup_->setChassisStatus\(CHASSIS_AUTO_CONTROL_CB\);' 'FSM still routes the weapon auto branch into current chassis auto CB mode'
Check-Match $fsmContent 'chassis_setup_->setChassisStatus\(CHASSIS_AUTO_CONTROL_KFS\);' 'FSM still routes KFS auto branch into current chassis auto KFS mode'

if ($omniContent -match 'USE_RC10_AIRJOY') {
    Check-Match $omniContent '#if\s*!USE_RC10_AIRJOY' 'Lora integration keeps CRSF as the default branch'
    Check-Match $omniContent '#else' 'Lora integration introduces an alternate control-data branch'
    Check-Match $omniContent 'communication::Lora_communication::GetInstance\(\)->Task_Process\(\);' 'Lora branch processes incoming communication data'
    Check-Match $omniContent 'communication::Lora_communication::GetInstance\(\)->Tim_It_Process\(\);' 'Lora branch keeps timing-side processing hook'
    Check-Match $omniContent 'communication::Lora_communication::GetInstance\(\)->update_airjoy_data\(&airjoy_data_\);' 'Lora branch updates airjoy_data_ through the communication adapter'
}

Check-Match $mainCContent 'HAL_TIM_Base_Start_IT\(&htim6\);' 'main.c still starts TIM6 before entering the control loop'
Check-Match $mainCppContent 'HAL_TIM_Base_Start_IT\(&htim6\);' 'main.cpp still starts TIM6 before entering the control loop'
Check-Match $mainCContent '(?m)^\s*while\s*\(1\)\s*$' 'main.c keeps an active infinite loop'
Check-Match $mainCContent '(?m)^\s*__disable_irq\(\);\s*$' 'main.c keeps the hard-stop path in Error_Handler'

if ($mainCContent -match 'MX_USART2_UART_Init\(\);') {
    Check-Match $mainCContent 'MX_UART5_Init\(\);' 'main.c pulls in UART5 together with USART2 during Lora integration'
    Check-Match $mainCContent 'MX_TIM3_Init\(\);' 'main.c pulls in TIM3 together with USART2 during Lora integration'
    Check-NotMatch $mainCContent '//\s*while\s*\(1\)' 'main.c does not comment out the main loop during Lora integration'
}

if ($mainCppContent -match 'MX_USART2_UART_Init\(\);') {
    Check-Match $mainCppContent 'MX_UART5_Init\(\);' 'main.cpp mirrors UART5 init when USART2 is enabled'
    Check-Match $mainCppContent 'MX_TIM3_Init\(\);' 'main.cpp mirrors TIM3 init when USART2 is enabled'
}

if ($failures -ne 0) {
    Write-Host ""
    Write-Host "newrobo_merge_static test: FAIL ($failures failures)"
    exit 1
}

Write-Host ""
Write-Host 'newrobo_merge_static test: PASS'
