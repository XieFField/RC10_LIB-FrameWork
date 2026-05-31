$ErrorActionPreference = 'Stop'

$legacyRoot = Split-Path -Parent $PSScriptRoot
$testsRoot = Split-Path -Parent $legacyRoot
$jiaDocsRoot = Split-Path -Parent $testsRoot
$repo = Split-Path -Parent $jiaDocsRoot

$setupConfigCpp = Join-Path $repo 'User\Setup\Src\Setup_ConfigInit.cpp'
$usbDriverCpp = Join-Path $repo 'RC10_LIB\BSP_Driver\Src\BSP_USB_UART_Driver.cpp'
$crsfCpp = Join-Path $repo 'RC10_LIB\Module\Src\Module_CrsfReceiver.cpp'
$uvprojx = Join-Path $repo 'MDK-ARM\Frame_T.uvprojx'
$loraCpp = Join-Path $repo 'RC10_LIB\Module\Src\Module_lora.cpp'
$commCpp = Join-Path $repo 'RC10_LIB\Module\Src\Module_communication.cpp'
$serialCpp = Join-Path $repo 'RC10_LIB\Module\Src\Module_Serial1Protocol.cpp'
$gpioExtiCpp = Join-Path $repo 'RC10_LIB\Module\Src\RC_gpio_exti.cpp'

$setupConfigContent = Get-Content -LiteralPath $setupConfigCpp -Raw
$usbDriverContent = Get-Content -LiteralPath $usbDriverCpp -Raw
$crsfContent = Get-Content -LiteralPath $crsfCpp -Raw
$uvprojxContent = Get-Content -LiteralPath $uvprojx -Raw
$loraContent = Get-Content -LiteralPath $loraCpp -Raw
$commContent = Get-Content -LiteralPath $commCpp -Raw
$serialContent = Get-Content -LiteralPath $serialCpp -Raw
$gpioExtiContent = Get-Content -LiteralPath $gpioExtiCpp -Raw

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

Check-Match $setupConfigContent 'communication::Lora_communication::GetInstance\(\)->Init\(\);' 'setup config initializes the merged Lora communication singleton'
Check-Match $setupConfigContent 'CrsfReceiver\*\s*crsf_rc\s*=\s*CrsfReceiver::GetInstance\(&huart7\);' 'setup config still acquires the CRSF receiver singleton on huart7'
Check-Match $setupConfigContent 'crsf_rc->init\(\);' 'setup config still initializes the CRSF receiver path'

Check-NotMatch $usbDriverContent 'Serial1Protocol_Debug' 'USB UART driver does not retain Serial1Protocol debug coupling'
Check-Match $usbDriverContent 'HAL_UARTEx_ReceiveToIdle_DMA\(huart,\s*instance->rx_buffer,\s*instance->rx_buffer_size\);' 'USB UART driver still rearms ReceiveToIdle DMA after callback dispatch'
Check-Match $usbDriverContent 'if\s*\(\s*huart\s*==\s*&huart7\s*\)\s*\{\s*tx_done\s*=\s*true\s*;' 'USB UART driver still releases CRSF DMA completion through the shared UART TX callback'

Check-NotMatch $crsfContent 'Serial1Protocol_Debug' 'CRSF receiver does not depend on Serial1Protocol debug helpers'
Check-NotMatch $crsfContent 'Module_Serial1Protocol.h' 'CRSF receiver does not gain direct Serial1Protocol coupling during merge'
Check-NotMatch $crsfContent 'static\s+volatile\s+bool\s+tx_done' 'CRSF receiver exports tx_done instead of hiding it as a file-local static'
Check-Match $crsfContent 'volatile\s+bool\s+tx_done\s*=\s*true\s*;' 'CRSF receiver keeps a globally linkable tx_done flag for the shared UART TX completion path'

Check-Match $loraContent 'bsp_rx\.SetCallback\(RxCallback\);' 'Lora init registers the RX callback on the UART wrapper'
Check-Match $loraContent 'bsp_rx\.UART_Init\(\);' 'Lora init starts the wrapped UART receiver'
Check-Match $loraContent 'Comm_RxDMAToRxBuffer\(s_instance->lora_rx_huart,\s*len\);' 'Lora RX callback forwards DMA bytes into the communication FIFO'
Check-Match $loraContent 'GetSettingData\(command,\s*load1,\s*load2\);' 'Lora task loop consumes decoded setting frames'
Check-Match $loraContent 'kfs_\[0\]\s*=\s*command;' 'Lora task loop mirrors KFS command byte into cache slot 0'
Check-Match $loraContent 'kfs_\[1\]\s*=\s*load1;' 'Lora task loop mirrors KFS load1 byte into cache slot 1'
Check-Match $loraContent 'kfs_\[2\]\s*=\s*load2;' 'Lora task loop mirrors KFS load2 byte into cache slot 2'
Check-Match $loraContent 'if\s*\(\s*timer_tick_count\s*>=\s*2\s*\)' 'Lora timer hook still flushes pending frames on the two-tick cadence'
Check-Match $loraContent 'Comm_TxBufferToTxDMA\(lora_tx_huart\);' 'Lora EXTI path still triggers deferred TX DMA flush'

Check-Match $commContent 'byte1\s*==\s*0xAA\s*&&\s*byte2\s*==\s*0x55' 'communication parser still recognizes joystick frames by 0xAA55 header'
Check-Match $commContent 'byte1\s*==\s*0xAA\s*&&\s*byte2\s*==\s*0x66' 'communication parser still recognizes setting frames by 0xAA66 header'
Check-Match $commContent 'frame\.header\[0\]\s*=\s*0x55;' 'communication TX path still emits XYZ frame head byte 0x55'
Check-Match $commContent 'frame\.header\[1\]\s*=\s*0xAA;' 'communication TX path still emits XYZ frame head byte 0xAA'
Check-Match $commContent 'frame\.tail\s*=\s*0xED;' 'communication TX path still emits XYZ frame tail byte 0xED'
Check-Match $commContent 'SCB_CleanDCache_by_Addr\(\(uint32_t\*\)dma_tx_buf,\s*count\);' 'communication TX path still cleans DCache before DMA send'

Check-Match $serialContent 'HAL_UARTEx_ReceiveToIdle_DMA\(m_huart,\s*m_rx_buffer,\s*30\);' 'Serial1 init still arms ReceiveToIdle DMA on the 30-byte RX buffer'
Check-Match $serialContent 'uint8_t is_ack = \(received_data\[0\] == 0x00 &&' 'Serial1 processing still treats all-zero payloads as ACK frames'
Check-Match $serialContent 'storeReceivedData\(received_data,\s*received_parity\);' 'Serial1 processing still stores decoded business frames before responding'
Check-Match $serialContent 'sendAckFrame\(\);' 'Serial1 processing still answers decoded business frames with ACK'
Check-Match $serialContent 'send_data\[2\]\s*=\s*cmd;' 'Serial1 command sender still stores outbound commands in the third data byte'

Check-Match $gpioExtiContent 'gpio::GpioExti::All_EXTI_Prosess\(GPIO_Pin\);' 'global HAL EXTI callback still routes into the gpio::GpioExti multiplexer'

Check-Match $uvprojxContent '\.\.\\RC10_LIB\\Module\\Inc\\Module_lora\.h' 'Keil project still includes Module_lora.h in RC10_LIB/Module'
Check-Match $uvprojxContent '\.\.\\RC10_LIB\\Module\\Src\\Module_lora\.cpp' 'Keil project still includes Module_lora.cpp in RC10_LIB/Module'
Check-Match $uvprojxContent '\.\.\\RC10_LIB\\Module\\Inc\\Module_communication\.h' 'Keil project still includes Module_communication.h in RC10_LIB/Module'
Check-Match $uvprojxContent '\.\.\\RC10_LIB\\Module\\Src\\Module_communication\.cpp' 'Keil project still includes Module_communication.cpp in RC10_LIB/Module'
Check-Match $uvprojxContent '\.\.\\RC10_LIB\\Module\\Inc\\Module_Serial1Protocol\.h' 'Keil project still includes Module_Serial1Protocol.h in RC10_LIB/Module'
Check-Match $uvprojxContent '\.\.\\RC10_LIB\\Module\\Src\\Module_Serial1Protocol\.cpp' 'Keil project still includes Module_Serial1Protocol.cpp in RC10_LIB/Module'
Check-Match $uvprojxContent '\.\.\\RC10_LIB\\Module\\Src\\RC_gpio_exti\.cpp' 'Keil project still includes RC_gpio_exti.cpp in RC10_LIB/Module'
Check-Match $uvprojxContent '\.\.\\RC10_LIB\\Module\\Inc\\RC_gpio_exti\.h' 'Keil project still includes RC_gpio_exti.h in RC10_LIB/Module'

if ($failures -ne 0) {
    Write-Host ''
    Write-Host "protocol_merge_static test: FAIL ($failures failures)"
    exit 1
}

Write-Host ''
Write-Host 'protocol_merge_static test: PASS'
