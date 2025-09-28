#include "BSP_fdCAN_Driver.h"

// --- 全局变量与函数 ---
// 最多支持3个FDCAN总线实例
static fdCANbus* g_fdcan_bus_map[3] = {nullptr, nullptr, nullptr};

/**
 * @brief 注册一个fdCANbus实例以进行全局中断路由
 * @param bus 指向fdCANbus实例的指针
 */
void register_fdcan_bus_for_isr(fdCANbus* bus) 
{
    if (bus && bus->getbusID() > 0 && bus->getbusID() <= 3) 
        g_fdcan_bus_map[bus->getbusID() - 1] = bus;
    
}


                                    

// --- init: 启动任务并配置 CAN（filter/interrupt） ---
void fdCANbus::init() 
{
    FDCAN_FilterTypeDef sFilterConfig = {0};
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    // 配置一个接收所有标准帧的滤波器
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000; // Mask为0，表示不关心任何位，全部接收
    HAL_FDCAN_ConfigFilter(hfdcan_, &sFilterConfig);

    // 配置一个接收所有扩展帧的滤波器
    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = 1; // 使用不同的滤波器索引
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x00000000;
    HAL_FDCAN_ConfigFilter(hfdcan_, &sFilterConfig);

    // 启动FDCAN硬件
    if (HAL_FDCAN_Start(hfdcan_) != HAL_OK) 
    {
        // 错误处理
        HAL_FDCAN_Start_ERROR = true;
    }

    // 激活FIFO0新消息中断
    if (HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        // 错误处理
        HAL_FDCAN_ActivateNotification_ERROR = true;
    }

    // 注册自身到全局映射表
    register_fdcan_bus_for_isr(this);

    //任务启动
    rxTask_.start(tskIDLE_PRIORITY + 3, 256);
    schedulerTask_.start(tskIDLE_PRIORITY + 4, 256);
}

// --- registerMotor ---
bool fdCANbus::registerMotor(Motor_Base* m) 
{
    for (std::size_t i = 0; i < MAX_MOTORS; ++i) 
    {
        if (motorList_[i] == nullptr) 
        {
            motorList_[i] = m;
            return true;
        }
    }
    return false;
}

// --- sendFrame: 调用 HAL 发送 ---
bool fdCANbus::sendFrame(const CanFrame& cf) 
{
    // 使用互斥锁保护硬件访问，设置1ms的超时等待
    if (xSemaphoreTake(tx_mutex_, pdMS_TO_TICKS(1)) != pdTRUE) 
        return false; // 获取锁失败，直接返回，不阻塞调度任务
    

    if (!hfdcan_) 
    {
        xSemaphoreGive(tx_mutex_);
        return false;
    }

    FDCAN_TxHeaderTypeDef tx_header;
    std::memset(&tx_header, 0, sizeof(tx_header));
    tx_header.Identifier = cf.ID;
    tx_header.IdType = (cf.isextended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID);
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8; // 学院目前所有电机都是8字节
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &tx_header, const_cast<uint8_t*>(cf.data));

    xSemaphoreGive(tx_mutex_); // 释放锁

    return status == HAL_OK;
}

// --- pushRxFromISR ---
bool fdCANbus::pushRxFromISR(const CanFrame& cf, BaseType_t* pxHigherPriorityTaskWoken) 
{
    return rxQueue_.sendFromISR(cf, pxHigherPriorityTaskWoken);
}

// --- rxTaskBody ---
void fdCANbus::rxTaskbody() 
{
    CanFrame cf;
    for (;;) 
    {
        if (rxQueue_.recv(cf, portMAX_DELAY)) 
        {

            for (std::size_t i = 0; i < MAX_MOTORS; ++i) 
            {
                Motor_Base* m = motorList_[i];

                if (m && m->matchesFrame(cf)) 
                   m->updateFeedback(cf);
            }

        }
    }
}

// --- schedulerTaskBody ---
void fdCANbus::schedulerTaskbody() 
{
    CanFrame frames_to_send[MAX_MOTORS * 2];
    for (;;) 
    {
        // 永久阻塞等待，直到被TIM中断的ISR唤醒
        xSemaphoreTake(schedSem_, portMAX_DELAY);

         // 1. 更新所有电机的控制状态
        for (std::size_t i = 0; i < MAX_MOTORS; ++i)
        {
            Motor_Base* m = motorList_[i];
            if (m) 
                m->update();
            
        }

        // 2. 打包所有电机的指令

        std::size_t frameCnt = 0;
        for (std::size_t i = 0; i < MAX_MOTORS; ++i) 
        {
            Motor_Base* m = motorList_[i];
            if (!m) 
                continue;
            frameCnt += m->packCommand(&frames_to_send[frameCnt], (sizeof(frames_to_send)/sizeof(frames_to_send[0])) - frameCnt);

            if (frameCnt >= (sizeof(frames_to_send)/sizeof(frames_to_send[0]))) 
                break;
        }
    
        // 3. 发送所有打包好的指令
        for (std::size_t j = 0; j < frameCnt; ++j)
            sendFrame(frames_to_send[j]);
    }
}


// --- matchesFrameDefault ---
bool fdCANbus::matchesFrameDefault(const CanFrame& cf, uint32_t targetId, bool isExt) {
    return (cf.isextended == isExt) && (cf.ID == targetId);
}

// --- RxTask / SchedTask 构造函数 ---
fdCANbus::RxTask::RxTask(fdCANbus* parent) 
    : RtosTask("CAN_Rx", 0), parent_(parent) {}

fdCANbus::SchedTask::SchedTask(fdCANbus* parent) 
    : RtosTask("CAN_Sched", 0), parent_(parent) {}

// --- RxTask / SchedTask run ---
void fdCANbus::RxTask::run() 
{
    parent_->rxTaskbody();
}

void fdCANbus::SchedTask::run() 
{
    parent_->schedulerTaskbody();
}

// --- 全局C风格回调函数 ---

/**
 * @brief 全局的FDCAN接收中断处理函数
 * @param hfdcan 触发中断的FDCAN句柄
 */
extern "C" void fdcan_global_rx_isr(FDCAN_HandleTypeDef* hfdcan) 
{
    fdCANbus* target_bus = nullptr;
    for (int i = 0; i < 3; ++i) 
    {
        if (g_fdcan_bus_map[i] && g_fdcan_bus_map[i]->getFDCANHandle() == hfdcan) 
        {
            target_bus = g_fdcan_bus_map[i];
            break;
        }
    }

    if (!target_bus) 
        return;

    CanFrame rx_frame;
    FDCAN_RxHeaderTypeDef rx_header;
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_frame.data) == HAL_OK) 
    {
        rx_frame.ID = rx_header.Identifier;
        rx_frame.isextended = (rx_header.IdType == FDCAN_EXTENDED_ID);
        rx_frame.DLC = (rx_header.DataLength >> 16) & 0x0F;
        BaseType_t higher_priority_task_woken = pdFALSE;
        target_bus->pushRxFromISR(rx_frame, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

/**
 * @brief 全局的调度器Tick中断处理函数
 */
extern "C" void fdcan_global_scheduler_tick_isr() 
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    for (int i = 0; i < 3; ++i) 
    {
        if (g_fdcan_bus_map[i]) 
        {
            xSemaphoreGiveFromISR(g_fdcan_bus_map[i]->schedSem_, &higher_priority_task_woken);
        }
    }
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/**
 * @brief  重写 HAL_FDCAN_RxFifo0Callback 弱函数
 * @param  hfdcan FDCAN句柄
 * @param  RxFifo0ITs FIFO0中断标志
 * @note   此函数会在 HAL_FDCAN_IRQHandler 中被自动调用
 */
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    // 直接调用全局中断处理函数
    fdcan_global_rx_isr(hfdcan);
  }
}
