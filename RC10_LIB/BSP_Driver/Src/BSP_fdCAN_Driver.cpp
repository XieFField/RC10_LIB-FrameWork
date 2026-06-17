#include "BSP_fdCAN_Driver.h"

// --- 全局变量与函数 ---
// 最多支持3个FDCAN总线实例
 static fdCANbus* g_fdcan_bus_map[3] = {nullptr, nullptr, nullptr};

// 为类内 static 成员提供定义
#if FD_CAN_DEBUG
//CanFrame fdCANbus::debug_last_frames_[fdCANbus::MAX_MOTORS * 2] = {0};
#endif


#if FD_CAN_DEBUG
fdCANbus* g_fdcan_bus_map_dbg[3] = {nullptr, nullptr, nullptr}; // 全局可见别名
#endif

volatile FdcanDiagSnapshot g_fdcan_diag[3] = {};

static int fdcan_diag_bus_index(FDCAN_HandleTypeDef* hfdcan)
{
    if (hfdcan == &hfdcan1)
        return 0;
    if (hfdcan == &hfdcan2)
        return 1;
    if (hfdcan == &hfdcan3)
        return 2;
    return -1;
}

static void fdcan_diag_capture(FDCAN_HandleTypeDef* hfdcan, uint32_t errorStatusITs)
{
    int idx = fdcan_diag_bus_index(hfdcan);
    if (idx < 0)
        return;

    volatile FdcanDiagSnapshot& s = g_fdcan_diag[idx];
    s.snapshot_count++;
    s.last_error_status_its = errorStatusITs;
    s.last_hal_error = HAL_FDCAN_GetError(hfdcan);

    s.ir = hfdcan->Instance->IR;
    s.ie = hfdcan->Instance->IE;
    s.psr = hfdcan->Instance->PSR;
    s.ecr = hfdcan->Instance->ECR;
    s.cccr = hfdcan->Instance->CCCR;
    s.nbtp = hfdcan->Instance->NBTP;
    s.dbtp = hfdcan->Instance->DBTP;
    s.rxf0s = hfdcan->Instance->RXF0S;
    s.rxf1s = hfdcan->Instance->RXF1S;

    s.lec = s.psr & FDCAN_PSR_LEC;
    s.dlec = (s.psr & FDCAN_PSR_DLEC) >> FDCAN_PSR_DLEC_Pos;
    s.rec = (s.ecr & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos;
    s.tec = (s.ecr & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos;

    RCC_PeriphCLKInitTypeDef periphClk = {0};
    HAL_RCCEx_GetPeriphCLKConfig(&periphClk);
    s.fdcan_clk_source = periphClk.FdcanClockSelection;
    s.fdcan_clk_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
}

/**
 * @brief 注册一个fdCANbus实例以进行全局中断路由
 * @param bus 指向fdCANbus实例的指针
 */
void register_fdcan_bus_for_isr(fdCANbus* bus) 
{
    if(!bus) 
        return;
    FDCAN_HandleTypeDef* h = bus->getFDCANHandle();
    if(h == &hfdcan1)
    {       
        g_fdcan_bus_map[0] = bus;
#if FD_CAN_DEBUG
        g_fdcan_bus_map_dbg[0] = bus;
#endif
    }
    else if(h == &hfdcan2)
    {
        g_fdcan_bus_map[1] = bus;
#if FD_CAN_DEBUG
        g_fdcan_bus_map_dbg[1] = bus;
#endif
    }

    else if(h == &hfdcan3)  
    {
        g_fdcan_bus_map[2] = bus;
#if FD_CAN_DEBUG
        g_fdcan_bus_map_dbg[2] = bus;
#endif
    }
}


                                    

void fdCANbus::init() 
{
    if(can_init_done_) 
        return; 
    FDCAN_FilterTypeDef sFilterConfig = {0};
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    // 配置一个接收所有标准帧的滤波器
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;
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
        HAL_FDCAN_Start_ERROR = 1;
        Error_Handler();
    }

    // 激活FIFO0新消息中断 和 BusOff中断
    if (HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0) != HAL_OK) {
        // 错误处理
        HAL_FDCAN_ActivateNotification_ERROR = 1;
        Error_Handler();
    }

    // 注册自身到全局映射表
    register_fdcan_bus_for_isr(this);

    // 记录初始化后首帧诊断快照（用于核对FDCAN时钟/时序寄存器）
    fdcan_diag_capture(hfdcan_, 0u);

    rxTask_.start(tskIDLE_PRIORITY + 3, 512);
    schedulerTask_.start(tskIDLE_PRIORITY + 4, 512);

    can_init_done_ = true;
}

bool fdCANbus::registerMotor(Motor_Base* m) 
{
    if (m->bus() != this) return false;

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
volatile HAL_StatusTypeDef success ; 
volatile int add_before = 0;
volatile int add_after = 0;

bool fdCANbus::sendFrame(const CanFrame& cf) 
{

     if (xSemaphoreTake(tx_mutex_, pdMS_TO_TICKS(1)) != pdTRUE) 
         return false; 
    

     if (!hfdcan_) 
     {
         xSemaphoreGive(tx_mutex_);
         return false;
     }

    FDCAN_TxHeaderTypeDef tx_header;

    // 确保发送数据 4 字节对齐
    alignas(4) uint8_t aligned_buf[8];
    std::memcpy(aligned_buf, cf.data, 8);


    tx_header.Identifier = cf.ID;
    tx_header.IdType = (cf.isextended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID);
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8; // 学院目前所有电机都是8字节
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;
    
    
    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &tx_header, aligned_buf);
    success = status;
    xSemaphoreGive(tx_mutex_); // 释放锁
    
    return status == HAL_OK;
}


bool fdCANbus::pushRxFromISR(const CanFrame& cf, BaseType_t* pxHigherPriorityTaskWoken) 
{
    return rxQueue_.sendFromISR(cf, pxHigherPriorityTaskWoken);
}


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

                if (m && m->bus() == this && m->matchesFrame(cf)) 
                   m->updateFeedback(cf);
            }

            for (std::size_t j = 0; j < 3; j++)
            {
                if(oid_encoder_[j] != nullptr && oid_encoder_[j]->matchesFrame(cf) && oid_encoder_[j]->bus() == this)
                {
                    oid_encoder_[j]->updateFeedback(cf);
                }
            }

        }
    }
}


void fdCANbus::schedulerTaskbody() 
{
    CanFrame frames_to_send[kMaxFrames];
    for (;;) 
    {
        xSemaphoreTake(schedSem_, portMAX_DELAY);

        if (bus_off_flag_)
        {
            // Handle Bus Off Recovery
            HAL_FDCAN_Stop(hfdcan_);
            HAL_FDCAN_Start(hfdcan_);
            HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0);
            bus_off_flag_ = false;
        }

        for (std::size_t i = 0; i < MAX_MOTORS; ++i)
        {
            Motor_Base* m = motorList_[i];
            if (m && (m->get_controlCnt() +1u >= static_cast<uint16_t>(1000 / m->get_controlFrequency()))) //防止重设控制频率导致pid计算错配
                m->update();
            
        }

        std::size_t frameCnt = 0; //计数值，记录打包了多少帧
        for (std::size_t i = 0; i < MAX_MOTORS; ++i) 
        {
            Motor_Base* m = motorList_[i];
            OIDEncoder* o = nullptr;
            if(i < 3)
                o = oid_encoder_[i];
            if (!m && !o) 
                continue;
                
            // bool due = true;
            // if(m && m->get_controlFrequency() != 1000) // 如果不是默认频率，则进行分频判断
            // {
            //     const uint16_t divider = static_cast<uint16_t>(1000 / m->get_controlFrequency()); // 计算分频器
            //     due = (m->get_controlCnt() + 1u >= divider); // 判断是否触发控制周期
            // }

            // if(!due && m)
            // {
            //     m->increment_controlCnt();
            //     continue; // 不到控制周期，不打包
            // }

            // bool o_due = true;
            // if(o && o->get_controlFrequency() != 1000) // 如果不是默认频率，则进行分频判断
            // {
            //     const uint16_t divider = static_cast<uint16_t>(1000 / o->get_controlFrequency()); // 计算分频器
            //     o_due = (o->get_controlCnt() + 1u >= divider); // 判断是否触发控制周期
            // }

            // if(!o_due && o)
            // {
            //     o->increment_controlCnt();
            //     continue; // 不到控制周期，不打包
            // }

            // if(m)
            // {
            //     frameCnt += m->packCommand(&frames_to_send[frameCnt], (sizeof(frames_to_send)/sizeof(frames_to_send[0])) - frameCnt);
            //     m->reset_controlCnt(); // 重置计数器，准备下一个控制周期
            // }

            // if(o)
            // {
            //     frameCnt += o->packCommand(&frames_to_send[frameCnt], (sizeof(frames_to_send)/sizeof(frames_to_send[0])) - frameCnt);
            //     o->reset_controlCnt();
            // }

            bool m_send = false, o_send = false;
            if (m) 
            {
                if (m->get_controlFrequency() != 1000) 
                {
                    uint16_t div = 1000 / m->get_controlFrequency();
                    if (m->get_controlCnt() + 1u >= div)
                        m_send = true;
                    else
                        m->increment_controlCnt();
                } 
                else 
                {
                    m_send = true;
                }
            }

            // o 频控（独立）
            if (o) 
            {
                if (o->get_controlFrequency() != 1000) 
                {
                    uint16_t div = 1000 / o->get_controlFrequency();
                    if (o->get_controlCnt() + 1u >= div)
                        o_send = true;
                    else
                        o->increment_controlCnt();
                } 
                else 
                {
                    o_send = true;
                }
            }

            if (m_send) 
            { 
                frameCnt += m->packCommand(&frames_to_send[frameCnt], (sizeof(frames_to_send)/sizeof(frames_to_send[0])) - frameCnt); 
                m->reset_controlCnt(); 
            }
            if (o_send) 
            { 
                frameCnt += o->packCommand(&frames_to_send[frameCnt], (sizeof(frames_to_send)/sizeof(frames_to_send[0])) - frameCnt); 
                o->reset_controlCnt(); 
            }
            if (frameCnt >= (sizeof(frames_to_send)/sizeof(frames_to_send[0]))) 
                break;
        }
    
#if FD_CAN_DEBUG
        debug_last_frame_count_ = frameCnt;
        for(std::size_t k=0; k<frameCnt && k< (kMaxFrames); ++k)
            debug_last_frames_[k] = frames_to_send[k];
        
#endif

        for (std::size_t j = 0; j < frameCnt; ++j)
        {
            if (!sendFrame(frames_to_send[j]))
            {
                static uint32_t tx_fail_count = 0;
                tx_fail_count++;
                (void)tx_fail_count; // 断点观察用，确认是否发生过发送失败
            }
        }
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

//extern "C"

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
    BaseType_t higher_priority_task_woken = pdFALSE;

    // 循环读取直到 FIFO 为空，防止高负载下数据积压
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_frame.data) == HAL_OK) 
        {
            rx_frame.ID = rx_header.Identifier;
            rx_frame.isextended = (rx_header.IdType == FDCAN_EXTENDED_ID);
            rx_frame.DLC = (rx_header.DataLength >> 16) & 0x0F;
            
            BaseType_t current_woken = pdFALSE;
            target_bus->pushRxFromISR(rx_frame, &current_woken);
            
            if(current_woken == pdTRUE)
            {
                higher_priority_task_woken = pdTRUE;
            }
        }
        else
        {
            break; // 读取失败或 FIFO 空
        }
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
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
 * @brief  FDCAN Error Status Callback
 * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
 *         the configuration information for the specified FDCAN.
 * @param  ErrorStatusITs Error Status Interrupts flags
 * @retval None
 */
extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    int idx = fdcan_diag_bus_index(hfdcan); 
    if (idx >= 0)
    {
        g_fdcan_diag[idx].error_status_cb_count++;
        fdcan_diag_capture(hfdcan, ErrorStatusITs);
    }

    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0)
    {
        if (idx >= 0)
            g_fdcan_diag[idx].bus_off_count++;

        // Bus Off detected, set flag and wake up task to recover
        for (int i = 0; i < 3; ++i) 
        {
            if (g_fdcan_bus_map[i] && g_fdcan_bus_map[i]->getFDCANHandle() == hfdcan) 
            {
                g_fdcan_bus_map[i]->setBusOffFlag();
                // Wake up scheduler task to handle recovery immediately
                BaseType_t higher_priority_task_woken = pdFALSE;
                xSemaphoreGiveFromISR(g_fdcan_bus_map[i]->schedSem_, &higher_priority_task_woken);
                portYIELD_FROM_ISR(higher_priority_task_woken);
                break;
            }
        }
    }
}

/**
 * @brief  重写 HAL_FDCAN_RxFifo0Callback 弱函数
 * @param  hfdcan FDCAN句柄
 * @param  RxFifo0ITs FIFO0中断标志
 * @note   此函数会在 HAL_FDCAN_IRQHandler 中被自动调用
 */
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    int idx = fdcan_diag_bus_index(hfdcan);
    if (idx >= 0)
    {
        g_fdcan_diag[idx].rx_fifo0_cb_count++;
        fdcan_diag_capture(hfdcan, 0u);
    }

  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    // 直接调用全局中断处理函数
    fdcan_global_rx_isr(hfdcan);
  }
}

fdCANbus* fdCANbus::getInstance(FDCAN_HandleTypeDef* hfdcan)
{

    static fdCANbus instance1(&hfdcan1);
    static fdCANbus instance2(&hfdcan2);
    static fdCANbus instance3(&hfdcan3);

    
    fdCANbus* inst = nullptr;
    if(hfdcan == &hfdcan1) 
        inst = &instance1;
    else if(hfdcan == &hfdcan2) 
        inst = &instance2;
    else if(hfdcan == &hfdcan3) 
        inst = &instance3;
    else 
        return nullptr;

    if(inst)
    {
        if(hfdcan == &hfdcan1 && g_fdcan_bus_map[0] != inst) 
        {
            g_fdcan_bus_map[0] = inst;
#if FD_CAN_DEBUG
            g_fdcan_bus_map_dbg[0] = inst;
#endif
        }
        if(hfdcan == &hfdcan2 && g_fdcan_bus_map[1] != inst) 
        {
            g_fdcan_bus_map[1] = inst;
#if FD_CAN_DEBUG
            g_fdcan_bus_map_dbg[1] = inst;
#endif

        }
        if(hfdcan == &hfdcan3 && g_fdcan_bus_map[2] != inst)
        {
            g_fdcan_bus_map[2] = inst;
#if FD_CAN_DEBUG
            g_fdcan_bus_map_dbg[2] = inst;
#endif

        }
    }

    return inst;
}

fdCANbus::fdCANbus(FDCAN_HandleTypeDef* hfdcan)
    : hfdcan_(hfdcan),
      rxQueue_(512), // [Fix] 增加队列深度，空载高转速下防止丢帧导致相位混叠
      rxTask_(this),
      schedulerTask_(this)
{
    for (std::size_t i = 0; i < MAX_MOTORS; ++i) 
            motorList_[i] = nullptr;

        tx_mutex_ = xSemaphoreCreateMutex(); //创建互斥锁
        schedSem_ = xSemaphoreCreateBinary(); //创建二值信号量
}


