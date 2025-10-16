/**
 * @file    BSP_Driver/Src/BSP_fdCAN_Driver.h
 * @brief   BSP Driver for fdCAN communication
 * @author  XieFField
 * @version 1.0
 * 
 */

 /* |(*^▽^*)/ 测试通过喵 */

#ifndef __BSP_FDCAN_DRIVER_H
#define __BSP_FDCAN_DRIVER_H

/*
    __________     _________    _   __   __              
   / ____/ __ \   / ____/   |  / | / /  / /_  __  _______
  / /_  / / / /  / /   / /| | /  |/ /  / __ \/ / / / ___/
 / __/ / /_/ /  / /___/ ___ |/ /|  /  / /_/ / /_/ (__  ) 
/_/   /_____/   \____/_/  |_/_/ |_/  /_.___/\__,_/____/  
                                                         
*/

#pragma once

#ifdef __cplusplus
extern "C" 
{
    #include "fdcan.h"
    #include<cstdint>
    #include "stm32h7xx_hal.h"
    #include "cmsis_os.h"
    #include "BSP_CanFrame.h"
}
#endif

#include "APP_tool.h"
#include "BSP_RTOS.h"
#include "Motor_Base.h" 
#include <cstring>

    class Motor_Base; // 前置声明

/** 
  * fdCANbus：管理一条fdCAN总线
  * - 每一路CAN生成一个实例
  * - 管理静态的motorList 最大8个
  * - ISR接收只单纯push 原始的 CanFrame 到 rxQueue_
  * - rxTask_ 负责从 rxQueue_ pop 并分发给 motor -> motor->updateFeedback()
  * - schedulerTask_ 每 1ms 调度 motorList，收集 packCommand() 并调用 sendFrame()，从而实现1kHz的发送频率
  * - 哈基米
  * @attention 此类不做任何具体的报文解析，全部交给电机类
 */
class fdCANbus;

extern "C" void fdcan_global_rx_isr(FDCAN_HandleTypeDef* hfdcan);
extern "C" void fdcan_global_scheduler_tick_isr();

class fdCANbus{
public:
    // 最大电机数（每路）
    static constexpr size_t MAX_MOTORS = 8;

    fdCANbus(FDCAN_HandleTypeDef* hfdcan, uint32_t bus_id): hfdcan_(hfdcan)
    , bus_id_(bus_id)
    , rxQueue_(64)                // 队列长度可调整
    , rxTask_(this)
    , schedulerTask_(this)
    {
        for (std::size_t i = 0; i < MAX_MOTORS; ++i) 
            motorList_[i] = nullptr;

        tx_mutex_ = xSemaphoreCreateMutex(); //创建互斥锁
        schedSem_ = xSemaphoreCreateBinary();
    }

    ~fdCANbus()
    {
        // 任务删除通常在系统停机，不在析构里自动删除
    }

    /**
     * @brief 初始化 (滤波/中断/启动任务)
     */
    void init();

    /**
     * @brief 静态注册电机
     */
    bool registerMotor(Motor_Base* m);

    /**
     * @brief 发送
     * @param cf 要发送的帧
     * @return true 发送成功，false 发送失败
     * @attention 由scheduler调用
     */
    bool sendFrame(const CanFrame& cf);

    /**
     * @brief 从ISR中接收数据并推入接收队列
     * @param cf 要接收的帧
     */
    bool pushRxFromISR(const CanFrame& cf, BaseType_t* pxHigherPriorityTaskWoken);

    uint8_t getbusID() const{return bus_id_;}

    // 在中断/ISR 中调用，唤醒该 fdCANbus 的 scheduler task
    // pxHigherPriorityTaskWoken 可以从 ISR 传入并用于 portYIELD_FROM_ISR
    //用了别的方式实现，这个已经不必了，日后若要取消定时中断唤醒，可以打开来用
    //static void notifySchedulerFromISR(BaseType_t* pxHigherPriorityTaskWoken);

    FDCAN_HandleTypeDef* getFDCANHandle() const { return hfdcan_; }
    
    SemaphoreHandle_t tx_mutex_; // 发送互斥锁
    SemaphoreHandle_t schedSem_; // 调度任务信号量
protected:
    
    FDCAN_HandleTypeDef* hfdcan_; //protected character
    uint32_t bus_id_;

    /**
     * @brief Rx任务主体
     */
    void rxTaskbody();

    /**
     * @brief 1kHz 调度任务主体
     */
    void schedulerTaskbody();

    // 默认匹配函数（子类或 motor 可 override motor.matchesFrame）
    static bool matchesFrameDefault(const CanFrame& cf, uint32_t targetId, bool isExt);

    Motor_Base * motorList_[MAX_MOTORS];// 电机列表

    RtosQueue<CanFrame> rxQueue_;

    /**
     * @brief 接收任务类
     */
    class RxTask : public RtosTask{
    public:
        explicit RxTask(fdCANbus* parent);
    protected:
       virtual void run() override;
    private:
        fdCANbus* parent_;
    };

    /**
     * @brief 调度器任务类
     */
    class SchedTask : public RtosTask {
    public:
        explicit SchedTask(fdCANbus* parent);
    protected:
       virtual void run() override;
    private:
        fdCANbus* parent_;
    };

    // 成员实例
    RxTask rxTask_;
    SchedTask schedulerTask_;

    bool HAL_FDCAN_Start_ERROR = false; // 记录 HAL_FDCAN_Start 是否成功

    bool HAL_FDCAN_ActivateNotification_ERROR = false; // 记录 HAL_FDCAN_ActivateNotification 是否成功
};



#endif /* __BSP_FDCAN_DRIVER_H */