/***************************************************************************
 *                                                                         *
 *               RRRRR    TTTTT    OOOOO    SSSS                           *
 *               R    R     T      O    O   S                              *
 *               R    R     T      O    O   SSSS                           *
 *               RRRRR      T      O    O       S                          *
 *               R   R      T      O    O       S                          *
 *               R    R     T      OOOOO    SSSS                           *
 *                                                                         *
 ***************************************************************************/

/**
 * @file BSP_RTOS.h
 * @author XieFField
 * @version 1.0
 * @brief freeRTOS任务调度父类
 * @attention hpp文件只允许被C++文件引用
 */
//111




#ifndef BSP_RTOS_H
#define BSP_RTOS_H

#pragma once


#ifdef __cplusplus

extern "C" 
{
    #include "stm32h7xx_hal.h"
    #include "cmsis_os.h"
    #include "FreeRTOS.h"
    #include "task.h"
    #include "queue.h"
    #include "semphr.h"
    #include <cstdint>
}

enum class TaskPrio {
    LOW = tskIDLE_PRIORITY+1,
    NORMAL = tskIDLE_PRIORITY+3,
    HIGH = tskIDLE_PRIORITY+5
};

/**
 * @brief FreeRTOS任务封装类
 *       继承该类并实现 run() 方法，即可创建一个 FreeRTOS 任务。
 *       用户在创建完任务对象后，需要使用 start() 方法注册并启动任务。
*/

class RtosTask{
public:
    /**
     * @brief 构造函数
     * @param name 任务名
     * @param period 任务周期 (Ticks)。
     *               - period > 0 (默认 = 1): 创建一个周期性任务，用户需实现 loop()。
     *               - period = 0           : 创建一个事件驱动任务，用户需重写 run()。
     */
    explicit RtosTask(const char* name, TickType_t period = 1) 
        : name_(name), handle_(nullptr), period_(period) {}

    virtual ~RtosTask() 
    {
        // 还是不在此处做任务的自动删除好了。系统停止会自动删除
    }

    /**
     * @brief 启动任务，用于注册任务到 FreeRTOS
     * @param priority 任务优先级
     * @param stackWords 任务栈大小，单位为字（4字节）
     * @param arg 传递给任务的参数
     * @return true 启动成功，false 启动失败
     *         后续用户可以通过bool返回值判断任务是否创建成功
     *         若创建失败，handle_将保持为nullptr
     */
    bool start(UBaseType_t priority, uint32_t stackWords = 128, void* arg = nullptr) 
    {
        BaseType_t res = xTaskCreate(&RtosTask::taskEntry,
                                     name_,
                                     (configSTACK_DEPTH_TYPE)stackWords,
                                     this,
                                     priority,
                                     &handle_);
        return res == pdPASS;
    }

    /**
     * @brief 获取任务句柄
     * @return 任务句柄
     */
    TaskHandle_t handle() const { return handle_; }

protected:
    
    /**
     * @brief [事件驱动任务] 的主函数。
     *        如果任务是事件驱动的 (period=0)，则重写此函数。
     *        必须在内部包含 for(;;) 和阻塞调用。
     */
    virtual void run() 
    { 
        // 提供一个默认的空实现
        for(;;)
        {
            osDelay(1000); // 默认每秒阻塞一次，防止空循环
        }
    }

    /**
     * @brief [周期性任务] 的循环体。
     *        如果任务是周期性的 (period>0)，则重写此函数。
     */
    virtual void loop() 
    {
        // 提供一个默认的空实现
    }

private:

    /**
     * @brief 任务入口函数
     */
    static void taskEntry(void* pvParameters) 
    {
        RtosTask* self = static_cast<RtosTask*>(pvParameters);
        if (self->period_ > 0) 
        {
            // 周期性任务模式
            for (;;) 
            {
                self->loop();
                osDelay(self->period_);
            }
        } 
        else  // 事件驱动/一次性任务模式
            self->run();
         // 如果 run() 返回或循环退出，删除任务
        vTaskDelete(nullptr);
    }

    const char* name_;
    TaskHandle_t handle_;
    TickType_t period_;
};

/**
 * @brief FreeRTOS队列封装类
 *        在构造函数
 * @attention 如果要在中断中发送队列，请使用 sendFromISR() 方法
 */
/*==============================================================
 * RtosQueue: 队列封装类
 * 用法：(〃'▽'〃)
 *   RtosQueue<int> queue(8); // 创建一个长度为8的int类型队列
 *   queue.send(123);         // 发送数据
 *   int value; 
 *   queue.recv(value); // 接收数据
 *      (*^▽^*)

*/
template<typename T>
class RtosQueue{
public:
    /**
     * @brief 构造函数，创建一个队列
     * @param len 队列长度，默认8
     */
    explicit RtosQueue(size_t len = 8) {queue_ = xQueueCreate((UBaseType_t)len, sizeof(T));}

    ~RtosQueue() 
    {
        // 删除队列
        if (queue_) 
            vQueueDelete(queue_);
    }

    /**
     * @brief 发送数据到队列
     * @param item 要发送的数据
     * @param ticks 等待时间，默认不等待
     * @return true 发送成功，false 发送失败
     */
    bool send(const T& item, TickType_t ticks = 0) 
    {
        if (!queue_) 
            return false;
        return xQueueSend(queue_, &item, ticks) == pdTRUE;
    }

    /**
     * @brief 从ISR中发送数据到队列
     * @param item 要发送的数据
     * @param pxHigherPriorityTaskWoken 如果发送数据后有更高优先级的任务被唤醒，置为pdTRUE
     * @return true 发送成功，false 发送失败
     */
    bool sendFromISR(const T& item, BaseType_t* pxHigherPriorityTaskWoken) 
    {
        if (!queue_) 
            return false;
        return xQueueSendFromISR(queue_, &item, pxHigherPriorityTaskWoken) == pdTRUE;
    }

    /**
     * @brief 从队列接收数据
     * @param out 接收数据的存放位置
     * @param ticks 等待时间，默认无限等待
     * @return true 接收成功，false 接收失败
     *       若队列为空且等待时间到达，返回false
     */
    bool recv(T& out, TickType_t ticks = portMAX_DELAY) 
    {
        if (!queue_) 
            return false;
        return xQueueReceive(queue_, &out, ticks) == pdTRUE;
    }

    /**
     * @brief 获取底层队列句柄
     * @return 队列句柄
     */
    QueueHandle_t raw() const { return queue_; }

private:
    QueueHandle_t queue_;
};


#endif


#endif