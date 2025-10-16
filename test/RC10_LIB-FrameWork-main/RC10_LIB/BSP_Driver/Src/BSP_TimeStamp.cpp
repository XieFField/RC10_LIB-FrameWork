#include "BSP_TimeStamp.h"

// 初始化静态成员变量
TIM_HandleTypeDef* TimeStamp::s_htim_ = nullptr;
volatile uint64_t TimeStamp::s_overflow_count_ = 0;

// 私有构造函数
TimeStamp::TimeStamp() 
{
    // do nothing
}

// 获取单例实例
TimeStamp& TimeStamp::getInstance() 
{
    static TimeStamp instance;
    return instance;
}

// 初始化
void TimeStamp::init(TIM_HandleTypeDef* htim) 
{
    if (s_htim_ == nullptr && htim != nullptr)
    {
        s_htim_ = htim;
        // 启动定时器并使能更新中断
        HAL_TIM_Base_Start_IT(s_htim_);
    }
}

// 溢出中断回调
void TimeStamp::overflowCallback() 
{
    // 检查定时器是否已初始化且中断标志位已设置
    if (s_htim_ != nullptr && __HAL_TIM_GET_FLAG(s_htim_, TIM_FLAG_UPDATE) != RESET) 
    {
        if (__HAL_TIM_GET_IT_SOURCE(s_htim_, TIM_IT_UPDATE) != RESET) 
        {
            __HAL_TIM_CLEAR_IT(s_htim_, TIM_IT_UPDATE);
            s_overflow_count_++;
        }
    }
}

// 获取微秒
uint64_t TimeStamp::getMicroseconds() const 
{
    if (!s_htim_) return 0;

    uint64_t overflow_val;
    uint32_t counter_val;

    // --- 进入临界区 ---
    uint32_t primask = __get_PRIMASK(); // 保存当前的中断状态
    __disable_irq();                    // 禁止所有中断

    // 在中断被禁止的情况下，安全地读取
    overflow_val = s_overflow_count_;
    counter_val = s_htim_->Instance->CNT;

    // 检查在读取CNT后，是否刚好发生了溢出中断（标志位被硬件置位）
    // 这是为了处理在读取CNT的瞬间，溢出中断即将发生但还未被处理的情况
    if (__HAL_TIM_GET_FLAG(s_htim_, TIM_FLAG_UPDATE) != RESET)
    {
        // 如果标志位为1，说明CNT已经翻转，但overflow_val还没来得及+1
        //之前就是因为这里64位溢出而导致会出现随机的无穷大
        // 所以我们再次读取一次，确保拿到最新的值
        overflow_val = s_overflow_count_ + 1;
    }

    // --- 退出临界区 ---
    if (!primask) // 如果之前中断是开启的，就恢复它
    {
        __enable_irq();
    }

    const uint64_t period = s_htim_->Instance->ARR + 1;
    return (overflow_val * period) + counter_val;
}

// 获取毫秒
uint64_t TimeStamp::getMilliseconds() const 
{
    return getMicroseconds() / 1000;
}

// 获取秒
float TimeStamp::getSeconds() const 
{
    return static_cast<float>(getMicroseconds()) * 1e-6f;
}


