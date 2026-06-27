#include "Module_HWT.h"


HWT101CT::HWT101CT(uint16_t rx_buffer_size,uint8_t *rx_buffer,UART_HandleTypeDef *uart_handle) 
    :UART_(rx_buffer_size,rx_buffer,uart_handle),
		uart_instance_(nullptr)
    , uart_initialized_(false)
    , rx_buffer_{0}
{
}

HWT101CT* HWT101CT::GetInstance(UART_HandleTypeDef *uart_handle) 
{
	  static uint8_t static_rx_buffer[64] = {0};
	  static HWT101CT instance(64,static_rx_buffer,uart_handle);
    return &instance;
}

// 初始化UART
void HWT101CT::InitUART() 
{
    if (uart_initialized_) 
	{
        return; // 已经初始化过
    }
    UART_HandleTypeDef *uart_handle=HWT101CT::UART_::GetUartHandle();

    uart_instance_ = InstanceManager::GetInstanceByUartHandle(uart_handle);
    
    // 初始化UART
    uart_instance_->UART_Init();
    
    uart_initialized_ = true;
}
float HWT101CT::calculateYaw(uint8_t YawH, uint8_t YawL)
{
    int16_t raw_yaw = (YawH << 8) | YawL;
    return ((float)raw_yaw / 32768.0f) * 180.0f;
}

uint8_t HWT101CT::calculateChecksum()
{
    return FRAME_HEADER_1 + FRAME_HEADER_2 + frame.reserved[0] + frame.reserved[1] +
           frame.reserved[2] + frame.reserved[3] + frame.YawL + frame.YawH + frame.VL + frame.VH;
}


void HWT101CT::Callback_Fuc(uint8_t *buf, uint16_t len)
{
    // 检验数据包的长度是否正确
    if(len != 0x16)
    {
        return;
    }
    // 检验数据包的两份数据的包头是否正确
    if(buf[0] != 0x55)
    {
        return;
    }
    if(buf[11] != 0x55)
    {
        return;
    }
    // 检验数据包的两份数据的id是否正确
    if(buf[1] != 0x52)
    {
        return;
    }
    if(buf[12] != 0x53)
    {
        return;
    }
    // 检验数据包的两份数据的SUM是否正确
    uint8_t sum = 0;
    for(uint8_t j = 0; j < 10; j++)
    {
        sum += buf[j];
    }
    if(sum != buf[10])
    {
        return;
    }
    sum = 0;
    for(uint8_t j = 11; j < 21; j++)
    {
        sum += buf[j];
    }
    if(sum != buf[21])
    {
        return;
    }

    orin_yawz = (int16_t)((buf[5] << 8 | buf[4]) + 2) / 32768.0f * 2000.0f; // 单位：deg/s
    orin_yaw = (int16_t)(buf[18] << 8 | buf[17]) / 32768.0f * 180.0f; // 单位：deg

    processDecodedData(orin_yaw);       
}

void HWT101CT::processDecodedData(float yaw)
{
    if (if_init)
    {
        init_count++;
        if (init_count > 6)
        {
            if_init = false;
            init_yaw = yaw;
            init_count = 0;
        }
    }
    else
    {
        yaw_tf(yaw);
    }
}
void HWT101CT::yaw_tf(float nowyaw)
{
    delta_angle = nowyaw - init_yaw;
    if (delta_angle >= 180.0f)
    {
        delta_angle -= 360.0f;
    }
    else if (delta_angle < -180.0f)
    {
        delta_angle += 360.0f;
    }
    real_yaw = delta_angle;

    yaw_rad = jia::degToRadF32(real_yaw);

    yaw_speed_rad = jia::degToRadF32(orin_yawz);
}

float HWT101CT::get_yaw_speed_rad()
{
    return yaw_speed_rad;
}

float HWT101CT::get_heading()
{
    return real_yaw;
}
float HWT101CT::get_yaw_rad()
{
    return yaw_rad;
}
void HWT101CT::imu_rst()
{

    if_init = true;
}

void HWT101CT::imu_reset_heading(float reheading)
{
    init_yaw = reheading;
}
