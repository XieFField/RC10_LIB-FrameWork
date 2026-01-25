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
	uint8_t i = 0;
	uint8_t CRC_check[2];//CRC校验位，此文件未启用
	uint8_t break_flag = 1;
	while(i < len && break_flag == 1)
	{
	    switch (rx_state)
    {
    case WAITING_FOR_HEADER_1:
        if (buf[i] == FRAME_HEADER_1)
        {
            rx_state = WAITING_FOR_HEADER_2;
        }
				i++;
        break;

    case WAITING_FOR_HEADER_2:
        if (buf[i] == FRAME_HEADER_2)
        {
            rx_state = WAITING_FOR_RESERVED_1;
            reserved_index = 0;
        }
        else
        {
            rx_state = WAITING_FOR_HEADER_1;
        }
				i++;
        break;

    case WAITING_FOR_RESERVED_1:
    case WAITING_FOR_RESERVED_2:
    case WAITING_FOR_RESERVED_3:
    case WAITING_FOR_RESERVED_4:
        frame.reserved[reserved_index++] = buf[i];
        if (reserved_index >= 4)
        {
            rx_state = WAITING_FOR_YAWL;
        }
        else
        {
            rx_state = static_cast<RxState>(rx_state + 1);
        }
				i++;
        break;

    case WAITING_FOR_YAWL:
        frame.YawL = buf[i];
        rx_state = WAITING_FOR_YAWH;
		    i++;
        break;

    case WAITING_FOR_YAWH:
        frame.YawH = buf[i];
        rx_state = WAITING_FOR_VL;
				i++;
        break;

    case WAITING_FOR_VL:
        frame.VL = buf[i];
        rx_state = WAITING_FOR_VH;
				i++;
        break;

    case WAITING_FOR_VH:
        frame.VH = buf[i];
        rx_state = WAITING_FOR_CHECKSUM;
				i++;
        break;

    case WAITING_FOR_CHECKSUM:
        frame.checksum = buf[i];
        calculated_checksum = calculateChecksum();

        if (calculated_checksum == frame.checksum)
        {
            orin_yaw = -calculateYaw(frame.YawH, frame.YawL);
            processDecodedData(orin_yaw);       
        }
				break_flag = 0;
        rx_state = WAITING_FOR_HEADER_1;
        break;

    default:
        rx_state = WAITING_FOR_HEADER_1;
        break;
    }
	}
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
    now_time = HAL_GetTick();

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
    yaw_rad = real_yaw * 0.0174533f;
    if (last_update_time != 0)
    {
        delta_time = (float)(now_time - last_update_time) / 1000.0f;
        if (delta_time != 0.0f && last_yaw != 0.0f)
        {
            yaw_speed_rad = (yaw_rad - last_yaw) / delta_time;
        }
    }
    last_yaw = yaw_rad;

    last_update_time = now_time;
}

float HWT101CT::get_yaw_speed_rad()
{
    return yaw_speed_rad;
}

uint32_t HWT101CT::get_update_time()
{
    return last_update_time;
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
