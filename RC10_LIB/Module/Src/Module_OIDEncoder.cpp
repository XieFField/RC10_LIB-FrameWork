#include "Module_OIDEncoder.h"


std::size_t OIDEncoder::packCommand(CanFrame outFrames[], std::size_t maxFrames)
{

    if(!is_init_ || maxFrames == 0)
        return 0;

    CanFrame &cf = outFrames[0];
    cf.DLC = 8;
    cf.isextended = isExtended_;
    cf.ID = device_id_;
    std::memset(cf.data, 0, 8); // 清空数据区，避免残留数据干扰
    switch(now_cmd_)
    {
        case OID_CMD::READ_ANGLE: //读取编码器值
        {
            cf.DLC = 8;
            cf.data[0] = 0x04; //数据长度
            cf.data[1] = static_cast<uint8_t>(device_id_) & 0xFF; //编码器地址
            cf.data[2] = static_cast<uint8_t>(now_cmd_) & 0xFF; //指令码
            cf.data[3] = 0x00; //数据1

            now_cmd_ = OID_CMD::NONE_; // 读命令发送后立即清除，避免重复发送
            break;
        }

        case OID_CMD::SET_ENCODER_ID: //设置编码器地址
        {
            cf.DLC = 8;
            cf.data[0] = 0x04; //数据长度
            cf.data[1] = static_cast<uint8_t>(device_id_) & 0xFF; //编码器地址
            cf.data[2] = static_cast<uint8_t>(now_cmd_) & 0xFF; //指令码
            cf.data[3] = data_[0]; //数据1

            now_cmd_ = OID_CMD::NONE_; // 设置命令发送后立即清除，避免重复发送
            break;
        }

        case OID_CMD::SET_BOARD_RATE: //设置波特率
        {
            cf.DLC = 8;
            cf.data[0] = 0x04; //数据长度
            cf.data[1] = static_cast<uint8_t>(device_id_) & 0xFF; //编码器地址
            cf.data[2] = static_cast<uint8_t>(now_cmd_) & 0xFF; //指令码
            cf.data[3] = data_[0]; //数据1

            now_cmd_ = OID_CMD::NONE_; // 设置命令发送后立即清除，避免重复发送
            break;
        }

        case OID_CMD::SET_ENCODER_MODE:
        {
            cf.DLC = 8;
            cf.data[0] = 0x04; //数据长度
            cf.data[1] = static_cast<uint8_t>(device_id_) & 0xFF; //编码器地址
            cf.data[2] = static_cast<uint8_t>(now_cmd_) & 0xFF; //指令码
            cf.data[3] = data_[0]; //数据1

            now_cmd_ = OID_CMD::NONE_; // 设置命令发送后立即清除，避免重复发送
            break;
        }

        case OID_CMD::SET_FEEDBACK_TIME:
        {
            cf.DLC = 8;
            cf.data[0] = 0x05; //数据长度
            cf.data[1] = static_cast<uint8_t>(device_id_) & 0xFF; //编码器地址
            cf.data[2] = static_cast<uint8_t>(now_cmd_) & 0xFF; //指令码
            cf.data[3] = data_[1]; //数据1
            cf.data[4] = data_[0]; //数据2

            now_cmd_ = OID_CMD::NONE_; // 设置命令发送后立即清除，避免重复发送
            break;
        }

        case OID_CMD::SET_ZERO:
        {
            cf.DLC = 8;
            cf.data[0] = 0x04; //数据长度
            cf.data[1] = static_cast<uint8_t>(device_id_) & 0xFF; //编码器地址
            cf.data[2] = static_cast<uint8_t>(now_cmd_) & 0xFF; //指令码
            cf.data[3] = 0x00; //数据1

            now_cmd_ = OID_CMD::NONE_; // 设置命令发送后立即清除，避免重复发送
            break;
        }

        case OID_CMD::SET_REVERSE:
        {
            cf.DLC = 8;
            cf.data[0] = 0x04; //数据长度
            cf.data[1] = static_cast<uint8_t>(device_id_) & 0xFF; //编码器地址
            cf.data[2] = static_cast<uint8_t>(now_cmd_) & 0xFF; //指令码
            cf.data[3] = data_[0]; //数据1

            now_cmd_ = OID_CMD::NONE_; // 设置命令发送后立即清除，避免重复发送
            break;
        }

        case OID_CMD::READ_ANGLE_SPEED:
        {
            cf.DLC = 8;
            cf.data[0] = 0x04; //数据长度
            cf.data[1] = static_cast<uint8_t>(device_id_) & 0xFF; //编码器地址
            cf.data[2] = static_cast<uint8_t>(now_cmd_) & 0xFF; //指令码
            cf.data[3] = 0x00; //数据1

            now_cmd_ = OID_CMD::NONE_; // 读命令发送后立即清除，避免重复发送
            break;
        }

        case OID_CMD::SET_ANGLE_SPEED_HZ:
        {
            cf.DLC = 8;
            cf.data[0] = 0x05; //数据长度
            cf.data[1] = static_cast<uint8_t>(device_id_) & 0xFF; //编码器地址
            cf.data[2] = static_cast<uint8_t>(now_cmd_) & 0xFF; //指令码
            cf.data[3] = data_[0]; //数据1
            cf.data[4] = data_[1]; //数据2

            now_cmd_ = OID_CMD::NONE_; // 设置命令发送后立即清除，避免重复发送

            this->angle_speed_hz_ =1.0f / static_cast<float>(static_cast<int16_t>(data_[0]) << 8 | data_[1]) * 1000.0f; 
            break;
        }

        case OID_CMD::SET_NOW_ANGLE:
        {
            cf.DLC = 8;
            cf.data[0] = 0x07; //数据长度
            cf.data[1] = static_cast<uint8_t>(device_id_) & 0xFF; //编码器地址
            cf.data[2] = static_cast<uint8_t>(now_cmd_) & 0xFF; //指令码
            cf.data[3] = data_[0]; //数据1
            cf.data[4] = data_[1]; //数据2
            cf.data[5] = data_[2]; //数据3
            cf.data[6] = data_[3]; //数据4

            now_cmd_ = OID_CMD::NONE_; // 设置命令发送后立即清除，避免重复发送
            break;
        }

        case OID_CMD::SET_MID_ANGLE:
        {
            cf.DLC = 8;
            cf.data[0] = 0x04;
            cf.data[1] = static_cast<uint8_t>(device_id_) & 0xFF; //编码器地址
            cf.data[2] = static_cast<uint8_t>(now_cmd_) & 0xFF; //指令码
            cf.data[3] = 0x01;
            now_cmd_ = OID_CMD::NONE_; // 设置命令发送后立即清除，避免重复发送
            break;
        }

        
        case OID_CMD::NONE_:
        {
            return 0; // 无需发送
        }
    }

    return 1;
}

void OIDEncoder::updateFeedback(const CanFrame& cf)
{
    if(!is_init_)
    {
        return;
    }
    uint8_t recevied_cmd = cf.data[2]; //指令码

    switch(recevied_cmd)
    {
        case 0x01: //OID::READ_ANGLE 反馈
        {
            for(int i = 0; i < 8; ++i)
                feedback_data_[i] = cf.data[i];

            //encoder_raw_ = (cf.data[3] << 24) | (cf.data[4] << 16) | (cf.data[5] << 8) | cf.data[6];
            encoder_raw_ = (cf.data[6] << 24) | (cf.data[5] << 16) | (cf.data[4] << 8) | cf.data[3]; //注意字节顺序
            real_encoder_angle_ = static_cast<float>(encoder_raw_) * 360.0f / static_cast<float>(range_);
            angle_ = static_cast<float>(static_cast<int32_t>(encoder_raw_) - static_cast<int32_t>(mid_angle_raw_)) * 360.0f / static_cast<float>(range_);
            break;
        }

        case 0x02: //SET_ENCODER_ID 反馈
        {
            for(int i = 0; i < 8; ++i)
                feedback_data_[i] = cf.data[i];
            break;
        }

        case 0x03: //SET_BOARD_RATE 反馈
        {
            for(int i = 0; i < 8; ++i)
                feedback_data_[i] = cf.data[i];

            break;
        }

        case 0x04: //SET_ENCODER_MODE 反馈
        {
            for(int i = 0; i < 8; ++i)
                feedback_data_[i] = cf.data[i];

            break;
        }

        case 0x05: //SET_FEEDBACK_TIME 反馈
        {
            for(int i = 0; i < 8; ++i)
                feedback_data_[i] = cf.data[i];

            break;
        }

        case 0x06: //SET_ZERO 反馈
        {
            for(int i = 0; i < 8; ++i)
                feedback_data_[i] = cf.data[i];

            break;
        }

        case 0x07: //SET_REVERSE 反馈
        {
            for(int i = 0; i < 8; ++i)
                feedback_data_[i] = cf.data[i];

            break;
        }

        case 0x0A: //READ_ANGLE_SPEED 反馈
        {
            for(int i = 0; i < 8; ++i)
                feedback_data_[i] = cf.data[i];

            //this->angle_speed_raw_ = (cf.data[3] << 24) | (cf.data[4] << 16) | (cf.data[5] << 8) | cf.data[6];
            this->angle_speed_raw_ = (cf.data[6] << 24) | (cf.data[5] << 16) | (cf.data[4] << 8) | cf.data[3]; //注意字节顺序
            this->read_rpm_ = this->angle_speed_raw_ / this->range_ / ( 1/ this->angle_speed_hz_)  * 60.0f; // 转速 = 角速度 / (1/采样时间) * 60
            break;
        }

        case 0x0B: //SET_ANGLE_SPEED_HZ 反馈
        {
            for(int i = 0; i < 8; ++i)
                feedback_data_[i] = cf.data[i];

            break;
        }

        case 0x0D: //SET_NOW_ANGLE 反馈
        {
            for(int i = 0; i < 8; ++i)
                feedback_data_[i] = cf.data[i];

            break;
        }
    } 
}

OIDEncoder::testtask::testtask(OIDEncoder* parent)
    : RtosTask("OID_TestTask", 1), parent_(parent) {}
 
int test_index = 100;
int8_t encoder_id = 91;
int8_t reverse = 0;
int32_t now_angle_raw = 0;
float now_angle = 0.0f; 
void OIDEncoder::testtask::loop()
{
    if(!parent_->is_init_)
    {
        return;
    }

    switch(test_index)
    {
        case 0:
        {
            parent_->set_encoder_id(encoder_id);
            break;
        }

        case 1:
        {
            parent_->set_auto_feedback();
            break;
        }

        case 2:
        {
            parent_->set_feeedback_time(10000); // 10ms反馈一次
            break;
        }

        case 3:
        {
            parent_->set_zero();
            break;
        }

        case 4:
        {
            parent_->set_reverse(reverse);
            break;
        }

        case 5:
        {
            parent_->set_now_angle_raw(static_cast<int32_t>(now_angle_raw));
            break;
        }

        case 6:
        {
            parent_->set_mid_angle();
            break;
        }

        case 7:
        {
            parent_->read_angle_cmd();
            break;
        }

        case 8:
        {
            parent_->set_now_angle(now_angle);
            break;
        }

        default:
            break;
    }
}