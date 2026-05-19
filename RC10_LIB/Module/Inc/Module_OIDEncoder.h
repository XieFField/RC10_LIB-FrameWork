/**
 * @file Module_OIDEncoder.h
 * @brief OID编码器模块声明
 * @author XieFField
 * @version 1.0
 */

#ifndef MODULE_OID_ENCODER_H
#define MODULE_OID_ENCODER_H

#pragma once

#ifdef __cplusplus
extern "C"
{
}
#endif

#ifdef __cplusplus

#include "BSP_CanFrame.h"
#include <cstdint>
#include <cstddef>
#include <cstring>
#include "BSP_RTOS.h"

class fdCANbus; // 前置声明

namespace OID_Encoder
{
    
    typedef enum OID_CMD
    {
        READ_ANGLE = 0x01,
        SET_ENCODER_ID = 0x02, // uint8_t 编码器地址，范围0~255
        SET_BOARD_RATE = 0x03, //慎用 0~4 0x00：500K（默认）；0x01:1M 0x02：250K；0x03:125K 0x04：100K
        SET_ENCODER_MODE = 0x04, //初始化时候设置一次即可，数据码：0x00： 
                                    //查询 0x02：自动返回编码器角速度值 0xAA：自动返回编码器值
        SET_FEEDBACK_TIME = 0x05, //设置自动返回的时间间隔，单位微秒，数值范围：50~65535 uint16 
                                    // 下发：[0x05][0x01][0x05][0xE8][0x03]
                                    // 回传速度过快的话，用上位机软件设置其他参数很容易失败，建议100hz即可
        SET_ZERO = 0x06, //设置当前位置为零点
        SET_REVERSE = 0x07, //设置正反转 data: 0x00顺时针正转 0x01逆时针正转

        READ_ANGLE_SPEED = 0x0A, // 读取编码器角速度值。 return int32, 
        SET_ANGLE_SPEED_HZ = 0x0B, //设置编码器角速度采样时间：0~65535单位ms int16_t 

        SET_MID_ANGLE = 0x0C, //设置编码器中位角度值 

        SET_NOW_ANGLE = 0x0D, //设置编码器当前角度值，数据格式：int32 数值范围：0~X（X为单圈分辨率*圈数）
        NONE_ = 0x00,
    };


} // namespace OID_Encoder

using namespace OID_Encoder;
class OIDEncoder {
public:
    OIDEncoder(uint32_t id, fdCANbus* bus, int32_t range, int16_t max_rounds)
        : device_id_(id), isExtended_(false), bus_(bus), range_(range), testTask_(this), max_rounds_(max_rounds)
    {
        
    }

    ~OIDEncoder() = default;

    uint32_t id() const { return device_id_; }
    fdCANbus* bus() const { return bus_; }

    void init_test_task()// 主要用于设置ENCODER_ID 和 ENCODER_MODE
    {
        testTask_.start(osPriorityNormal);
    }

    void init()
    {
        this->mid_angle_raw_ = static_cast<uint32_t>(max_rounds_ * range_ / 2); //默认中位角为最大值的一半
        set_auto_feedback();
        set_feeedback_time(20000); // 20ms反馈一次
       is_init_ = true;
    }

    /**
     * 发送： 0x04（数据长度）+0x01（编码器地址）+0x01（指令码）+0x00（数据1）
     * 接收：0X07（数据长度）+0X01（编码器地址）+0X01（指令码）+0x00012345（数据）
     */

    void updateFeedback(const CanFrame& cf);
    bool matchesFrame(const CanFrame& cf)const
    {
        // 简单匹配：ID和是否扩展帧必须匹配
        return ((cf.ID == device_id_) && (cf.isextended == isExtended_)) 
            || cf.data[1] == static_cast<uint8_t>(device_id_);
    }

    

    std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames);

    void setCmd(OID_CMD cmd)
    {
        now_cmd_ = cmd;
    }

    void update(){/*do nothing*/}


    void read_angle_cmd()
    {
        set_cmd(OID_CMD::READ_ANGLE);
    }

    void set_auto_feedback()
    {
        set_cmd(OID_CMD::SET_ENCODER_MODE, static_cast<uint8_t>(0xAA)); //设置自动返回编码器值
    }

    void set_feeedback_time(uint16_t time_us)
    {
        set_cmd(OID_CMD::SET_FEEDBACK_TIME, time_us);
    }

    void set_zero()
    {
        set_cmd(OID_CMD::SET_ZERO);
    }

    void set_encoder_id(uint8_t id)
    {
        set_cmd(OID_CMD::SET_ENCODER_ID, id);
        device_id_ = id; // 同步更新当前ID，确保后续通信正常
    }

    void set_reverse(uint8_t reverse)
    {
        set_cmd(OID_CMD::SET_REVERSE, reverse);
    }

    void set_now_angle(float angle)
    {
        uint32_t angle_raw = static_cast<uint32_t>((angle / 360.0f) * static_cast<float>(range_));
        set_cmd(OID_CMD::SET_NOW_ANGLE, static_cast<uint32_t>(angle_raw));
    }

    void set_now_angle_raw(uint32_t angle_raw)
    {
        set_cmd(OID_CMD::SET_NOW_ANGLE, angle_raw);
    }

    void set_mid_angle()
    {
        set_cmd(OID_CMD::SET_MID_ANGLE);
    }

    float get_angle()const
    {
        return angle_;
    }

    float get_real_encoder_angle()const
    {
        return real_encoder_angle_;
    }

    uint32_t get_encoder_raw()const
    {
        return encoder_raw_;
    }
    
    float get_rpm()const
    {
        
        return read_rpm_;
    }

    bool is_init()const
    {
        return is_init_;
    }
    uint16_t get_controlFrequency() const { return control_Frequency_; }
    uint16_t get_controlCnt() const { return control_cnt; }
    void reset_controlCnt() { control_cnt = 0; }
    void increment_controlCnt() { control_cnt++; }
private:
    /**
     * @brief 无需设置目标值的cmd接口
     */
    void set_cmd(OID_CMD cmd)
    {
        now_cmd_ = cmd;
    }

    /**
     * @brief 需要设置目标值的cmd接口，data1的具体含义根据cmd不同而不同
     * @attention SET_ENCODER_ID SET_BOARD_RATE 
     */
    void set_cmd(OID_CMD cmd, uint8_t data1)
    {
        now_cmd_ = cmd;
        data_[0] = data1;
    }

    /**
     * @brief 需要设置目标值的cmd接口，data1的具体含义根据cmd不同而不同
     * @attention SET_ANGLE_SPEED_HZ SET_FEEDBACK_TIME
     */
    void set_cmd(OID_CMD cmd, uint16_t data1)
    {
        now_cmd_ = cmd;
        data_[0] = (data1 >> 8) & 0xFF;
        data_[1] = data1 & 0xFF;
    }

    /**
     * @brief 需要设置目标值的cmd接口，data1的具体含义根据cmd不同而不同
     * @attention SET_NOW_ANGLE
     */
    void set_cmd(OID_CMD cmd, uint32_t data1)
    {
        now_cmd_ = cmd;
        data_[0] = (data1 >> 24) & 0xFF;
        data_[1] = (data1 >> 16) & 0xFF;
        data_[2] = (data1 >> 8) & 0xFF;
        data_[3] = data1 & 0xFF;
    }

    bool is_init_ = false;
    uint32_t device_id_;
    bool isExtended_;
    fdCANbus* bus_;
    uint32_t range_; //编码器线程
    int16_t max_rounds_; //编码器最大累计圈数

    uint32_t mid_angle_raw_ = 0; //编码器中位角度对应的原始值


    uint8_t data_[4] = {0}; //通用数据区，具体含义根据cmd不同而不同
    uint8_t feedback_data_[8] = {0}; //通用反馈数据区，具体含义根据cmd不同而不同

    float angle_ = 0.0f; //当前角度，单位度 //减速前的角度值
    float real_encoder_angle_ = 0.0f;
    uint32_t encoder_raw_ = 0; //原始编码器值

    uint8_t retrun_id_ = 0; //当前帧的返回id，范围0~255

    int32_t angle_speed_raw_ = 0; //原始角速度值 int32
    float read_rpm_ = 0.0f; //当前角速度
    float angle_speed_hz_ = 0.0f; //角速度采样时间，单位Hz

    OID_CMD now_cmd_ = OID_CMD::NONE_;

    uint16_t control_cnt = 0;
    uint16_t control_Frequency_ = 100; // 默认控制频率 Hz，重设的控制频率必须是100的整数倍

protected:    
    class testtask: public RtosTask
    {
        public: 
            explicit testtask(OIDEncoder* parent);

        protected:
            void loop() override;

        private:
            OIDEncoder* parent_;
    };

    testtask testTask_;
};



#endif
#endif // MODULE_OID_ENCODER_H