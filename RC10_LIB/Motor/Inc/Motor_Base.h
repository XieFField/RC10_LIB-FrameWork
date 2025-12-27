/**
 * @file Motor_Base.h
 * @author XieFField
 * @brief ������ඨ��
 * @version 1.0
 * @date 2025-09-16
 */
#ifndef MOTOR_BASE_H
#define MOTOR_BASE_H

#pragma once
#ifdef __cplusplus

#endif // __cplusplus
#include "BSP_CanFrame.h"
#include <cstdint>
#include <cstddef>
class fdCANbus; // ǰ������

//����������ͨ�ýӿ�
class Motor_Base {
public:
    Motor_Base(uint32_t id, bool isExt, fdCANbus* bus)
    {
        motor_id_ = id;
        isExtended_ = isExt;
        bus_ = bus;
    };
    virtual ~Motor_Base(){};

    // Ŀ���趨
    virtual void setTargetRPM(float rpm_set){};
    virtual void setTargetCurrent(float current_set){};
    virtual void setTargetAngle(float angle_set){};
    virtual void setTargetTotalAngle(float totalAngle_set){};

    // �����Ը��º���������ִ�п����߼����ڵ������񱻻��ѣ���������ִ�С�
    virtual void update(){};
    
    // ������ȡ
    virtual float getRPM() const { return 0.0f; }   
    virtual float getCurrent() const { return 0.0f; }
    virtual float getAngle() const { return 0.0f; }
    virtual float getTotalAngle() const { return 0.0f; }

    
    /**
     * @brief ������Ŀ�����������CAN֡
     * @param outFrames ���ڴ�Ŵ����CAN֡������
     * @param maxFrames ������������
     * @return ʵ�ʴ����CAN֡����
     */
    virtual std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) = 0;

    
    /**
     * @brief �������������Ե���ķ���CAN֡
     */
    virtual void updateFeedback(const CanFrame& cf) = 0;

    /**
     * @brief ��������CAN֡�Ƿ����ڴ˵��
     * @return ���ƥ���򷵻�true�����򷵻�false
     */
    virtual bool matchesFrame(const CanFrame& cf) const
    {
        (void)cf;
        return false;
    }

    virtual float get_GearRatio() const { return GEAR_RATIO; }

    float getTargetRPM() const { return target_rpm_; }
    float getTargetCurrent() const { return target_current_; }
    float getTargetAngle() const { return target_angle_; }
    float getTargetTotalAngle() const { return target_totalAngle_; }
    

    fdCANbus* bus() const { return bus_; }
    uint32_t getID() const { return motor_id_; }
protected:
    uint32_t motor_id_;
    bool isExtended_;
    fdCANbus* bus_;

    // Ŀ��/״̬��
    float target_rpm_ = 0.0f; //ת��
    float target_current_= 0.0f; //����
    float target_angle_ = 0.0f; //�Ƕ�
    float target_totalAngle_ = 0.0f; //�ܽǶ�
    
    float GEAR_RATIO = 1.0f; // ���ٱȣ�Ĭ��Ϊ1
    float rpm_ = 0.0f;
    float current_ = 0.0f;
    float angle_ = 0.0f;
    float totalAngle_ = 0.0f;
    float temperature_ = 0.0f; //�¶�

};




#endif // MOTOR_BASE_H
