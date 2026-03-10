/**
 * @file Motor_DJI.h
 * @author XieFField
 * @brief �󽮵����
 * @version 1.0
 * @date 2025-09-19
 * 
 * ���ļ������� M3508 M2006 GM6020��װ
 * 
 */

/*
��װ˼·�ο��˽����Ŀ�ܣ�����������Ƴ������������DJI������Ի�������
*/

#ifndef __DJI_MOTOR_H
#define __DJI_MOTOR_H
#pragma once

#ifdef __cplusplus
extern "C" {
    #include <stdint.h>
    #include "BSP_CanFrame.h"
}
#endif

/*
    ____      ______     __  _______  __________  ____ 
   / __ \    / /  _/    /  |/  / __ \/_  __/ __ \/ __ \
  / / / /_  / // /     / /|_/ / / / / / / / / / / /_/ /
 / /_/ / /_/ // /     / /  / / /_/ / / / / /_/ / _, _/ 
/_____/\____/___/    /_/  /_/\____/ /_/  \____/_/ |_|  
*/


#ifdef __cplusplus

#include "Motor_Base.h"
#include "APP_tool.h"
#include "APP_PID.h"
#include <cstring>
#include "Module_Encoder.h"

typedef enum {
    M3508_Type,
    M2006_Type,
    GM6020_Type
} DJI_MotorType;

//
/**
 * @brief     ������4�����֡
 * @attention �������Ҫ��ͬһ·CAN�ϴ���GM6020��M3508/M2006����ȷ��GM6020����Ƭ��
 *            ��M3508/M2006����Ƭ�ϡ���ΪM3508/M2006��ID��Χ��0x201��0x208, 
 *            ��GM6020��ID��Χ��0x205��0x211
 */
class DJI_Motor : public Motor_Base {
public:
    /**
     * @brief     ������4�����֡
     * @attention �������Ҫ��ͬһ·CAN�ϴ���GM6020��M3508/M2006����ȷ��GM6020����Ƭ��
     *            ��M3508/M2006����Ƭ�ϡ���ΪM3508/M2006��ID��Χ��0x201��0x208, 
     *            ��GM6020��ID��Χ��0x205��0x211
     */
    DJI_Motor(DJI_MotorType type, uint32_t id, fdCANbus *bus);
    ~DJI_Motor(){};

    bool matchesFrame(const CanFrame& cf) const override
    {
        if(cf.isextended) 
            return false; // DJI���ֻ�ñ�׼֡

        if(type_ == GM6020_Type)//��0x204+������ID
        {
            if(cf.ID < (0x204 + 1) || cf.ID > (0x204 + 7))
                return false; // �Ƿ�ID

            else if(cf.ID >= (0x204 + 1) && cf.ID <= (0x204 + 7))
                return (cf.ID == (0x204 + motor_id_));
            
            else
                return false; // �Ƿ�ID 
        }
        if(type_ == M3508_Type || type_ == M2006_Type)//��0x200+������ID 1~8
        {
            if(cf.ID < (0x200 + 1) || cf.ID > (0x200 + 8))
                return false; // �Ƿ�ID

            else if(cf.ID >= (0x200 + 1) && cf.ID <= (0x200 + 8))
                return (cf.ID == (0x200 + motor_id_));

            else
                return false; // �Ƿ�ID
        }
        else
            return false; // δ֪����
    }

    /**
     * @brief ���������ֵת��Ϊʵ�ʵ���ֵ
     */
    float virtualCurrent_to_realCurrent(int16_t virtualCurrent);

    /**
     * @brief ��ʵ�ʵ���ֵת��Ϊ�������ֵ
     */
    int16_t realCurrent_to_virtualCurrent(float realCurrent);


    std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) override
    {
        return 0; // �� DJI_Group ������
    }

    void updateFeedback(const CanFrame& cf) override;

    void setTargetCurrent(float current_set) override
    {
        target_current_ = current_set;
    }

    DJI_MotorType getType() const { return type_; }

    void reset_AnglePidTimePSC(int reset_value = 0)
    {
        anglePid_timePSC = reset_value;
    }

    /**
     * @brief ������������·�����¶�λ��ָ��ֵ���ض���ƫ����
     */
    void relocate_totalAngle(float now_totalAngle)
    {
        // [Fix] �����ض�λ�߼�������Ϊ�����Ƕȣ���ת��Ϊת�ӽǶ����ñ�����
        encoder_.relocate_totalAngle(now_totalAngle * get_GearRatio());
        totalAngle_ = encoder_.getTotalAngle() / get_GearRatio();

        this->angle_ = fmodf(this->totalAngle_, 360.0f);
        if(this->angle_ < 0) 
            this->angle_ += 360.0f;
    }

protected:
    int anglePid_timePSC = 10; //�Ƕ�ʱ���Ƶ Ĭ��Ϊ 10 ������Ƶ��Ϊ100Hz
    int anglePid_timeCnt = 0; //�Ƕ�ʱ�����

private:
     //1~8
    DJI_MotorType type_;
    int16_t encoder_raw_ = 0; //ԭʼ������ֵ

    

//common
    virtual uint32_t recv_id() const{return 0x200;}

    Encoder encoder_; //������ʵ��
};

uint32_t send_idLow();
uint32_t send_idHigh();
uint32_t send_idLow6020();
uint32_t send_idHigh6020();

//------- DJI_Group ------- ������� -----------------------------
// 
/**
 * @brief     ������4�����֡
 * @attention �������Ҫ��ͬһ·CAN�ϴ���GM6020��M3508/M2006����ȷ��GM6020����Ƭ��
 *            ��M3508/M2006����Ƭ�ϡ���ΪM3508/M2006��ID��Χ��0x201��0x208, 
 *            ��GM6020��ID��Χ��0x205��0x211
 */
class DJI_Group : public Motor_Base {
public:
    static constexpr std::size_t MAX_GROUP_SIZE = 4;
    
    DJI_Group(uint32_t baseTxId, fdCANbus* bus);
    ~DJI_Group() = default;

    /**
     * @brief ע����
     */
    bool addMotor(DJI_Motor* motor);

    std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) override;

    void updateFeedback(const CanFrame& cf) override
    {
        (void)cf;
    }

    void update() override
    {
        //do nothing
    }

private:
    uint32_t baseTxID_;
    
    DJI_Motor* motors_p[MAX_GROUP_SIZE] = {nullptr, nullptr, nullptr, nullptr};
    uint8_t motor_count_ = 0;

    bool containsGM6020 = false; //�Ƿ����GM6020, M3508/M2006����GM6020����

    int calcSlot(uint32_t motorID, DJI_MotorType type) const; // �����λ
};

#define M3508_DECRATION (3591.0f/187.0f) //���ٱ�
#define M3508_KT 0.01562f // 3508����Ȧת�س��� ��λ��N.M/A

// ����ģʽö��
typedef enum {
    CURRENT_CONTROL, // ������������
    SPEED_CONTROL,   // �ٶȱջ�����
    ANGLE_CONTROL,    // �Ƕȱջ�����
    TOTAL_ANGLE_CONTROL // �ܽǶȱջ�����
} ControlMode;

class M3508 : public DJI_Motor {
public:
    M3508(uint32_t motor_id, fdCANbus* bus);
    ~M3508() {};

    void pid_init(const PID_Param_Config& speed_params, float speed_tdRatio, const PID_Param_Config& angle_params, float angle_I_Separa);

    // ���ƽӿ�
    void setTargetCurrent(float current_set) override;
    void setTargetRPM(float rpm_set) override;
    void setTargetAngle(float angle_set) override;
    void setTargetTotalAngle(float totalAngle_set) override;

    void update() override; //�����Ը���

    // ��ȡ�����״̬
    float getRPM() const override;
    float getAngle() const override;
    float getTotalAngle() const override;

    void reset_GearRatio(float reset_value){GEAR_RATIO = reset_value;}

    /**
     * @brief ���ٻ�ǰ�����꣬����Ŀ��ת�ٵ�ת��Ϊ�������̶ȵĵ������
     *        ʹ���������ٵ�Ŀ��ת������ʱ����ǿ���ǰ���̡�
     * @param kf ǰ����(Ŀ��ת�٣�rpm) -> (������) ���ӳ�ϵ��
     *           ��ÿ���ж�Ȳ���: ̲ת��D����ʱ�Ĵ���ΪVn(rpm)���¶ȽϻǶ͸ö�Ĵ�������Ϊ I_ss(mA)
     *           kf = I_ss / Vn. ������Ĵ���Ŀ��Ȳ㿪Ƿ�����.
     *           ΪʹÿΪ0ʱ�ǰ�ǰ�������д�Ĭ��Ϊ0.
     */
    void set_speed_ff_gain(float kf) { speed_pid_.set_ff_gain(kf); }

    float get_GearRatio() const override { return GEAR_RATIO; }
private:
    
    ControlMode mode_ = CURRENT_CONTROL;
    float GEAR_RATIO = 19.2032f; // ���ٱ�

    PID_Incremental speed_pid_;
    PID_Position angle_pid_;
};

#define M2006_DECRATION 36.0f //���ٱ�
class M2006 : public DJI_Motor {
public:
    M2006(uint32_t motor_id, fdCANbus* bus);
    ~M2006() {};

    void pid_init(const PID_Param_Config& speed_params, float speed_I_Separa, const PID_Param_Config& angle_params, float angle_I_Separa);

    // ���ƽӿ�
    void setTargetCurrent(float current_set) override;
    void setTargetRPM(float rpm_set) override;
    void setTargetAngle(float angle_set) override;
    void setTargetTotalAngle(float totalAngle_set) override;

    void update() override;

    // ��ȡ�����״̬
    float getRPM() const override;
    float getAngle() const override;
    float getTotalAngle() const override;
    float get_GearRatio() const override { return GEAR_RATIO; }
    void reset_GearRatio(float reset_value){GEAR_RATIO = reset_value;}

    /**
     * @brief ���ٻ�ǰ�����꣬��M3508 set_speed_ff_gain ˵�������
     * @param kf ǰ����(Ŀ��ת�٣�rpm) -> (������) ���ӳ�ϵ��
     */
    void set_speed_ff_gain(float kf) { speed_pid_.set_ff_gain(kf); }


private:
    ControlMode mode_ = CURRENT_CONTROL;
    float GEAR_RATIO = 36.0f;

    PID_Incremental speed_pid_;
    PID_Position angle_pid_;
};

class GM6020 : public DJI_Motor {
public:
    GM6020(uint32_t motor_id, fdCANbus* bus);
    ~GM6020() {};

    void pid_init(const PID_Param_Config& speed_params, float speed_I_Separa, const PID_Param_Config& angle_params, float angle_I_Separa);

    // ���ƽӿ�
    void setTargetCurrent(float current_set) override;
    void setTargetRPM(float rpm_set) override;
    void setTargetAngle(float angle_set) override;
    void setTargetTotalAngle(float totalAngle_set) override;

    void update() override;

    // ��ȡ�����״̬ (GM6020�޼�����)
    float getRPM() const override { return rpm_; }
    float getAngle() const override { return angle_; }
    float getTotalAngle() const override { return totalAngle_; }

    /**
     * @brief ���ٻ�ǰ�����꣬��M3508 set_speed_ff_gain ˵�������
     * @param kf ǰ����(Ŀ��ת�٣�rpm) -> (������) ���ӳ�ϵ��
     */
    void set_speed_ff_gain(float kf) { speed_pid_.set_ff_gain(kf); }

private:
    ControlMode mode_ = CURRENT_CONTROL;
    float GEAR_RATIO = 1.0f;

    PID_Incremental speed_pid_;
    PID_Position angle_pid_;
};


#endif



#endif // __DJI_MOTOR_H