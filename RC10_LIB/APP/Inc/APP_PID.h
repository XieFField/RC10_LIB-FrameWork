/**
 * @file APP_PID.h
 * @author XieFField
 * @brief PID ������
 * @version 1.0
 * @date 2025-09-17
 */

/*
    �˴������㷨�Ϻܴ�̶Ƚ���˽����Ĵ��룬�ڴ˱�ʾ��л
    ֮����û��ֱ��ʹ�ã�����Ϊ�������ѧ�߼���Ȼ���ʣ�����
    ������ͽṹ������C++�����ǰ��൱���˽ṹ����ʹ�ã�
    ����Ϊ�ƻ��˷�װ�ԣ�������׳
    =================================================
    �������㷨�ϣ�����Ϊ��Щ�ط����ԸĽ�����Ҫ�������ڲ���
    ʱ��ļ����ϣ��Լ����ֿ����͵��߼������������ֱ����
    ��������е���ڼ�������������ϵͳ�𵴡�
    =================================================
*/

#ifndef __APP_PID_H
#define __APP_PID_H

#pragma once

#ifdef __cplusplus

extern "C" {

#include "stm32h7xx_hal.h" 

}
#include <cstdint>
#include <stdbool.h>
#include <cmath>
#include "APP_tool.h"
#include "BSP_TimeStamp.h"

/*
    ______ _            __  _____  ____________     ______
   /  _/ /( )_____     /  |/  /\ \/ / ____/ __ \   / / / /
   / // __/// ___/    / /|_/ /  \  / / __/ / / /  / / / / 
 _/ // /_  (__  )    / /  / /   / / /_/ / /_/ /  /_/_/_/  
/___/\__/ /____/    /_/  /_/   /_/\____/\____/  (_|_|_)   
*/                                                          

typedef struct {
    float kp;
    float ki;
    float kd;
    float I_Outlimit; // �����޷�
    bool isIOutlimit; // �Ƿ����û����޷�
    float output_limit;   // ����޷�
    float deadband;      // ����
} PID_Param_Config;

// λ��ʽ PID �����������������λ��֣�΢�����У����ַ���ĸĽ�ʽPID Ĭ�Ͽ���Ƶ��100Hz
class PID_Position {
public:
    /**
     * @brief ���캯��
     * @param pid_Param_Config PID �������ýṹ��
     * @param I_SeparaThreshold ���ַ�����ֵ����������ֵ���ڸ�ֵʱ��������ۼ�
     */
    PID_Position(PID_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, false, 0.0f, 0.0f}, 
        float I_SeparaThreshold = 0.0f)
        : params_(params), I_SeparaThreshold_(I_SeparaThreshold) 
        {
            reset();
        }

    bool max_output = false; //���ֿ�����

    /**
     * @brief ����PID���
     * @param target Ŀ��ֵ
     * @param feedback ��ǰ����ֵ
     * @return PID�������
     */
    float pid_calc(float target, float feedback);
    

    /**
     * @brief ����PID���������ڲ�״̬
     *        ��Ҫ�������û��������ʷֵ��
     */
    void reset()
    {
        I_Term = 0.0f;
        P_Term = 0.0f;
        D_Term = 0.0f;
        output_ = 0.0f;
        error_last_ = 0.0f;
        feedback_last_ = 0.0f;
        isFirst_ = true; // ���� isFirst_ ��־
    }

    /**
     * @brief ��̬�޸�PID����
     * @param params �µ�PID����
     */
    void set_params(const PID_Param_Config& params, float I_SeparaThreshold);

    /**
     * @brief ��PID�趨Ϊѭ��ģʽ
     * @param range ѭ����Χ������360��
     * @brief ����Ϊѭ��ģʽ��ʱ��PID���Զ�ѡ�������·��������
     *        350�ȵ�10�ȣ���������20�ȣ������ǵ�����340��
     * @param offset ƫ������Ĭ��Ϊ0
     */
    void set_as_circular()
    {
        is_circular_ = true;
    }

    /**
     * @brief ��PID����Ϊ����ģʽ
     */
    void set_as_linear()
    {
        is_circular_ = false;
    }

    float get_P_Term() const { return P_Term; }
    float get_I_Term() const { return I_Term; }
    float get_D_Term() const { return D_Term; }
    float get_dt() const { return dt_; }

    void reset_dt_error(float set)
    {
        dt_error_ = set;
    }
private:
    float I_Term = 0;			/* ��������� */
    float P_Term = 0;			/* ��������� */
    float D_Term = 0;			/* ΢������� */
    float output_ = 0.0f;     // ���ֵ
    PID_Param_Config params_;
    float I_SeparaThreshold_;

    float error_ = 0.0f;              // ��ǰ���
    float error_last_ = 0.0f;       // �ϴ����
    float feedback_last_ = 0.0f;    // �ϴη���ֵ

    float dt_ = 0.01f;             // ����ʱ�䣬��λ��
    float last_time_s_ = 0.0f;      // �ϴε��õ�ʱ�䣬��λ��
    bool isFirst_ = true; // �Ƿ�Ϊ��һ�μ���
    float dt_error_ = 0.01f; //dtĬ��ֵ
    // ѭ���趨
    bool is_circular_ = false;
};

// ����ʽ PID ������ ����΢�ָ�����
class PID_Incremental {
public:
    /** 
     * @brief ���캯��
     * @param params PID �������ýṹ��
     * @param td_ratio ΢�ָ������������ӣ���Χ0.0~1.0��0��ʾ������΢�ָ�����.td_ratioԽ�����Խ�졣
     *
    */
    PID_Incremental(PID_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, false, 0.0f, 0.0f}, float td_ratio = 0.0f)
        : params_(params), td_ratio_(td_ratio) 
    {
        reset();
    }

        /**
         * @brief ��������ʽPID���
         * @param target Ŀ��ֵ
         * @param feedback ��ǰ����ֵ
         * @return PID�������
         */
        float pid_calc(float target, float feedback);

        /**
         * @brief ����PID���������ڲ�״̬
         *        ��Ҫ�������û��������ʷֵ��
         */
        void reset()
        {
            error_ = 0.0f;
            error_last_ = 0.0f;
            error_earlier_ = 0.0f;
            output_ = 0.0f; // ���� output_
            output_last_ = 0.0f;
            td_v1_ = 0.0f;
            td_v2_ = 0.0f;
            isFirst_ = true; // ���� isFirst_ ��־
        }

        /**
         * @brief ��̬�޸�PID����
         * @param params �µ�PID����
         */
        void set_params(const PID_Param_Config& params, float td_ratio);

        float get_dt() const { return dt_; }
        
private:
    void calc_track_D(float expect, float dt); //΢�ָ�����
    bool isFirst_ = true; // �Ƿ�Ϊ��һ�μ���

    float error_ = 0.0f, error_last_ = 0.0f, error_earlier_ = 0.0f; // ��ǰ���ϴ������ϴ����
    float output_ = 0.0f;       // ��ǰ�����
    float output_last_ = 0.0f;  // �ϴ������

    PID_Param_Config params_;

    float I_Term = 0.0f; // ���������
    float P_Term = 0.0f; // ���������
    float D_Term = 0.0f; // ΢�������

    float td_ratio_ = 0.0f; // track_D��������

    float td_v1_ = 0.0f; //���ٵ�Ŀ��λ��
    float td_v2_ = 0.0f; //���ٵ�Ŀ���ٶ�

    float dt_ = 0.001f;             // ����ʱ�䣬��λ��
    float last_time_s_ = 0.0f;      // �ϴε��õ�ʱ�䣬��λ��
};


extern PID_Param_Config m3508_speed_pid_params;
extern PID_Param_Config m3508_angle_pid_params;

extern PID_Param_Config m2006_speed_pid_params;
extern PID_Param_Config m2006_angle_pid_params;

extern PID_Param_Config track_pid_params;
#endif

#endif