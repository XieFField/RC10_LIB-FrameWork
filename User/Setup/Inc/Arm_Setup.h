/**
 * @file Arm_setup.h
 * @author XieFField  
 * @brief �������˶�����ʵ��
 *        KFS��������1 ~ 12 ʹ��ʱ�� index = KFSNum -1
 * @version 1.0
 *  ���Ի����Ĵ������˶�����
 * @version 2.0
 *  ��ʼд�Զ�ʰȡ���?
 *      ����ʺɽ�ѵ� O(��_��)O �����ε��ľ�д�ľ� 
 *      ��������ġ�����ʺɽ�ѻ����Ժ���������Լ�������Ӳ����:>
 * 
 * @version 3.0
 *   ������������̨���ֵ��Զ����ƣ������������ƺ��沿��
 * 
 * @version 4.0
 *   д�����Զ����Ʋ��֣�Ŀǰ�ѷ��ֵ�Bug���޸���
 * 
 * @version 5.0
 *   ����������ϵ�У׼�߼����Լ������Ļ�е�����?����λ�á�
 *   �����޸�Ϊ����е����̨��פsafe�߶�Ϊ0.20�ף�
 *              �����е�����?��safe�߶����£����е�����?�Ļ��Χ��Ϊ60��~135��(rotate_angle, ����motor_angle)
 *              
 *              �������Զ�ģʽ�£�state_toTargetHight�׶�Ҳ�����ظù��򣬼���ʰȡ�߶ȵ���0.20��
 *              ҲҪ�ȵ���̨ת�����������½�
 * 
 * @version 6.0 ���µ�Ȧģʽ�Ĳ߻�����
 *   ����planB[��Ȧģʽ]���Զ�ʰȡ��������Ȧ��ת��ִ�й����е����rotate�Ƕ�Ϊ270��(abs(���?-�յ�) <= 270)����ζ����̨��ֹ��Ȧ��ת����ת3/4Ȧ��
 *   ��Ҫת������(��ֹ������߲������?����)���ۼ��߹��Ƕ�λ��[���������㣬�����?(һ�����ض�λ��λ��)��ʼ]��ҲӦ��Ӧ�����ֲٵ��С�
 *   �������?0�ȣ�����0->90(state_Align)[���·��]->270(carrying)[�̳�state_Align��ת����]->[0/180 ��̨��Ҫ�ߺ�state_carrying�෴�ķ���)(state_return)
 * 
 *   �������?180�ȣ�����180->90(state_Align)[���·��]->270(carrying)[�̳�state_Align��ת����]->[0/180 ��̨��Ҫ�ߺ�state_carring�෴�ķ���]
 *   ����state_return�׶Σ���Ҫ����̨���ߵ���߸߶ȣ�?0.4m��������̨��ת��Ŀ��λ��ͬʱ���С�
 *   ��˼������ܲ��������?�ýӿڡ�
 *   
 *   ʹ��ģʽ����void setRotateMultiTurn(bool isMulti)���趨��̨��Ȧ�Լ���Ȧģʽ
 *   һ��������̨�ĵ�Ȧ�Ͷ�Ȧ��ȫ������
 * 
 * @version 7.0
 *   ��version 6.0�Ļ����ϣ���ԭ����only_oneģʽ����չ��twoģʽ
 *   ��twoģʽ�£���е�ۻ�����ʰȡ����KFS
 *   1. ִ�к�only_oneģʽһ�������̣�ʰȡ��һ��KFS
 *   2. ��ʰȡ��һ��KFS��state_return�׶Σ���е�ۻ�ǰ���ڶ���KFS�ĳ�ʼλ��(0/180��)���������ߵ���ȫ�߶�(0.2m)
 *   3. Ȼ�����ڶ���KFS��ʰȡ����
 *   4. �ڶ���KFS��ʰȡ���̺͵�һ�����ƣ�ʰȡ��ɺ�state_return����ʼλ��(0��)��������
 * 
 *  @version 8.0    
 *    ��autoģʽ�����auot_onlyOne���в����޸�
 *    ɾȥcarrying�׶Σ���Ϊaim_extִ���꣬����ȡ��KFS��ֱ��return�س�ʼλ��
 *    ��Ϊ����̨��ת�����KFS��������auto_onlyone�У�return�׶�����̨��ת�س�ʼλ�õ�ͬʱ��������߸߶�?
 *    (ͬ����Ҫ���ذ�ȫ�߶��µİ�ȫ�Ƕ�����)
 *    ��return����λ�ò��̶�Ϊ0�ȣ����Ǻ���ʼλ���෴������ʼ��0����return��180�ȣ�����ʼ��180�ȣ���return��0��
 *    (�����Ǻ��н������෴)��������return�׶ε���̨��ת���Ը���sign_align�׶ε���ת���ԡ�
 * 
 *  @version 9.0  ����������һ��
 *  �°��е�۵�����?  ���ϰ������в��ٲ�ͬ����Ҫ��д
    (1)���Ƕ����� 
        ִ��state_to_waitStillnesş����ߣ�����pitch����Ϊ90��
        ����ִ��state_alignStillness���� �ӽ�֮��ִ��state_extStillness�쳤��Ŀ��KFSλ��
        Ȼ��ִ��state_lowerStillness���͵�Ŀ��KFSλ�ã��������̡�(Lower�׶ν����ٽ�߶Ⱥ�ͣ�£��ȴ�canExtend�������½���Ŀ��λ��)
        ֮��ִ��state_launchStillnesş������ȫ�߶ȣ����ִ��state_backStillness���س�ʼλ�á�


    (2)���ǲ�����
        ִ��state_to_waitStillnesş����ߣ�����pitch����Ϊ0��
        ����ִ��state_alignStillness���� �ӽ�֮��ִ��state_lowerStillness���͵�Ŀ��KFSλ�ã��������̡�
        ֮��ִ��state_extStillness�쳤����ȫλ�ã����ִ��state_backStillness���س�ʼλ�á�
 */

#ifndef __ARM_SETUP_H
#define __ARM_SETUP_H

#ifdef __cplusplus
extern "C" {
#include "stdint.h"
}
#endif



#ifdef __cplusplus
#include "BSP_RTOS.h"
#include "Robot_Arm.h"
#include "APP_Tool.h"
#include "Module_Air_joy.h"
#include "Motor_DJI.h"
#include "BSP_TimeStamp.h"
#include "APP_debugTool.h"
#include "FSMstauts_enum.h"
#include "APP_CoordConvert.h"
#include "AutoCtrler.h"
#include "Module_CrsfReceiver.h"
#include "Locate_Setup.h"

// #include "usart.h"

#define ARM_AUTO_DEBUG_NOCHASSIS  0 //�o�ױP�£���̓�M�����M����C�Ԅ�߉݋
#define ARM_VERSION 0 //�汾�ţ� ����1����ζ���Ƕ��������ںϰ汾 �����?0���Ǵ������汾


typedef struct{
    bool init_flag = false;
    uint8_t debug_start = 0; //���Կ�ʼ��־ == 1 ��ʼ����

    uint8_t auto_start = 0; //�Զ����Կ�ʼ��־ == 1 ��ʼ�Զ�����

    float calibrate_startTime = 0; 
    bool calibrate_start = false;
    bool is_calibrating = false;

    float last_right_x = 0.0f; //�ϴ���ҡ�˺�������
    float last_right_y = 0.0f; //�ϴ���ҡ����������

    int8_t last_manual_extend = 0; //�ϴ��ֶ���չ״̬
    int8_t last_manual_sucker = 0; //�ϴ��ֶ�����״̬

    int8_t last_manual_pitch = 0; //�ϴ��ֶ�����״̬

    int8_t pitch_switch_offset = 0; //��������ƫ�ư�
    int8_t extend_switch_offset = 0; // ��չ����ƫ�ư�
    int8_t sucker_switch_offset = 0; // ���̿���ƫ�ư�

    uint8_t button_click_state = 0;
    uint8_t is_store_acting = 0; //����ִ�д洢�����ı�־λ

    
    uint8_t last_manual_store = 0; //�����ֲٺʹ洢�л����ж�
}arm_ctrl_status_S;


typedef enum{
    STATE_TO_WAIT,
    STATE_ALIGN,
    STATE_LOWER,
    STATE_EXT,
    STATE_LAUNCH,
    STATE_BACK,
    STATE_STORE,
    STATE_DONE,
    STATE_OVER,
}ARM_AUTO_STILLNESS_E;

typedef enum{
    ONLY_ONE,
    TWO,
    NONE_KFS,
}KFS_NUM_E;

typedef struct{
    int targetKFS[3] = {0,0,0};
    int now_targetIndex = 0;
    KFS_NUM_E kfs_num = ONLY_ONE;
    bool start_to_autoctrl = false;

    Point2D now_armPosition = {5.0f, 8.60f, 0.0f}; //��е�۵�ǰλ��

    Point2D now_ChassisPosition = {5.0f, 8.60f, 0.0f}; //���̵�ǰλ��

    Point2D now_chassis_speed = {0.0f, 0.0f, 0.0f}; //��ǰ�����ٶȣ���λ��ÿ��

    Point2D targetKFS_pos[2] = {{0.0f,0.0f,0.0f}, {0.0f,0.0f,0.0f}}; //Ŀ��KFSλ��

    /**
     * @brief ��ת·������ �������ʾ�Ƕ��������������ʾ�Ƕȸ�����
     *                    ����Ϊ��ʱ����ת������Ϊ˳ʱ����ת
     */
    Rotate_Strategy_E current_strategy = ROTATE_PATH_SHORTEST; 
    ARM_AUTO_STILLNESS_E now_state = STATE_DONE;
    MF_AutoCtrler::PathInformation_S pathInfo; 
    struct{
        bool isrecalcPath = false; //�Ƿ����¼���·��
        bool canExtend = false; //�Ƿ�������?
        float reach_finishTimeStore = 0.0f; //�洢����Ŀ��λ�õ�ʱ���?
        bool isExtReach = false;
        bool canChassisStart = false; //�Ƿ���Կ�ʼ�����ƶ�?

        bool isbackdone = false; //������ɱ��?
        float back_time = 0.0f; //����ʱ��
        int8_t pitch_state[2] = {0,0}; //��¼Ŀ��KFS��pitch״̬��0����������1��������
    }flag;
}ARM_AUTO_S;


const float MF_high[12] = 
{
    0.4f, 0.2f, 0.4f,
    0.2f, 0.4f, 0.6f,
    0.4f, 0.6f, 0.4f,
    0.2f, 0.4f, 0.2f
};

class ArmSetup: public RtosTask ,public Robot_Arm {
public:
    ArmSetup(Arm_InitData_S init_Data)
        : Robot_Arm(init_Data), RtosTask("ArmSetup", 1) 
    {
    }

    bool isArmcalibrated() const
    {
        if(arm_ctrlStatus.is_calibrating)
            return true;
        else
            return false;
    }

    void init(M3508 *motor_ArmLaunch, M2006 *motor_ArmStretch, 
        M3508 *motor_ArmRotate, DM_Motor *motor_ArmPitch)
    {
        this->registerMotor_Launch(motor_ArmLaunch);
        this->registerMotor_Stretch(motor_ArmStretch);
        this->registerMotor_Rotate(motor_ArmRotate);
        this->registerMotor_Pitch(motor_ArmPitch);

        // this->setPitchReversed(true); //�����������?
        this->setStretchReversed(true); //��չ�������?
        this->setRotateReversed(false);
        this->setLaunchReversed(true); //�����������?
        start(osPriorityHigh-1, 512); 
        arm_ctrlStatus.init_flag = true;
    }

    void setArmStatus(ARM_Status_E status)
    {
        // δ���У׼ʱ��ֻ����������У׼�?�����ⱻ�ϲ�״̬����ǰ�е��ֲ�/����
        if(status != ARM_CALIBRATE && !isArmcalibrated())
            return;

        arm_status_ = status;
    }

    /**
     * @brief ����Ŀ��ץȡ÷��׮���?
     * @param KFS1 ��һ��KFS����Χ0~12
     * @param KFS2 �ڶ���KFS����Χ0~12
     * @brief 0����û��Ҫʰȡ��
     */
    bool set_TargetKFS(int KFS1, int KFS2)
    {
        if(KFS1 >=0 && KFS1 <=12)
            auto_ctrl_.targetKFS[0] = KFS1;
        else
            auto_ctrl_.targetKFS[0] = 0;
        if(KFS2 >=0 && KFS2 <=12)
            auto_ctrl_.targetKFS[1] = KFS2;
        else
            auto_ctrl_.targetKFS[1] = 0;
        
        if(KFS1 != 0 && KFS2 !=0)
            auto_ctrl_.kfs_num = TWO;
        else
            auto_ctrl_.kfs_num = ONLY_ONE;

        if(KFS1 == 0 && KFS2 == 0)
            return false; //û��Ŀ��KFS������ʧ��
        else
        {
            auto_ctrl_.targetKFS_pos[0] = MF_AutoCtrler::MapNum_RealPos[MF_AutoCtrler::MFNum_TransforMapNum(auto_ctrl_.targetKFS[0])-1];
            if(KFS2 != 0)
            {
                auto_ctrl_.targetKFS_pos[1] = MF_AutoCtrler::MapNum_RealPos[MF_AutoCtrler::MFNum_TransforMapNum(auto_ctrl_.targetKFS[1])-1];
            }
            else
            {
                auto_ctrl_.targetKFS_pos[1] = {0.0f, 0.0f, 0.0f};
            }
        }

        MF_AutoCtrler::PathInformation_S temp = MF_AutoCtrler::PathInformation_calc(auto_ctrl_.now_ChassisPosition,
                                       auto_ctrl_.targetKFS[0], 
                                        auto_ctrl_.targetKFS[1]);
        auto_ctrl_.pathInfo.entranceMap = temp.entranceMap;
        
        for(int i=0; i<2; i++)
        {
            auto_ctrl_.pathInfo.MFroad[i] = temp.MFroad[i];
        }

        for(int i=0; i<12; i++)
        {
            auto_ctrl_.pathInfo.mustPastMap[i] = temp.mustPastMap[i];
        }

        for(int i=0; i<2; i++)
        {
            auto_ctrl_.pathInfo.Index_MFroad[i] = temp.Index_MFroad[i];
        }

        if(MF_high[auto_ctrl_.targetKFS[0]-1] == 0.6f)
            auto_ctrl_.flag.pitch_state[0] = 0; //�ߵ�KFS��������ˮƽ������
        else
            auto_ctrl_.flag.pitch_state[0] = 1; //�͵�KFS������ֱ���£�����

        if(MF_high[auto_ctrl_.targetKFS[1]-1] == 0.6f)
            auto_ctrl_.flag.pitch_state[1] = 0; //�ߵ�KFS��������ˮƽ������
        else
            
            auto_ctrl_.flag.pitch_state[1] = 1; //�͵�KFS������ֱ���£�����


#if ARM_AUTO_DEBUG_NOCHASSIS
        auto_ctrl_.now_ChassisPosition.x = MF_AutoCtrler::MapNum_RealPos[temp.MFroad[0]-1].x;
        auto_ctrl_.now_ChassisPosition.y = MF_AutoCtrler::MapNum_RealPos[temp.MFroad[0]-1].y - 3.0f;
#endif
        return true;
    }

    void set_Arm_autoStart(uint8_t start)
    {
        if(start == 1)
            arm_ctrlStatus.auto_start = 1;
        else    
            arm_ctrlStatus.auto_start = 0;
    }

    bool isArmAutoStart() const
    {
        return auto_ctrl_.start_to_autoctrl;
    }

    /**
     * @brief ��ͣ��ʰȡ�Զ�ģʽ�£�����״̬�����ã�
     *        �����Ƿ���Խ�����չ�׶�?
     */
    void setAutocanExtend(bool canExtend)
    {
        auto_ctrl_.flag.canExtend = canExtend;
    }

    /**
     * @brief ��ͣ��ʰȡ�Զ�ģʽ�£�����״̬�����ã�
     *       �����Ƿ���Խ�������ƶ��׶�
     */
    bool isAutoChassisCanStart()
    {
        return auto_ctrl_.flag.canChassisStart;
    }

    
    enum class store_state{
        idle,
        laucnh_state,
        rotate_state,
        lower_state,
        outstate1, //ȡ��ר��
        outstate2,
    };
private:

    void start_toAutoCtrl(bool start)
    {
        if(start)
            auto_ctrl_.start_to_autoctrl = true;

        else
            auto_ctrl_.start_to_autoctrl = false;
    }

    RmPocketData_t airjoy_data_; //ҡ��ֵΪ -1 ~ 1

    Debug_Printf debug_uart = Debug_Printf(&huart8);

    //���ƺ���
    void manualControl();
    bool manual_store();
    bool manual_takeout();

    void autoControl();
    void stop();
    void idle();
    void debug();

    //�ϵ�У׼M2006���λ��?
    void calibrateMotor();

    //=======================
    //�Զ�ͣ��ʰȡ˽�ܺ���[ͣ��ʰȡ]

    void auto_stillnessOne();
    void auto_stillnessTwo();

    bool state_to_waitStillness(int targetKFS);
    bool state_alignStillness(int targetKFS);
    bool state_lowerStillness(int targetKFS);
    bool state_extStillness(int targetKFS);
    bool state_launchStillness(int targetKFS);
    bool state_backStillness(int targetKFS);

    //=======================
    /**
     * @brief ��ȫ����ͨ�ýӿڣ����ݵ�ǰ��̨�߶�Լ����ת�Ƕ�
     * ����
     * 1. H < 0.03m: [60��, 185��] (�ض�λ/���͸߶�����)
     * 2. 0.03m <= H < Safe_H: [60��, 185��] (��е��λ��������)
     * 3. H >= Safe_H: [0��, 360��] (��ȫ�߶�)
     * ˵��������/���صĽǶȾ�Ϊ rotate_angle����̨�Ƕȣ��ǵ���Ƕȣ�?
     */
    bool isRotateAllowed(float rotate_angle_deg) const
    {
        const float h = this->get_currentJointStatus().launchJoint_Height_;
        const float safe_h = init_data_.safe_height_;

        // ���ظ�������һ��
        const float norm_deg = rotate_angle_deg;

        if(h < 0.03f) return (norm_deg >= 60.0f && norm_deg <= 185.0f);
        if(h < safe_h - 0.01f) return (norm_deg >= 60.0f && norm_deg <= 185.0f);
        return true;
    }

    /**
     * @brief ���ط��ϰ�ȫ�����ĽǶȣ�
     * - H < 0.03m: ǯ�Ƶ� [60��, 185��]
     * - 0.03m <= H < Safe_H: ǯ�Ƶ� [60��, 185��]
     * - H >= Safe_H: ����ԭ�Ƕ�
     * @param desired_deg ��������ת�Ƕȣ���̨�Ƕȣ��ǵ���Ƕȣ�?
     * @return ���ϰ�ȫ��������ת�Ƕ�
     */
    float sanitizeRotateAngle(float desired_deg) const
    {
        const float h = this->get_currentJointStatus().launchJoint_Height_;
        const float safe_h = init_data_.safe_height_;

        if(h >= safe_h - 0.01f) return desired_deg;

        // ���ظ�������һ��
        const float norm_deg = desired_deg;

        if(norm_deg < 60.0f) return 60.0f;
        if(norm_deg > 179.9f && norm_deg < 270.0f) return 180.0f;
        if(norm_deg >= 270.0f) return 60.0f;
        return norm_deg;
    }


    
protected:

    /**
     * @brief ��û�е�۵����?��λ��(ҲΪ��������λ��)
     * @details ��ʵ�֣�������һ���սӿڣ�����������Զ��߼���ʵ��? 
     */
    Point2D get_nowArmPosition()
    {
        #if ARM_AUTO_DEBUG_NOCHASSIS
            return get_nowChassisPose();
        #else
            Locate_Setup *locate_ptr = Locate_Setup::getInstance();

            Point2D arm_pos;
            arm_pos = locate_ptr->get_ArmPos_inWorld();
            return arm_pos;
        #endif
    }
    /**
     * @brief Ԥ���ӿں�����ȫ����õ�ǰ�����ٶ�?
     * @return 
     */
    Point2D get_nowChassisSpeed()
    {
        #if ARM_AUTO_DEBUG_NOCHASSIS

        Point2D speed = {0.0f, 0.0f, 0.0f};
        if(arm_ctrlStatus.auto_start == 1)
        {
            bool inTargetMap = MF_AutoCtrler::isInTargetMap(auto_ctrl_.now_ChassisPosition,
                                                auto_ctrl_.pathInfo.MFroad[auto_ctrl_.now_targetIndex],
                                                0.1f);
            if(inTargetMap)
            {
                auto_ctrl_.flag.canExtend = true;
            }

            if(auto_ctrl_.flag.canChassisStart || !inTargetMap)
                speed = {0.0f, 1.0f, 0.0f};
            else
                speed = {0.0f, 0.0f, 0.0f};
        }
        return speed;
             
             
        #else

            // Locate_Setup *locate_ptr = Locate_Setup::getInstance();

            // return locate_ptr->get_FK_ChassisSpeed_inWorld();
            Locate_Setup *locate_ptr = Locate_Setup::getInstance();
            Point2D speed = {0};
            speed.x = locate_ptr->get_RobotSpeed_inWorld().x;
            speed.y = locate_ptr->get_RobotSpeed_inWorld().y;
            return speed;
        #endif
    }

    /**
     * @brief ��õ���λ��?
     */

    Point2D get_nowChassisPose()
    {

        #if ARM_AUTO_DEBUG_NOCHASSIS

        Point2D pose = auto_ctrl_.now_ChassisPosition;
        Point2D speed = get_nowChassisSpeed();


        pose.x += speed.x * get_dt();
                
        pose.y += speed.y * get_dt();                    

        return pose;

        #else

        Point2D pose = {0};

        Locate_Setup *locate_ptr = Locate_Setup::getInstance();
        pose.x = locate_ptr->get_RobotPos_inWorld().x;
        pose.y = locate_ptr->get_RobotPos_inWorld().y;
        pose.theta = locate_ptr->get_RobotPos_inWorld().yaw;

        return pose;
        #endif
    }

    void loop() override;

    arm_ctrl_status_S arm_ctrlStatus = {
        .init_flag = false,
        .debug_start = 1,
        .calibrate_startTime = 0,
        .calibrate_start = false,
        .is_calibrating = false,

    };


    ARM_Status_E arm_status_ = ARM_MANUAL_CONTROL;
    ARM_Status_E last_arm_status_ = ARM_MANUAL_CONTROL;

    Joint_Status_S last_joint_status_ = {0.0f, 0.0f, 0.0f, 0.0f};
    Joint_Status_S target_joint_status_ = {0.0f, 0.0f, 0.0f, 0.0f};

    ARM_AUTO_S auto_ctrl_;

    struct {
        
        float rotate_rate = 0.1f;
        float launch_rate = 0.03f;
        int cnt = 0;
    }manual_control;

    ButtonDetector button_detector_1 = ButtonDetector(0.200f); //��ť1�ĵ�˫���������?350ms˫���ж�ʱ��
    float rotate_accum_initial_motor_total_ = 0.0f;


    store_state store_state_ = store_state::idle; //�洢��ȡ����״̬����
    /**
     * @brief ��ȡ��ǰ�ۼ���ת�Ƕ�
     */
    float get_accuum_roatate_angle()
    {
        if(!arm_ctrlStatus.is_calibrating)
            return 0.0f;
        
        float delta = motor_rotate_->getTotalAngle() - this->rotate_accum_initial_motor_total_;
        return this->MotorTotalAngle_to_rotateAngle(delta);
    }

    /**
     * @brief �ֲٵ��У���ת�Ƕ����ƣ���ֹ��Ȧ����
     */
    float manual_roate_clamp(float desired_deg)
    {
        constexpr float MAX_ROTATE_ANGLE = 270.0f; //������?�Ƕȣ���λ��

        //���·��?
        float diff = desired_deg - this->get_currentJointStatus().rotateJoint_angle_;
        if (diff > 180.0f)       diff -= 360.0f;
        else if (diff < -180.0f) diff += 360.0f;

        float new_accum = this->get_accuum_roatate_angle() + diff;

        if(new_accum > MAX_ROTATE_ANGLE)
        {
            diff = MAX_ROTATE_ANGLE - this->get_accuum_roatate_angle();
        }
        else if(new_accum < -MAX_ROTATE_ANGLE)
        {
            diff = -MAX_ROTATE_ANGLE - this->get_accuum_roatate_angle();
        }

        float clamped_deg = this->get_currentJointStatus().rotateJoint_angle_ + diff;
        return normalize_deg_0_360(clamped_deg);
    }  

    /**
     * @brief �Զ�ģʽ�°�ȫ��ת��Ŀ��Ƕ�?
     * @param target_deg ��������̨��һ���Ƕȣ�0~360��
     * @note �ڲ��Զ�ѡ�񲻻ᳬ���ۼ� ��270�� ����ת���ԣ���ֱ�����õ��Ŀ��?
     * @param final_deg ���ʵ�����õİ��?��ת�Ƕȣ���̨�Ƕȣ��ǵ���Ƕȣ�?
      * @param strategy_used ���ʵ��ʹ�õ����?����
     */
    void safe_rotate_to(float target_deg)
    { 
        struct Option{
            Rotate_Strategy_E strategy;
            float diff;
            bool vaild;
        };

        float diff_short = target_deg - this->get_currentJointStatus().rotateJoint_angle_;
        if (diff_short > 180.0f)       diff_short -= 360.0f;
        else if (diff_short < -180.0f) diff_short += 360.0f;

        //����diff
        float diff_pos = target_deg - this->get_currentJointStatus().rotateJoint_angle_;
        if(diff_pos < 0 ) diff_pos += 360.0f;
        //����diff
        float diff_neg = target_deg - this->get_currentJointStatus().rotateJoint_angle_;
        if(diff_neg > 0 ) diff_neg -= 360.0f;

        float accum = get_accuum_roatate_angle();

        constexpr float LIMIT = 270.0f;

        Option options[3] = {
            {ROTATE_PATH_SHORTEST, diff_short, (accum + diff_short >= -LIMIT - 0.01f && accum + diff_short <= LIMIT + 0.01f)},
            {ROTATE_PATH_POSITIVE, diff_pos,   (accum + diff_pos   >= -LIMIT - 0.01f && accum + diff_pos   <= LIMIT + 0.01f)},
            {ROTATE_PATH_NEGATIVE, diff_neg,   (accum + diff_neg   >= -LIMIT - 0.01f && accum + diff_neg   <= LIMIT + 0.01f)}
        };

        Option* best_option = nullptr;
        float min_abs = 1e9f;

        for(auto&opt: options)
        {
            if(opt.vaild && std::fabs(opt.diff) < min_abs)
            {
                best_option = &opt;
                min_abs = _tool_Abs(opt.diff);
            }
        }

        this->setRotateStrategy(best_option->strategy);
        float final_deg = normalize_deg_0_360(this->get_currentJointStatus().rotateJoint_angle_ + best_option->diff);
        this->set_RotateAngle(final_deg);
    }
};


extern Arm_InitData_S arm_initData;

#endif //__cplusplus


#endif // __ARM_SETUP_H

