/**
 * @file Arm_setup.h
 * @author XieFField  
 * @brief 串联臂运动控制实现
 *        KFS索引采用1 ~ 12 使用时候 index = KFSNum -1
 * @version 1.0
 *  测试基础的串联臂运动控制
 * @version 2.0
 *  开始写自动拾取相关
 *      依旧屎山堆叠 O(∩_∩)O 经典梦到哪句写哪句 
 *      酣畅淋漓的。。。屎山堆积，以后多用数组以及、、、硬编码:>
 * 
 * @version 3.0
 *   基本完善了云台部分的自动控制，等罗麒麟完善后面部分
 * 
 * @version 4.0
 *   写完了自动控制部分，目前把发现的Bug都修复了
 * 
 * @version 5.0
 *   重新设计了上电校准逻辑，以及后续的机械臂云台禁区位置。
 *   将会修改为，机械臂云台常驻safe高度为0.20米；
 *              如果机械臂云台在safe高度以下，则机械臂云台的活动范围仅为60度~135度(rotate_angle, 不是motor_angle)
 *              
 *              包括在自动模式下，state_toTargetHight阶段也会遵守该规则，即便拾取高度低于0.20米
 *              也要等到云台转到禁区外再下降
 * 
 * @version 6.0 更新单圈模式的策划方案
 *   构造planB[单圈模式]的自动拾取，即不多圈旋转；执行过程中的最大rotate角度为270度(abs(起点-终点) <= 270)，意味着云台禁止多圈旋转，旋转3/4圈后
 *   需要转回来，(防止电机电线缠绕云台底座)，累计走过角度位移[含正负计算，从起点(一般是重定位的位置)开始]，也应当应用在手操当中。
 *   若起点是0度，则是0->90(state_Align)[最短路径]->270(carrying)[继承state_Align旋转方向]->[0/180 云台需要走和state_carrying相反的方向)(state_return)
 * 
 *   若起点是180度，则是180->90(state_Align)[最短路径]->270(carrying)[继承state_Align旋转方向]->[0/180 云台需要走和state_carring相反的方向]
 *   且在state_return阶段，需要将云台升高到最高高度（0.4m）【和云台旋转到目标位置同时进行】
 *   在思考这个能不能做成通用接口。
 *   
 *   使用模式设置void setRotateMultiTurn(bool isMulti)，设定云台多圈以及单圈模式
 *   一旦设置云台的单圈和多圈，全局适用
 * 
 * @version 7.0
 *   就version 6.0的基础上，从原本的only_one模式，扩展到two模式
 *   在two模式下，机械臂会依次拾取两个KFS
 *   1. 执行和only_one模式一样的流程，拾取第一个KFS
 *   2. 在拾取第一个KFS的state_return阶段，机械臂会前往第二个KFS的初始位置(0/180度)，并且升高到安全高度(0.2m)
 *   3. 然后进入第二个KFS的拾取流程
 *   4. 第二个KFS的拾取流程和第一个类似，拾取完成后state_return到初始位置(0度)，结束。
 * 
 *  @version 8.0    
 *    对auto模式下面的auot_onlyOne进行策略修改
 *    删去carrying阶段，改为aim_ext执行完，即吸取到KFS后，直接return回初始位置
 *    但为了云台旋转不会打到KFS，所以在auto_onlyone中，return阶段在云台旋转回初始位置的同时，升到最高高度
 *    (同样需要遵守安全高度下的安全角度限制)
 *    且return到的位置不固定为0度，而是和起始位置相反，若起始是0，则return到180度，若起始是180度，则return到0度
 *    (本质是和行进方向相反)，而且在return阶段的云台旋转策略跟随sign_align阶段的旋转策略。
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

#define ARM_AUTO_DEBUG_NOCHASSIS  0 //無底盤下，用虛擬坐標進行驗證自動邏輯
#define ARM_AUTOMOVE 0 //0:停下拾取KFS，1:行进间拾取KFS



typedef struct{
    bool init_flag = false;
    uint8_t debug_start = 0; //调试开始标志 == 1 开始调试

    uint8_t auto_start = 0; //自动调试开始标志 == 1 开始自动调试

    float calibrate_startTime = 0; 
    bool calibrate_start = false;
    bool is_calibrating = false;

    float last_right_x = 0.0f; //上次右摇杆横向数据
    float last_right_y = 0.0f; //上次右摇杆纵向数据

    int8_t last_manual_extend = 0; //上次手动伸展状态
    int8_t last_manual_sucker = 0; //上次手动吸盘状态

    int8_t last_manual_pitch = 0; //上次手动俯仰状态

    int8_t pitch_switch_offset = 0; //俯仰开关偏移绑定
    int8_t extend_switch_offset = 0; // 伸展开关偏移绑定
    int8_t sucker_switch_offset = 0; // 吸盘开关偏移绑定
}arm_ctrl_status_S;


typedef enum{
    STATE_TO_WAIT,
    STATE_ALIGN,
    STATE_LOWER,
    STATE_EXT,
    STATE_LAUNCH,
    STATE_BACK,
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

    Point2D now_armPosition = {5.0f, 8.60f, 0.0f}; //机械臂当前位置

    Point2D now_ChassisPosition = {5.0f, 8.60f, 0.0f}; //底盘当前位置

    Point2D now_chassis_speed = {0.0f, 0.0f, 0.0f}; //当前底盘速度，单位米每秒

    Point2D targetKFS_pos[2] = {{0.0f,0.0f,0.0f}, {0.0f,0.0f,0.0f}}; //目标KFS位置

    struct {
        Point2D PA;
        Point2D PB;
    }PointPAB[2];

    /**
     * @brief 旋转路径策略 正方向表示角度正增，负方向表示角度负增；
     *                    正增为逆时针旋转，负增为顺时针旋转
     */
    Rotate_Strategy_E current_strategy = ROTATE_PATH_SHORTEST; 
    ARM_AUTO_STILLNESS_E now_state = STATE_DONE;
    MF_AutoCtrler::PathInformation_S pathInfo; 
    struct{
        bool isrecalcPath = false; //是否重新计算路径
        bool canExtend = false; //是否可以伸展
        float reach_finishTimeStore = 0.0f; //存储到达目标位置的时间戳
        bool isExtReach = false;
        bool canChassisStart = false; //是否可以开始底盘移动

        bool isbackdone = false; //返回完成标志
        float back_time = 0.0f; //返回时间
        int8_t pitch_state[2] = {0,0}; //记录目标KFS的pitch状态，0代表侧吸，1代表顶吸
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

        // this->setPitchReversed(true); //俯仰电机反向
        this->setStretchReversed(true); //伸展电机反向
        this->setRotateReversed(false);
        this->setLaunchReversed(true); //升降电机反向
        start(osPriorityHigh-1, 512); 
        setRotateMultiTurn(false); //单圈模式
        arm_ctrlStatus.init_flag = true;
    }

    void setArmStatus(ARM_Status_E status)
    {
        // 未完成校准时，只允许保持在校准态，避免被上层状态机提前切到手操/空闲
        if(status != ARM_CALIBRATE && !isArmcalibrated())
            return;

        arm_status_ = status;
    }

    /**
     * @brief 设置云台多圈/单圈模式
     *        默认单圈模式，即rotate_multiTurn_ = false
     *        单圈模式意味着云台禁止多圈旋转，旋转3/4圈后需要转回来，(防止电机电线缠绕云台底座)
     */
    void setRotateMultiTurn(bool isMulti)
    {
        rotate_multiTurn_ = isMulti;
    }



    /**
     * @brief 设置目标抓取梅花桩编号
     * @param KFS1 第一个KFS，范围0~12
     * @param KFS2 第二个KFS，范围0~12
     * @brief 0代表没有要拾取的
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
            return false; //没有目标KFS，设置失败
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
            auto_ctrl_.flag.pitch_state[0] = 0; //高的KFS保持吸盘水平，侧吸
        else
            auto_ctrl_.flag.pitch_state[0] = 1; //低的KFS吸盘竖直向下，顶吸

        if(MF_high[auto_ctrl_.targetKFS[1]-1] == 0.6f)
            auto_ctrl_.flag.pitch_state[1] = 0; //高的KFS保持吸盘水平，侧吸
        else
            
            auto_ctrl_.flag.pitch_state[1] = 1; //低的KFS吸盘竖直向下，顶吸


#if ARM_AUTO_DEBUG_NOCHASSIS
        auto_ctrl_.now_ChassisPosition.x = MF_AutoCtrler::MapNum_RealPos[temp.MFroad[0] - 1].x;
        auto_ctrl_.now_ChassisPosition.y = MF_AutoCtrler::MapNum_RealPos[temp.MFroad[0] - 1].y - 2.0f;
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
     * @brief 在停下拾取自动模式下，由主状态机调用，
     *        设置是否可以进入伸展阶段
     */
    void setAutocanExtend(bool canExtend)
    {
        auto_ctrl_.flag.canExtend = canExtend;
    }

    /**
     * @brief 在停下拾取自动模式下，由主状态机调用，
     *       返回是否可以进入底盘移动阶段
     */
    bool isAutoChassisCanStart()
    {
        return auto_ctrl_.flag.canChassisStart;
    }
private:

    void start_toAutoCtrl(bool start)
    {
        if(start)
            auto_ctrl_.start_to_autoctrl = true;

        else
            auto_ctrl_.start_to_autoctrl = false;
    }

    RmPocketData_t airjoy_data_; //摇杆值为 -1 ~ 1

    Debug_Printf debug_uart = Debug_Printf(&huart8);

    //控制函数
    void manualControl();
    void autoControl();
    void stop();
    void idle();
    void debug();

    //上电校准M2006电机位置
    void calibrateMotor();

    //=======================
    //自动停下拾取私密函数[停下拾取]

    void auto_stillnessOne();
    void auto_stillnessTwo();

    bool state_to_waitStillness(int targetKFS);
    bool state_alignStillness(int targetKFS);
    bool state_lowerStillness(int targetKFS);
    bool state_extStillness(int targetKFS);
    bool state_launchStillness(int targetKFS);
    bool state_backStillness(int targetKFS);
    // bool state_doneStillness(int targetKFS);

    //=======================
    /**
     * @brief 安全禁区通用接口：根据当前云台高度约束旋转角度
     * 规则：
     * 1. H < 0.03m: [60°, 185°] (重定位/极低高度区间)
     * 2. 0.03m <= H < Safe_H: [60°, 185°] (机械限位干涉区间)
     * 3. H >= Safe_H: [0°, 360°] (安全高度)
     * 说明：传入/返回的角度均为 rotate_angle（云台角度，非电机角度）
     */
    bool isRotateAllowed(float rotate_angle_deg) const
    {
        const float h = this->get_currentJointStatus().launchJoint_Height_;
        const float safe_h = init_data_.safe_height;
        
        // 归一化到 0-360
        float norm_deg = fmodf(rotate_angle_deg, 360.0f);
        if(norm_deg < 0.0f) norm_deg += 360.0f;

        if(h < 0.03f)
        {
            return (norm_deg >= 60.0f && norm_deg <= 185.0f);
        }
        else if(h < safe_h - 0.01f)
        {
            return (norm_deg >= 60.0f && norm_deg <= 185.0f);
        }
        return true;
    }

    /**
     * @brief 返回符合安全禁区的角度：
     * - H < 0.03m: 钳制到 [60°, 185°]
     * - 0.03m <= H < Safe_H: 钳制到 [60°, 185°]
     * - H >= Safe_H: 保持原角度
     * @param desired_deg 期望的旋转角度（云台角度，非电机角度）
     * @return 符合安全禁区的旋转角度
     */
    float sanitizeRotateAngle(float desired_deg) const
    {
        const float h = this->get_currentJointStatus().launchJoint_Height_;
        const float safe_h = init_data_.safe_height;
        
        if(h < safe_h - 0.01f)
        {
            // 归一化到 0-360
            float norm_deg = fmodf(desired_deg, 360.0f);
            if(norm_deg < 0.0f) norm_deg += 360.0f;

            if(h < 0.03f)
            {
                if(norm_deg < 60.0f ) return 60.0f;
                if(norm_deg > 185.0f && norm_deg < 270.0f) return 180.0f;
                if(norm_deg >= 270.0f) return 60.0f;
                return norm_deg;
            }
            else
            {
                if(norm_deg < 60.0f) return 60.0f;
                if(norm_deg > 185.0f && norm_deg < 270.0f) return 180.0f; // 185~270区间钳制到180
                if(norm_deg >= 270.0f) return 60.0f; // 270~360(即-90~0)区间钳制到60
                return norm_deg;
            }
        }
        return desired_deg;
    }

    /**
     * @brief 执行安全门：在任何位置控制前调用，返回是否允许并输出安全角度
     * @param desired_deg 期望的旋转角度（云台角度，非电机角度）
     * @param safe_out_deg 输出的安全旋转角度
     */
    bool safetyGate_ForRotate(float desired_deg, float& safe_out_deg) const
    {
        bool ok = isRotateAllowed(desired_deg);
        safe_out_deg = sanitizeRotateAngle(desired_deg);
        return ok;
    }

protected:

    

    /**
     * @brief 获得机械臂底座原点位置(也为云天中心位置)
     * @details 待实现，现在留一个空接口，方便先完成自动逻辑的实现 
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
     * @brief 预留接口后续补全，获得当前底盘速度
     * @return 
     */
    Point2D get_nowChassisSpeed()
    {
        #if ARM_AUTO_DEBUG_NOCHASSIS

        Point2D speed = {0.0f, 0.0f, 0.0f};
        if(arm_ctrlStatus.auto_start == 1)
           speed = {0.0f, 1.0f, 0.0f};

        else
             speed = {0.0f, 0.0f, 0.0f};
        return speed;
        
       if(MF_AutoCtrler::isInTargetMap(auto_ctrl_.now_ChassisPosition,
                                           auto_ctrl_.pathInfo.MFroad[auto_ctrl_.now_targetIndex],
                                           0.03f))
       {
            if(auto_ctrl_.flag.canChassisStart == 1)
                speed = {0.0f, 1.0f, 0.0f};
            else
                speed = {0.0f, 0.0f, 0.0f};
       }
             
             
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
     * @brief 获得底盘位姿
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

    bool rotate_multiTurn_ = false; //云台多圈模式标志，默认单圈模式

    ARM_AUTO_S auto_ctrl_;

    // 单圈模式相关变量
    Rotate_Strategy_E recorded_align_strategy_ = ROTATE_PATH_SHORTEST;
    Rotate_Strategy_E recorded_carrying_strategy_ = ROTATE_PATH_SHORTEST;

    struct {
        
        float rotate_rate = 3.0f;
        float launch_rate = 0.03f;
        int cnt = 0;
    }manual_control;
};


extern Arm_InitData_S arm_initData;

#endif //__cplusplus


#endif // __ARM_SETUP_H

