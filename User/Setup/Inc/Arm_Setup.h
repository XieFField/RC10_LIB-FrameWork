/**
 * @file Arm_setup.h
 * @author XieFField  
 * @brief 机械臂应用层控制逻辑
 *        KFS范围1 ~ 12  nowindex = KFSNum -1
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
#include "Module_lora.h"


// #include "usart.h"

#define ARM_AUTO_DEBUG_NOCHASSIS  0 //无底盘下的模拟调试开关 1开启 0关闭
#define ARM_VERSION 0 //版本号 已无用

typedef enum{
    INSIDE,
    OUTSIDE,
}Store_MANUAL_E;

typedef enum{
    C3Z_DONE,
    C3Z_PUTDOWN,
    C3Z_PICKUP,
    C3Z_PUTDOWN_NEXT,
}ARM_C3Z_E;

typedef struct{
    bool is_c3z_start = false; //是否开始挑战赛3区流程
    bool can_pickup = false; //是否可以拾取
    bool can_putdown = false; //是否可以放下

    bool is_putdown_done = false; //放下完成标志
    bool is_pickup_done = false; //拾取完成标志

    uint8_t c3z_acting = 0; //当前动作 0无动作 1放下 2拾取 3放下NEXT

    ARM_C3Z_E now_state = C3Z_DONE; //当前状态
}C3Z_S;

typedef struct{
    bool init_flag = false;
    uint8_t debug_start = 0; //调试开始标志 == 1 开始调试

    uint8_t auto_start = 0; //自动开始标志 == 1 开始自动

    float calibrate_startTime = 0; 
    bool calibrate_start = false;
    bool is_calibrating = false;

    float last_right_x = 0.0f; //上一次摇杆右横轴的值，用于检测摇杆状态变化
    float last_right_y = 0.0f; //上一次摇杆右纵轴的值，用于检测摇杆状态变化

    int8_t last_manual_extend = 0; //上一次手动展状态
    int8_t last_manual_sucker = 0; //上一次手动吸盘状态
    int8_t last_manual_pitch = 0; //上一次手动pitch状态
    int8_t last_manual_store_sucker = 0; //上一次手动存储位吸盘状态

    int8_t pitch_switch_offset = 0; //pitch开关偏移
    int8_t extend_switch_offset = 0; //展开关偏移
    int8_t sucker_switch_offset = 0; // 吸盘开关偏移
    int8_t store_suker_switch_offset = 0; // 存储吸盘开关偏移


#if !USE_RC10_AIRJOY
    uint8_t button_click_state = 0;
#endif
    uint8_t is_store_acting = 0; //手操作存储状态 0无动作 1取出 2存储 3拾取 4放下
    
    uint8_t last_manual_store = 0; //上一次手动存储状态

    bool can_putdown = 0;

    Store_MANUAL_E store_manual_mode = OUTSIDE; //存储位手动模式，默认外部
    uint8_t comp_takeout_now[2] = {0}; //当前取出位置 0 为初始即无，1为内，2为外

    bool is_putdown_done = false; //放下完成标志

    bool pickup_stage1_ready = false; // 拾取预置位完成，等待第二次按下
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
    TWO_OR_THREE,
    NONE_KFS,
}KFS_NUM_E;

typedef struct{
    int targetKFS[3] = {0,0,0};
    int now_targetIndex = 0;
    KFS_NUM_E kfs_num = ONLY_ONE;
    bool start_to_autoctrl = false;

    Point2D now_armPosition = {5.0f, 8.60f, 0.0f}; //机械臂当前位置

    Point2D now_ChassisPosition = {5.0f, 8.60f, 0.0f}; //底盘当前位置

    Point2D now_chassis_speed = {0.0f, 0.0f, 0.0f}; //当前底盘速度，单位：米/秒

    Point2D targetKFS_pos[2] = {{0.0f,0.0f,0.0f}, {0.0f,0.0f,0.0f}}; //目标KFS位置

    /**
     * @brief 路径规划 
     *                   
     */ 
    ARM_AUTO_STILLNESS_E now_state = STATE_DONE;
    MF_AutoCtrler::PathInformation_S pathInfo; 
    struct{
        bool isrecalcPath = false; //是否计算出完整路径
        bool canExtend = false; //是否可以展开，满足展开条件后外部接口自动置true
        float reach_finishTimeStore = 0.0f; //吸取KFS完成时间点，单位秒
        bool isExtReach = false;
        bool canChassisStart = false; //是否可以启动底盘

        bool isbackdone = false; //是否返回完成
        float back_time = 0.0f; //返回时长
        float target_high [3] = {0.0f, 0.0f, 0.0f}; //目标KFS高度
        bool is_up_catch = false;
        bool canChassisStart_triggered = false;
    }flag;
    float target_lowerhigh[3] = {0.0f, 0.0f, 0.0f}; //目标KFS拾取高度
}ARM_AUTO_S;




inline float GetKFSHeight(int8_t mfNum)
{
    const float* arr = (MF_AutoCtrler::get_color() == 1) ? 
        MF_AutoCtrler::MF_high_blue : MF_AutoCtrler::MF_high_blue;
    return arr[mfNum - 1];
}

class ArmSetup: public RtosTask ,public Robot_Arm {
public:
    ArmSetup(Arm_InitData_S init_Data)
        : Robot_Arm(init_Data), RtosTask("ArmSetup", 1) 
    {
    }

    void set_c3z_start(bool start)
    {
        c3z_ctrl_.is_c3z_start = start;
    }

    void set_c3z_can_pickup(bool pickup)
    {
        c3z_ctrl_.can_pickup = pickup;
    }

    void set_c3z_can_putdown(bool putdown)
    {
        c3z_ctrl_.can_putdown = putdown;
    }

    bool is_c3z_pickup_done() const
    {
        return c3z_ctrl_.is_pickup_done;
    }

    bool is_c3z_putdown_done() const
    {
        return c3z_ctrl_.is_putdown_done;
    }

    bool is_c3z_start() const
    {
        return c3z_ctrl_.is_c3z_start;
    }

    bool isArmcalibrated() const
    {
        if(arm_ctrlStatus.is_calibrating)
            return true;
        else
            return false;
    }

    /**
     * @brief 接收底盘是否可以放置的信号
     */
    void can_putdown(bool is_or_not)
    {
        arm_ctrlStatus.can_putdown = is_or_not;
    }

    void init(M3508 *motor_ArmLaunch, M2006 *motor_ArmStretch, 
        M3508 *motor_ArmRotate, DM_Motor *motor_ArmPitch)
    {
        this->registerMotor_Launch(motor_ArmLaunch);
        this->registerMotor_Stretch(motor_ArmStretch);
        this->registerMotor_Rotate(motor_ArmRotate);
        this->registerMotor_Pitch(motor_ArmPitch);

         this->setPitchReversed(false); //锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟?
        this->setStretchReversed(true); //锟斤拷展锟斤拷锟斤拷锟斤拷锟?
        this->setRotateReversed(false);
        this->setLaunchReversed(false); //锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟?
        start(osPriorityHigh-1, 512); 
        arm_ctrlStatus.init_flag = true;
    }

    void setArmStatus(ARM_Status_E status)
    {
        if(status != ARM_CALIBRATE && !isArmcalibrated())
            return;

        arm_status_ = status;
    }

    bool is_putdown_done()
    {
        return arm_ctrlStatus.is_putdown_done;
    }

    Store_MANUAL_E get_store_side()
    {
        return arm_ctrlStatus.store_manual_mode;
    }

    void set_store_side(Store_MANUAL_E side)
    {
        arm_ctrlStatus.store_manual_mode = side;
    }

    /**
     * @brief 计算目标KFS对应的路径
     * @param KFS1 第一个KFS编号，范围0~12
     * @param KFS2 第二个KFS编号，范围0~12
     * @brief 0表示没有要拾取的KFS，如果KFS1和KFS2都为0，则返回false表示没有有效目标；如果至少有一个KFS编号有效，则计算路径并返回true
     */
    bool set_TargetKFS(int KFS1, int KFS2, int KFS3)
    {
        if(KFS1 >=0 && KFS1 <=12)
            auto_ctrl_.targetKFS[0] = KFS1;
        else
            auto_ctrl_.targetKFS[0] = 0;
        if(KFS2 >=0 && KFS2 <=12)
            auto_ctrl_.targetKFS[1] = KFS2;
        else
            auto_ctrl_.targetKFS[1] = 0;
        if(KFS3 >=0 && KFS3 <=12)
            auto_ctrl_.targetKFS[2] = KFS3;
        else
            auto_ctrl_.targetKFS[2] = 0;

        if(KFS1 != 0 && KFS2 !=0)
            auto_ctrl_.kfs_num = TWO_OR_THREE;
        else
            auto_ctrl_.kfs_num = ONLY_ONE;

        if(KFS1 == 0 && KFS2 == 0 && KFS3 == 0)
            return false; //没锟斤拷目锟斤拷KFS锟斤拷锟斤拷锟斤拷失锟斤拷
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
            if(KFS3 != 0)
            {
                auto_ctrl_.targetKFS_pos[2] = MF_AutoCtrler::MapNum_RealPos[MF_AutoCtrler::MFNum_TransforMapNum(auto_ctrl_.targetKFS[2])-1];
            }
            else
            {
                auto_ctrl_.targetKFS_pos[2] = {0.0f, 0.0f, 0.0f};
            }
        }

        auto_ctrl_.pathInfo = MF_AutoCtrler::PathInformation_calc(auto_ctrl_.now_ChassisPosition,
                                       auto_ctrl_.targetKFS[0],
                                        auto_ctrl_.targetKFS[1], auto_ctrl_.targetKFS[2]);
        for(int i = 0; i < 3; i++)
        {
            auto_ctrl_.flag.target_high[i] = GetKFSHeight(auto_ctrl_.targetKFS[i]);
        }

        auto_ctrl_.flag.is_up_catch = (auto_ctrl_.now_targetIndex == 0x00 
            && auto_ctrl_.start_to_autoctrl == 1
            && GetKFSHeight(auto_ctrl_.targetKFS[auto_ctrl_.now_targetIndex]) == 0.2f 
            && auto_ctrl_.targetKFS[2] != 0);

#if ARM_AUTO_DEBUG_NOCHASSIS
        auto_ctrl_.now_ChassisPosition.x = MF_AutoCtrler::MapNum_RealPos[temp.MFroad[0]-1].x;
        auto_ctrl_.now_ChassisPosition.y = MF_AutoCtrler::MapNum_RealPos[temp.MFroad[0]-1].y - 3.0f;
#endif

    float targetLowerHeight = 0.0f; // 目标kfs高度
    float catch_offset = 0.0f;
    for(int i = 0; i < 3; i++)
    {
        if (auto_ctrl_.now_targetIndex == 0 && GetKFSHeight(auto_ctrl_.targetKFS[auto_ctrl_.now_targetIndex]) != 0.2f && auto_ctrl_.kfs_num == TWO_OR_THREE)
            // catch_offset = -0.07f;
            catch_offset = -0.05f;
        else if (auto_ctrl_.now_targetIndex == 1)
            catch_offset = 0.00f;
        else
            catch_offset = 0.0f;
        float kfs_h = GetKFSHeight(auto_ctrl_.targetKFS[i]);
        
        if (auto_ctrl_.flag.is_up_catch && kfs_h == 0.2f)
            targetLowerHeight = this->init_data_.up_20cm_lower_height_;
        if (kfs_h == 0.2f)
            targetLowerHeight = init_data_.catch_20height;
        else if (kfs_h == 0.4f)
            targetLowerHeight = this->init_data_.catch_40height + catch_offset;
        else if (kfs_h == 0.6f)
            targetLowerHeight = this->init_data_.catch_60height + catch_offset;
        else
            targetLowerHeight = this->init_data_.max_launchCatch_Height_ + catch_offset;

        auto_ctrl_.target_lowerhigh[i] = targetLowerHeight;
    }
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
     * @brief 设置自动展开功能
     * @param canExtend 是否可以展开
     * @details 锟斤拷锟斤拷锟角凤拷锟斤拷越锟斤拷锟斤拷锟秸癸拷锥锟?
     */
    void setAutocanExtend(bool canExtend)
    {
        auto_ctrl_.flag.canExtend = canExtend;
    }

    /**
     * @brief 返回是否底盘可以启动
     *       
     */
    bool isAutoChassisCanStart()
    {
        return auto_ctrl_.flag.canChassisStart;
    }

    void clearChassisStartFlag()
    {
        auto_ctrl_.flag.canChassisStart = false;
    }

    void pulseChassisStart()
    {
        if (!auto_ctrl_.flag.canChassisStart_triggered)
        {
            auto_ctrl_.flag.canChassisStart = true;
            auto_ctrl_.flag.canChassisStart_triggered = true;
        }
    }


    enum class store_state{
        idle,
        laucnh_state,
        rotate_state,
        lower_state,
        outstate1, //取锟斤拷专锟斤拷
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

#if !USE_RC10_AIRJOY
    RmPocketData_t airjoy_data_; // -1 ~ 1
#else
    communication::RC10_AirJoy_Data_S airjoy_data_; // -1 ~ 1
#endif

    Debug_Printf debug_uart = Debug_Printf(&huart8);

    //控制函数相关
    void manualControl();
    void manualControl_lowLevel();

    void arm_d_pad_ctrl(); //十字键统一处理

    bool c3z_putdown_ctrl(); //挑战赛3区放下控制
    bool c3z_pickup_ctrl(); //挑战赛3区拾取控制

    bool manual_store(uint8_t kfs_index); //存儲kfs
    bool manual_takeout(uint8_t kfs_index); //取出存储kfs
    bool manual_pickup(); //拾取地上的kfs
    bool manual_pickup_preposition(); //拾取阶段1：预置位
    bool manual_putdown(); //放下kfs

    void challenge_3zone();

    bool test();

    void autoControl();
    void stop();
    void idle();
    void debug();

    bool combine_idle();

    //校准
    void calibrateMotor();

    //=======================
    //自动控制相关状态函数

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
     * @brief 判断旋转是否被允许
     * @param rotate_angle_deg 期望的旋转角度（度）
     * @return 旋转是否被允许
     * @details 根据当前关节高度判断旋转是否安全
     *  H < lock_height_: 只允许在初始位置，即0度，不允许转动。
     *  lock_height_ <= H < Safe_H: 只允许在 0 ~ 135度的范围内转动
     *  H >= Safe_H: 允许在 135 ~ 360/0度范围内转动 不允许返回135~rotate_end之内的禁区角度 
     */
    bool isRotateAllowed(float rotate_angle_deg) const
    {
        const float h = this->get_currentJointStatus().launchJoint_Height_;
        const float safe_h = init_data_.safe_height_;

        const float norm_deg = rotate_angle_deg;

        if (h < init_data_.lock_height_) {
            // 只有下降前在0.0±3.0度时才强制锁定到0°
            if (pre_descent_angle_ <= 3.0f || pre_descent_angle_ >= 357.0f)
                return false;
        }
        if(h < safe_h - 0.01f) return (norm_deg >= 0.0f && norm_deg <= 135.0f);
        return true;
    }

    /**
     * @brief 返回安全的旋转角度
     * @details 根据当前关节高度裁剪旋转角度
     * - H < lock_height_:  0.0f
     * - lock_height_ <= H < Safe_H: 旋转范围 [0度, 135度]
     * - H >= Safe_H: 允许在 135 ~ 360/0度范围内转动 不允许返回135~rotate_end之内的禁区角度 
     * @param desired_deg 期望的旋转角度（度）
     * @return 安全的旋转角度
     */
    float sanitizeRotateAngle(float desired_deg) const
    {
        const float h = this->get_currentJointStatus().launchJoint_Height_;
        const float safe_h = init_data_.safe_height_;
        const float lock_h = init_data_.lock_height_;
        const float re = init_data_.rotate_end;

        if (h < lock_h) {
            // 只有下降前在0.0±3.0度时才强制锁定到0°
            if (pre_descent_angle_ <= 3.0f || pre_descent_angle_ >= 357.0f)
                return 0.0f;
            // 否则走safe_h限制
        }

        if (h < safe_h - 0.01f)
        {
            float cur = this->get_currentJointStatus().rotateJoint_angle_;
            bool in_storage_zone = (cur >= re && cur <= 360.0f)
                                || (cur >= 0.0f && cur <= 135.0f) ;
            if (in_storage_zone && this->get_currentJointStatus().launchJoint_Height_ > init_data_.store_height_outside_ - 0.01f)
                return desired_deg;

            const float norm_deg = desired_deg;
            if (norm_deg < 0.0f)   return 0.0f;
            if (norm_deg > 135.0f && norm_deg < 270.0f) return 135.0f;
            if (norm_deg >= 270.0f) return 0.0f;
            return norm_deg;
        }

        return desired_deg;
    }


    
protected:

    /**
     * @brief 获取当前机械臂位置
     * @details 实时获取机械臂当前位置
     * @return 当前机械臂位置
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
     * @brief 底盘速度
     * @return 底盘速度
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
            if(inTargetMap && auto_ctrl_.flag.canChassisStart == false)
            {
                speed = {0.0f, 0.0f, 0.0f};
            }

            if(auto_ctrl_.now_state != STATE_EXT && auto_ctrl_.now_state != STATE_LAUNCH)
            {
                speed = { 0.0f, 0.5f, 0.0f};
            }
            else if(inTargetMap && auto_ctrl_.flag.canChassisStart == true)
            {
                speed = {0.0f, 0.5f, 0.0f};
            }


            // if(auto_ctrl_.flag.canChassisStart || !inTargetMap)
            //     speed = {0.0f, 1.0f, 0.0f};
            // else
            //     speed = {0.0f, 0.0f, 0.0f};
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
     * @brief 获取当前底盘位置
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
    C3Z_S c3z_ctrl_;  //挑战赛3区专用

    struct {
        
        float rotate_rate = 0.1f;
        float launch_rate = 0.03f;
        int cnt = 0;
    }manual_control;

    float pre_descent_angle_ = 0.0f; // 下降前云台角度：h>=lock_h时持续更新，h<lock_h时冻结

#if USE_RC10_AIRJOY
    int8_t is_d_pad_up_clicked = 0;
    int8_t is_d_pad_down_clicked = 0;
    int8_t is_d_pad_left_clicked = 0;
    int8_t is_d_pad_right_clicked = 0;
#endif

    ButtonDetector button_detector_1 = ButtonDetector(0.200f); //双击三击检测器，200ms间隔


    bool calibration_seen_ = false;
    store_state store_state_ = store_state::idle; //存储状态机状态
};


extern Arm_InitData_S arm_initData;

#endif //__cplusplus


#endif // __ARM_SETUP_H