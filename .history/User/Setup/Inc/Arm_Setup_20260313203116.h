/**
 * @file Arm_setup.h
 * @author XieFField  70er66
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

    int8_t extend_switch_offset = 0; // 伸展开关偏移绑定
    int8_t sucker_switch_offset = 0; // 吸盘开关偏移绑定
}arm_ctrl_status_S;

#if ARM_AUTOMOVE
typedef enum{
    STATE_TO_TARGET_HIGHT, //阶段1：升高到对应高度
    STATE_SIGN_ALIGN,            //阶段2：旋转对齐，打开吸盘
    STATE_AIM_EXT,         //阶段3：伸展预判
    STATE_CARRYING,        //阶段4：吸附后搬回
    STATE_RETURN,          //阶段5：返回初始位置
    STATE_DONE,            //待机，还未进入梅花林
}ARM_AUTO_E;
#endif


typedef enum{
    STATE_TO_WAIT,
    STATE_ALIGN,
    STATE_LOWER,
    STATE_EXT,
    STATE_LAUNCH,
    STATE_BACK,
    STATE_DONE
}ARM_AUTO_STILLNESS_E;

typedef enum{
    ONLY_ONE,
    TWO,
}KFS_NUM_E;

typedef struct{
    Point2D entranceMap;
    Point2D bestB1;     //前一桩
    Point2D bestBMF1;   //正对桩
    Point2D bestB2;
    Point2D bestBMF2;
    Point2D exitMap;
}autopathPos_S;

typedef struct{
    const float stretch_time_s = 0.3f; //伸展时间，单位秒

    float gimbal_max_rad = 0.0f; //云台最大旋转角速度，单位弧度每秒
    float rotateSpeedRate_ = 0.8f; //云台旋转速度比例
}arm_timeset_S;

typedef struct{
    int targetKFS[2] = {0,0};
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

    MF_AutoCtrler::Direction_E KFS_Movedirection[2] = {MF_AutoCtrler::NONE, MF_AutoCtrler::NONE}; //目标KFS方向

    MF_AutoCtrler::PathNode_S path;

    autopathPos_S pathPos;
    /**
     * @brief 旋转路径策略 正方向表示角度正增，负方向表示角度负增；
     *                    正增为逆时针旋转，负增为顺时针旋转
     */
    Rotate_Strategy_E current_strategy = ROTATE_PATH_SHORTEST; 
    
#if ARM_AUTOMOVE
    ARM_AUTO_E now_state = STATE_DONE;
    int gimbal_calcCount = 0; //云台预判计算计数

    ARM_AUTO_E state = STATE_DONE;
    struct{

        bool is_toPlace = false; //是否到达可放置状态
    }store[2];
    struct{
        bool align_done = false; //对齐完成标志
        bool ext_done = false;   //伸展完成标志
        bool carry_done = false; //搬运完成标志
        bool return_done = false; //返回完成标志

        bool ext_started = false; //伸展开始标志
        bool issafetoLower = false;
        bool is_reachingTarget = false; //是否到达目标位置

        float reach_finishTime = 0.0f; //到达目标位置的时间戳
        const float safe_height = 0.14f; //安全高度，单位米  待定
        bool isrecalcPath = false; //是否重新计算路径
    }flag;

    arm_timeset_S time_set;
#else
    ARM_AUTO_STILLNESS_E now_state = STATE_DONE;

    struct{
        bool isrecalcPath = false; //是否重新计算路径
        bool canExtend = false; //是否可以伸展
    }flag;

#endif
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
        #if ARM_AUTOMOVE
        auto_ctrl_.time_set.gimbal_max_rad = (100.0f * init_Data.rotate_gearRatio_ * PI)/(180.0f * 60.0f) ; //云台最大角速度(rad/s)
        #endif
    }

    bool isArmcalibrated() const
    {
        if(arm_ctrlStatus.is_calibrating)
            return true;
        else
            return false;
    }

    void init(M3508 *motor_ArmLaunch, M2006 *motor_ArmStretch, 
        M3508 *motor_ArmRotate, M2006 *motor_ArmPitch)
    {
        this->registerMotor_Launch(motor_ArmLaunch);
        this->registerMotor_Stretch(motor_ArmStretch);
        this->registerMotor_Rotate(motor_ArmRotate);
        this->registerMotor_Pitch(motor_ArmPitch);

        this->setPitchReversed(true); //俯仰电机反向
        this->setStretchReversed(false); //伸展电机不反向
        this->setRotateReversed(false);
        this->setLaunchReversed(true); //升降电机反向
        start(osPriorityHigh-1, 512); 
        setRotateMultiTurn(false); //单圈模式
        arm_ctrlStatus.init_flag = true;
    }

    void setArmStatus(ARM_Status_E status)
    {
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

        auto_ctrl_.targetKFS_pos[0] = MF_AutoCtrler::MapNum_RealPos[MF_AutoCtrler::MFNum_TransforMapNum(auto_ctrl_.targetKFS[0])-1];
        auto_ctrl_.targetKFS_pos[1] = MF_AutoCtrler::MapNum_RealPos[MF_AutoCtrler::MFNum_TransforMapNum(auto_ctrl_.targetKFS[1])-1];
        
        MF_AutoCtrler::get_MoveDiretion(auto_ctrl_.now_armPosition,
                                        auto_ctrl_.targetKFS[0], auto_ctrl_.targetKFS[1],
                                        auto_ctrl_.KFS_Movedirection);

        auto_ctrl_.now_targetIndex = 0;

        MF_AutoCtrler::PathNode_S temp = MF_AutoCtrler::PathNodeResult_calc(auto_ctrl_.now_armPosition,
                                       auto_ctrl_.targetKFS[0], 
                                        auto_ctrl_.targetKFS[1],26);
                                        
        auto_ctrl_.path.bestB1 = temp.bestB1;
        auto_ctrl_.path.bestBMF1 = temp.bestBMF1;
        auto_ctrl_.path.bestB2 = temp.bestB2;
        auto_ctrl_.path.bestBMF2 = temp.bestBMF2;

        auto_ctrl_.pathPos.bestB1 = MF_AutoCtrler::MapCenterWorld(auto_ctrl_.path.bestB1);
        auto_ctrl_.pathPos.bestBMF1 = MF_AutoCtrler::MapCenterWorld(auto_ctrl_.path.bestBMF1);
        auto_ctrl_.pathPos.bestB2 = MF_AutoCtrler::MapCenterWorld(auto_ctrl_.path.bestB2);
        auto_ctrl_.pathPos.bestBMF2 = MF_AutoCtrler::MapCenterWorld(auto_ctrl_.path.bestBMF2);
        auto_ctrl_.pathPos.entranceMap = MF_AutoCtrler::MapCenterWorld(auto_ctrl_.path.entranceMap);
        auto_ctrl_.pathPos.exitMap = MF_AutoCtrler::MapCenterWorld(auto_ctrl_.path.exitMap);



#if ARM_AUTO_DEBUG_NOCHASSIS
        //auto_ctrl_.now_ChassisPosition = auto_ctrl_.pathPos.bestB1 ; //初始化底盘位置为前一桩位置
        auto_ctrl_.now_ChassisPosition.y = 8.60f;
        auto_ctrl_.now_ChassisPosition.x = 5.0f; //假设已经到达前一桩正前方0.5米处
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

    //自动控制流程私密函数[行进间拾取]

    void state_toTargetHight(int targetKFS);
    void state_signAlign(int targetKFS ,bool &align_done);
    bool state_aimExt(int targetKFS);
    void state_carrying(int targetKFS, bool &carrying_done);
    bool state_return(int next_targetKFS);

    void auto_onlyOne();
    void auto_two();
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
    bool state_doneStillness(int targetKFS);
    void set_chassisStartStillness(){}

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

    /**
     * @brief 云台碰撞检测
     * 
     * @param px 机械臂末端X坐标，单位米
     * @param py 机械臂末端Y坐标，单位米
     * @param pivot_x 机械臂基座X坐标，单位米
     * @param pivot_y 机械臂基座Y坐标，单位米
     * @param arm_world_angle_deg 机械臂在世界坐标系下的绝对角度 (度)
     * @param L_arm 机械臂长度，单位米
     * @param W_arm 机械臂宽度，单位米
     * @return true 碰撞
     */
    bool check_Arm_collision(float px, float py, 
                            float pivot_x, float pivot_y, 
                            float arm_angle_deg, float L_arm, 
                            float W_arm);

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
           speed = {-0.6f, 0.0f, 0.0f};

        else
             speed = {0.0f, 0.0f, 0.0f};
        return speed;

        #else

            // Locate_Setup *locate_ptr = Locate_Setup::getInstance();

            // return locate_ptr->get_FK_ChassisSpeed_inWorld();
            Locate_Setup *locate_ptr = Locate_Setup::getInstance();
            Point2D speed = {0};
            speed.x = locate_ptr->get_RobotSpeed_inWorld().x/rate_forspeed;
            speed.y = locate_ptr->get_RobotSpeed_inWorld().y/rate_forspeed;
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

    /**
     * @brief 这里的输入是已经初始化的targetKFS的index (0 或 1)
     * @return 云台旋转时候末端需要避障的PA PB点 
     * 
     * 
     */
    void get_GimbalMF_PAPB(int target_KFSIndex, Point2D& PA, Point2D& PB)
    {   
        if(target_KFSIndex != 0 && target_KFSIndex != 1) 
            return;

        // 1. 获取 KFS 中心坐标 (直接从结构体获取，无需转换)
        Point2D KFS_Pos = auto_ctrl_.targetKFS_pos[target_KFSIndex];
        
        // 2. 获取参考点 (路径起点 B)
        Point2D Robot_Pos;
        if(target_KFSIndex == 0) 
            Robot_Pos = auto_ctrl_.pathPos.bestB1;
        else 
            Robot_Pos = auto_ctrl_.pathPos.bestB2;
        
        // 3. 单元格半宽 (1.2m / 2 = 0.6m)
        float half_cell = 0.6f; 
        
        // 4. 获取运动方向
        MF_AutoCtrler::Direction_E dir = auto_ctrl_.KFS_Movedirection[target_KFSIndex];
        
        // 5. 逻辑修正：PA 必须是运动方向上先遇到的点 (Near Corner)
        if (dir == MF_AutoCtrler::Positive_X || dir == MF_AutoCtrler::Negative_X) 
        {
            // 水平运动，比较 Y 坐标
            if (KFS_Pos.y > Robot_Pos.y) 
            {
                // KFS 在上方 (North)，底盘在下方通过 -> 障碍面是 KFS 下表面
                float common_y = KFS_Pos.y - half_cell;
                if (dir == MF_AutoCtrler::Positive_X) 
                {
                    PA.x = KFS_Pos.x - half_cell; PA.y = common_y; // Left-Bottom
                    PB.x = KFS_Pos.x + half_cell; PB.y = common_y; // Right-Bottom
                } 
                else 
                { // Negative_X
                    PA.x = KFS_Pos.x + half_cell; PA.y = common_y; // Right-Bottom
                    PB.x = KFS_Pos.x - half_cell; PB.y = common_y; // Left-Bottom
                }
            } 
            else 
            {
                // KFS 在下方 (South) -> 障碍面是 KFS 上表面
                float common_y = KFS_Pos.y + half_cell;
                if (dir == MF_AutoCtrler::Positive_X) 
                {
                    PA.x = KFS_Pos.x - half_cell; PA.y = common_y; // Left-Top
                    PB.x = KFS_Pos.x + half_cell; PB.y = common_y; // Right-Top
                } 
                else 
                { // Negative_X
                    PA.x = KFS_Pos.x + half_cell; PA.y = common_y; // Right-Top
                    PB.x = KFS_Pos.x - half_cell; PB.y = common_y; // Left-Top
                }
            }
        } 
        else 
        {

            // 垂直运动，比较 X 坐标
            if (KFS_Pos.x > Robot_Pos.x) 
            {
                // KFS 在右侧 (East) -> 障碍面是 KFS 左表面
                float common_x = KFS_Pos.x - half_cell;
                if (dir == MF_AutoCtrler::Positive_Y) 
                {
                    PA.x = common_x; PA.y = KFS_Pos.y - half_cell; // Left-Bottom
                    PB.x = common_x; PB.y = KFS_Pos.y + half_cell; // Left-Top
                } 
                else 
                { // Negative_Y
                    PA.x = common_x; PA.y = KFS_Pos.y + half_cell; // Left-Top
                    PB.x = common_x; PB.y = KFS_Pos.y - half_cell; // Left-Bottom
                }
            } 
            else 
            {
                // KFS 在左侧 (West) -> 障碍面是 KFS 右表面
                float common_x = KFS_Pos.x + half_cell;
                if (dir == MF_AutoCtrler::Positive_Y) 
                {
                    PA.x = common_x; PA.y = KFS_Pos.y - half_cell; // Right-Bottom
                    PB.x = common_x; PB.y = KFS_Pos.y + half_cell; // Right-Top
                } 
                else 
                { // Negative_Y
                    PA.x = common_x; PA.y = KFS_Pos.y + half_cell; // Right-Top
                    PB.x = common_x; PB.y = KFS_Pos.y - half_cell; // Left-Bottom
                }
            }
        }
    }
    
    float rate_forspeed =1.0f;
};


extern Arm_InitData_S arm_initData;

#endif //__cplusplus


#endif // __ARM_SETUP_H

