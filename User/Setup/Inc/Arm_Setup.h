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

typedef struct{
    bool init_flag = false;

    
    uint8_t debug_start = 1; //调试开始标志 == 1 开始调试

    float calibrate_startTime = 0; 
    bool calibrate_start = false;
    bool is_calibrating = false;

}arm_ctrl_status_S;

typedef enum{
    STATE_TO_TARGET_HIGHT, //阶段1：升高到对应高度
    SIGN_ALIGN,            //阶段2：旋转对齐，打开吸盘
    STATE_AIM_EXT,         //阶段3：伸展预判
    STATE_CARRYING,        //阶段4：吸附后搬回
    STATE_RETURN,          //阶段5：返回初始位置
    STATE_DONE,            //待机
}ARM_AUTO_E;

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
    const float stretch_time_s = 0.613f; //伸展时间，单位秒

    float gimbal_max_rad = 0.0f; //云台最大旋转角速度，单位弧度每秒
}arm_timeset_S;

typedef struct{
    int targetKFS[2] = {0,0};
    int now_targetIndex = 0;
    KFS_NUM_E kfs_num = ONLY_ONE;
    ARM_AUTO_E now_state = STATE_DONE;

    Point2D now_armPosition = {0.0f, 0.0f, 0.0f}; //机械臂当前位置

    Point2D now_ChassisPosition = {0.0f, 0.0f, 0.0f}; //底盘当前位置

    Point2D targetKFS_pos[2] = {{0.0f,0.0f,0.0f}, {0.0f,0.0f,0.0f}}; //目标KFS位置

    //Point2D point_PAB[2] = {{0.0f,0.0f,0.0f}, {0.0f,0.0f,0.0f}}; //PA PB
    struct {
        Point2D PA;
        Point2D PB;
    }PointPAB[2];

    MF_AutoCtrler::Direction_E KFS_Movedirection[2] = {MF_AutoCtrler::NONE, MF_AutoCtrler::NONE}; //目标KFS方向

    MF_AutoCtrler::PathNode_S path;

    autopathPos_S pathPos;

    int gimbal_calcCount = 0; //云台预判计算计数

    float dt = 0.01f; //控制周期，单位秒

    float now_chassis_speed = 1.0f; //当前底盘速度，单位米每秒

    const float arm_width = 0.12f; //机械臂宽度，单位米

    int gimbal_calcHz = 100; //云台预判计算频率

    arm_timeset_S time_set;


    /**
     * @brief 旋转路径策略 正方向表示角度正增，负方向表示角度负增；
     *                    正增为逆时针旋转，负增为顺时针旋转
     */
    Rotate_Strategy_E current_strategy = ROTATE_PATH_SHORTEST; 
}ARM_AUTO_S;





const float MF_high[12] = 
{
    40.0f, 20.0f, 40.0f,
    20.0f, 40.0f, 60.0f,
    40.0f, 60.0f, 40.0f,
    20.0f, 40.0f, 20.0f
};

class ArmSetup: public RtosTask ,public Robot_Arm {
public:
    ArmSetup(Arm_InitData_S init_Data)
        : Robot_Arm(init_Data), RtosTask("ArmSetup", 1) 
    {
        auto_ctrl_.time_set.gimbal_max_rad = (400.0f * init_Data.rotate_gearRatio_ * PI)/(180.0f * 60.0f); //云台最大角速度(rad/s)
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

        start(osPriorityNormal, 256);

        arm_ctrlStatus.init_flag = true;
    }

    void setArmStatus(ARM_Status_E status)
    {
        arm_status_ = status;
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

        auto_ctrl_.targetKFS_pos[0] = MF_AutoCtrler::MapNum_RealPos[auto_ctrl_.targetKFS[0] - 1];
        auto_ctrl_.targetKFS_pos[1] = MF_AutoCtrler::MapNum_RealPos[auto_ctrl_.targetKFS[1] - 1];
        
        MF_AutoCtrler::get_MoveDiretion(auto_ctrl_.now_armPosition,
                                        auto_ctrl_.targetKFS[0], auto_ctrl_.targetKFS[1],
                                        auto_ctrl_.KFS_Movedirection);

        auto_ctrl_.now_targetIndex = 0;

        MF_AutoCtrler::PathNode_S temp = MF_AutoCtrler::PathNodeResult_calc(auto_ctrl_.now_armPosition,
                                        auto_ctrl_.targetKFS[0], auto_ctrl_.targetKFS[1]);
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

        return true;
    }
private:
    Debug_Printf debug_uart = Debug_Printf(&huart1);

    //控制函数
    void manualControl();
    void autoControl();
    void stop();
    void idle();
    void debug();

    //上电校准M2006电机位置
    void calibrateM2006();

    //自动控制流程私密函数

    void state_toTargetHight(int targetKFS);
    void state_signAlign(int targetKFS);
    void state_aimExt(int targetKFS);
    void state_carrying(int targetKFS);
    void state_return(int next_targetKFS);


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

    }
    /**
     * @brief 预留接口后续补全，获得当前底盘速度
     * @return 
     */
    Point2D get_nowChassisSpeed()
    {

    }

    /**
     * @brief 获得底盘位姿
     */

    Point2D get_nowChassisPose()
    {

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

    /**
     * @brief 这里的输入是已经初始化的targetKFS的index
     * @return 云台旋转时候末端需要避障的PA PB点 
     * 
     * @details 前面不规范创建的结构体，导致现在只能把这坨屎山堆下去
     */
    void get_GimbalMF_PAPB(int target_KFSIndex, Point2D& PA, Point2D& PB)
    {   
        /*
            0~3索引分别对应从左下角的点，逆时针到左上角的点
        */
        Point2D targetMF_p[4];
        for(int i = 0; i < 4; i++)
            targetMF_p[i] = {0.0f, 0.0f, 0.0f};
        Point2D targetBestMF_p[4];
        for(int i = 0; i < 4; i++)
            targetBestMF_p[i] = {0.0f, 0.0f, 0.0f};

        Point2D targetbestB_p[4];
            for(int i = 0; i < 4; i++)
                targetbestB_p[i] = {0.0f, 0.0f, 0.0f};

        if(target_KFSIndex != 0 && target_KFSIndex !=1)
            return;

        Point2D result[2]= {{0}, {0}}; //0 PA 1 PB


        for(int i = 0; i < 4; i++)
        {
            switch(i)
            {
                case 0:
                {
                    targetMF_p[i].x = auto_ctrl_.targetKFS_pos[target_KFSIndex].x - 0.6f;
                    targetMF_p[i].y = auto_ctrl_.targetKFS_pos[target_KFSIndex].y - 0.6f;

                    if(target_KFSIndex == 0)
                    {
                        targetBestMF_p[i].x = auto_ctrl_.pathPos.bestBMF1.x - 0.6f;
                        targetBestMF_p[i].y = auto_ctrl_.pathPos.bestBMF1.y - 0.6f;

                        targetbestB_p[i].x = auto_ctrl_.pathPos.bestB1.x - 0.6f;
                        targetbestB_p[i].y = auto_ctrl_.pathPos.bestB1.y - 0.6f;
                    }
                    else
                    {
                        targetBestMF_p[i].x = auto_ctrl_.pathPos.bestBMF2.x - 0.6f;
                        targetBestMF_p[i].y = auto_ctrl_.pathPos.bestBMF2.y - 0.6f;

                        targetbestB_p[i].x = auto_ctrl_.pathPos.bestB2.x - 0.6f;
                        targetbestB_p[i].y = auto_ctrl_.pathPos.bestB2.y - 0.6f;
                    }
                    break;
                }

                case 1:
                {
                    targetMF_p[i].x = auto_ctrl_.targetKFS_pos[target_KFSIndex].x + 0.6f;
                    targetMF_p[i].y = auto_ctrl_.targetKFS_pos[target_KFSIndex].y - 0.6f;

                    if(target_KFSIndex == 0)
                    {
                        targetBestMF_p[i].x = auto_ctrl_.pathPos.bestBMF1.x + 0.6f;
                        targetBestMF_p[i].y = auto_ctrl_.pathPos.bestBMF1.y - 0.6f;

                        targetbestB_p[i].x = auto_ctrl_.pathPos.bestB1.x + 0.6f;
                        targetbestB_p[i].y = auto_ctrl_.pathPos.bestB1.y - 0.6f;
                    }
                    else
                    {
                        targetBestMF_p[i].x = auto_ctrl_.pathPos.bestBMF2.x + 0.6f;
                        targetBestMF_p[i].y = auto_ctrl_.pathPos.bestBMF2.y - 0.6f;

                        targetbestB_p[i].x = auto_ctrl_.pathPos.bestB2.x + 0.6f;
                        targetbestB_p[i].y = auto_ctrl_.pathPos.bestB2.y - 0.6f;
                    }

                    break;
                }

                case 2:
                {
                    targetMF_p[i].x = auto_ctrl_.targetKFS_pos[target_KFSIndex].x + 0.6f;
                    targetMF_p[i].y = auto_ctrl_.targetKFS_pos[target_KFSIndex].y + 0.6f;

                    if(target_KFSIndex == 0)
                    {
                        targetBestMF_p[i].x = auto_ctrl_.pathPos.bestBMF1.x + 0.6f;
                        targetBestMF_p[i].y = auto_ctrl_.pathPos.bestBMF1.y + 0.6f;

                        targetbestB_p[i].x = auto_ctrl_.pathPos.bestB1.x + 0.6f;
                        targetbestB_p[i].y = auto_ctrl_.pathPos.bestB1.y + 0.6f;
                    }
                    else
                    {
                        targetBestMF_p[i].x = auto_ctrl_.pathPos.bestBMF2.x + 0.6f;
                        targetBestMF_p[i].y = auto_ctrl_.pathPos.bestBMF2.y + 0.6f;

                        targetbestB_p[i].x = auto_ctrl_.pathPos.bestB2.x + 0.6f;
                        targetbestB_p[i].y = auto_ctrl_.pathPos.bestB2.y + 0.6f;
                    }

                    break;
                }

                case 3:
                {
                    targetMF_p[i].x = auto_ctrl_.targetKFS_pos[target_KFSIndex].x - 0.6f;
                    targetMF_p[i].y = auto_ctrl_.targetKFS_pos[target_KFSIndex].y + 0.6f;

                    if(target_KFSIndex == 0)
                    {
                        targetBestMF_p[i].x = auto_ctrl_.pathPos.bestBMF1.x - 0.6f;
                        targetBestMF_p[i].y = auto_ctrl_.pathPos.bestBMF1.y + 0.6f;

                        targetbestB_p[i].x = auto_ctrl_.pathPos.bestB1.x - 0.6f;
                        targetbestB_p[i].y = auto_ctrl_.pathPos.bestB1.y + 0.6f;
                    }
                    else
                    {
                        targetBestMF_p[i].x = auto_ctrl_.pathPos.bestBMF2.x - 0.6f;
                        targetBestMF_p[i].y = auto_ctrl_.pathPos.bestBMF2.y + 0.6f;

                        targetbestB_p[i].x = auto_ctrl_.pathPos.bestB2.x - 0.6f;
                        targetbestB_p[i].y = auto_ctrl_.pathPos.bestB2.y + 0.6f;
                    }

                    break;
                }

                default:
                    break;
            }
        }

        //判断重合的点，如果MF、bestB、bestBMF都重合，即PA，只有MF、bestBMF重合，即为PB

        Point2D temp[2] = {{0,0,0}, {0,0,0}}; 
        int temp_index = 0;
        
        // 1. 找出 MF 和 BMF 的重合点
        for(int i = 0; i < 4; i++)
        {
            for(int j = 0; j < 4; j++)
            {
            
                if(std::abs(targetMF_p[i].x - targetBestMF_p[j].x) < 0.001f &&
                std::abs(targetMF_p[i].y - targetBestMF_p[j].y) < 0.001f)
                {
                    if(temp_index < 2) {
                        temp[temp_index] = targetMF_p[i];
                        temp_index++;
                    }
                    break; // 找到一个重合点后跳出内层循环
                }
            }
        }

        // 2. 区分 PA 和 PB
        
        for(int i = 0; i < temp_index; i++)
        {
            bool is_PA = false;
            for(int j = 0; j < 4; j++)
            {
                // 判断是否与 bestB 重合
                if(std::abs(temp[i].x - targetbestB_p[j].x) < 0.001f &&
                std::abs(temp[i].y - targetbestB_p[j].y) < 0.001f)
                {
                    result[0] = temp[i]; // 既重合 bestB，又是 MF/BMF 公共点 -> PA
                    is_PA = true;
                    break; // 确认为 PA，跳出
                }
            }
            
            //遍历完所有 bestB 的角后，如果都不是，才是 PB
            if(!is_PA)
                result[1] = temp[i]; // 只是 MF/BMF 公共点 -> PB
            
        }

        PA = result[0];
        PB = result[1];
    }
    
};


extern Arm_InitData_S arm_initData;

#endif //__cplusplus


#endif // __ARM_SETUP_H

