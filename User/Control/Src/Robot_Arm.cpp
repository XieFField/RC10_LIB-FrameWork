#include "Robot_Arm.h"
#include <iostream>

Robot_Arm::Robot_Arm(Arm_InitData_S init_Data)
    : init_data_(init_Data)
{
}

float ramp_rate = 15000;
void Robot_Arm::update()
{
    /*电机当前的角度转换成关节当前的角度 */
    now_time_s_ = TimeStamp::getInstance().getSeconds();

    if(!time_initialized_)
    {
        last_time_s_ = now_time_s_;
        time_initialized_ = true;
        // 首次对齐，后续基于“目标”积分
        target_joint_angle_ = joint_angle_;
        return;
    }

    dt_ = now_time_s_ - last_time_s_;
    last_time_s_ = now_time_s_;

    if (motor_rotate_ != nullptr)
    {
        ramped_rotateMotorAngle_ = motor_rotate_->getTotalAngle();
    }
    
    if(motor_rotate_ != nullptr)
    {
        // 直接读取电机总角度，映射回机械臂的 0-360 度
        float raw_angle = MotorTotalAngle_to_rotateAngle(motor_rotate_->getTotalAngle());
        joint_angle_.rotateJoint_angle_ = normalize_deg_0_360(raw_angle);
    }
    if(motor_stretch_ != nullptr)
        joint_angle_.stretchJoint_Length_ = MotorTotalAngle_to_stretchLength(motor_stretch_->getTotalAngle());
    if(motor_launch_ != nullptr)
        joint_angle_.launchJoint_Height_ = MotorTotalAngle_to_launchHeight(motor_launch_->getTotalAngle());
    if(motor_pitch_ != nullptr)
        joint_angle_.suckerJoint_angle_ = MotorTotalAngle_to_pitchAngle(motor_pitch_->getTotalAngle());


    forwardKinematics(arm_forward_);

    if(control_mode_ == TARGET_POSITION_MODE)
        inverseKinematics(arm_target_);

    else if(control_mode_ == MANUAL_MOTOR_POSITION_MODE)
    {
        // 手动电机位置模式下的处理
        target_joint_angle_.launchJoint_Height_  = constrain(target_joint_angle_.launchJoint_Height_,  0.0f, init_data_.max_launchHeight_);
        target_joint_angle_.stretchJoint_Length_ = constrain(target_joint_angle_.stretchJoint_Length_, 0.0f, init_data_.max_stretchLength_);
       
        target_joint_angle_.rotateJoint_angle_ = calc_rotate_targetByStrategy(
            joint_angle_.rotateJoint_angle_,
            target_joint_angle_.rotateJoint_angle_
        );
    }
    else if(control_mode_ == CURRENT_CONTROL_MODE)
        // 电流控制模式下的处理
        return; // 直接返回，不进行位置更新
    

  
    // 机械臂位置更新
    float target_rotateMotorAngle = 0.0f;
    float target_stretchMotorAngle = 0.0f;
    float target_launchMotorAngle = 0.0f;
    float target_pitchMotorAngle = 0.0f;

    // 对旋转通道：计算基于最近圈数的绝对目标
    if (motor_rotate_ != nullptr)
    {
        float current_arm_total = MotorTotalAngle_to_rotateAngle(motor_rotate_->getTotalAngle());
        
        float target_arm_mod = normalize_deg_0_360(target_joint_angle_.rotateJoint_angle_);

        // 计算 k 值 (Round to nearest integer)
        // 增加 0.5f 偏移确保 round 行为在正负数一致 (虽然 roundf 已处理)
        float diff = current_arm_total - target_arm_mod;
        float k = roundf(diff / 360.0f);

        float target_arm_total = target_arm_mod + k * 360.0f;
        
        target_rotateMotorAngle = rotateAngle_to_MotorTotalAngle(target_arm_total);

		setRampRotateMaxSpeed(ramp_rate); // 可调参数，按需设置
        ramp_rotate_target_ = caculate_rotate_target(motor_rotate_->getTotalAngle(),target_rotateMotorAngle);
   
		motor_rotate_->setTargetTotalAngle(ramp_rotate_target_);
			
    }

    target_stretchMotorAngle = stretchLength_to_MotorTotalAngle(target_joint_angle_.stretchJoint_Length_);
    target_launchMotorAngle = launchHeight_to_MotorTotalAngle(target_joint_angle_.launchJoint_Height_);
    target_pitchMotorAngle = pitchAngle_to_MotorTotalAngle(target_joint_angle_.suckerJoint_angle_);
    /*暂时不做斜坡处理*/

    if(motor_stretch_ != nullptr)
        motor_stretch_->setTargetTotalAngle(target_stretchMotorAngle);

    if(motor_launch_ != nullptr)
        motor_launch_->setTargetTotalAngle(target_launchMotorAngle);

    if(motor_pitch_ != nullptr)
        motor_pitch_->setTargetTotalAngle(init_data_.max_pitchRPM_, target_pitchMotorAngle);

    if(sucker_status_ == SUCK)
        HAL_GPIO_WritePin(init_data_.Sucker_GPIO_Port, init_data_.Sucker_GPIO_Pin, GPIO_PIN_SET);
    
    else
        HAL_GPIO_WritePin(init_data_.Sucker_GPIO_Port, init_data_.Sucker_GPIO_Pin, GPIO_PIN_RESET);
    
}

void Robot_Arm::inverseKinematics(Arm_Point_S target_point)
{
    // 旋转角（度）
    float raw_deg;
    if (std::abs(target_point.x) < 1e-6f && std::abs(target_point.y) < 1e-6f)
        raw_deg = joint_angle_.rotateJoint_angle_;  // 奇异点：保持当前角
    else
        raw_deg = atan2f(target_point.y, target_point.x) * 180.0f / PI;

    // 就近包裹，保证目标也是 0-360
    target_joint_angle_.rotateJoint_angle_ = normalize_deg_0_360(raw_deg);

    target_joint_angle_.launchJoint_Height_ = target_point.z;
    target_joint_angle_.stretchJoint_Length_ = sqrt(target_point.x * target_point.x + target_point.y * target_point.y) - init_data_.arm_length_;



    /*角度制 */
   
    target_joint_angle_.suckerJoint_angle_ = target_point.suckerJoint_status_;

    target_joint_angle_.launchJoint_Height_ = constrain(target_joint_angle_.launchJoint_Height_,
                                                    0.0f,
                                                    init_data_.max_launchHeight_);
    target_joint_angle_.stretchJoint_Length_ = constrain(target_joint_angle_.stretchJoint_Length_,
                                                     0.0f,
                                                     init_data_.max_stretchLength_);
}


bool Robot_Arm::forwardKinematics(Arm_Point_S& out) const
{
    /*末端关节位置*/
    float theta = joint_angle_.rotateJoint_angle_ * 3.1415926f / 180.0f;
    float Ltot  = init_data_.arm_length_ + joint_angle_.stretchJoint_Length_;

    out.x = Ltot * cosf(theta);
    out.y = Ltot * sinf(theta);
    out.z = joint_angle_.launchJoint_Height_;

    out.suckerJoint_status_ = joint_angle_.suckerJoint_angle_;
    return true;
}

float Robot_Arm::calc_rotate_targetByStrategy(float current_cont_angle, float target_raw_0_360)
{
    //连续角度归一化至0~360
    
    float current_mod = fmodf(current_cont_angle, 360.0f);
    if(current_mod < 0)
        current_mod += 360.0f;

    //归一化目标角度，防止越界
    float target_mod = fmodf(target_raw_0_360, 360.0f);
    if(target_mod < 0)
        target_mod += 360.0f;

    float diff = target_mod - current_mod;

    // 当接近目标角时，强制切换到最短路径，避免过冲后持续单向绕圈无法收敛
    float shortest_diff = diff;
    if (shortest_diff > 180.0f)
        shortest_diff -= 360.0f;
    else if (shortest_diff < -180.0f)
        shortest_diff += 360.0f;

    if (_tool_Abs(shortest_diff) < 10.0f)
        return current_cont_angle + shortest_diff;

    switch(rotate_strategy_)
    {
        case ROTATE_PATH_SHORTEST:
            // 最短路径：差值限制在 -180 到 +180 之间
            if (diff > 180.0f)       
                diff -= 360.0f;
            else if (diff < -180.0f) 
                diff += 360.0f;
            break;

        case ROTATE_PATH_POSITIVE:
            // 正方向：关节角度必须增加，即 diff 必须 > 0
            if (diff < 0.0f) 
                diff += 360.0f;
            break;

        case ROTATE_PATH_NEGATIVE:
            // 负方向：关节角度必须减小，即 diff 必须 < 0
            if (diff > 0.0f) 
                diff -= 360.0f;
            break;
    }

    return current_cont_angle + diff;
}





























/*

void test()
{
    // 1. 初始化机械臂参数
    Arm_InitData_S arm_params;
    arm_params.max_launchHeight_ = 1.0f;    // 最大升降高度 1米
    arm_params.max_stretchLength_ = 2.0f;   // 最大伸展长度 2米
    arm_params.arm_length_ = 0.5f;          // 机械臂基础长度 0.5米
    
    arm_params.stretch_Ratio_ = 0.01f;      // 伸展电机转一圈伸展0.01米
    arm_params.launch_Ratio_ = 0.005f;      // 升降电机转一圈升降0.005米  
    arm_params.rotate_gearRatio_ = 1.0f;    // 旋转电机转一圈机械臂转1度
    arm_params.pitch_gearRatio_ = 1.0f;     // 俯仰电机转一圈末端转1度

    // 2. 创建机械臂对象
    Robot_Arm my_arm(arm_params);
    
    // 3. 设置末端连杆长度（如果有的话）
    my_arm.setEndLinkLength(0.2f);  // 吸盘臂长0.2米

    // 4. 设置目标关节角度（方式一：通过设置目标位置，逆解会自动计算关节角度）
    Arm_Point_S target;
    target.x = 1.2f;                // 目标x坐标 1.2米
    target.y = 0.8f;                // 目标y坐标 0.8米  
    target.z = 0.5f;                // 目标z坐标 0.5米
    target.suckerJoint_status_ = 30.0f;  // 末端关节30度
    
    my_arm.setArmTarget(target);
    
    // 5. 更新机械臂状态（这会触发逆运动学计算）
    my_arm.update();

    // 6. 计算正运动学验证位置
    Arm_Point_S calculated_position;
    if (my_arm.forwardKinematics(calculated_position)) 
    {
        std::cout << "正运动学计算结果:" << std::endl;
        std::cout << "X: " << calculated_position.x << " 米" << std::endl;
        std::cout << "Y: " << calculated_position.y << " 米" << std::endl; 
        std::cout << "Z: " << calculated_position.z << " 米" << std::endl;
        std::cout << "末端关节角度: " << calculated_position.suckerJoint_status_ << " 度" << std::endl;
    }
}
    */