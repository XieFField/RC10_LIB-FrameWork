#include "Robot_Arm.h"
#include <iostream>

Robot_Arm::Robot_Arm(Arm_InitData_S init_Data)
    : init_data_(init_Data)
{
}

void Robot_Arm::update()
{
    /*电机当前的角度转换成关节当前的角度 */

    if(motor_rotate_ != nullptr)
        joint_angle_.rotateJoint_angle_ = MotorTotalAngle_to_rotateAngle(motor_rotate_->getTotalAngle());
    if(motor_stretch_ != nullptr)
        joint_angle_.stretchJoint_angle_ = MotorTotalAngle_to_stretchLength(motor_stretch_->getTotalAngle());
    if(motor_launch_ != nullptr)
        joint_angle_.launchJoint_angle_ = MotorTotalAngle_to_launchHeight(motor_launch_->getTargetTotalAngle());
    if(motor_pitch_ != nullptr)
        joint_angle_.suckerJoint_angle_ = MotorTotalAngle_to_pitchAngle(motor_pitch_->getTotalAngle());


    forwardKinematics(arm_forward_);


    if(control_mode_ == TARGET_POSITION_MODE)
        inverseKinematics(arm_target_);
    else if(control_mode_ == MANUAL_JOINT_SPEED_MODE)
    {
        // 手动关节速度模式下的处理
        jacobianMatrix();
    }
    else if(control_mode_ == MANUAL_MOTOR_POSITION_MODE)
    {
        // 手动电机位置模式下的处理
    }
  
  
    // 机械臂位置更新
    float target_rotateMotorAngle = 0.0f;
    float target_stretchMotorAngle = 0.0f;
    float target_launchMotorAngle = 0.0f;
    float target_pitchMotorAngle = 0.0f;

    target_rotateMotorAngle = rotateAngle_to_MotorTotalAngle(target_joint_angle_.rotateJoint_angle_);
    target_stretchMotorAngle = stretchLength_to_MotorTotalAngle(target_joint_angle_.stretchJoint_angle_);
    target_launchMotorAngle = launchHeight_to_MotorTotalAngle(target_joint_angle_.launchJoint_angle_);
    target_pitchMotorAngle = pitchAngle_to_MotorTotalAngle(target_joint_angle_.suckerJoint_angle_);
    /*暂时不做斜坡处理*/

    if(motor_rotate_ != nullptr)
        motor_rotate_->setTargetTotalAngle(target_rotateMotorAngle);

    if(motor_stretch_ != nullptr)
        motor_stretch_->setTargetTotalAngle(target_stretchMotorAngle);

    if(motor_launch_ != nullptr)
        motor_launch_->setTargetTotalAngle(target_launchMotorAngle);

    if(motor_pitch_ != nullptr)
        motor_pitch_->setTargetTotalAngle(target_pitchMotorAngle);
}

void Robot_Arm::inverseKinematics(Arm_Point_S target_point)
{
    if (std::abs(target_point.x) < 1e-6 && std::abs(target_point.y) < 1e-6) 
        target_joint_angle_.rotateJoint_angle_ = 0.0f; // 处理奇异点
     
    else
        target_joint_angle_.rotateJoint_angle_ = atan2(target_point.y, target_point.x) * 180.0f / PI;
    
    target_joint_angle_.launchJoint_angle_ = target_point.z;
    target_joint_angle_.stretchJoint_angle_ = sqrt(target_point.x * target_point.x + target_point.y * target_point.y) - init_data_.arm_length_;

    /*角度制 */
    target_joint_angle_.rotateJoint_angle_ = atan2(target_point.y, target_point.x) * 180.0f / 3.1415926f;
    target_joint_angle_.suckerJoint_angle_ = target_point.suckerJoint_status_;
}


bool Robot_Arm::forwardKinematics(Arm_Point_S& out) const
{


    /*吸盘位置*/
    // float theta = target_rotate_angle_ * 3.1415926f / 180.0f;
    // float alpha = target_pitch_angle_ * 3.1415926f / 180.0f;

    // float Ltot = init_data_.arm_length_ + target_stretch_length_;
    // float pjx = Ltot * cosf(theta);
    // float pjy = Ltot * sinf(theta);
    // float pjz = target_launch_height_;

    // // Rz(theta) * Ry(alpha) * [Ls,0,0]^T
    // float v_x = init_data_.end_link_length_ * cosf(alpha);
    // float v_y = 0.0f;
    // float v_z = -init_data_.end_link_length_ * sinf(alpha);

    // float vsx = cosf(theta) * v_x - sinf(theta) * v_y;
    // float vsy = sinf(theta) * v_x + cosf(theta) * v_y;
    // float vsz = v_z;

    // out.x = pjx + vsx;
    // out.y = pjy + vsy;
    // out.z = pjz + vsz;
    
    // out.suckerJoint_status_ = target_pitch_angle_;

    /*末端关节位置*/
    float theta = joint_angle_.rotateJoint_angle_ * 3.1415926f / 180.0f;
    float Ltot  = init_data_.arm_length_ + joint_angle_.stretchJoint_angle_;

    out.x = Ltot * cosf(theta);
    out.y = Ltot * sinf(theta);
    out.z = joint_angle_.launchJoint_angle_;

    out.suckerJoint_status_ = joint_angle_.suckerJoint_angle_;
    return true;
}

void Robot_Arm::jacobianMatrix()
{
    float now_time_s = TimeStamp::getInstance().getSeconds();
    if(!time_initialized_)
    {
        last_time_s_ = now_time_s;
        time_initialized_ = true;
        return;
    }

    dt_ = now_time_s - last_time_s_;
    last_time_s_ = now_time_s;

    if(dt_ <= 1e-6 || dt_ >0.1f)
        return; //异常值

    //速度限幅

    float vx= manual_vx_, vy = manual_vy_, vz = manual_vz_;

    float vxy_2 = vx*vx + vy*vy, vmax_xy_2 = vmax_xy_ * vmax_xy_;
    if(vxy_2 > vmax_xy_2)
    {
        float scale = 0;
        arm_sqrt_f32(vxy_2, &scale);
        scale = vmax_xy_ /(scale + 1e-6f);
        vx *= scale;
        vy *= scale;
    }

    constrain(vz, -vmax_z_, vmax_z_);
        
    //构造Jv[3x3], q = [h, d ,theta]
    const float h = joint_angle_.launchJoint_angle_;
    const float d = joint_angle_.stretchJoint_angle_;
    const float theta_deg = rad_to_deg(joint_angle_.rotateJoint_angle_);

    float c = arm_cos_f32(theta_deg);
    float s = arm_sin_f32(theta_deg);

    const float L = init_data_.arm_length_ + d;

    float Jv_data[9] = {
        0.0f,   c, -L*s,
        0.0f,   s,  L*c,
        1.0f, 0.0f, 0.0f
    };

    //阻尼最小二乘 qdot = J^T (J J^T + λ^2 I)^-1 v
    arm_matrix_instance_f32 Jv, JT, S, Sl, Sl_inv, vec_v, vec_tmp, vec_qdot;

    float JT_data[9], S_data[9], Sl_data[9], Sl_inv_data[9];
    float v_data[3] = {vx, vy, vz};
    float tmp_data[3]={0,0,0}, qdot_data[3]={0,0,0};

    arm_mat_init_f32(&Jv, 3, 3, Jv_data);
    arm_mat_init_f32(&JT, 3, 3, JT_data);
    arm_mat_init_f32(&S, 3, 3, S_data);
    arm_mat_init_f32(&Sl, 3, 3, Sl_data);
    arm_mat_init_f32(&Sl_inv, 3, 3, Sl_inv_data);
    arm_mat_init_f32(&vec_v, 3, 1, v_data);
    arm_mat_init_f32(&vec_tmp, 3, 1, tmp_data);
    arm_mat_init_f32(&vec_qdot, 3, 1, qdot_data);


    // JT = Jv^T
    if(arm_mat_trans_f32(&Jv, &JT) != ARM_MATH_SUCCESS)
        return;

    // S = Jv * JT
    if(arm_mat_mult_f32(&Jv, &JT, &S) != ARM_MATH_SUCCESS)
        return;

    //Sl = S + λ^2 I
    const float lambda_2 = jac_lambda_ * jac_lambda_;
    for(uint32_t r=0; r < 3; ++r)
    {
        for(uint32_t c_idx = 0; c_idx < 3; ++c_idx)
            Sl_data[r*3 + c_idx] = S_data[r*3 + c_idx];

        Sl_data[r*3 + r] += lambda_2;
    }

    //Sl_inv = Sl^-1
    if(arm_mat_inverse_f32(&Sl, &Sl_inv) != ARM_MATH_SUCCESS)
        return;

    // tmp = Sl_inv * v
    if(arm_mat_mult_f32(&Sl_inv, &vec_v, &vec_tmp) != ARM_MATH_SUCCESS)
        return;

    //qdot = JT * tmp
    if(arm_mat_mult_f32(&JT, &vec_tmp, &vec_qdot) != ARM_MATH_SUCCESS)
        return;

    //关节速度限幅 
    
    float hdot = qdot_data[0];
    float ddot = qdot_data[1];

    float thetadot_deg = rad_to_deg(qdot_data[2]);

    constrain(hdot, -hdot_max_, hdot_max_);
    constrain(ddot, -ddot_max_, ddot_max_);
    constrain(thetadot_deg, -thetadot_deg_max_, thetadot_deg_max_);
    
    //速度积分更新到目标关节
    target_joint_angle_.launchJoint_angle_ = h + hdot * dt_;
    target_joint_angle_.stretchJoint_angle_ = d + ddot * dt_;
    target_joint_angle_.rotateJoint_angle_ = joint_angle_.rotateJoint_angle_ + thetadot_deg * dt_;

    //位置限幅
    constrain(target_joint_angle_.launchJoint_angle_, 0.0f, init_data_.max_launchHeight_);
    constrain(target_joint_angle_.stretchJoint_angle_, 0.0f, init_data_.max_stretchLength_);
    constrain(target_joint_angle_.rotateJoint_angle_, -180.0f, 180.0f);
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