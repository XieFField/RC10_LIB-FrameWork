#include "Motor_DJI.h"

//M3508 and M2006
     uint32_t send_idLow(){return 0x200;}
     uint32_t send_idHigh(){return 0x1FF;}

//GM6020
     uint32_t send_idLow6020() {return 0x1FE;}
     uint32_t send_idHigh6020() {return 0x2FE;}

DJI_Motor::DJI_Motor(DJI_MotorType type, uint32_t id, fdCANbus *bus)
    : Motor_Base(id, false, bus), type_(type)//, motor_id_(id)
    {}

void DJI_Motor::updateFeedback(const CanFrame &cf)
{
    // 解析反馈帧
//    if(cf.DLC != 8)   //实测，大疆的CAN包我没收到DLC。。。
//        return; // 非法帧长

    const uint8_t* data = cf.data;
    // 解析通用部分
    
    uint16_t encoder_raw = (data[0] << 8) | data[1];
    int16_t rpm_raw = (data[2] << 8) | data[3];
    int16_t current_raw = (data[4] << 8) | data[5];
    int8_t temperature_raw = data[6];
    int8_t null = data[7]; // 保留字节，未使用

    this->encoder_raw_ = encoder_raw;
    // this->angle_ = Encoder_to_angle(encoder_raw);
    encoder_.update(encoder_raw);
    //静态类型转化
    this->rpm_ = static_cast<float>(rpm_raw) /  get_GearRatio();
    this->current_ = static_cast<float>(current_raw);
    this->temperature_ = static_cast<float>(temperature_raw);

    this->totalAngle_ = encoder_.getTotalAngle() / get_GearRatio();
    this->angle_ = fmodf(this->totalAngle_, 360.0f);
    if(this->angle_ < 0) 
        this->angle_ += 360.0f;
    
}

float DJI_Motor::virtualCurrent_to_realCurrent(int16_t virtualCurrent)
{
    switch(type_)
    {
        case M3508_Type:
            return static_cast<float>(virtualCurrent) / 16384.0f * 20000.0f; // 20000mA满量程
        case M2006_Type:
            return static_cast<float>(virtualCurrent) / 10000.0f * 10000.0f; // 10000mA满量程
        case GM6020_Type:
            return static_cast<float>(virtualCurrent) / 16384.0f * 3000.0f;  //3000mA满量程
        default:
            return 0.0f;
    }
}

int16_t DJI_Motor::realCurrent_to_virtualCurrent(float realCurrent)
{
    switch(type_)
    {
        case M3508_Type:
            return static_cast<int16_t>(realCurrent / 20000.0f * 16384.0f); // 20A满量程
        case M2006_Type:
            return static_cast<int16_t>(realCurrent / 10000.0f * 10000.0f); // 10A满量程
        case GM6020_Type:
            return static_cast<int16_t>(realCurrent / 3000.0f * 16384.0f);  //3A满量程
        default:
            return 0;
    }
}
    
//====DJI_Group 负责打包4电机合帧====

DJI_Group::DJI_Group(uint32_t baseTxId, fdCANbus* bus)
    : Motor_Base(baseTxId, false, bus), baseTxID_(baseTxId)
    {

    }


bool DJI_Group::addMotor(DJI_Motor* motor)
{
    if(!motor)
        return false;

    if(motor_count_ >= MAX_GROUP_SIZE)
        return false; // 超过最大数量

    if(motor->getID() < 1 || motor->getID() > 8)
        return false; // 不合法ID范围

    DJI_MotorType type = motor->getType();
    uint32_t mid = motor->getID();

    if(type == GM6020_Type)
    {
        if(baseTxID_ != send_idLow6020() && baseTxID_ != send_idHigh6020())
            return false; 
        
        if(baseTxID_ == send_idLow6020())
        {
            if(!(mid >= 1 && mid <=4))
                return false; // 低片只能加1~4号电机
        }

        if(baseTxID_ == send_idHigh6020())
        {
            if(!(mid >=5 && mid <=7))
                return false; // 高片只能加5~7号电机
        }
    }
    else if(type == M3508_Type || type == M2006_Type)
    {
        if(baseTxID_ != send_idLow() && baseTxID_ != send_idHigh())
            return false; 
        
        if(baseTxID_ == send_idLow())
        {
            if(!(mid >= 1 && mid <=4))
                return false; // 低片只能加1~4号电机
        }

        if(baseTxID_ == send_idHigh())
        {
            if(!(mid >=5 && mid <=8))
                return false; // 高片只能加5~8号电机
        }
    }
    else
        return false; // 不支持的型号
    
    
    if(motor_count_ == 0)
    {
        if(type == GM6020_Type)
            containsGM6020 = true;
        else
            containsGM6020 = false;
    }
    else 
    {
        if(type != GM6020_Type && containsGM6020)
            return false; // 不能在同一片上混用GM6020和其他型号
        else if(type == GM6020_Type && !containsGM6020)
            return false; // 不能在同一片上混用GM6020和其他型号
    }

    int slot = calcSlot(mid, type);
    if(slot < 0 || slot >= static_cast<int>(MAX_GROUP_SIZE))
        return false;

    if(motors_p[slot] != nullptr)
        return false; //被占用

    motors_p[slot] = motor;
    motor_count_++;
    return true;
}

int DJI_Group::calcSlot(uint32_t mid, DJI_MotorType type)const
{
    if(type == GM6020_Type) 
    {
        if(baseTxID_ == send_idLow6020()) 
        { // 1~4
            if(mid >=1 && mid <=4) return static_cast<int>(mid) - 1;
        } 
        else if(baseTxID_ == send_idHigh6020()) 
        { // 5~7 (占前3槽位)
            if(mid >=5 && mid <=7) return static_cast<int>(mid) - 5;
        }
        return -1;
    } 
    else 
    { // M3508 / M2006
        if(baseTxID_ == send_idLow()) 
        {        // 1~4
            if(mid >=1 && mid <=4) return static_cast<int>(mid) - 1;
        } 
        else if(baseTxID_ == send_idHigh()) 
        { // 5~8
            if(mid >=5 && mid <=8) return static_cast<int>(mid) - 5;
        }
        return -1;
    }
}

std::size_t DJI_Group::packCommand(CanFrame outFrames[], std::size_t maxFrames)
{
    if(maxFrames <1 || motor_count_ == 0)
        return 0; // 无法打包
    // 初始化帧
    CanFrame &f = outFrames[0];
    f.ID = baseTxID_;
    f.isextended = false;
    f.DLC = 8;
    std::memset(f.data, 0, 8);
    for(std::size_t i = 0; i < 4; ++i)
    { 
        DJI_Motor* m = motors_p[i];
        if(!m)
            continue;
        int16_t current = m->realCurrent_to_virtualCurrent(m->getTargetCurrent());
        f.data[i*2] = (current >> 8) & 0xFF;
        f.data[i*2 + 1] = current & 0xFF;
    }
    return 1;
}
/*======================= M3508 =======================*/

M3508::M3508(uint32_t motor_id, fdCANbus* bus)
    : DJI_Motor(M3508_Type, motor_id, bus) 
{
    
}

float M3508::getAngle() const
{
    return angle_;
}

float M3508::getTotalAngle() const
{
    return totalAngle_;
}

float M3508::getRPM() const
{
    return rpm_;
}

void M3508::pid_init(const PID_Param_Config& speed_params, float speed_tdRatio, const PID_Param_Config& angle_params, float angle_I_Separa)
{
    speed_pid_.set_params(speed_params, speed_tdRatio);
    angle_pid_.set_params(angle_params, angle_I_Separa);
}

void M3508::setTargetCurrent(float current_set)
{
    mode_ = CURRENT_CONTROL;
    target_current_ = current_set;
}

void M3508::setTargetRPM(float rpm_set)
{
    mode_ = SPEED_CONTROL;
    target_rpm_ = rpm_set;
}

void M3508::setTargetAngle(float angle_set)
{
    mode_ = ANGLE_CONTROL;
    target_angle_ = angle_set;
    angle_pid_.set_as_circular(); //最小路径处理
}


void M3508::setTargetTotalAngle(float totalAngle_set)
{
    mode_ = TOTAL_ANGLE_CONTROL;
    target_totalAngle_ = totalAngle_set;
    angle_pid_.set_as_linear(); //线性处理
}

//volatile float cur = 0;

void M3508::update()
{
    switch(mode_)
    {
        case TOTAL_ANGLE_CONTROL:
        {
            anglePid_timeCnt++;
            if(anglePid_timeCnt >= anglePid_timePSC)
            {
                float expected_rpm = angle_pid_.pid_calc(target_totalAngle_, getTotalAngle());
                target_rpm_ = expected_rpm;
                anglePid_timeCnt = 0;
            }
            target_current_ = speed_pid_.pid_calc(target_rpm_, this->rpm_);
            //cur = target_current_;
            break;
        }
        //注意：此处不break，继续执行速度环计算
        case SPEED_CONTROL:
        {
            // 目标值 target_rpm_ 和反馈值 this->rpm_ 都已经是输出轴转速，尺度统一
            target_current_ = speed_pid_.pid_calc(target_rpm_, this->rpm_);
            //cur = target_current_;
            break;
        }

        case ANGLE_CONTROL:
        {
            anglePid_timeCnt++;
            if(anglePid_timeCnt >= anglePid_timePSC)
            {
                float expected_rpm = angle_pid_.pid_calc(target_angle_, getAngle());
                target_rpm_ = expected_rpm;
                anglePid_timeCnt = 0;
            }
            target_current_ = speed_pid_.pid_calc(target_rpm_, this->rpm_);
            break;
        }
        
        case CURRENT_CONTROL:
        {
            // 直接使用 target_current_
            break;
        }

        default :
        {
            break;
        }
    }
}

/*------------ M2006 ------------*/

M2006::M2006(uint32_t motor_id, fdCANbus* bus)
    : DJI_Motor(M2006_Type, motor_id, bus)
{
}

float M2006::getAngle() const
{
    return angle_;
}

float M2006::getTotalAngle() const
{
    return totalAngle_;
}

float M2006::getRPM() const
{
    return rpm_;
}

void M2006::pid_init(const PID_Param_Config& speed_params, float speed_tdRatio, const PID_Param_Config& angle_params, float angle_I_Separa)
{
    speed_pid_.set_params(speed_params, speed_tdRatio);
    angle_pid_.set_params(angle_params, angle_I_Separa);
    angle_pid_.set_as_circular(); //最小路径处理
}

void M2006::setTargetCurrent(float current_set)
{
    mode_ = CURRENT_CONTROL;
    target_current_ = current_set;
}

void M2006::setTargetRPM(float rpm_set)
{
    mode_ = SPEED_CONTROL;
    target_rpm_ = rpm_set;
}

void M2006::setTargetAngle(float angle_set)
{
    mode_ = ANGLE_CONTROL;
    target_angle_ = angle_set;
    angle_pid_.set_as_circular(); //最小路径处理
}

void M2006::setTargetTotalAngle(float totalAngle_set)
{
    mode_ = TOTAL_ANGLE_CONTROL;
    target_totalAngle_ = totalAngle_set;
    angle_pid_.set_as_linear(); //线性处理
}

void M2006::update()
{
    switch(mode_)
    {
        case TOTAL_ANGLE_CONTROL:
        {
            anglePid_timeCnt++;
            if(anglePid_timeCnt >= anglePid_timePSC)
            {
                float expected_rpm = angle_pid_.pid_calc(target_totalAngle_, getTotalAngle());
                target_rpm_ = expected_rpm;
                anglePid_timeCnt = 0;
            }
            target_current_ = speed_pid_.pid_calc(target_rpm_, this->rpm_);
            //cur = target_current_;
            break;
            // Fallthrough to speed control
        }

        case SPEED_CONTROL:
        {
            // 目标值 target_rpm_ 和反馈值 this->rpm_ 都已经是输出轴转速，尺度统一
            target_current_ = speed_pid_.pid_calc(target_rpm_, this->rpm_);
            break;
        }

        case ANGLE_CONTROL:
        {
            anglePid_timeCnt++;
            if(anglePid_timeCnt >= anglePid_timePSC)
            {
                float expected_rpm = angle_pid_.pid_calc(target_angle_, getAngle());
                target_rpm_ = expected_rpm;
                anglePid_timeCnt = 0;
            }
            target_current_ = speed_pid_.pid_calc(target_rpm_, this->rpm_);
            break;
        }

        case CURRENT_CONTROL:
        {
            break;
        }

        default :
        {
            break;
        }
    }
}

/*------------GM6020------------*/

GM6020::GM6020(uint32_t motor_id, fdCANbus* bus)
    : DJI_Motor(GM6020_Type, motor_id, bus)
{
}

void GM6020::pid_init(const PID_Param_Config& speed_params, float speed_tdRatio, const PID_Param_Config& angle_params, float angle_I_Separa)
{
    speed_pid_.set_params(speed_params, speed_tdRatio);
    angle_pid_.set_params(angle_params, angle_I_Separa);
}

void GM6020::setTargetCurrent(float current_set)
{
    mode_ = CURRENT_CONTROL;
    target_current_ = current_set;
}

void GM6020::setTargetRPM(float rpm_set)
{
    mode_ = SPEED_CONTROL;
    target_rpm_ = rpm_set;
}

void GM6020::setTargetAngle(float angle_set)
{
    mode_ = ANGLE_CONTROL;
    target_angle_ = angle_set;
    angle_pid_.set_as_circular(); //最小路径处理
}

void GM6020::setTargetTotalAngle(float totalAngle_set)
{
    mode_ = TOTAL_ANGLE_CONTROL;
    target_totalAngle_ = totalAngle_set;
    angle_pid_.set_as_linear(); //线性处理
}

void GM6020::update()
{
    switch(mode_)
    {
        case TOTAL_ANGLE_CONTROL:
        {
            anglePid_timeCnt++;
            if(anglePid_timeCnt >= anglePid_timePSC)
            {

                float expected_rpm = angle_pid_.pid_calc(target_totalAngle_, getTotalAngle());
                target_rpm_ = expected_rpm;
                anglePid_timeCnt = 0;
            }
            target_current_ = speed_pid_.pid_calc(target_rpm_, this->rpm_);
            //cur = target_current_;
            break;
            // Fallthrough to speed control
        }

        case SPEED_CONTROL:
        {
            // GM6020没有减速比，直接使用目标转速
            // 目标值是输出轴转速 (target_rpm_)
            // 反馈值也是输出轴转速 (this->rpm_)
            // 两者尺度统一，PID可以正确工作
            target_current_ = speed_pid_.pid_calc(target_rpm_, this->rpm_);
            break;
        }

        case ANGLE_CONTROL:
        {
            anglePid_timeCnt++;
            if(anglePid_timeCnt >= anglePid_timePSC)
            {

                float expected_rpm = angle_pid_.pid_calc(target_angle_, getAngle());
                target_rpm_ = expected_rpm;
                anglePid_timeCnt = 0;
            }
            target_current_ = speed_pid_.pid_calc(target_rpm_, this->rpm_);
            break;
        }

        case CURRENT_CONTROL:
        {
            break;
        }

        default :
        {
            break;
        }
    }
}
