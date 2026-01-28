#include "FSM_Controller.h"



void FSM_Controller::loop()
{
    if(!init_flag_)
        return;

    CrsfReceiver::GetInstance(&huart7)->process();

    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);


    switch(airjoy_data_.SWB)
    {
        case 0x00:
            robot_status_ = ALL_STOP;
            break;

        case 0x01:
            robot_status_ = MANUAL_CONTROL;
            break;  

        case 0x02:
            robot_status_ = AUTO_CONTROL;
            break;
    }

    if(arm_setup_->isArmcalibrated() == false || weaponSage_setup_->isWeaponSageCalibrated() == false)
    {
        robot_status_ = ALL_STOP;
    }

   switch (robot_status_)
   {
    case ALL_STOP:
        all_stop();
        break;

    case MANUAL_CONTROL:
        // 手动控制逻辑实现
        manual_ctrl();
        break;

    case AUTO_CONTROL:
        // 自动控制逻辑实现
        auto_ctrl();
        break;
    case DEBUG_MODE:
        // 调试模式逻辑实现
        debug();
        break;

    default:
        break;
   }



   if(airjoy_data_.SWA ==0x01 && airjoy_data_.SWC==0x00)
   {
	   static uint8_t iiii = 0;
	   
        // if(airjoy_data_.SWA == 0x01)
        // { 
            //重定位
        if(airjoy_data_.botton_click ==1 && iiii == 0)
        {
            Locate_Setup::getInstance()->Relocte_ToLader();
            
            iiii++;
        }
            
        else
        {
            Locate_Setup::getInstance()->set_startToLRL(false);
			iiii = 0;
        }
    }
    else
    {
        Locate_Setup::getInstance()->set_startToLRL(false);
    }

   last_robot_status_ = robot_status_;
}

void FSM_Controller::all_stop()
{
   // 停止所有机构动作的实现


    if(arm_setup_->isArmcalibrated() == true)
        arm_setup_->setArmStatus(ARM_STOP);
    else
        arm_setup_->setArmStatus(ARM_CALIBRATE);

    if(weaponSage_setup_->isWeaponSageCalibrated() == true)
        weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_STOP);
    else
        weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_CALIBRATE);
       
    chassis_setup_->setChassisStatus(CHASSIS_STOP);
}

void FSM_Controller::manual_ctrl()
{
    switch(airjoy_data_.SWC)
    {
        case 0x00:
        {
            chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_A);
            arm_setup_->setArmStatus(ARM_IDLE);
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);

            break;
        }
        case 0x01:
        {
            chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_B);
            arm_setup_->setArmStatus(ARM_MANUAL_CONTROL);
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);
            break;  
        }
        case 0x02:
        {
            chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_B);
            arm_setup_->setArmStatus(ARM_IDLE);
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_MANUAL_CONTROL);
            break;
        }
    }
}


void FSM_Controller::auto_ctrl()
{
    // 半自动控制模式下的实现
    // arm_setup_->setArmStatus(ARM_AUTO_CONTROL);
    // chassis_setup_->setChassisStatus(CHASSIS_AUTO_CONTROL);

    switch(airjoy_data_.SWC)
    {
        //无操作，进入底盘手操模式
        case 0x00:
        {
            chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_A);
            arm_setup_->setArmStatus(ARM_IDLE);
            break;
        }

        //arm进入自动模式，底盘进入锁定模式
        case 0x01:
        {
            //暂时不把路径规划部分纳入
            chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_B);

            arm_setup_->setArmStatus(ARM_IDLE);
            break;
        }

        //weaponSage进入自动模式
        case 0x02:
        {
            //weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_AUTO_CONTROL);
            chassis_setup_->setChassisStatus(CHASSIS_AUTO_CONTROL);
            arm_setup_->setArmStatus(ARM_IDLE);
            break;
        }
    }
    //arm_setup_->setArmStatus(ARM_AUTO_CONTROL);
}


void FSM_Controller::debug()
{
   // 调试模式下的实现

    // arm_setup_->setArmStatus(ARM_AUTO_CONTROL);
}

