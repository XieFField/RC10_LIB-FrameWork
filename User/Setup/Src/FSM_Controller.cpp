#include "FSM_Controller.h"

void FSM_Controller::loop()
{
    if(!init_flag_) 
        return;
    CrsfReceiver::GetInstance(&huart7)->send_kfsandSpear(crsf_send_s.rsf_send_data.kfs1, crsf_send_s.rsf_send_data.kfs2, 
																	crsf_send_s.rsf_send_data.Spear);
    CrsfReceiver::GetInstance(&huart7)->process();
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);
    
   
    switch(airjoy_data_.SWB)
    {
        case 0x00:
            robot_status_ = ALL_STOP;
            if(airjoy_data_.SWA == 0x01)
            {
                switch(airjoy_data_.SWC)
                {
                    case 0x00:
                        Stop_set_stauts = RELOCATE;
                        break;
                    case 0x01:
                        Stop_set_stauts = SET_KFS;
                        break;

                    case 0x02:
                        Stop_set_stauts = SET_SPEAR;
                        break;
                }
            }
            else
            {
                Stop_set_stauts = NONE;
            }
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
        // ï¿½Ö¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ß¼ï¿½Êµï¿½ï¿½
        manual_ctrl();
        break;

    case AUTO_CONTROL:
        // ï¿½Ô¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ß¼ï¿½Êµï¿½ï¿½
        auto_ctrl();
        break;
    case DEBUG_MODE:
        // ï¿½ï¿½ï¿½ï¿½Ä£Ê½ï¿½ß¼ï¿½Êµï¿½ï¿½
        debug();
        break;

    default:
        break;
   }



//    if(airjoy_data_.SWA ==0x01 && airjoy_data_.SWC==0x00)
//    {
// 	   static uint8_t iiii = 0;
	   
//         // if(airjoy_data_.SWA == 0x01)
//         // { 
//             //ï¿½Ø¶ï¿½Î»
//         if(airjoy_data_.botton_click ==1 && iiii == 0)
//         {
//             Locate_Setup::getInstance()->Relocte_ToLader();
            
//             iiii++;
//         }
            
//         else
//         {
//             Locate_Setup::getInstance()->set_startToLRL(false);
// 			iiii = 0;
//         }
//     }
//     else
//     {
//         Locate_Setup::getInstance()->set_startToLRL(false);
//     }

// 	if(airjoy_data_.SWA ==0x01 && airjoy_data_.SWC!=0x00)
// 	{
// 		if(airjoy_data_.SWC==0x01) 	
// 				state123 = 1;	
	
// 		if(state123 == 1)
// 		{			
		
// 		if(airjoy_data_.right_x>0.5)
// 		{
// 				count123++;
// 				if(count123>=500)
// 				{target.kfs1++;	
// 					count123=0;
// 				}
// 		}
// 		else if(airjoy_data_.right_x<-0.5)
// 		{
// 			count123++;
// 			if(count123>=500)
// 			{
// 				target.kfs1--;	
// 				count123=0;
// 			}
// 		}
// 		else
// 				count123 = 0;
// 			if(airjoy_data_.scroll_wheel==1)
// 			{
// 				target.kfs1=target.kfs2=0;
// 			}
// 			if(airjoy_data_.botton_click ==1&&state123==1) 
// 					state123=2;
// 		}
// 		if(state123==2)
// 		{
// 			if(airjoy_data_.right_x>0.5)
// 		{
// 			count123++;
// 			if(count123>=500)target.kfs2++;	
// 		}
// 		if(airjoy_data_.right_x<-0.5)
// 		{
// 			count123++;
// 			if(count123>=500)target.kfs2--;
// 		}
// 		if(airjoy_data_.scroll_wheel==1)
// 		{
// 			CrsfReceiver::GetInstance(&huart7)->send_kfsandSpear(0,0,target.Spear);
// 			state123 =1;
// 		}
// 		}
// 		if(airjoy_data_.botton_click ==1&&state123==2)
// 		{
// 			target.kfs1=target.kfs2=0;
// 			state123=0;
// 		}
// 		if(airjoy_data_.SWC==0x02)
// 		{
				
// 		}
//     }
}


void FSM_Controller::all_stop()
{
   // Í£Ö¹Ä£Ê½+Ä¿±êÉèÖÃÄ£Ê½

    if(arm_setup_->isArmcalibrated() == true)
        arm_setup_->setArmStatus(ARM_STOP);
    else
        arm_setup_->setArmStatus(ARM_CALIBRATE);

    if(weaponSage_setup_->isWeaponSageCalibrated() == true)
        weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_STOP);
    else
        weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_CALIBRATE);
       
    chassis_setup_->setChassisStatus(CHASSIS_STOP);
    stop_modeswitch();
}

void FSM_Controller::stop_modeswitch()

{
    switch(Stop_set_stauts)
    {
        case NONE:
        {
            
            crsf_send_s.isread_srollWheelSpear = false;
            break;
        }
            

        case RELOCATE:
        {
            crsf_send_s.isread_srollWheelKFS = false;
            crsf_send_s.isread_srollWheelSpear = false;


            static uint8_t iiii = 0;
        
            if(airjoy_data_.botton_click ==1 && iiii == 0)
            {
                Locate_Setup::getInstance()->Relocte_ToLader();
                
                iiii++;
            }   
            else if(airjoy_data_.botton_click ==0)
            {
                iiii = 0;
            }
            break;
        }

        case SET_KFS:
        {
            crsf_send_s.isread_srollWheelSpear = false;
            if(!crsf_send_s.isread_srollWheelKFS)
            {
                crsf_send_s.sroll_wheel_last = airjoy_data_.scroll_wheel;
                crsf_send_s.isread_srollWheelKFS = true;
            }
            else
            {
                if(airjoy_data_.scroll_wheel != crsf_send_s.sroll_wheel_last)
                {
                    crsf_send_s.kfs_setDone = false;
                    crsf_send_s.isread_srollWheelKFS = false;
                    crsf_send_s.rsf_send_data.kfs1 = 0;
                    crsf_send_s.rsf_send_data.kfs2 = 0;
                    crsf_send_s.now_setKFSindex = 0;
                }
            }

            if(!crsf_send_s.kfs_setDone)
            {
                if(_tool_Abs(airjoy_data_.right_x) > 0.5f)
                {
                    crsf_send_s.count++;
                    if(crsf_send_s.count>= 200 && airjoy_data_.right_x <0.0f)
                    {
                        if(crsf_send_s.now_setKFSindex == 0)
                        {
                            crsf_send_s.rsf_send_data.kfs1++;
                            if(crsf_send_s.rsf_send_data.kfs1>12) crsf_send_s.rsf_send_data.kfs1=12;
                            crsf_send_s.count = 0;
                        }
                        else if(crsf_send_s.now_setKFSindex == 1)
                        {
                            crsf_send_s.rsf_send_data.kfs2++;
                            if(crsf_send_s.rsf_send_data.kfs2>12) crsf_send_s.rsf_send_data.kfs2=12;
                            crsf_send_s.count = 0;
                        }                 
                    }
                    else if(crsf_send_s.count>= 200 && airjoy_data_.right_x > 0.0f)
                    {
                        if(crsf_send_s.now_setKFSindex == 0)
                        {
                            crsf_send_s.rsf_send_data.kfs1--;
                            if(crsf_send_s.rsf_send_data.kfs1<0) crsf_send_s.rsf_send_data.kfs1=0;
                             if(crsf_send_s.rsf_send_data.kfs1>13) crsf_send_s.rsf_send_data.kfs1=0;
                            crsf_send_s.count = 0;
                        }
                        else if(crsf_send_s.now_setKFSindex == 1)
                        {
                            crsf_send_s.rsf_send_data.kfs2--;
                            if(crsf_send_s.rsf_send_data.kfs2<0) crsf_send_s.rsf_send_data.kfs2=0;
                            if(crsf_send_s.rsf_send_data.kfs2>13) crsf_send_s.rsf_send_data.kfs2=0;
                            crsf_send_s.count = 0;
                        }                 
                    }

                }
                else
                {
                    static uint8_t is_click = 0;
                    if(airjoy_data_.botton_click == 1 && is_click == 0)
                    {
                        switch(crsf_send_s.now_setKFSindex)
                        {
                            case 0:
                                target_KFS[0] = crsf_send_s.rsf_send_data.kfs1;
                                crsf_send_s.now_setKFSindex = 1;
                                break;
                            case 1:
                                target_KFS[1] = crsf_send_s.rsf_send_data.kfs2;
                                crsf_send_s.now_setKFSindex = 2;
                                crsf_send_s.kfs_setDone = true;
                                break;
                        }
                        is_click = 1;
                    }
                    else if(airjoy_data_.botton_click == 0)
                    {
                        is_click = 0;
                    }
                    crsf_send_s.count = 0;
                }
            }
            break;
        }

        case SET_SPEAR:
        {
            crsf_send_s.isread_srollWheelKFS = false;
            if(!crsf_send_s.isread_srollWheelSpear)
            {
                crsf_send_s.sroll_wheel_last = airjoy_data_.scroll_wheel;
                crsf_send_s.isread_srollWheelSpear = true;
                crsf_send_s.spear_setDone = false;
            }
            else
            {
                if(airjoy_data_.scroll_wheel != crsf_send_s.sroll_wheel_last)
                {
                    crsf_send_s.spear_setDone = false;
                    crsf_send_s.isread_srollWheelSpear = false;
                    crsf_send_s.rsf_send_data.Spear = 0;
                }
            }

            if(!crsf_send_s.spear_setDone)
            {
                if(_tool_Abs(airjoy_data_.right_x) > 0.5f)
                {
                    crsf_send_s.count++;
                    if(crsf_send_s.count>= 200 && airjoy_data_.right_x <0.0f)
                    {
                        crsf_send_s.rsf_send_data.Spear++;
                        if(crsf_send_s.rsf_send_data.Spear > 4) crsf_send_s.rsf_send_data.Spear = 4;
                        crsf_send_s.count = 0;
                    }
                    else if(crsf_send_s.count>= 200 && airjoy_data_.right_x > 0.0f)
                    {
                        crsf_send_s.rsf_send_data.Spear--;
                        if(crsf_send_s.rsf_send_data.Spear < 0) crsf_send_s.rsf_send_data.Spear = 0;
                        crsf_send_s.count = 0;
                    }

                }
                else
                {
                    static uint8_t is_click = 0;
                    if(airjoy_data_.botton_click == 1 && is_click == 0)
                    {
                        target_spear = crsf_send_s.rsf_send_data.Spear;
                        crsf_send_s.spear_setDone = true;
                        is_click = 1;
                    }
                    else if(airjoy_data_.botton_click == 0)
                    {
                        is_click = 0;
                    }
                    crsf_send_s.count = 0;
                }
            }

            break;
        }
    }
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
    // ×Ô¶¯Ä£Ê½
    // arm_setup_->setArmStatus(ARM_AUTO_CONTROL);
    // chassis_setup_->setChassisStatus(CHASSIS_AUTO_CONTROL);

    switch(airjoy_data_.SWC)
    {
        //µ×ÅÌÊÖ¶¯Ä£Ê½
        case 0x00:
        {
            chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_A);
            arm_setup_->setArmStatus(ARM_IDLE);
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);
            break;
        }

        //arm×Ô¶¯Ä£Ê½
        case 0x01:
        {
            //ÔÝÊ±²»°ÑÂ·¾¶¹æ»®²¿·ÖÄÉÈë
            chassis_setup_->setChassisStatus(CHASSIS_AUTO_CONTROL);

            arm_setup_->setArmStatus(ARM_AUTO_CONTROL);
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);
            break;
        }

        //weaponSage×Ô¶¯Ä£Ê½
        case 0x02:
        {
			weaponSage_setup_->Set_End_Flag(chassis_setup_->GetReach_flag());
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);
            chassis_setup_->setChassisStatus(CHASSIS_AUTO_CONTROL);
            arm_setup_->setArmStatus(ARM_IDLE);
            break;
        }
    }
    //arm_setup_->setArmStatus(ARM_AUTO_CONTROL);
}


void FSM_Controller::debug()
{
   // µ÷ÊÔ

    // arm_setup_->setArmStatus(ARM_AUTO_CONTROL);
}
