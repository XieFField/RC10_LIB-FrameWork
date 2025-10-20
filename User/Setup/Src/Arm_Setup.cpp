#include "Arm_setup.h"

void ArmSetup::loop()
{
    if(!init_flag)
        return;
    
    static uint64_t last_us = 0;
    uint64_t now_us = TimeStamp::getInstance().getMicroseconds();
    if(last_us == 0) 
    { 
        last_us = now_us; 
        return; 
    }
    uint64_t dt_us = (now_us >= last_us) ? (now_us - last_us) : 0;
    last_us = now_us;
    if(dt_us == 0) 
        return;
    if(dt_us > 200000) 
        dt_us = 200000; 
    float dt = dt_us * 1e-6f;

    
}


