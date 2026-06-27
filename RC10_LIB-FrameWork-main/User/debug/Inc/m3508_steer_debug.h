#ifndef M3508_STEER_DEBUG_H
#define M3508_STEER_DEBUG_H

#ifdef __cplusplus
extern "C"{

}
#endif
#ifdef __cplusplus

#include "BSP_RTOS.h"
#include "Motor_DJI.h"
#include "Module_CrsfReceiver.h"
#include "APP_debugTool.h"
#include "APP_tool.h"

class M3508_Steer_Debug : public RtosTask
{
public:
    M3508_Steer_Debug(): RtosTask("M3508_Steer_Debug\0", 1){}
    ~M3508_Steer_Debug() {}

    void init(M3508* steer_motor1, M3508* steer_motor2, M3508* steer_motor3, M3508* steer_motor4)
    {
        this->steer[0] = steer_motor1;
        this->steer[1] = steer_motor2;
        this->steer[2] = steer_motor3;
        this->steer[3] = steer_motor4;

        start(osPriorityNormal, 512);
    }



protected:

private:
    void loop() override;

    RmPocketData_t airjoy_data_;

    int8_t test_index = 0;

    float test_target_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float target_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float k = 350.0f;
    M3508 *steer[4] = {nullptr, nullptr, nullptr, nullptr};

    Debug_Printf debug_uart = Debug_Printf(&huart8);
};

#endif

#endif // M3508_STEER_DEBUG_H