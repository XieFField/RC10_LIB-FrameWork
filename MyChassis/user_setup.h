#ifndef __USER_SETUP_H_
#define __USER_SETUP_H_


class MyChassisController : public RtosTask
{
public:
    MyChassisController() : RtosTask("ChassisCtrl", 10) {} // 10ms loop

protected:
    void init();
    void loop() override;
 
};

#endif