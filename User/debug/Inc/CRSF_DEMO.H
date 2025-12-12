#include "BSP_RTOS.h"
#include "Module_CrsfReceiver.h"
class crsf_demo : public RtosTask{
public:
    crsf_demo() : RtosTask("crsf_demo", 1) {} 

    void init(CrsfReceiver *crsf) 
    {
			
        crsf_test_ = crsf;
				crsf_test_->init();
        start(osPriorityNormal, 256);
    }
		int a = 0;
    CrsfReceiver *crsf;
private:
    void loop() override;
    CrsfReceiver *crsf_test_;
    RmPocketData_t data;
} ;