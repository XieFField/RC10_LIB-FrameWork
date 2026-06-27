#ifndef __BAP_CANFRAME_H
#define __BAP_CANFRAME_H

#include<cstdint>

    struct CanFrame
    {
        /* data */
        uint32_t ID;
        bool isextended;
        uint8_t DLC;        //Data Length Code
        uint8_t data[8];    //Data
    };

#endif /* __BAP_CANFRAME_H */