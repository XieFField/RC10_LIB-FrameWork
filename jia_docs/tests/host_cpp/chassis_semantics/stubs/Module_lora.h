#ifndef TEST_TDD_MODULE_LORA_H
#define TEST_TDD_MODULE_LORA_H

namespace communication
{
    struct RC10_AirJoy_Data_S
    {
        float left_x = 0.0f;
        float left_y = 0.0f;
        float right_y = 0.0f;
        float right_x = 0.0f;
    };

    class Lora_communication
    {
    public:
        static Lora_communication *GetInstance()
        {
            static Lora_communication instance;
            return &instance;
        }

        void update_airjoy_data(RC10_AirJoy_Data_S *data)
        {
            if (data != nullptr)
            {
                *data = {};
            }
        }
    };
} // namespace communication

#endif
