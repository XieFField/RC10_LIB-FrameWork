#include "main.h"

TestPhotogateState g_test_photogates[4] = {
    {kPHOTOGATE_1_GPIO_Port, kPHOTOGATE_1_Pin, GPIO_PIN_RESET},
    {kPHOTOGATE_2_GPIO_Port, kPHOTOGATE_2_Pin, GPIO_PIN_RESET},
    {kPHOTOGATE_3_GPIO_Port, kPHOTOGATE_3_Pin, GPIO_PIN_RESET},
    {kPHOTOGATE_4_GPIO_Port, kPHOTOGATE_4_Pin, GPIO_PIN_RESET},
};

TestJustFloatCapture g_test_justfloat_capture{};

static float g_test_time_seconds = 0.0f;

GPIO_PinState testHostReadPhotogate(GPIO_TypeDef *port, unsigned short pin)
{
    for (const TestPhotogateState &gate : g_test_photogates)
    {
        if (gate.port == port && gate.pin == pin)
        {
            return gate.state;
        }
    }
    return GPIO_PIN_RESET;
}

void testHostSetPhotogate(GPIO_TypeDef *port, unsigned short pin, GPIO_PinState state)
{
    for (TestPhotogateState &gate : g_test_photogates)
    {
        if (gate.port == port && gate.pin == pin)
        {
            gate.state = state;
            return;
        }
    }
}

void testHostResetPhotogates()
{
    for (TestPhotogateState &gate : g_test_photogates)
    {
        gate.state = GPIO_PIN_RESET;
    }
}

void testHostSetTimeSeconds(float seconds)
{
    g_test_time_seconds = seconds;
}

void testHostAdvanceTimeSeconds(float delta_seconds)
{
    g_test_time_seconds += delta_seconds;
}

void testHostResetTimeSeconds()
{
    g_test_time_seconds = 0.0f;
}

float testHostGetTimeSeconds()
{
    return g_test_time_seconds;
}
