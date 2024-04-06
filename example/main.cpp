#include <ch32v003fun.h>
#include "../src/SoftPWM-CH32.h"

int main()
{
    SystemInit();

    // Needed to SoftPWM configure TIMER
    SoftPWMInitialize();

    // Configure pin as SoftPWM
    SoftPWMConfigurePIN(PA1);

    uint8_t Value = 0;

    while (true)
    {
        // Define a value for pin
        SoftPWMWrite(PA1, Value);
        Value++;
        Delay_Ms(10);
    }
}