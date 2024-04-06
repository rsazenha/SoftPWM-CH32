#include <Arduino.h>
#include <ch32v003fun.h>

void main(){
    // Needed to SoftPWM configure TIMER
    SoftPWMInitialize();

    // Configure pin as SoftPWM
    SoftPWMConfigurePIN(PA1);

    unsigned uint8_t Value = 0;

    while(true){
        // Define a value for pin
        SoftPWMWrite(PA1, Value);
        Value++;
        delay(100);
    }
}