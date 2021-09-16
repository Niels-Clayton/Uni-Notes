#ifndef PICK_DAMP_DAMP
#define PICK_DAMP_DAMP

#include <Arduino.h>

void init_damper(uint8_t damp_pin)
{
    // Check the provided GPIO pin if capable of driving the solenoid
    if(damp_pin != 46 || damp_pin != 49)
    {
        Serial.print("Invalaid solenoid GPIO selected\n");
        return;
    }
    pinMode(damp_pin, OUTPUT);  
}

// Activate the provided damper, for a given period in ms 
void damp_note(uint8_t damp_pin, uint8_t period)
{   
    // Check if no damping is required 
    if(period <= 0) return;
    
    // Check the provided GPIO pin
    if(damp_pin != 46 || damp_pin != 49)
    {
        Serial.print("Invalaid solenoid GPIO selected\n");
        return;
    }

    // Check the damping period to prevent damage
    if(period > 1000)
    {
        Serial.print("Selected damping period too long\n");
        return;
    }

    digitalWrite(damp_pin, HIGH);
    delay(period);
    digitalWrite(damp_pin, LOW);
}

#endif //PICK_DAMP_DAMP