#ifndef PICK_DAMP_PICK
#define PICK_DAMP_PICK

#include <FlexyStepper.h>

//Pin out definitions for the A4988
const uint8_t MS1 = 36;
const uint8_t MS2 = 34;
const uint8_t MS3 = 32;
const uint8_t RST = 30;
const uint8_t SLP = 28;
const uint8_t STEP = 26;
const uint8_t DIR = 24;

FlexyStepper Picker;

const float n_picks = 5.0f; // number of picks on wheel

void init_picker(uint8_t u_step)
{
    pinMode(MS1, OUTPUT);
    pinMode(MS2, OUTPUT);
    pinMode(MS3, OUTPUT);
    pinMode(RST, OUTPUT);
    pinMode(SLP, OUTPUT);

    digitalWrite(RST, HIGH);
    digitalWrite(SLP, HIGH);

    // set up micro stepping
    switch (u_step)
    {
    case 1: //full
        digitalWrite(MS1, LOW);
        digitalWrite(MS2, LOW);
        digitalWrite(MS3, LOW);
        break;
    case 2: //half
        digitalWrite(MS1, HIGH);
        digitalWrite(MS2, LOW);
        digitalWrite(MS3, LOW);
        break;
    case 4: //quarter
        digitalWrite(MS1, LOW);
        digitalWrite(MS2, HIGH);
        digitalWrite(MS3, LOW);
        break;
    case 8: //eighth
        digitalWrite(MS1, HIGH);
        digitalWrite(MS2, HIGH);
        digitalWrite(MS3, LOW);
        break;
    case 16: //sixteenth
        digitalWrite(MS1, HIGH);
        digitalWrite(MS2, HIGH);
        digitalWrite(MS3, HIGH);
        break;

    default: //full step
        digitalWrite(MS1, LOW);
        digitalWrite(MS2, LOW);
        digitalWrite(MS3, LOW);
        break;
    }

    Picker.connectToPins(STEP, DIR);                     //attach pins to class
    Picker.setStepsPerRevolution((float)(200 * u_step)); //update steps/rev with u_step

    //To meet required 120pick per minute, rev/s = (120/60)/5 = 0.4rev/s
    Picker.setSpeedInRevolutionsPerSecond(2.0f);
    Picker.setAccelerationInRevolutionsPerSecondPerSecond(5.0f);
}

void pick_note(uint32_t sustain_period)
{
    Picker.moveRelativeInRevolutions(1.0f / n_picks); //blocking motion call to advance pick-wheel by 1 position
    delay(sustain_period);
}

#endif // PICK_DAMP_PICK