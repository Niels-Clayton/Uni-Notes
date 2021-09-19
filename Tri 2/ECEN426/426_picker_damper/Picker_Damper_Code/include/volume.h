#ifndef PICK_DAMP_VOL
#define PICK_DAMP_VOL

#include <Arduino.h>
#include <Servo.h>

// Volume control servo definitions
Servo Volume;               // Servo class to control the volume servo
const uint8_t vol_max = 40; // Max rotation of the servo
const uint8_t vol_min = 0;  // Min rotation of the servo

// Enum to map the 9 volume levels linearly to keys
enum volume_lvls
{
  vol_0 = 0,
  vol_1 = (vol_max / 8) * 1,
  vol_2 = (vol_max / 8) * 2,
  vol_3 = (vol_max / 8) * 3,
  vol_4 = (vol_max / 8) * 4,
  vol_5 = (vol_max / 8) * 5,
  vol_6 = (vol_max / 8) * 6,
  vol_7 = (vol_max / 8) * 7,
  vol_8 = vol_max
};

// Setup the volume control servo
void init_volume(uint8_t vol_pin)
{
  // Attach the volume control pin to the servo class
  Volume.attach(vol_pin);
}

// Change the current volume level
void set_volume(uint8_t servo_pos)
{
  // Check if the new value is within the specified range
  if (servo_pos > vol_max || servo_pos < vol_min)
  {
    Serial.print("Invalid volume given\n");
    return;
  }

  // Move the servo to the next position
  Volume.write(servo_pos);
  // delay(500); //optional delay to ensure motion complete
}

#endif //PICK_DAMP_VOL