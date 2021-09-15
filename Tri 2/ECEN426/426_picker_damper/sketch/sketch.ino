#include <Servo.h>

// Volume control servo definitions
Servo volume;
const uint8_t vol_pin = 14;
const uint8_t vol_max = 40;
const uint8_t vol_min = 0;

enum volumes{vol_0 = 0,             vol_1 = (vol_max/8)*1, vol_2 = (vol_max/8)*2, 
             vol_3 = (vol_max/8)*3, vol_4 = (vol_max/8)*4, vol_5 = (vol_max/8)*5, 
             vol_6 = (vol_max/8)*6, vol_7 = (vol_max/8)*7, vol_8 = vol_max};


// Function definitions
void set_volume(uint8_t servo_pos);


void setup() 
{
  Serial.begin(115200); // Any baud rate should work

  // setup the volume servo
  volume.attach(vol_pin);
}


void loop() 
{

  set_volume(vol_0);
  delay(500);
  set_volume(vol_1);
  delay(500);
  set_volume(vol_2);
  delay(500);  
  set_volume(vol_3);
  delay(500);  
  set_volume(vol_4);
  delay(500);  
  set_volume(vol_5);
  delay(500);  
  set_volume(vol_6);
  delay(500);  
  set_volume(vol_7);
  delay(500);  
  set_volume(vol_8);
  delay(500); 
  
}

void set_volume(uint8_t servo_pos)
{
  if(servo_pos > vol_max || servo_pos < vol_min)
  {
    Serial.print("Invalid volume given\n");
    return;
  }
  volume.write(servo_pos);
}



