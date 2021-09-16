#include "volume.h"
#include "damper.h"

const uint8_t vol_pin  = 14;
const uint8_t damp_pin = 49;

// Function Definitions

void play_note(uint8_t volume, uint8_t note_period, uint8_t damper_pin, uint8_t damp_period);

void setup() 
{
  Serial.begin(115200); // Any baud rate should work

  // init the volume servo control
  init_volume(vol_pin);

  // init the solenoid damper
  init_damper(damp_pin);
}


void loop() 
{

  play_note(vol_0, 100, damp_pin, 500);
  delay(500);
  
  play_note(vol_1, 200, damp_pin, 500);
  delay(500);
  
  play_note(vol_2, 300, damp_pin, 500);
  delay(500);  
  
  play_note(vol_3, 400, damp_pin, 500);
  delay(500);  
  
  play_note(vol_4, 500, damp_pin, 500);
  delay(500);  
  
  play_note(vol_5, 600, damp_pin, 500);
  delay(500);  
  
  play_note(vol_6, 700, damp_pin, 500);
  delay(500);  
  
  play_note(vol_7, 800, damp_pin, 500);
  delay(500);  
  
  play_note(vol_8, 900, damp_pin, 500);
  delay(500); 
  
}

void play_note(uint8_t volume, uint8_t note_period, uint8_t damper_pin, uint8_t damp_period)
{
  set_volume(volume);

  // TODO create picker function, should take note period, and delay for that long after picking
  // pick_note(uint8_t note_period);

  damp_note(damper_pin, damp_period);
}
