#include "volume.h"
#include "picker.h"
#include "damper.h"

const uint8_t VOL_PIN = 14;
const uint8_t DAMP_PIN = 49;

// Function Definitions
void play_note(uint8_t volume, uint32_t sustain_period, uint8_t damper_pin, uint32_t damp_period);

void setup()
{
  Serial.begin(115200); // Any baud rate should work

  // init the volume servo control
  init_volume(VOL_PIN);

  //init picking stepper motor control
  init_picker(8);

  // init the solenoid damper
  init_damper(DAMP_PIN);
}

void loop()
{
  play_note(vol_0, 100, DAMP_PIN, 500);
  delay(500);

  play_note(vol_1, 200, DAMP_PIN, 500);
  delay(500);

  play_note(vol_2, 300, DAMP_PIN, 500);
  delay(500);

  play_note(vol_3, 400, DAMP_PIN, 500);
  delay(500);

  play_note(vol_4, 500, DAMP_PIN, 500);
  delay(500);

  play_note(vol_5, 600, DAMP_PIN, 500);
  delay(500);

  play_note(vol_6, 700, DAMP_PIN, 500);
  delay(500);

  play_note(vol_7, 800, DAMP_PIN, 500);
  delay(500);

  play_note(vol_8, 900, DAMP_PIN, 500);
  delay(500);
}

void play_note(uint8_t volume, uint32_t sustain_period, uint8_t damper_pin, uint32_t damp_period)
{
  set_volume(volume);

  pick_note(sustain_period);

  damp_note(damper_pin, damp_period);
}