#include <Arduino.h>

const unsigned int BUZZ = 26; // Check schematic and see pin connection to buzzer
const unsigned int BUZZ_CHANNEL = 0; //Selecting PWM channel 0

const unsigned int octave = 5;

void setup() {
  // Stop the right motor by setting pin 14 low
  // this pin floats high or is pulled
  // high during the bootloader phase for some reason
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);

  ledcAttachPin(BUZZ, BUZZ_CHANNEL);
}

void loop() {

/*
  ledcWriteNote(BUZZ_CHANNEL, NOTE_C, 8);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, octave);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_E, octave);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_F, octave);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_G, octave);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_A, octave);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_B, octave);
  delay(500);
*/
/*
  ledcWriteNote(BUZZ_CHANNEL, NOTE_F, octave);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_E, octave);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, octave);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_F, octave);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_E, octave);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, octave);
  delay(500);
  // next part
  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, octave);
  delay(250);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, octave);
  delay(250);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, octave);
  delay(250);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, octave);
  delay(250);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_E, octave);
  delay(250);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_E, octave);
  delay(250);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_E, octave);
  delay(250);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_E, octave);
  delay(250);
// next part
  ledcWriteNote(BUZZ_CHANNEL, NOTE_F, octave);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_E, octave);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, octave);
*/
// u8x8   write stuff on the screen

  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, 7);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_F, 7);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, 8);
  delay(2000);

  ledcWriteNote(BUZZ_CHANNEL, NOTE_A, 14);
  delay(1000);

  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, 7);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_F, 7);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, 8);
  delay(2000);


  ledcWriteNote(BUZZ_CHANNEL, NOTE_E, 8);
  delay(1500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_F, 8);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_E, 8);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_F, 8);
  delay(500);

  ledcWriteNote(BUZZ_CHANNEL, NOTE_E, 8);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_C, 8);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_A, 8);
  delay(2000);



  ledcWriteNote(BUZZ_CHANNEL, NOTE_A, 8);
  delay(1000);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, 7);
  delay(1000);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_F, 7);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_G, 7);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_A, 8);
  delay(3000);


  ledcWriteNote(BUZZ_CHANNEL, NOTE_A, 8);
  delay(1000);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, 7);
  delay(1000);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_F, 7);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_G, 7);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_E, 7);
  delay(3000);



// second run through



  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, 7);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_F, 7);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, 8);
  delay(2000);

  ledcWriteNote(BUZZ_CHANNEL, NOTE_A, 14);
  delay(1000);

  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, 7);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_F, 7);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, 8);
  delay(2000);


  ledcWriteNote(BUZZ_CHANNEL, NOTE_E, 8);
  delay(1500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_F, 8);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_E, 8);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_F, 8);
  delay(500);

  ledcWriteNote(BUZZ_CHANNEL, NOTE_E, 8);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_C, 8);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_A, 8);
  delay(2000);



  ledcWriteNote(BUZZ_CHANNEL, NOTE_A, 8);
  delay(1000);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, 7);
  delay(1000);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_F, 7);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_G, 7);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_A, 8);
  delay(2000);


  ledcWriteNote(BUZZ_CHANNEL, NOTE_A, 8);
  delay(1000);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_A, 8);
  delay(1000);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, 7);
  delay(6000);

  delay(100000000000);


}
