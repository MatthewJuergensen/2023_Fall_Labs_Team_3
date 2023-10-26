#include <Arduino.h>

const unsigned int M1_IN_1 = 13;
const unsigned int M1_IN_2 = 12;
const unsigned int M2_IN_1 = 25;
const unsigned int M2_IN_2 = 14;

const unsigned int M1_IN_1_CHANNEL = 0;
const unsigned int M1_IN_2_CHANNEL = 1;
const unsigned int M2_IN_1_CHANNEL = 2;
const unsigned int M2_IN_2_CHANNEL = 3;

const unsigned int M1_I_SENSE = 35;
const unsigned int M2_I_SENSE = 34;

const unsigned int PWM_VALUE = 512; // Do not give max PWM. Robot will move fast

const int freq = 5000;
const int resolution = 8;


void setup() {
  Serial.begin(115200);
  
  ledcSetup(M1_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M1_IN_2_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_2_CHANNEL, freq, resolution);

  ledcAttachPin(M1_IN_1, M1_IN_1_CHANNEL);
  ledcAttachPin(M1_IN_2, M1_IN_2_CHANNEL);
  ledcAttachPin(M2_IN_1, M2_IN_1_CHANNEL);
  ledcAttachPin(M2_IN_2, M2_IN_2_CHANNEL);

  pinMode(M1_I_SENSE, INPUT);
  pinMode(M2_I_SENSE, INPUT);

  //set right motor to stop
  ledcWrite(M2_IN_2_CHANNEL, 0);
}

//positive value = forwards, negative value = backwards, 0 value = brake
void left(float value){
  if (value == 0){
    ledcWrite(M2_IN_1_CHANNEL, 0);
    ledcWrite(M2_IN_2_CHANNEL, 0);
    return;
  }
  if (value > 0){
    if (value > 255.0){
      value = 255.0;
    }
    ledcWrite(M2_IN_1_CHANNEL, 0);
    ledcWrite(M2_IN_2_CHANNEL, (uint32_t) value);
  }
  if (value < 0){
    if (value < -255.0){
      value = -255.0;
    }
    ledcWrite(M2_IN_1_CHANNEL, (uint32_t) -value);
    ledcWrite(M2_IN_2_CHANNEL, 0);
  }
}

void right(float value){
  if (value == 0){
    ledcWrite(M1_IN_1_CHANNEL, 0);
    ledcWrite(M1_IN_2_CHANNEL, 0);
    return;
  }
  if (value > 0){
    if (value > 255.0){
      value = 255.0;
    }
    ledcWrite(M1_IN_1_CHANNEL, 0);
    ledcWrite(M1_IN_2_CHANNEL, (uint32_t) value);
  }
  if (value < 0){
    if (value < -255.0){
      value = -255.0;
    }
    ledcWrite(M1_IN_1_CHANNEL, (uint32_t) -value);
    ledcWrite(M1_IN_2_CHANNEL, 0);
  }
}

void loop() {
  //alternate forwards and backwards
  // left(100);
  // right(100);
  // delay(1000);
  // left(0);
  // right(0);
  // delay(1000);
  // left(-100);
  // right(-100);
  // delay(1000);
  // left(0);
  // right(0);
  // delay(1000);

  //pwm test
  for (int i = 0; i <= 255; i+=15){
    Serial.print("Speed: ");
    Serial.println(i);
    left(i);
    right(i);
    delay(500);
  }
}
