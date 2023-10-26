#include <Arduino.h>
#include <Encoder.h>
#include <Adafruit_MCP3008.h>

//line sensor stuff
Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;

int adc1_buf[8];
int adc2_buf[8];

int lines[13];

//encoder stuff
const unsigned int M1_ENC_A = 39;
const unsigned int M1_ENC_B = 38;
const unsigned int M2_ENC_A = 37;
const unsigned int M2_ENC_B = 36;

//motor stuff
const unsigned int M1_IN_1 = 13;
const unsigned int M1_IN_2 = 12;
const unsigned int M2_IN_1 = 25;
const unsigned int M2_IN_2 = 14;

const unsigned int M1_IN_1_CHANNEL = 8;
const unsigned int M1_IN_2_CHANNEL = 9;
const unsigned int M2_IN_1_CHANNEL = 10;
const unsigned int M2_IN_2_CHANNEL = 11;

const unsigned int M1_I_SENSE = 35;
const unsigned int M2_I_SENSE = 34;

const unsigned int PWM_VALUE = 512; // Max PWM given 8 bit resolution

const int freq = 5000;
const int resolution = 8;

//line position
int position = 0;

//base speed
float base_speed = 90;

//step speed
float step_speed = 5;

//light = 0, dark = 1
void readADC() {
  for (int i = 0; i < 8; i++) {
    adc1_buf[i] = adc1.readADC(i);
    adc2_buf[i] = adc2.readADC(i);

    if (i<7) {
      //Serial.print((adc1_buf[i]>700)?"0":"1"); Serial.print("\t");
      lines[i*2] = (adc1_buf[i]>700)?1:0;
    }

    if (i<6) {
      //Serial.print((adc2_buf[i]>700)?"0":"1"); Serial.print("\t");
      lines[i*2+1] = (adc2_buf[i]>700)?1:0;
    }
  }
}

void detectLinePosition() {
  readADC();
  int prev = 0;
  int curr = 0;
  int upi = 0;
  for (int i = 0; i<13; i++){
    curr = lines[i];
    if (prev != curr){
      if (curr > 0){
        upi = i;
      }
      else{
        position = (upi+i)/2;
      }
    }
    prev = curr;
  }
  if(curr > 0){
    position = (upi+12)/2;
  }
}

void setup() {
  // Stop the right motor by setting pin 14 low
  // this pin floats high or is pulled
  // high during the bootloader phase for some reason
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);

  Serial.begin(115200);
  //line sensor setup
  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);

  //motor setup
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
}

void loop() {
  detectLinePosition();
  if(position > 6){
    //if line on left side
    //slow left wheel
    ledcWrite(M1_IN_1_CHANNEL, 0);
    ledcWrite(M1_IN_2_CHANNEL, base_speed-(position-6)*step_speed);
    //fast right wheel
    ledcWrite(M2_IN_1_CHANNEL, 0);
    ledcWrite(M2_IN_2_CHANNEL, base_speed+(position-6)*step_speed);
  }
  else{
    //if line on right side
    //slow right wheel
    ledcWrite(M2_IN_1_CHANNEL, 0);
    ledcWrite(M2_IN_2_CHANNEL, base_speed-(6-position)*step_speed);
    //fast left wheel
    ledcWrite(M1_IN_1_CHANNEL, 0);
    ledcWrite(M1_IN_2_CHANNEL, base_speed+(6-position)*step_speed);
  }
}