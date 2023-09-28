#include <Arduino.h>
#include <Encoder.h>
#include <Adafruit_MCP3008.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//line sensor stuff
Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

Adafruit_MPU6050 mpu;

const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;

int adc1_buf[8];
int adc2_buf[8];

int lines[13];

int last_turn = 0;
int y_count = 0;

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
float base_speed = 85;

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

void setLeftMotor(float value){
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

void setRightMotor(float value){
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


void rotate(float target_degrees){
  //convert target degrees to radians
  target_degrees *= 0.01745; //degrees*pi/180
  //radians rotated
  float r = 0;
  
  //accuracy of rotation
  float acc = 0.02;

  //motor speed 0-255
  float speed = 100;

  //time step in milliseconds
  unsigned int step = 5;
  //time step in seconds
  float step_time = (float) step/1000.0;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  while (r-target_degrees > acc || r-target_degrees < -acc){
    if (target_degrees > r){
      setRightMotor(speed);
      setLeftMotor(-speed);
    }
    if (target_degrees < r){
      //rotate 
      setRightMotor(-speed);
      setLeftMotor(speed);
    }
    delay(step);
    mpu.getEvent(&a, &g, &temp);
    delay(step);
    r += g.gyro.z * step_time*2;
  }
}

bool turn_aval(){
  if ((540 < lines[1] < 650) || (540<(lines[12])<650)){
    return true;
  }
}

bool yellow(){
  for(int i; i<13; i++){
    if (lines[i]<540){
      return true;
    }
  } 
}

bool deadend(){
  int t;
  for(int i; i<13; i++){
    if(lines[i]>680){
      t++;
    }
  }
  if(t==13){
    return true;
  }
}

bool T_inter(){
  int t;
  for(int i;i<13;i++){
    if (lines[i] > 650){
      t++;
    }
    if(t==13){
      return true;
    }
    
  }
}

void turn(){
  if(540<lines[0]<650){
    rotate(90);
    last_turn = 2;
  }
  if(540<lines[12],650){
    rotate(-90);
    last_turn = 1;
  }
}

void turn_all(){
  rotate(180);
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

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  detectLinePosition();
  //Serial.print(adc2.readADC(1));
  //Serial.println();
  //delay(1000);
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
/*
  if(turn_aval()){
    if(T_inter()){
      if(last_turn==0){
        Serial.print(y_count);
        //switch state
      }
      else{
        if(last_turn==1){
          rotate(90);
        }
        else{
          rotate(-90);
        }
        last_turn = 0;
      }
    }
    else{
      turn();
      if(yellow()){
        turn_all();
        y_count++;
      }
      if(deadend()){
        turn_all();
      }
    }
  }

*/





}