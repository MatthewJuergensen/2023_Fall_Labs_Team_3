#include <Arduino.h>
#include <Encoder.h>
#include <Adafruit_MCP3008.h>

const unsigned int M1_ENC_A = 39;
const unsigned int M1_ENC_B = 38;
const unsigned int M2_ENC_A = 37;
const unsigned int M2_ENC_B = 36;

const unsigned int M1_IN_1 = 13;
const unsigned int M1_IN_2 = 12;
const unsigned int M2_IN_1 = 25;
const unsigned int M2_IN_2 = 14;

const unsigned int M1_IN_1_CHANNEL = 0;
const unsigned int M1_IN_2_CHANNEL = 1;
const unsigned int M2_IN_1_CHANNEL = 2;
const unsigned int M2_IN_2_CHANNEL = 3;

const unsigned int PWM_VALUE = 1023; // Do not give max PWM. Robot will move fast

const int freq = 5000;
const int resolution = 10;


long prevT = 0;
float eprev = 0;
float ei = 0;

int base_pwm;
float prevpos = 0;

// PID constants
float kp = 1;
float kd = 0.0;
float ki = 0;


void setrightpwm(float value){
  if (value == 0){
    ledcWrite(M2_IN_1_CHANNEL, 0);
    ledcWrite(M2_IN_2_CHANNEL, 0);
    return;
  }
  if (value > 0){
    if(value > PWM_VALUE){
      value = PWM_VALUE;
    }
    ledcWrite(M2_IN_1_CHANNEL, 0);
    ledcWrite(M2_IN_2_CHANNEL, (int) value);
  }
  if (value < 0){
    if(value < -PWM_VALUE){
      value = -PWM_VALUE;
    }
    ledcWrite(M2_IN_1_CHANNEL, (int) -value);
    ledcWrite(M2_IN_2_CHANNEL, 0);
  }
}

void setleftpwm(float value){
  if (value == 0){
    ledcWrite(M1_IN_1_CHANNEL, 0);
    ledcWrite(M1_IN_2_CHANNEL, 0);
    return;
  }
  if (value > 0){
    if(value > PWM_VALUE){
      value = PWM_VALUE;
    }
    ledcWrite(M1_IN_1_CHANNEL, 0);
    ledcWrite(M1_IN_2_CHANNEL, (int) value);
  }
  if (value < 0){
    if(value < -PWM_VALUE){
      value = -PWM_VALUE;
    }
    ledcWrite(M1_IN_1_CHANNEL, (int) -value);
    ledcWrite(M1_IN_2_CHANNEL, 0);
  }
}

void turn(float angle){
  
}

void setup() {
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);

  ledcSetup(M1_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M1_IN_2_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_2_CHANNEL, freq, resolution);

  ledcAttachPin(M1_IN_1, M1_IN_1_CHANNEL);
  ledcAttachPin(M1_IN_2, M1_IN_2_CHANNEL);
  ledcAttachPin(M2_IN_1, M2_IN_1_CHANNEL);
  ledcAttachPin(M2_IN_2, M2_IN_2_CHANNEL);


  Serial.begin(115200);
}

void loop() {
  Encoder encleft(M1_ENC_A, M1_ENC_B);
  Encoder encright(M2_ENC_A, M2_ENC_B);

  base_pwm = 350;
  

  while(1){
    // long currT = micros();
    // float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    // prevT = currT;

    int r_enc_count = encright.read();
    int l_enc_count = encleft.read();

    int e = l_enc_count + r_enc_count;

    int de = e-eprev;

    // integral
    ei = ei + e;
  
    // control signal
    int u = kp*e + kd*de + ki*ei;

    setleftpwm(base_pwm - u);
    setrightpwm(base_pwm + u);

    // setleftpwm(base_pwm);
    // setrightpwm(base_pwm);
    Serial.print("l_enc:");
    Serial.print(l_enc_count);
    Serial.print("\tr_enc:");
    Serial.print(r_enc_count);
    Serial.print("\tleftpwm");
    Serial.print(base_pwm - u);
    Serial.print("\trightpwm");
    Serial.println(base_pwm + u);

    eprev = e;
  }

}

