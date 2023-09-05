#include <util/atomic.h>

#define ENCODER_A 2
#define ENCODER_B 3
#define M1 9
#define M2 10

volatile int theta = 0;
float resolution = 1188;

int vel = 0;
int ang = 0;

void setup(){
  Serial.begin(57600);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder, RISING);
}

void loop(){
  float posicion;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    posicion = float(theta*360.0/resolution);
  }

  pwmOut(0);

  Serial.println(posicion*PI/180.0);
}

void encoder(){
  int b = digitalRead(ENCODER_B);
  if(b > 0){
    theta++;
  }
  else{
    theta--;
  }
}

void pwmOut(float out){
  if(out > 0){
    //drive motor CW
    analogWrite(M2, out);
    analogWrite(M1, 0);
  }
  else{
    // drive motor CCW
    analogWrite(M2, 0);
    analogWrite(M1, abs(out));
  }
}
