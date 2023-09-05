#include <util/atomic.h>
#include <ros.h>
#include <rospy_tutorials/Floats.h>

#define ENCODER_A       2
#define ENCODER_B       3
#define M1              9
#define M2              10

ros::NodeHandle nh;

double pos = 0, vel = 0, output = 0;
volatile int theta = 0;
volatile long last_pos = 0;
unsigned long timeold = 0.0;
unsigned long now = 0.0;
unsigned long lasttimepub;
float resolution = 1185.36;
float rpm = 0.0;

rospy_tutorials::Floats joint_state;

void set_angle_cb( const rospy_tutorials::Floats& cmd_msg){
  output= cmd_msg.data[0]; 
}

ros::Subscriber<rospy_tutorials::Floats> sub("/joints_to_aurdino", set_angle_cb);
ros::Publisher pub("/joint_states_from_arduino", &joint_state);


void setup() {
  //TCCR1B = TCCR1B & B11111000 | B00000101;
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  Serial.begin(57600);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder, RISING); 
}

void loop() {  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = (float (theta*360.0/resolution));
  }

  now = millis();
  if( (now-timeold) >= 100){    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      rpm = float( (60.0 * 1000.0 * (theta-last_pos)) / ((now - timeold) * resolution) ) ;
      timeold = now;
      last_pos = theta;
    }  
  }

  pwmOut(output);

  if ((now - lasttimepub)> 100)
  {
    joint_state.data_length=2;
    joint_state.data[0]=pos*(3.1416/180.0); //pos;
    joint_state.data[1]=rpm*(3.1416/180.0); //vel;
    pub.publish(&joint_state);
    lasttimepub=now;
  }

  nh.spinOnce();
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

void pwmOut(float out) {                                
  if (out > 0) {
    analogWrite(M2, out);                             // drive motor CW
    analogWrite(M1, 0);
  }
  else {
    analogWrite(M2, 0);
    analogWrite(M1, abs(out));                        // drive motor CCW
  }
}
