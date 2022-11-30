//libraries

#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

/* deterine the maximum velocity of the motors and max propertionate the target accordingly
for radius=m 0.035 meters and max rpm 200 maximum velocity is 0.73304m/s


Motor-1 is the right motor A and Motor-2 is the left motor* B*/
//===================================Motor-pins-declaration==========================================
//motor-1
#define A1 2
#define B1 7
#define pwmPin1 9
#define dirPin1 11
//motor-1
#define A2 3
#define B2 6
#define pwmPin2 10
#define dirPin2 12

float max_vel=0.73304;

//====================================PID variables========================================================
//time
long prevT = 0; //previou time
//currenttime and deltaT declartion in loop

//position
int posPrev_1 = 0; //previous position
int posPrev_2 = 0;

//velocity
float v_1,v_2 ;
float v1_Prev=0;
float v2_Prev=0;

float rpm_1 =0.0;
float rpm_2= 0.0;
// Use the "volatile" directive for variables used in an interrupt
volatile int posi_1 = 0;
volatile int posi_2 = 0;  

//low pass filter
float v_Filt_1 = 0;
float v_Prev_1 = 0;

float v_Filt_2 = 0;
float v_Prev_2 = 0;

//PI
float eintegral_1 = 0;
float kp_1 = 5;
float ki_1 = 10;
float e_1=0 ;
float u_1=0;

float eintegral_2 = 0;
float kp_2 = 5;
float ki_2 = 10;
float e_2=0 ;
float u_2=0;

//=======================================subscriber===============================================

float v_teleop=0.00;
float w_teleop=0.00;
ros::NodeHandle nh;

void messageCb( const geometry_msgs::Twist& msg){
  
  v_teleop = msg.linear.x;
  w_teleop = msg.angular.z;

}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

//========================================Diffenrtial drive variables==============================

float motor_dis=0.10;
float wheel_rad=0.035; //in meters


float vt_1 = 0.00;
float vt_2 = 0.00;




void setup() {
  Serial.begin(115200);
//======================================= motor driver setup ===============================================
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2 OUTPUT);
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);

  analogWrite(pwmPin1, LOW);
  analogWrite(pwmPin2, LOW);
  
  
  
//=========================================encoder pins setup====================================
  //motor1
  pinMode(A1,INPUT_PULLUP);
  pinMode(B1,INPUT_PULLUP);
  //motor2
  pinMode(A2,INPUT_PULLUP);
  pinMode(B2,INPUT_PULLUP);
  

  attachInterrupt(digitalPinToInterrupt(A1),Funct1,RISING);
  attachInterrupt(digitalPinToInterrupt(A2),Funct2,RISING);
//=========================================================subscriber================================

 nh.initNode();
 nh.subscribe(sub);
 
}



void loop() {

  // read the position and velocity
  int pos_1 = 0;
  int pos_2 = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos_1 = posi_1;
  pos_2 = posi_2;
  interrupts(); // turn interrupts back on

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  v_1 = (pos_1 - posPrev_1)/deltaT;
  v_2 = (pos_2 - posPrev_2)/deltaT;
  posPrev_1 = pos_1;
  posPrev_2 = pos_2;
  prevT = currT;

  /* Convert count/s to RPM
  rpm_1 = v_1/600.0*60.0;
  rpm_2= v_2/600.0*60.0;*/


  // Low-pass filter (25 Hz cutoff)
  v1_Filt = 0.854*v1_Filt + 0.0728*v_1 + 0.0728*v1_Prev;
  v1_Prev = v_1;
  
  v2Filt = 0.854*v2_Filt + 0.0728*v_2 + 0.0728*v2_Prev;
  v2_Prev = v_2;
  /*v1_Filt = 0.854*v1_Filt + 0.0728*rpm_1 + 0.0728*v1_Prev;
  v1_Prev = rpm_1;
  
  v2Filt = 0.854*v2_Filt + 0.0728*v2 + 0.0728*v2_Prev;
  v2_Prev = rpm_2;/*/

  // Set a target
  vt_1 = (( (2.00*v_teleop) + (w_teleop)*motor_dis)/(2.00*wheel_rad) ); //if very less the multipy to get a finite velocity
  vt_2 = (( (2.00*v_teleop) - (w_teleop)*motor_dis)/(2.00*wheel_rad) );

  if(vt_1 > max_vel){
    vt_1 =max_vel ;
    }
   if(vt_2> max_vel){
    vt_2 =max_vel ;
    }


// ==================================Compute the control signal u===================================
 //defining kp and ki
  //motor1
  
  e_1 = vt_1 - v1_Filt;
  eintegral_1 = eintegral_1 + e_1*deltaT;
  u_1 = (kp_1*e_1) + (ki_1*eintegral_1);
  
  //motor2
  e_2= vt_2 - v2_Filt;
  eintegral_2 = eintegral_2 + e_2*deltaT;
  u_2 = (kp_2*e_2) + (ki_2*eintegral_2);
  
// ==========================================Set the motor speed and direction================

  //method1- change the polarity of one motor compared to other motor and encoder function b==0
  
  int dir1 = 0  //0 for forward direction 1 for backward ,motor [A or right or 1]
  if (u_1<0){
    dir1 = 1;
  }

  int dir2 = 0; //motor[ B or left or 2]
  if (u_2<0){
    dir2 = 1;
  }

  int pwm_1 = (int) fabs(u_1);
  if(pwm_1 > 255){
   pwm_1 = 255;
  }

  int pwm_2 = (int) fabs(u_2);
  if(pwm_2 > 255){
    pwm_2 = 255;
  }
  setMotor(dir1,pwmPin1,pwm_1,dirPin1);
  setMotor(dir2,pwmPin2,pwm_2,dirPin2);
  /* //motor1
  digitalWrite(dirPin1,dir1);
  analogWrite(pwmPin1,pwm_1);
 //motor2
  digitalWrite(dirPin2,dir2);
  analogWrite(pwmPin2,pwm_2);*/

  /*/method2- change the direction initilization  of one motor compared to other motor

  int dir1 = 0  //0 for forward direction 1 for backward ,motor [A or right or 1] refernce frame
  if (u_1<0){
    dir1 = 1;
  }

  int dir2 = 1; ////1 for forward direction 0 for backwardmotor[ B or left or 2] reference frame
  if (u_2<0){
    dir2 = 0;
  }

  int pwm_1 = (int) fabs(u_1);
  if(pwm_1 > 255){
   pwm_1 = 255;
  }

  int pwm_2 = (int) fabs(u_2);
  if(pwm_2 > 255){
    pwm_2 = 255;
  }
 //motor1
  digitalWrite(dirPin1,dir1);
  analogWrite(pwmPin1,pwm_1);
 //motor2
  digitalWrite(dirPin2,dir2);
  analogWrite(pwmPin2,pwm_2);

  */
  Serial.print("v1_Filt = ");
  Serial.print(v1_Filt);
  Serial.print("\t ");
  Serial.print("v2_Filt = ");
  Serial.print(v2_Filt);
  Serial.println();
  
 //subscriber
  nh.spinOnce();
}




//++++++++++++++++++++++++++Motor setup+++++++++++++++++++++++++++++++//

void setMotor(int dir, int pwmPin, int pwm, int dirPin){
  
  if(dir == 0){ 
    // Turn one way
    digitalWrite(dirPin,LOW);
    
  }
  else if(dir == 1){
    // Turn the other way
    digitalWrite(dirPin,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(dirPin,-1);  
  }

  analogWrite(pwmPin,pwm); // Motor speed
}



////////////////////encoder_feedback//////////////////////
void Funct1(){
  // Read encoder B when ENCA rises
  int b1 = digitalRead(B1);
  int increment_1 = 0;
  if(b1>0){
    // If B is high, increment forward
    increment_1 = 1;
  }
  else{
    // Otherwise, increment backward
    increment_1 = -1;
  }
  posi_1 = posi_1 + increment_1;


}

void Funct2(){
  // Read encoder B when ENCA rises motor2
  int b2 = digitalRead(B2);
  int increment_2 = 0;
  if(b2>0){         //b2<0
    // If B is high, increment forward
    increment_2 = 1;
  }
  else{
    // Otherwise, increment backward
    increment_2 = -1;
  }
  posi_2 = posi_2 + increment_2;

}
