//libraries

#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>



/*Motor-1 is the right motor and Motor-2 is the left motor*/
//===================================Motor-pins-declaration==========================================
//motor-1
#define A1 2
#define B1 7
#define address1=0x02 >> 1 //Default slave address of this drive is 0x10. Arduino works on 7-bit I2C Addressing, so the Address value is shifted to right by 1 bit.

//motor-1
#define A2 3
#define B2 6
#define address2=0x04 >> 1

int max_speed=255;

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

float motor_dis=10.00;
float wheel_rad=3.5;


float vt_1 = 0.00;
float vt_2 = 0.00;




void setup() {
  Serial.begin(115200);
//======================================= i2c setup===============================================
//motor1
  Wire.begin();
  delay(100);
  Wire.beginTransmission(address1);
  Wire.beginTransmission(address2);
  Wire.write(0);
  Wire.write(max_speed); // LSB Byte of Maximum Speed Value
  Wire.write(0); // MSB Byte of Maximum Speed Value
  Wire.endTransmission();
  delay(100);,
  
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
  vt_1 = (( (2.00*v_teleop) + (w_teleop)*motor_dis)/(2.00*wheel_rad) );
  vt_2 = (( (2.00*v_teleop) - (w_teleop)*motor_dis)/(2.00*wheel_rad) );

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
  
  // Set the motor speed and direction
  int dir_1 = 1;
  if (u_1<0){
    dir_1 = -1;
  }

  int dir_2 = 1;
  if (u_2<0){
    dir_2 = -1;
  }
  
  int pwr_1 = (int) fabs(u_1);
  if(pwr_1 > 255){
    pwr_1 = 255;
  }

  int pwr_2 = (int) fabs(u_2);
  if(pwr_2 > 255){
    pwr_2 = 255;
  }
  setMotor(dir_1,pwr_1,address1);
  setMotor(dir_2,pwr_2,address1);
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

void setMotor(int dir, int pwr, int address){
 
  if(dir == 1){ 
  Wire.beginTransmission(address);  //send the slave address
  Wire.write(1); //send the command variable for speed
  Wire.write(pwr); //LSB Byte of Speed to be set
  Wire.write(0); //MSB Byte of Speed to be set
  Wire.endTransmission(); //send I2C stop 
  }
  else if(dir == -1){
    Wire.beginTransmission(address);  //send the slave address
  Wire.write(1); //send the command variable for speed
  Wire.write(1); //LSB Byte of Speed to be set
  Wire.write(pwr); //MSB Byte of Speed to be set
  Wire.endTransmission(); //send I2C stop 
  }
  else{
    Wire.beginTransmission(address);  //send the slave address
  Wire.write(1); //send the command variable for speed
  Wire.write(0); //LSB Byte of Speed to be set
  Wire.write(0); //MSB Byte of Speed to be set
  Wire.endTransmission(); //send I2C stop 
    
  }
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
  // Read encoder B when ENCA rises
  int b2 = digitalRead(B2);
  int increment_2 = 0;
  if(b2>0){
    // If B is high, increment forward
    increment_2 = 1;
  }
  else{
    // Otherwise, increment backward
    increment_2 = -1;
  }
  posi_2 = posi_2 + increment_2;

}
