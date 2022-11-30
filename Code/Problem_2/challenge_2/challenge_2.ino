/*
Connections of Motor Driver and Arduino
Motor       -      Arduino
GND         -      GND
SCL         -      SCL
SDA         -      SDA
*/

#include "Wire.h"
#define I2C_ADDRESS1 0x02 >> 1 //Default slave address of this drive is 0x10. Arduino works on 7-bit I2C Addressing, so the Address value is shifted to right by 1 bit.
#define I2C_ADDRESS2 0x04 >> 1
#define MAX_SPEED 255  //Set Maximum Speed (0-255)
#define CB_0 0 // Command Byte "0" to Read/Write Motor Max Speed 
#define CB_1 1 // Command Byte "1" to Read/Write Motor Speed and Direction
//1
const int A1=2; //encoderA to pin3 and encoderB to pin8
const int B1=8;
volatile long tick1 =0; //this where the bug is present //unsigned
//2
const int A2=3; //encoderA to pin3 and encoderB to pin8
const int B2=9;
volatile long tick2 =0; //this where the bug is present //unsigned


 int Encoder_cpr1 ;
 int Encoder_cpr2 ;
 int distance = ?;
 
void setup(){
  Serial.begin(115200);
  Wire.begin();
  delay(100);
  Wire.beginTransmission(I2C_ADDRESS1);
   Wire.beginTransmission(I2C_ADDRESS2);
  Wire.write(CB_0);
  Wire.write(MAX_SPEED); // LSB Byte of Maximum Speed Value
  Wire.write(0); // MSB Byte of Maximum Speed Value
  Wire.endTransmission();
  delay(100);

  pinMode(A1,INPUT_PULLUP);
  pinMode(B1,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(A1),Funct1,RISING);
  pinMode(A2,INPUT_PULLUP);
  pinMode(B2,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(A2),Funct2,RISING);

    //////////////
  Encoder_cpr1=((distance/44)*7*133600*radius); //wheel 
 // Encoder_cpr2=((distance/44)*7*133600*radius); 
}

void loop(){
    //1
  if( tick1 <= Encoder_cpr1){
  Wire.beginTransmission(I2C_ADDRESS1);  //send the slave address
  Wire.write(CB_1); //send the command variable for speed
  Wire.write(255); //LSB Byte of Speed to be set
  Wire.write(0); //MSB Byte of Speed to be set
  Wire.endTransmission(); //send I2C stop 
  delay(1);
 }else{
  Wire.beginTransmission(I2C_ADDRESS1);  //send the slave address
  Wire.write(CB_1); //send the command variable for speed
  Wire.write(0); //LSB Byte of Speed to be set
  Wire.write(0); //MSB Byte of Speed to be set
  Wire.endTransmission(); //send I2C stop 
  delay(100);}


  //2
  if( tick2 >= Encoder_cpr1){
   Wire.beginTransmission(I2C_ADDRESS2);  //send the slave address
  Wire.write(CB_1); //send the command variable for speed
  Wire.write(255); //LSB Byte of Speed to be set
  Wire.write(0); //MSB Byte of Speed to be set
  Wire.endTransmission(); //send I2C stop 
  delay(100);
 }else{
  Wire.beginTransmission(I2C_ADDRESS2);  //send the slave address
  Wire.write(CB_1); //send the command variable for speed
  Wire.write(0); //LSB Byte of Speed to be set
  Wire.write(0); //MSB Byte of Speed to be set
  Wire.endTransmission(); //send I2C stop 
  delay(100);}
}

//1
void Funct1(){
 int b1 = digitalRead(B1);
  if(b1== 1){
    tick1 =tick1+1;
      }
  else if(b1==0){
   tick1=tick1-1;
   
  }

}

//2
void Funct2(){
 int b2 = digitalRead(B2);
  if(b2== 1){
    tick2 =tick2+1;
      }
  else if(b2==0){
   tick2=tick2-1;
   
  }

}
