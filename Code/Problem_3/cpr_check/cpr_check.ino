
#include <util/atomic.h>
#define A1 2
#define B1 5

#define pwmPin1 9
#define dirPin1 11
int b1;
volatile long posi ;
int cpr=33300;
int x=1;
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
//======================================= motor driver setup ===============================================
  pinMode(dirPin1, OUTPUT);
  pinMode(pwmPin1, OUTPUT);

  pinMode(A1,INPUT);
  pinMode(B1,INPUT_PULLUP);
  
 attachInterrupt(digitalPinToInterrupt(A1),Funct1,RISING);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  
  if(x==1){delay(1000); posi =0;x=2;analogWrite(pwmPin1,0);
  }
  int dir1=0;
 digitalWrite(dirPin1,dir1);analogWrite(pwmPin1,0);
 ////////////////////////////////////
  if (posi < 33300){
  
  analogWrite(pwmPin1,200);
  }else{
  
  analogWrite(pwmPin1,0);
  }
  Serial.print(b1);Serial.print("\t");Serial.println(posi);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Funct1(){
  // Read encoder B when ENCA rises
   b1 = digitalRead(B1);
  int increment_1 = 0;
  if(b1>0){
    // If B is high, increment forward
    increment_1 = 1;
  }
  else{
    // Otherwise, increment backward
    increment_1 = -1;
  }
  posi = posi + increment_1;



}
