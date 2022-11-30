//motor-1
#define A1 3
#define B1 6
#define pwmPin1 10
#define dirPin1 12
int interval =1000 ; 
long pM=0.0;

volatile long posi ;
volatile long x ;
int cpr= 33300 ;
int cpr_meter = 2356;

 int pwm =225;
 float rpm = 0.00;
 float vel=0.00;
 float vel1, vel2, vel3 ,vel21,vel22;
 float rpm2,rpm3 ;
 
 float w=0.00;
 
  float d=0.10;
  float r=0.035; 
  

long prevT=0.00;
int posPrev=0;

//======================================= ===============================================
   
void setup() {
  // put your setup code here, to run once:
Serial.begin(2000000);
//
  pinMode(dirPin1, OUTPUT);
  pinMode(pwmPin1, OUTPUT);

  pinMode(A1,INPUT_PULLUP);
  pinMode(B1,INPUT_PULLUP);
  
 attachInterrupt(digitalPinToInterrupt(A1),Funct1,RISING);
  
}

//======================================= ===============================================

void loop() {

  int pos_1 = 0;
  
  noInterrupts(); // disable interrupts temporarily while reading
  pos_1 = posi;
  
  interrupts(); // turn interrupts back on


  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6; 
  vel = (pos_1 - posPrev)/deltaT; // encoder counnts per second
 
  posPrev = pos_1;
  
  prevT = currT;

  rpm =(float) ((vel * 7.00*60.00)/( 2.00* 22.00* r));  //vel in cps not mps
  vel1 = (float) ((rpm * 2.00* 22.00* r)/( 7.00*60.00));
  //----------------------------------------------------------------------

  
  long cM = millis();
  if (cM - pM > interval) {
    pM = cM;
    rpm2 =   (float) ((x *60.00)/( cpr));
    vel21 =  (float) (x /( cpr_meter));
   x=0;
  vel22 = (float) ((rpm2 * 2.00* 22.00* r)/( 7.00*60.00));
   
  }
  
  //------------------------------------------------------------------------ 
  rpm3= (float)((vel/cpr)*60.0);
  vel3 = (float) ((rpm3 * 2.00* 22.00* r)/( 7.00*60.00));
  //====---------------------------------------------------------------------
  
  int dir1=0;
 
  digitalWrite(dirPin1,dir1);
  analogWrite(pwmPin1,pwm);

 Serial.print("pwm = ");
 Serial.println(pwm);
 
 
 Serial.print("vel = ");
 Serial.print(vel);
 Serial.print("\t");
 Serial.print("rpm = ");
 Serial.print(rpm);
 Serial.print("\t");
 Serial.print("vel1 = ");
 Serial.println(vel1);
 
 
 Serial.print("vel21 = ");
 Serial.print(vel21);
 Serial.print("\t");
 
 Serial.print("vel22 = ");
 Serial.println(vel22);
 Serial.print("\t");
 Serial.print("rpm2 = ");
 Serial.println(rpm2);

 Serial.print("vel3 = ");
 Serial.print(vel3);
 Serial.print("\t");
 Serial.print("rpm3 = ");
 Serial.println(rpm3);
 delay(500);
 }
//======================================= ===============================================

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
  posi = posi + increment_1;

  x = x+ increment_1;

}
