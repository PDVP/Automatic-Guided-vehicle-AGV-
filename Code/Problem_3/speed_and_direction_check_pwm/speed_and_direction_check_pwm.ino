//motor-1 left 
#define A1 2
#define B1 5
#define pwmPin1 9
#define dirPin1 11
//motor-2 right
#define A2 3
#define B2 6
#define pwmPin2 10
#define dirPin2 12

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
//======================================= motor driver setup ===============================================
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  
}

void loop() {
 int dir1=0;
 int dir2=1;
digitalWrite(dirPin1,dir1);
  analogWrite(pwmPin1,50);
 //motor2
  digitalWrite(dirPin2,dir2);
  analogWrite(pwmPin2,50);
}
