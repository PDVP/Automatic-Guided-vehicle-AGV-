

// Pins
#define A1 2
#define B1 3
#define address1=0x02 >> 1 //Default slave address of this drive is 0x10. Arduino works on 7-bit I2C Addressing, so the Address value is shifted to right by 1 bit.

// globals
long prevT = 0; //previou time
int posPrev = 0; //previous position

// Use the "volatile" directive for variables used in an interrupt
volatile int pos_i = 0;
volatile float velocity_2 = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

float eintegral = 0;

void setup() {
   Serial.begin(115200);
  Wire.begin();
  delay(100);
  Wire.beginTransmission(address1);
  
  Wire.write(0);
  Wire.write(MAX_SPEED); // LSB Byte of Maximum Speed Value
  Wire.write(0); // MSB Byte of Maximum Speed Value
  Wire.endTransmission();
  delay(100);,

  pinMode(A1,INPUT_PULLUP);
  pinMode(B1,INPUT_PULLUP);
  

  attachInterrupt(digitalPinToInterrupt(A1),Funct1,RISING);
}

void loop() {

  // read the position and velocity
  int pos = 0;
  float velocity2 = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i;
  velocity2 = velocity_2;
  interrupts(); // turn interrupts back on

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1/600.0*60.0;
  float v2 = velocity2/600.0*60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

  // Set a target
  float vt = target;

  // Compute the control signal u
  float kp = 5;
  float ki = 10;
  float e = vt-v1Filt;
  eintegral = eintegral + e*deltaT;
  
  float u = kp*e + ki*eintegral;

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }
  setMotor(dir,pwr,address1);

  Serial.print(vt);
  Serial.print(" ");
  Serial.print(v1Filt);
  Serial.println();
  delay(1);
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
  int b = digitalRead(B1);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_2= increment/deltaT;
  prevT_i = currT;
}
