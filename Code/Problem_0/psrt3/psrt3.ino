
#define ENCA 3// YELLOW
#define ENCB 8 // WHITE

int encoderPin1 = 2;
int encoderPin2 = 5;

#define pwmPin1 9
#define dirPin1 11

volatile int lastEncoded = 0;
volatile long encoderValue = 0;

long lastencoderValue = 0;

int lastMSB = 0;
int lastLSB = 0;

void setup() {
  Serial.begin (115200);

  pinMode(encoderPin1, INPUT); 
  pinMode(encoderPin2, INPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(pwmPin1, OUTPUT);


  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt ENCODERV1 (pin 3) 
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);

}
int x= 1;

void loop(){ 
 
 

  Serial.println(encoderValue);
 
}
void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue ++;

  lastEncoded = encoded; //store this value for next time
}
