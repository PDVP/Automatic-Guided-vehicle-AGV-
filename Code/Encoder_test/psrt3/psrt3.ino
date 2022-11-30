#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 3// YELLOW
#define ENCB 8 // WHITE

volatile long posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT_PULLUP);
  pinMode(ENCB,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}

void loop() {
  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }

  Serial.println(posi);
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi=posi+100;
  }
  else{
   posi= posi-100;
  }
}
