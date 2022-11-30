const int A=2;
const int B=3;
volatile long tick =0; //this where the bug is present //unsigned
 void setup(){
  Serial.begin(115200);
  pinMode(A,INPUT_PULLUP);
  pinMode(B,INPUT);
  attachInterrupt(digitalPinToInterrupt(A),Funct,RISING); 
  
}
void loop(){
 
  Serial.println(tick);
}
void Funct(){
 int b = digitalRead(B);
  if(b== 1){
    tick =tick+1;
      }
  else if(b==0){
   tick=tick-1;
   
  }

}
