//--------------------------+++++++++++++++++++++++++++++++++++++++++--
#define CHANNEL_NUMBER 2//set the number of channel
#define CHANNEL_DEFAULT_VALUE 1500 //set the default value
#define CHANNEL_DEFAULT_VALUE_2 1500 //set the default value
#define FRAME_LENGTH 4800 //set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 1660 //set the pulse length in microseconds (The PPM signal pulse width must range from 600us to 2.4ms. The motor speed will be zero at PPM pulse width of 1.51ms)
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
//1
#define sigPin1 9  //set PPM signal output pin on the arduino
//2
#define sigPin1 10
int ppm[CHANNEL_NUMBER];
//1--------------------------+++++++++++++++++++++++++++++++++++++++++--
const int A1=3;
const int B1=8;
volatile long tick1 =0;

//2
const int A2=3;
const int B2=8;
volatile long tick2 =0;


//ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ
void setup(){  

//--------------------------+++++++++++++++++++++++++++++++++++++++++-----------------------------  //initiallize default ppm values
  //initiallize default ppm values
  for(int i=0; i<CHANNEL_NUMBER; i++){
      ppm[i]= CHANNEL_DEFAULT_VALUE;
      
  }

  pinMode(sigPin1, OUTPUT);
  digitalWrite(sigPin1, !onState);  //set the PPM signal pin to the default state (off)
  pinMode(sigPin2, OUTPUT);
  digitalWrite(sigPin2, !onState); 
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
  //--------------------------+++++++++++++++++++++++++++++++++++++++++-----------------------------
   Serial.begin(115200);
   //1
  pinMode(A1,INPUT_PULLUP);
  pinMode(B2,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(A1),Funct1,RISING);
  //2
    pinMode(A1,INPUT_PULLUP);
  pinMode(B2,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(A2),Funct2,RISING);
}





//ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ

void loop(){
  
  Serial.print(tick1);Serial.print('\t');  Serial.println(tick2);
  }
  


//ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ
ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if (state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PULSE_LENGTH * 2;
    state = false;
  } else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= CHANNEL_NUMBER){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PULSE_LENGTH; 
      OCR1A = (FRAME_LENGTH - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PULSE_LENGTH) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}

//ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ

void Funct1(){
 int b1 = digitalRead(B1);
  if(b1== 1){
    tick1 =tick1+1;
      }
  else if(b1==0){
   tick1=tick1-1;
   
  }

}
void Funct2(){
 int b2 = digitalRead(B2);
  if(b2== 1){
    tick2 =tick2+1;
      }
  else if(b2==0){
   tick2=tick2-1;
   
  }

}
