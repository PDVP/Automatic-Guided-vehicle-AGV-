#define CHANNEL_NUMBER 1 //set the number of channel
#define CHANNEL_DEFAULT_VALUE 1500 //set the default value
#define FRAME_LENGTH 4800 //set the PPM frame length in microseconds (1ms = 1000µs)
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 9  //set PPM signal output pin on the arduino
int PULSE_LENGTH; //set the pulse length in microseconds (The PPM signal pulse width must range from 600us to 2.4ms. The motor speed will be zero at PPM pulse width of 1.51ms)
int ppm[CHANNEL_NUMBER];

////////////////////////
const int A=3; //encoderA to pin3 and encoderB to pin8
const int B=8;
volatile long tick =0; //this where the bug is present //unsigned


 int Encoder_cpr ;
 int distance = ?;
//ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZzzzzzzzzzzzzzzzzzzzzzzZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ
void setup(){  
   Serial.begin(115200);
  //initiallize default ppm values
  for(int i=0; i<CHANNEL_NUMBER; i++){
      ppm[i]= CHANNEL_DEFAULT_VALUE;
  }

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  
  pinMode(A,INPUT_PULLUP);
  pinMode(B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(A),Funct,RISING);
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();

  //////////////
  Encoder_cpr=((distance/44)*7*133600*radius); //wheel radius
}

//ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZzzzzzzzzzzzzzzzzzzzzzzZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ
void loop(){
  Serial.println(tick);
  }

//ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZzzzzzzzzzzzzzzzzzzzzzzZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ
ISR(TIMER1_COMPA_vect){  //leave this alone

  
static boolean state ;
 
 if( tick >= Encoder_cpr){
   PULSE_LENGTH = 1510 ;
   state = false;
 }else{ state = true;}
 
 
  
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

//ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZzzzzzzzzzzzzzzzzzzzzzzzzZZZZZZZZZZZZZZZZZZZZZzzz

void Funct(){
 int b = digitalRead(B);
  if(b== 1){
    tick =tick+100;
      }
  else if(b==0){
   tick=tick-100;
   
  }

}
