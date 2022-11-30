import RPi.GPIO as GPIO          
from time import sleep


GPIO.setmode(GPIO.BCM)

'Motor1'
Dr1 = 20
P1 = 21
a1=23
b1=24
tick1=0
GPIO.setup(Dr1,GPIO.OUT)
GPIO.setup(P1,GPIO.OUT)

GPIO.setup(a1,GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(b1,GPIO.INPUT)

pwm1 = GPIO.PWM(P1,100)
pwm1.start(0)

def Funct1():
  b = GPIO.input(b1)
  
  if(b== 1):

    tick1 =tick1+1
    
  elif(b==0):
   tick1=tick1-1

GPIO.add_event_detect(a1, GPIO.RISING, callback=Funct1, bouncetime=1)

'Motor2 -----------------------------------------------------------'
Dr2 = 20
P2 = 21
a2=23
b2=24
tick2=0
GPIO.setup(Dr2,GPIO.OUT)
GPIO.setup(P2,GPIO.OUT)

GPIO.setup(a2,GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(b2,GPIO.INPUT)
pwm2 = GPIO.PWM(P2,100)
pwm2.start(0)

def Funct2():
  b = GPIO.input(b2)
  
  if(b== 1):

    tick2=tick2+1
    
  elif(b==0):
   tick2=tick2-1

GPIO.add_event_detect(a2, GPIO.RISING, callback=Funct2, bouncetime=1)


distance =100 #in cms
radius = 3.5
Encoder_cpr=((distance/(44*radius))*7*133600); # radius= wheel radius

'-----------------------------------------------------------'

while(1):
    GPIO.output(Dr1,LOW)
    GPIO.output(Dr2,LOW)

    if( tick1 >= Encoder_cpr):
        pwm1= GPIO.ChangeDutyCycle(0)
    else:  
        pwm1= GPIO.ChangeDutyCycle(20)  

    if( tick2 >= Encoder_cpr):
        pwm2= GPIO.ChangeDutyCycle(0)
    else:  
        pwm2= GPIO.ChangeDutyCycle(20)
     
    print("Motor1.pos =", tick1 , "Motor2.pos = ",tick2)
    sleep(5)
