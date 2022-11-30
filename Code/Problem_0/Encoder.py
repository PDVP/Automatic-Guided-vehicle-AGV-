
import RPi.GPIO as GPIO          
from time import sleep

a1=23
b1=24
tick=0

GPIO.setmode(GPIO.BCM)
GPIO.setup(a1,GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(b1,GPIO.INPUT)

def Funct():
  b = GPIO.input(b1)
  
  if(b== 1):

    tick =tick+1
    
  elif(b==0):
   tick=tick-1

GPIO.add_event_detect(b1, GPIO.RISING, callback=Funct, bouncetime=1)



while(1):

 print("\n"+ tick)

   
  
'''https://www.youtube.com/watch?v=uIyoWnbjT5k'''





