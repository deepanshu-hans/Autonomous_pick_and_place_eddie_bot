import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

def CheckForDistance(pin):
    GPIO.setup(pin, GPIO.OUT)  
   
    GPIO.output(pin, 0)  
    time.sleep(0.000002)  
    GPIO.output(pin, 1)
    time.sleep(0.000005)  
    GPIO.output(pin, 0)  
  
    GPIO.setup(pin, GPIO.IN)  
  
  
    while GPIO.input(pin)==0:  
       starttime=time.time()  

  
    while GPIO.input(pin)==1:  
       endtime=time.time()  
        
    duration=endtime-starttime  
    distance=duration*34000/2
    
    return distance
    
while True:
    print('Left: ', CheckForDistance(36), 'Right: ' ,CheckForDistance(40))